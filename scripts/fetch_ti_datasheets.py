#!/usr/bin/env python3
"""Fetch TI datasheets into the datasheets/ directory (which is gitignored).

The PDFs are TI-copyrighted distribution files: fetched from ti.com at
runtime like the CMSIS packs, never committed. Mirrors
scripts/fetch_packs.py (same CLI shape, same skip-if-present +
SHA256-verified download).

Usage:
    python3 scripts/fetch_ti_datasheets.py           # fetch everything listed
    python3 scripts/fetch_ti_datasheets.py SPMS376E  # fetch a single document
    python3 scripts/fetch_ti_datasheets.py --list    # print the configured list
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATASHEETS_DIR = ROOT / "datasheets"

# lit: official TI literature number. Every TI datasheet cited anywhere in
# vioarm-core (grep SPMS) must appear here so a fresh machine reproduces
# datasheets/ byte-identically. URL pattern follows the TI convention:
#   https://www.ti.com/lit/ds/symlink/<lit-lowercase>.pdf
# Hashes pinned 2026-09-25 (Rev. E / Rev. B printings).
DATASHEETS: dict[str, dict[str, str]] = {
    "SPMS376E": {
        "title": "Tiva C Series TM4C123GH6PM Microcontroller Data Sheet",
        "url": "https://www.ti.com/lit/ds/symlink/spms376e.pdf",
        "sha256": "108350fb9d6cabfcb581d65e7cc7ab54fc197aa9a23cad17e13772c7dbd2930b",
    },
    "SPMS433B": {
        "title": "Tiva C Series TM4C1294NCPDT Microcontroller Data Sheet",
        "url": "https://www.ti.com/lit/ds/symlink/spms433b.pdf",
        "sha256": "41e70b4470c9272fd6ae9ca492f1be74cc653214df93e3365af2fc166353e8d6",
    },
}


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def download(url: str, dest: Path, want_hash: str) -> None:
    """Download with retries, resume, and atomic replace (mirrors
    fetch_packs.py: ti.com also resets mid-file; dest is only replaced
    after the hash verifies)."""
    part = dest.with_name(dest.name + ".part")
    for attempt in range(1, 6):
        try:
            resume_from = part.stat().st_size if part.exists() else 0
            req = urllib.request.Request(url)
            if resume_from:
                req.add_header("Range", f"bytes={resume_from}-")
            with urllib.request.urlopen(req, timeout=120) as resp:
                mode = "ab" if resume_from and resp.status == 206 else "wb"
                with open(part, mode) as f:
                    shutil.copyfileobj(resp, f)
            break
        except Exception as e:
            print(f"attempt {attempt}/5 failed: {e}")
            if attempt == 5:
                raise SystemExit(f"download failed after 5 attempts: {url}")
            time.sleep(min(60, 5 * attempt))
    got = sha256_of(part)
    if got != want_hash:
        part.unlink(missing_ok=True)
        raise SystemExit(f"hash mismatch on {dest.name}: got {got}, want {want_hash}")
    print(f"sha256 ok: {dest.name}")
    part.replace(dest)
    print(f"saved {dest} ({dest.stat().st_size} bytes)")


def fetch(name: str, force: bool = False) -> Path:
    if name not in DATASHEETS:
        print(
            f"error: unknown datasheet '{name}'; choices: {', '.join(DATASHEETS)}",
            file=sys.stderr,
        )
        sys.exit(1)

    doc = DATASHEETS[name]
    filename = f"{name.lower()}.pdf"
    dest = DATASHEETS_DIR / filename
    want_hash = doc["sha256"]

    DATASHEETS_DIR.mkdir(parents=True, exist_ok=True)

    if dest.exists() and not force:
        got = sha256_of(dest)
        if got != want_hash:
            print(f"hash mismatch on cached {dest.name}: got {got}, want {want_hash}")
            print("refetch with --force")
            sys.exit(1)
        print(f"already present: {dest.name} (use --force to refetch)")
        return dest

    print(f"fetching {doc['title']}\n  {doc['url']}")
    download(doc["url"], dest, want_hash)
    return dest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("names", nargs="*", help="datasheet literature numbers to fetch")
    parser.add_argument("--list", action="store_true", help="list configured datasheets")
    parser.add_argument("--force", action="store_true", help="refetch even if present")
    args = parser.parse_args()

    if args.list:
        for name, doc in DATASHEETS.items():
            print(f"{name}: {doc['title']}")
        return 0

    names = args.names or list(DATASHEETS)
    for name in names:
        fetch(name, force=args.force)
    return 0


if __name__ == "__main__":
    sys.exit(main())
