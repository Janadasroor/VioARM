#!/usr/bin/env python3
"""Fetch CMSIS device packs into the packs/ directory (which is gitignored).

Usage:
    python3 scripts/fetch_packs.py            # fetch everything listed in PACKS
    python3 scripts/fetch_packs.py STM32F1    # fetch a single family by name
    python3 scripts/fetch_packs.py --list     # print the configured pack list
"""

from __future__ import annotations

import argparse
import shutil
import sys
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PACKS_DIR = ROOT / "packs"

# name: (base_url, filename, extract_subdir_flag)
# URL pattern follows the CMSIS pack convention:
#   <base_url>/<Vendor>.<Name>.<Version>.pack
PACKS: dict[str, dict[str, str]] = {
    "STM32F1": {
        "vendor": "Keil",
        "name": "STM32F1xx_DFP",
        "version": "2.4.1",
        "url": "https://www.keil.com/pack/",
    },
}


def fetch(name: str) -> Path:
    if name not in PACKS:
        print(f"error: unknown pack '{name}'; choices: {', '.join(PACKS)}", file=sys.stderr)
        sys.exit(1)

    pack = PACKS[name]
    filename = f"{pack['vendor']}.{pack['name']}.{pack['version']}.pack"
    url = pack["url"] + filename
    dest = PACKS_DIR / filename
    extract_dir = PACKS_DIR / f"{pack['vendor']}.{pack['name']}"

    PACKS_DIR.mkdir(parents=True, exist_ok=True)

    if dest.exists() and extract_dir.exists():
        print(f"already present: {dest.name} (use --force to refetch)")
        return dest

    print(f"fetching {url}")
    with urllib.request.urlopen(url) as resp, open(dest, "wb") as f:
        shutil.copyfileobj(resp, f)
    print(f"saved {dest} ({dest.stat().st_size} bytes)")

    print(f"extracting to {extract_dir}")
    with zipfile.ZipFile(dest) as zf:
        zf.extractall(extract_dir)
    print(f"done: {extract_dir}")
    return dest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("names", nargs="*", help="pack family names to fetch")
    parser.add_argument("--list", action="store_true", help="list configured packs")
    args = parser.parse_args()

    if args.list:
        for name, pack in PACKS.items():
            print(f"{name}: {pack['vendor']}.{pack['name']} {pack['version']}")
        return 0

    names = args.names or list(PACKS)
    for name in names:
        fetch(name)
    return 0


if __name__ == "__main__":
    sys.exit(main())