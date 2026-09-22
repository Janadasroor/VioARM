#!/usr/bin/env python3
"""Fetch CMSIS device packs into the packs/ directory (which is gitignored).

Usage:
    python3 scripts/fetch_packs.py            # fetch everything listed in PACKS
    python3 scripts/fetch_packs.py STM32F1    # fetch a single family by name
    python3 scripts/fetch_packs.py --list     # print the configured pack list
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PACKS_DIR = ROOT / "packs"

# name: pack coordinates. Mirrors packs/PROVENANCE.md exactly (vendor,
# DFP name, version, SHA256); every pre-downloaded pack must appear here
# so a fresh machine reproduces packs/ byte-identically.
# URL pattern follows the CMSIS pack convention:
#   <base_url>/<Vendor>.<Name>.<Version>.pack
PACKS: dict[str, dict[str, str]] = {
    "STM32F0": {
        "vendor": "Keil",
        "name": "STM32F0xx_DFP",
        "version": "2.1.0",
        "url": "https://www.keil.com/pack/",
        "sha256": "ba8a448690a4cb7a0dd7284e179f93256e6d7b7710e4e8ca75b360a2456b8684",
    },
    "STM32F1": {
        "vendor": "Keil",
        "name": "STM32F1xx_DFP",
        "version": "2.4.1",
        "url": "https://www.keil.com/pack/",
        "sha256": "807ea15da5b172b916bbc47b2b87f1e621240ad208d38e82a417a2ef8191e9d1",
    },
    "STM32F2": {
        "vendor": "Keil",
        "name": "STM32F2xx_DFP",
        "version": "3.1.0",
        "url": "https://www.keil.com/pack/",
        "sha256": "0c36d60606fd52d559d7527ac780c9021014ab50c69418ec810a40ed2d7b6ae5",
    },
    "STM32F3": {
        "vendor": "Keil",
        "name": "STM32F3xx_DFP",
        "version": "2.2.2",
        "url": "https://www.keil.com/pack/",
        "sha256": "47a41da7268a2cba41eab8a2f6222ac76aad6cc67afc2f892e9069c46e038e30",
    },
    "STM32F4": {
        "vendor": "Keil",
        "name": "STM32F4xx_DFP",
        "version": "2.17.1",
        "url": "https://www.keil.com/pack/",
        "sha256": "c7e0f3595aa5ee4c2da2c0de79598b8a91fc5f4f77e16c3b39c674429fb0f4eb",
    },
    "STM32F7": {
        # PROVENANCE records the .pdsc hash (pack not retained locally);
        # no pack hash to verify against, so verification is skipped.
        "vendor": "Keil",
        "name": "STM32F7xx_DFP",
        "version": "2.15.1",
        "url": "https://www.keil.com/pack/",
    },
    "STM32G0": {
        # Same as F7: only the extracted tree + .pdsc hash exist locally.
        "vendor": "Keil",
        "name": "STM32G0xx_DFP",
        "version": "1.4.0",
        "url": "https://www.keil.com/pack/",
    },
    "STM32G4": {
        "vendor": "Keil",
        "name": "STM32G4xx_DFP",
        "version": "1.5.0",
        "url": "https://www.keil.com/pack/",
        "sha256": "5c878bb8b2de2120153b9570e789ea216b9a1a0903dbe1e154ceb1ff0a4d2972",
    },
    "STM32H5": {
        "vendor": "Keil",
        "name": "STM32H5xx_DFP",
        "version": "2.3.1",
        "url": "https://www.keil.com/pack/",
        "sha256": "906b2f8802845ed21c4bc31512a816065991bd9839f1441d0126a667a50347b1",
    },
    "STM32H7": {
        "vendor": "Keil",
        "name": "STM32H7xx_DFP",
        "version": "3.1.1",
        "url": "https://www.keil.com/pack/",
        "sha256": "53ef531aa7ec36ec5b9c489789fdeda6de5abeb3f9c1b91fb1d96f941b98bbd7",
    },
    "STM32L4": {
        "vendor": "Keil",
        "name": "STM32L4xx_DFP",
        "version": "2.6.2",
        "url": "https://www.keil.com/pack/",
        "sha256": "28c4985f0d0fcbcf2e97eea1cc496d0f243e8a54955008ed32e08c45c47c7c5b",
    },
    "STM32L5": {
        "vendor": "Keil",
        "name": "STM32L5xx_DFP",
        "version": "2.1.0",
        "url": "https://www.keil.com/pack/",
        "sha256": "190ebdd8615107f9be69cd7a8bd9710e4c4240ed69c5886a5d7694a6d8c74c31",
    },
    "STM32U5": {
        "vendor": "Keil",
        "name": "STM32U5xx_DFP",
        "version": "3.2.0",
        "url": "https://www.keil.com/pack/",
        "sha256": "e320687fe534f2fe6902e9bcdee981315abea26c4ca547142af9b9439e958be6",
    },
    "STM32WBA": {
        "vendor": "Keil",
        "name": "STM32WBAxx_DFP",
        "version": "2.2.0",
        "url": "https://www.keil.com/pack/",
        "sha256": "629532e024093307f0a46e47abbf1fe7826923720d9ad108684b0955316ea51a",
    },
    "ARM-CMSIS": {
        "vendor": "ARM",
        "name": "CMSIS",
        "version": "5.9.0",
        "url": "https://www.keil.com/pack/",
        "sha256": "14b366f2821ee5d32f0d3bf48ef9657ca45347261d0531263580848e9d36f8f4",
    },
    "TM4C": {
        "vendor": "Keil",
        "name": "TM4C_DFP",
        "version": "1.1.0",
        "url": "https://www.keil.com/pack/",
        "sha256": "127dcaf64e1b64674ae902239b6afa612701d8a73031492d65205f9aee2a5e8e",
    },
}


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch(name: str, force: bool = False) -> Path:
    if name not in PACKS:
        print(f"error: unknown pack '{name}'; choices: {', '.join(PACKS)}", file=sys.stderr)
        sys.exit(1)

    pack = PACKS[name]
    filename = f"{pack['vendor']}.{pack['name']}.{pack['version']}.pack"
    url = pack["url"] + filename
    dest = PACKS_DIR / filename
    extract_dir = PACKS_DIR / f"{pack['vendor']}.{pack['name']}"
    want_hash = pack.get("sha256")

    PACKS_DIR.mkdir(parents=True, exist_ok=True)

    if dest.exists() and extract_dir.exists() and not force:
        if want_hash:
            got = sha256_of(dest)
            if got != want_hash:
                print(f"hash mismatch on cached {dest.name}: got {got}, want {want_hash}")
                print("refetch with --force")
                sys.exit(1)
        print(f"already present: {dest.name} (use --force to refetch)")
        return dest

    print(f"fetching {url}")
    with urllib.request.urlopen(url) as resp, open(dest, "wb") as f:
        shutil.copyfileobj(resp, f)
    print(f"saved {dest} ({dest.stat().st_size} bytes)")

    if want_hash:
        got = sha256_of(dest)
        if got != want_hash:
            print(f"hash mismatch on {dest.name}: got {got}, want {want_hash}", file=sys.stderr)
            sys.exit(1)
        print(f"sha256 ok: {dest.name}")

    print(f"extracting to {extract_dir}")
    with zipfile.ZipFile(dest) as zf:
        zf.extractall(extract_dir)
    print(f"done: {extract_dir}")
    # Normalize SVD location: F0/F3 packs use CMSIS/SVD, F1 uses SVD
    cmsis_svd = extract_dir / "CMSIS" / "SVD"
    top_svd = extract_dir / "SVD"
    if not top_svd.exists() and cmsis_svd.exists():
        try:
            top_svd.symlink_to(cmsis_svd, target_is_directory=True)
        except Exception:
            shutil.copytree(cmsis_svd, top_svd)
        print(f"linked {cmsis_svd} -> {top_svd}")
    return dest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("names", nargs="*", help="pack family names to fetch")
    parser.add_argument("--list", action="store_true", help="list configured packs")
    parser.add_argument("--force", action="store_true", help="refetch even if present")
    args = parser.parse_args()

    if args.list:
        for name, pack in PACKS.items():
            print(f"{name}: {pack['vendor']}.{pack['name']} {pack['version']}")
        return 0

    names = args.names or list(PACKS)
    for name in names:
        fetch(name, force=args.force)
    return 0


if __name__ == "__main__":
    sys.exit(main())