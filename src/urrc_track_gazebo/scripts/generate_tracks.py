#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path
PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from urrc_tracks import TRACKS
from urrc_tracks.build import build_all

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Regenerate portable assets using local source data")
    parser.add_argument("tracks", nargs="*", metavar="TRACK", help="Names: " + ", ".join(TRACKS))
    args = parser.parse_args()
    if any(name not in TRACKS for name in args.tracks):
        parser.error("Unknown track. Available: " + ", ".join(TRACKS))
    build_all(PACKAGE, args.tracks or TRACKS)
