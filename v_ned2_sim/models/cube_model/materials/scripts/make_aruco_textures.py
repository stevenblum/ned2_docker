#!/usr/bin/env python3
"""
Generate ArUco marker PNGs to ./materials/textures as aruco_<ID>.png

Usage examples:
  python3 make_aruco_textures.py --start 1 --count 25
  python3 make_aruco_textures.py --ids 1 5 6 9 10
  python3 make_aruco_textures.py --start 6 --count 5 --dict DICT_4X4_100 --px 600

Requires OpenCV with aruco module. If missing:
  pip install --no-cache-dir opencv-contrib-python
"""
import argparse, os, sys
import cv2
from pathlib import Path

DICT_NAMES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

def parse_args():
    p = argparse.ArgumentParser(description="Generate ArUco textures")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--start", type=int, help="first ID")
    p.add_argument("--count", type=int, default=5, help="how many IDs from --start")
    g.add_argument("--ids", type=int, nargs="+", help="explicit list of IDs")
    p.add_argument("--dict", dest="dict_name", default="DICT_4X4_100", choices=DICT_NAMES.keys(),
                   help="ArUco dictionary")
    p.add_argument("--px", type=int, default=512, help="marker image size in pixels (square)")
    p.add_argument("--border", type=int, default=1, help="black border bits for drawMarker")
    p.add_argument("--out", default="materials/textures", help="output folder for PNGs")
    return p.parse_args()

def main():
    args = parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Determine IDs
    if args.ids:
        ids = args.ids
    else:
        if args.start < 0 or args.count < 1:
            print("Invalid --start/--count", file=sys.stderr)
            sys.exit(2)
        ids = list(range(args.start, args.start + args.count))

    # Validate dict size vs max ID (OpenCV will raise if out-of-range)
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_NAMES[args.dict_name])

    # Generate each PNG
    for id_ in ids:
        img = cv2.aruco.drawMarker(aruco_dict, id_, args.px, borderBits=args.border)
        out_path = out_dir / f"aruco_{id_}.png"
        ok = cv2.imwrite(str(out_path), img)
        if not ok:
            print(f"Failed to write {out_path}", file=sys.stderr)
            sys.exit(1)
        print(f"Wrote {out_path}")

    # Also drop a simple white 'blank.png' if helpful for bottoms
    blank_path = out_dir / "blank.png"
    if not blank_path.exists():
        import numpy as np
        cv2.imwrite(str(blank_path), 255 * np.ones((args.px, args.px), dtype=np.uint8))
        print(f"Wrote {blank_path}")

if __name__ == "__main__":
    main()
