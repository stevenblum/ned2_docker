#!/usr/bin/env python3
"""
make_aruco_textures.py
Generate ArUco marker PNGs with OpenCV (contrib). Works whether your OpenCV
exposes aruco.drawMarker or aruco.generateImageMarker.

Examples:

  *** Be in the "cube_model" folder
  
  # IDs 1..99 into ./materials/textures
  python3 make_aruco_textures.py --start 1 --count 99

  # Specific IDs into a custom folder with a different dictionary and size
  python3 make_aruco_textures.py --ids 1 6 11 16 --dict DICT_4X4_100 --px 512 --out materials/textures
"""
import argparse
import sys
from pathlib import Path

# --- OpenCV import & checks ---------------------------------------------------
try:
    import cv2
except Exception:
    sys.exit("ERROR: OpenCV not installed. Try: pip install --upgrade opencv-contrib-python")

if not hasattr(cv2, "aruco"):
    sys.exit("ERROR: cv2.aruco not found. Install the contrib build: pip install opencv-contrib-python")

# Pick a generator that exists on this version
def _pick_generator():
    if hasattr(cv2.aruco, "generateImageMarker"):
        def gen(dict_obj, marker_id, px, borderBits):
            return cv2.aruco.generateImageMarker(dict_obj, marker_id, px, borderBits=borderBits)
        return gen
    if hasattr(cv2.aruco, "drawMarker"):
        def gen(dict_obj, marker_id, px, borderBits):
            # drawMarker returns the image in newer OpenCV; older may fill a passed array.
            return cv2.aruco.drawMarker(dict_obj, marker_id, px, borderBits=borderBits)
        return gen
    return None

GEN = _pick_generator()
if GEN is None:
    sys.exit("ERROR: Neither aruco.generateImageMarker nor aruco.drawMarker is available. "
             "Upgrade opencv-contrib-python.")

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
    p = argparse.ArgumentParser(description="Generate ArUco marker PNGs")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--start", type=int, help="first marker ID (inclusive)")
    g.add_argument("--ids", type=int, nargs="+", help="explicit list of IDs")
    p.add_argument("--count", type=int, default=5, help="how many to generate from --start (default 5)")
    p.add_argument("--dict", dest="dict_name", default="DICT_4X4_100", choices=DICT_NAMES.keys(),
                   help="ArUco dictionary")
    p.add_argument("--px", type=int, default=512, help="image size (square pixels)")
    p.add_argument("--border", type=int, default=1, help="black border bits (default 1)")
    p.add_argument("--out", default="materials/textures", help="output directory")
    p.add_argument("--blank", action="store_true", help="also write a white blank.png of size --px")
    return p.parse_args()

def main():
    args = parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    if args.ids:
        ids = args.ids
    else:
        if args.start is None or args.count < 1:
            sys.exit("Invalid --start/--count")
        ids = list(range(args.start, args.start + args.count))

    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_NAMES[args.dict_name])

    for mid in ids:
        img = GEN(aruco_dict, mid, args.px, args.border)
        if img is None:
            sys.exit("OpenCV returned None from ArUco generator; try upgrading opencv-contrib-python.")
        out_path = out / f"aruco_{mid}.png"
        ok = cv2.imwrite(str(out_path), img)
        if not ok:
            sys.exit(f"Failed to write {out_path}")
        print("Wrote", out_path)

    if args.blank:
        try:
            import numpy as np
            blank_path = out / "blank.png"
            if cv2.imwrite(str(blank_path), 255 * np.ones((args.px, args.px), dtype="uint8")):
                print("Wrote", blank_path)
        except Exception:
            print("[warn] numpy not available; skipping blank.png")

if __name__ == "__main__":
    main()
