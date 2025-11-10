#!/usr/bin/env bash
set -Eeuo pipefail

# Usage:
#   ./spawn_aruco_cube.sh NAME START_ID [X Y Z [YAW] [DICT] [PX]]
#   NAME     : Gazebo model name (e.g., cube1)
#   START_ID : first ArUco ID (sides START..START+3, top START+4)
#   X,Y,Z    : position (default 0,0,0.05)
#   YAW      : yaw in radians (default 0)
#   DICT     : OpenCV dict name (default DICT_4X4_100)
#   PX       : PNG size in pixels (default 512)
#
# Examples:
#   ./spawn_aruco_cube.sh cube1 1
#   ./spawn_aruco_cube.sh cube2 6 0.1 0 0.05 1.5708 DICT_4X4_100 600

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XACRO_FILE="$DIR/aruco_cube.urdf.xacro"
MAT_DIR="$DIR/materials/scripts"
TEX_DIR="$DIR/materials/textures"
MAT_FILE="$MAT_DIR/aruco.material"

NAME="${1:-cube1}"
START="${2:-1}"
X="${3:-0}"
Y="${4:-0}"
Z="${5:-0.05}"
YAW="${6:-0}"
DICT="${7:-DICT_4X4_100}"
PX="${8:-512}"

require() { command -v "$1" >/dev/null 2>&1 || { echo "Missing '$1' in PATH"; exit 1; }; }

# --- deps ---
require rosservice
if command -v xacro >/dev/null 2>&1; then XACRO_BIN="xacro"; else require rosrun; XACRO_BIN="rosrun xacro xacro"; fi
[[ -f "$XACRO_FILE" ]] || { echo "Not found: $XACRO_FILE"; exit 1; }

mkdir -p "$MAT_DIR" "$TEX_DIR"

# --- ensure make_aruco_textures.py exists (use local or temp fallback) ---
PY_GEN="$DIR/make_aruco_textures.py"
if [[ ! -f "$PY_GEN" ]]; then
  PY_GEN="$(mktemp)"
  cat >"$PY_GEN" <<'PY'
#!/usr/bin/env python3
import argparse, os
from pathlib import Path
try:
  import cv2
  import numpy as np
  DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50, "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250, "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50, "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250, "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50, "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250, "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50, "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250, "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
  }
except Exception as e:
  raise SystemExit("OpenCV with aruco module is required. Try: pip install opencv-contrib-python") from e

ap = argparse.ArgumentParser()
ap.add_argument("--ids", type=int, nargs="+", required=True)
ap.add_argument("--dict", default="DICT_4X4_100", choices=list(DICTS.keys()))
ap.add_argument("--px", type=int, default=512)
ap.add_argument("--out", default="materials/textures")
args = ap.parse_args()

out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
aruco_dict = cv2.aruco.getPredefinedDictionary(DICTS[args.dict])
for id_ in args.ids:
  img = cv2.aruco.drawMarker(aruco_dict, id_, args.px, borderBits=1)
  cv2.imwrite(str(out / f"aruco_{id_}.png"), img)
# blank helper
blank = out / "blank.png"
if not blank.exists():
  import numpy as np
  cv2.imwrite(str(blank), 255*np.ones((args.px,args.px), dtype=np.uint8))
PY
  chmod +x "$PY_GEN"
fi

# --- generate missing textures on demand ---
IDS=("$START" "$((START+1))" "$((START+2))" "$((START+3))" "$((START+4))")
MISSING=()
for id in "${IDS[@]}"; do
  [[ -f "$TEX_DIR/aruco_${id}.png" ]] || MISSING+=("$id")
done

if (( ${#MISSING[@]} > 0 )); then
  echo "[spawn] generating missing textures via Python for IDs: ${MISSING[*]}"
  require python3
  python3 "$PY_GEN" --ids "${MISSING[@]}" --dict "$DICT" --px "$PX" --out "$TEX_DIR"
fi

# ensure blank exists (for bottoms if you ever want it)
[[ -f "$TEX_DIR/blank.png" ]] || python3 "$PY_GEN" --ids "${IDS[0]}" --dict "$DICT" --px "$PX" --out "$TEX_DIR" >/dev/null 2>&1 || true

# --- (re)generate minimal OGRE material entries for these IDs + blank ---
touch "$MAT_FILE"
URDF_TMP="" 
tmp="$(mktemp)"; trap 'rm -f "$tmp" "$URDF_TMP"' EXIT
# remove old definitions for current IDs and 'blank'
awk -v ids="$(printf "%s " "${IDS[@]}")" '
  BEGIN{ split(ids,A); for(i in A) K[A[i]]=1; K["blank"]=1 }
  { print }
' "$MAT_FILE" | awk '
  BEGIN{del=0}
  /^material[ \t]+/ {
    if ($2=="blank") {del=1}
    else {
      # get postfix (aruco_XX)
      split($2, a, "_"); if (a[1]=="aruco" && a[2]!="") {
        for(i in a){}; # noop
      }
    }
  }
  { print }
' >/dev/null 2>&1 # (we'll just rewrite the file fresh below for simplicity)

# simpler: rewrite a fresh file containing only what we need now
cat >"$MAT_FILE" <<EOF
material blank { technique { pass { lighting off ambient 1 1 1 1 diffuse 1 1 1 1 specular 0 0 0 1 0 } } }
EOF
for id in "${IDS[@]}"; do
cat >>"$MAT_FILE" <<EOF
material aruco_${id}
{
  technique
  {
    pass
    {
      lighting off
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 1 0
      texture_unit { texture aruco_${id}.png }
    }
  }
}
EOF
done

# ... after the DIR="$(cd ...)" assignment
MODEL_ROOT="$(dirname "$DIR")" # Parent directory of where the script lives

# Export the parent directory to GAZEBO_MODEL_PATH for model:// URIs
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$MODEL_ROOT"

# --- wait for Gazebo service ---
echo "[spawn] waiting for /gazebo/spawn_urdf_model ..."
for i in $(seq 1 120); do
  rosservice list 2>/dev/null | grep -q /gazebo/spawn_urdf_model && break
  sleep 1
  [[ $i -eq 120 ]] && { echo "Timeout waiting for Gazebo"; exit 1; }
done


# --- build URDF and spawn ---
URDF_TMP="$(mktemp)"
$XACRO_BIN "$XACRO_FILE" name:="$NAME"_link start_id:="$START" tex_root:="$DIR" > "$URDF_TMP"

# delete if exists
if rosservice list | grep -q /gazebo/delete_model; then
  if rosservice call /gazebo/get_model_state "{model_name: '$NAME', relative_entity_name: 'world'}" 2>/dev/null | grep -q "success: True"; then
    echo "[spawn] deleting existing model: $NAME"
    rosservice call /gazebo/delete_model "{model_name: '$NAME'}" || true
  fi
fi

# yaw -> quaternion
read -r QX QY QZ QW < <(python3 - "$YAW" <<'PY'
import math,sys
try:
    yaw=float(sys.argv[1]); r=p=0.0
    cy,sy=math.cos(yaw*0.5),math.sin(yaw*0.5)
    qw=cy; qx=0.0; qy=0.0; qz=sy
    print(qx,qy,qz,qw)
except IndexError:
    print("0.0 0.0 0.0 1.0") # Default to no rotation if arg missing/bad
except ValueError:
    print("0.0 0.0 0.0 1.0") # Handle if float conversion fails
PY
)

echo "[spawn] spawning $NAME: IDs $START..$((START+4)) at ($X,$Y,$Z) yaw=$YAW"
rosservice call /gazebo/spawn_urdf_model "model_name: '$NAME'
model_xml: '$(<"$URDF_TMP")'
robot_namespace: ''
initial_pose:
  position: {x: $X, y: $Y, z: $Z}
  orientation: {x: $QX, y: $QY, z: $QZ, w: $QW}
reference_frame: 'world'"
echo "[spawn] done."
