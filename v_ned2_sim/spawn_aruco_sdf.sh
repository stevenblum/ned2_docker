# spawn_cube.sh  (replace your current one)
#!/usr/bin/env bash
set -e

# Usage: ./spawn_cube.sh <NUM> [x y z [yaw]]
NUM="${1:?Usage: ./spawn_cube.sh <NUM> [x y z [yaw]]}"
X="${2:-0.15}"; Y="${3:-0.15}"; Z="${4:-0.05}"; YAW="${5:-0}"

# Where SDFs live (can override with env SDF_DIR)
SDF_DIR="${SDF_DIR:-/volume/models/cube_model}"
SDF="$SDF_DIR/cube${NUM}.sdf"
MODEL="cube${NUM}"

[[ -f /opt/ros/noetic/setup.bash ]] && source /opt/ros/noetic/setup.bash || true

# Wait for Gazebo spawn service
for i in {1..120}; do
  rosservice list 2>/dev/null | grep -q /gazebo/spawn_sdf_model && break
  sleep 1
  [[ $i -eq 120 ]] && { echo "Timeout waiting for /gazebo/spawn_sdf_model"; exit 1; }
done

[[ -f "$SDF" ]] || { echo "Missing: $SDF"; exit 1; }

rosservice call /gazebo/delete_model "{model_name: '$MODEL'}" >/dev/null 2>&1 || true

rosrun gazebo_ros spawn_model \
  -sdf -file "$SDF" \
  -model "$MODEL" \
  -x "$X" -y "$Y" -z "$Z" -Y "$YAW"

echo "[ok] spawned $MODEL from $SDF at ($X,$Y,$Z), yaw=$YAW"