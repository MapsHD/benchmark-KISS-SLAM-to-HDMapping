#!/bin/bash
# Runs KISS-SLAM on a bag and converts the result to an HDMapping session.
#
# KISS-SLAM runs OFFLINE, like PIN-SLAM and HP2-SLAM: KISS-ICP's dataloader
# reads the bag (ROS 1 .bag file or ROS 2 bag directory) directly through the
# rosbags library, so there is no ROS, no playback and no recording. It writes
# its trajectory in TUM format, stamped with the bag time of each scan. The
# converter then rebuilds the dense world-frame cloud from the bag and the
# poses and writes the HDMapping session. Live progress is shown in KISS-SLAM's
# own Open3D viewer (instead of RViz).
#
# Usage:
#   ./docker_session_run-kiss-slam.sh <input.bag | ros2_bag_dir> <output_dir>
#
# Environment variables:
#   CONFIG_FILE         - KISS-SLAM parameter file on the host (default: none =
#                         the author's defaults, see `kiss_slam_dump_config`)
#   TOPIC               - PointCloud2 topic in the bag (default: /livox/pointcloud)
#   VIS                 - 1 = KISS-SLAM's Open3D viewer (default: 1; auto-off without DISPLAY)
#   LIBGL_SW            - 1 = Mesa software rendering for the viewer (default: 1)
#   POINT_SKIP          - converter keeps every Nth point (default 1 = all)
#   CONVERT_RESULTS_DIR - container path of an existing KISS-SLAM results dir
#                         (e.g. /output/kiss-slam-results/<timestamp>): skip SLAM, only convert

IMAGE_NAME='kiss-slam_standalone'
OUTPUT_NAME='output_hdmapping-kiss-slam'
RESULTS_NAME='kiss-slam-results'

DATASET_CONTAINER_DIR='/kiss_ws/dataset'
OUTPUT_CONTAINER='/output'
CONFIG_MOUNT_CONTAINER='/kiss_ws/config/kiss_slam.yaml'

CONFIG_FILE="${CONFIG_FILE:-}"
TOPIC="${TOPIC:-/livox/pointcloud}"
VIS="${VIS:-1}"
POINT_SKIP="${POINT_SKIP:-1}"
CONVERT_RESULTS_DIR="${CONVERT_RESULTS_DIR:-}"

# Force Mesa software rendering by default so the viewer renders even when the
# host GPU driver is not exposed to the container.
LIBGL_SW="${LIBGL_SW:-1}"
if [[ "$LIBGL_SW" == "1" ]]; then LIBGL_ENV="1"; else LIBGL_ENV=""; fi

usage() {
  echo "Usage:"
  echo "  $0 <input.bag | ros2_bag_dir> <output_dir>"
  echo
  echo "  input.bag    : ROS 1 bag file with a PointCloud2 topic (e.g. reg-1.bag-pc.bag)"
  echo "  ros2_bag_dir : ROS 2 bag directory with a PointCloud2 topic"
  echo
  echo "If no arguments are provided, a GUI file selector will be used."
  echo
  echo "Environment variables:"
  echo "  CONFIG_FILE - KISS-SLAM parameter file on the host  (default: the author defaults)"
  echo "  TOPIC       - PointCloud2 topic                      (default: /livox/pointcloud)"
  echo "  VIS         - 1/0, KISS-SLAM's Open3D live viewer    (default: 1)"
  echo "  POINT_SKIP  - converter keeps every Nth point        (default: 1)"
  exit 1
}

echo "=== KISS-SLAM rosbag pipeline ==="

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
fi

if [[ $# -eq 2 ]]; then
  DATASET_HOST_PATH="$1"
  OUTPUT_HOST_DIR="$2"
elif [[ $# -eq 0 ]]; then
  command -v zenity >/dev/null || {
    echo "Error: zenity is not available"
    exit 1
  }
  DATASET_HOST_PATH=$(zenity --file-selection --title="Select BAG file (or ROS 2 bag directory)")
  OUTPUT_HOST_DIR=$(zenity --file-selection --directory --title="Select output directory")
else
  usage
fi

if [[ -z "$DATASET_HOST_PATH" || -z "$OUTPUT_HOST_DIR" ]]; then
  echo "Error: no file or directory selected"
  exit 1
fi

if [[ ! -e "$DATASET_HOST_PATH" ]]; then
  echo "Error: input does not exist: $DATASET_HOST_PATH"
  exit 1
fi

CONFIG_MOUNT=()
CONFIG_ARG=""
CONFIG_DESC="author defaults"
if [[ -n "$CONFIG_FILE" ]]; then
  if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "Error: CONFIG_FILE does not exist: $CONFIG_FILE"
    exit 1
  fi
  CONFIG_FILE=$(realpath "$CONFIG_FILE")
  CONFIG_MOUNT=(-v "$CONFIG_FILE":"$CONFIG_MOUNT_CONTAINER":ro)
  CONFIG_ARG="--config $CONFIG_MOUNT_CONTAINER"
  CONFIG_DESC="$CONFIG_FILE"
fi

mkdir -p "$OUTPUT_HOST_DIR"
DATASET_HOST_PATH=$(realpath "$DATASET_HOST_PATH")
OUTPUT_HOST_DIR=$(realpath "$OUTPUT_HOST_DIR")
DATASET_BASE=$(basename "$DATASET_HOST_PATH")

# Viewer needs X11 (same recipe as the RViz-based benchmarks).
VIS_ARG=""
if [[ "$VIS" == "1" && -n "$DISPLAY" ]]; then
  VIS_ARG="--visualize"
  xhost +local:docker >/dev/null 2>&1 || true
elif [[ "$VIS" == "1" ]]; then
  echo "No DISPLAY: the viewer is off."
fi

echo "Input           : $DATASET_HOST_PATH"
echo "Output dir      : $OUTPUT_HOST_DIR"
echo "Session         : $OUTPUT_HOST_DIR/$OUTPUT_NAME"
echo "Parameter file  : $CONFIG_DESC"
echo "Cloud topic     : $TOPIC"
echo "Viewer          : ${VIS_ARG:-off}"

RESULTS_ARG=""
if [[ -n "$CONVERT_RESULTS_DIR" ]]; then
  RESULTS_ARG="--results-dir $CONVERT_RESULTS_DIR"
  echo "SLAM            : skipped, converting $CONVERT_RESULTS_DIR"
fi

# ── Run KISS-SLAM, then convert, in one container ────────────────────────────
# HOME: open3d and matplotlib write caches there; the container user has none.
docker run -it --rm \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -e LIBGL_ALWAYS_SOFTWARE="$LIBGL_ENV" \
  -e HOME=/tmp/kiss-home \
  -e XDG_RUNTIME_DIR=/tmp/xdg \
  -e MPLBACKEND=Agg \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_DIR/$DATASET_BASE":ro \
  -v "$OUTPUT_HOST_DIR":"$OUTPUT_CONTAINER" \
  "${CONFIG_MOUNT[@]}" \
  "$IMAGE_NAME" \
  /bin/bash -c "
    mkdir -p /tmp/kiss-home /tmp/xdg '$OUTPUT_CONTAINER/logs'
    # A writable working directory: the image's /kiss_ws belongs to root.
    cd /tmp/kiss-home
    python3 -u /kiss_ws/scripts/kiss_slam_bench.py \
      --bag '$DATASET_CONTAINER_DIR/$DATASET_BASE' \
      --topic '$TOPIC' \
      --out-root '$OUTPUT_CONTAINER/$RESULTS_NAME' \
      --session '$OUTPUT_CONTAINER/$OUTPUT_NAME' \
      --point-skip '$POINT_SKIP' \
      $CONFIG_ARG $VIS_ARG $RESULTS_ARG 2>&1 | tee '$OUTPUT_CONTAINER/logs/kiss-slam.log'
    exit \${PIPESTATUS[0]}
  "
status=$?

if [[ $status -eq 42 ]]; then
  echo "=== ABORTED: input check failed, nothing was run ==="
  exit 1
elif [[ $status -ne 0 ]]; then
  echo "=== FAILED (exit $status): see $OUTPUT_HOST_DIR/logs/kiss-slam.log ==="
  exit 1
elif [[ ! -f "$OUTPUT_HOST_DIR/$OUTPUT_NAME/session.json" ]]; then
  echo "=== FAILED: no session.json in $OUTPUT_HOST_DIR/$OUTPUT_NAME; see $OUTPUT_HOST_DIR/logs/kiss-slam.log ==="
  exit 1
fi

echo "=== DONE === Results in: $OUTPUT_HOST_DIR/$OUTPUT_NAME"
