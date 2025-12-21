#!/bin/bash
#
# KISS-SLAM to HDMapping - Build and Run Script
#
# This script:
# 1. Checks if the Docker image exists, builds it if not
# 2. Runs KISS-SLAM on the specified rosbag dataset
# 3. Outputs HDMapping-compatible files
#
# Usage:
#   ./run_kiss_slam.sh <input_bag_path> <output_directory>
#
# Example:
#   ./run_kiss_slam.sh /home/kmro/dev/slams/datasets/charles/warm-up_large-room_bag /tmp/output
#

set -e

# Configuration
IMAGE_NAME="kiss-slam-to-hdmapping"
IMAGE_TAG="latest"
FULL_IMAGE="${IMAGE_NAME}:${IMAGE_TAG}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory (where Dockerfile is located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Function to print colored messages
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to print usage
print_usage() {
    echo ""
    echo "Usage: $0 <input_bag_path> <output_directory> [lidar_topic]"
    echo ""
    echo "Arguments:"
    echo "  input_bag_path    - Path to the ROS2 bag directory (containing .db3 file)"
    echo "  output_directory  - Path where HDMapping output will be saved"
    echo "  lidar_topic       - (Optional) LiDAR topic name, default: auto-detect"
    echo ""
    echo "Options:"
    echo "  --rebuild         - Force rebuild of Docker image"
    echo "  --no-cache        - Build Docker image without cache"
    echo "  --help            - Show this help message"
    echo ""
    echo "Example:"
    echo "  $0 /home/user/datasets/livox_bag /tmp/hdmapping_output"
    echo "  $0 /home/user/datasets/livox_bag /tmp/hdmapping_output /livox/lidar"
    echo ""
}

# Function to check if Docker is available
check_docker() {
    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed or not in PATH"
        exit 1
    fi

    if ! docker info &> /dev/null; then
        log_error "Docker daemon is not running or you don't have permission to use it"
        log_info "Try running: sudo systemctl start docker"
        log_info "Or add your user to docker group: sudo usermod -aG docker \$USER"
        exit 1
    fi
}

# Function to check if image exists
image_exists() {
    docker image inspect "${FULL_IMAGE}" &> /dev/null
}

# Function to build Docker image
build_image() {
    local build_args=""

    if [ "$NO_CACHE" = true ]; then
        build_args="--no-cache"
    fi

    log_info "Building Docker image: ${FULL_IMAGE}"
    log_info "This may take several minutes..."
    echo ""

    cd "${SCRIPT_DIR}"

    if docker build ${build_args} -t "${FULL_IMAGE}" .; then
        log_success "Docker image built successfully: ${FULL_IMAGE}"
    else
        log_error "Failed to build Docker image"
        exit 1
    fi
}

# Function to run KISS-SLAM pipeline
run_pipeline() {
    local input_bag="$1"
    local output_dir="$2"
    local lidar_topic="${3:-auto}"

    # Get absolute paths
    input_bag=$(realpath "${input_bag}")
    output_dir=$(realpath "${output_dir}")

    # Create output directory if it doesn't exist
    mkdir -p "${output_dir}"

    # Get the parent directory of input bag for mounting
    input_parent=$(dirname "${input_bag}")
    input_basename=$(basename "${input_bag}")

    log_info "Running KISS-SLAM pipeline..."
    log_info "  Input bag: ${input_bag}"
    log_info "  Output directory: ${output_dir}"
    log_info "  LiDAR topic: ${lidar_topic}"
    echo ""

    # Run Docker container (use -t if terminal is available, otherwise just -i)
    local docker_flags="-i --rm"
    if [ -t 1 ]; then
        docker_flags="-it --rm"
    fi

    docker run ${docker_flags} \
        -v "${input_bag}:/data/input_bag:ro" \
        -v "${output_dir}:/data/output" \
        "${FULL_IMAGE}" \
        /run_pipeline.sh /data/input_bag /data/output "${lidar_topic}"

    if [ $? -eq 0 ]; then
        log_success "Pipeline completed successfully!"
        echo ""
        log_info "Output files:"
        ls -la "${output_dir}"
        echo ""
        log_info "HDMapping session file: ${output_dir}/session.json"
        log_info "LAZ point cloud files: $(ls "${output_dir}"/*.laz 2>/dev/null | wc -l) files"
    else
        log_error "Pipeline failed"
        exit 1
    fi
}

# Parse command line arguments
REBUILD=false
NO_CACHE=false
INPUT_BAG=""
OUTPUT_DIR=""
LIDAR_TOPIC="auto"

while [[ $# -gt 0 ]]; do
    case $1 in
        --rebuild)
            REBUILD=true
            shift
            ;;
        --no-cache)
            NO_CACHE=true
            REBUILD=true
            shift
            ;;
        --help|-h)
            print_usage
            exit 0
            ;;
        -*)
            log_error "Unknown option: $1"
            print_usage
            exit 1
            ;;
        *)
            if [ -z "$INPUT_BAG" ]; then
                INPUT_BAG="$1"
            elif [ -z "$OUTPUT_DIR" ]; then
                OUTPUT_DIR="$1"
            elif [ "$LIDAR_TOPIC" = "auto" ]; then
                LIDAR_TOPIC="$1"
            else
                log_error "Too many arguments"
                print_usage
                exit 1
            fi
            shift
            ;;
    esac
done

# Validate arguments
if [ -z "$INPUT_BAG" ] || [ -z "$OUTPUT_DIR" ]; then
    log_error "Missing required arguments"
    print_usage
    exit 1
fi

# Check if input bag exists
if [ ! -d "$INPUT_BAG" ]; then
    log_error "Input bag directory does not exist: $INPUT_BAG"
    exit 1
fi

# Check if input bag contains .db3 file
if ! ls "${INPUT_BAG}"/*.db3 &> /dev/null; then
    log_error "No .db3 file found in input bag directory: $INPUT_BAG"
    log_info "Make sure this is a valid ROS2 bag directory"
    exit 1
fi

# Main execution
echo ""
echo "=============================================="
echo "  KISS-SLAM to HDMapping Pipeline"
echo "=============================================="
echo ""

# Check Docker
log_info "Checking Docker..."
check_docker
log_success "Docker is available"

# Check/Build Docker image
if [ "$REBUILD" = true ]; then
    log_info "Rebuild requested, building image..."
    build_image
elif image_exists; then
    log_success "Docker image exists: ${FULL_IMAGE}"
else
    log_warn "Docker image not found, building..."
    build_image
fi

echo ""

# Run the pipeline
run_pipeline "$INPUT_BAG" "$OUTPUT_DIR" "$LIDAR_TOPIC"

echo ""
echo "=============================================="
echo "  Done!"
echo "=============================================="
echo ""
