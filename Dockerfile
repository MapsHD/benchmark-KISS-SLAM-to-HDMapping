FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# ── Base tools and libraries ─────────────────────────────────────────────────
# Eigen and SuiteSparse (BLAS/LAPACK for CHOLMOD) as in KISS-SLAM's README; the
# X/GL libraries are for open3d and the Open3D viewer (Mesa DRI is the
# software-GL fallback); libusb because the open3d wheel links
# libusb-1.0.so.0 (its RealSense support) and fails to import without it.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    patch \
    ca-certificates \
    libeigen3-dev \
    libsuitesparse-dev \
    libgl1 \
    libegl1 \
    libgomp1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libx11-6 \
    libxi6 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# ── Python build tools and KISS-SLAM's dependencies ──────────────────────────
# cmake >= 3.28: KISS-SLAM's CMake fetches g2o and SuiteSparse with
# FetchContent SYSTEM/EXCLUDE_FROM_ALL; < 4 because CMake 4 refuses the old
# cmake_minimum_required of some fetched dependencies. scikit-build-core < 1:
# KISS-SLAM's pyproject uses the cmake.minimum-version key, which 1.x rejects.
# kiss-icp 1.2.3 and map_closures 2.0.2 are the newest releases at the pinned
# KISS-SLAM commit (2025-12-05); open3d 0.19.0 is its minimum.
# rosbags: KISS-ICP's rosbag dataloader. laspy + lazrs: the LAZ writer of the
# HDMapping converter. matplotlib: the pipeline's closure plot.
RUN pip3 install --no-cache-dir \
    "cmake>=3.28,<4" ninja "scikit-build-core<1" pybind11 \
    "kiss-icp==1.2.3" "map_closures==2.0.2" "open3d==0.19.0" \
    "rosbags==0.9.22" laspy lazrs matplotlib

WORKDIR /kiss_ws
COPY ./src/KISS-SLAM ./kiss-slam

# ── KISS-SLAM (local maps, MapClosures loop closing, g2o pose graph) ─────────
# g2o and SuiteSparse are fetched and built by KISS-SLAM's CMake (the apt g2o
# lacks the targets it needs); Eigen comes from apt. The define is passed
# through scikit-build-core's environment variable: Ubuntu 22.04's pip has no
# --config-settings. The pinned versions above already satisfy KISS-SLAM's
# requirements, so pip keeps them; the check below fails the build otherwise.
RUN test -f kiss-slam/pyproject.toml || \
        { echo "[build] src/KISS-SLAM is empty: clone with --recursive (or git submodule update --init)"; exit 1; } && \
    SKBUILD_CMAKE_DEFINE="USE_SYSTEM_G2O=OFF" \
    pip3 install --no-cache-dir --no-build-isolation ./kiss-slam && \
    python3 -c "from importlib.metadata import version as v; \
got = {p: v(p) for p in ('kiss_icp', 'map_closures', 'open3d')}; \
want = {'kiss_icp': '1.2.3', 'map_closures': '2.0.2', 'open3d': '0.19.0'}; \
assert got == want, f'dependency versions changed: {got}'; print('[build] versions', got)"

# ── Benchmark viewer patch: closing the viewer does not end the run ──────────
# See overlay/viewer/kiss_slam_viewer_keep_running.patch (visualisation only;
# the estimate is unchanged). Upstream, ESC/Q calls os._exit(0), which ends
# KISS-SLAM before it writes any result, and closing the window while paused
# hangs. Applied to the installed package in a late layer, so a viewer change
# does not rebuild g2o and SuiteSparse.
COPY ./overlay/viewer/ ./viewer/
RUN SITE=$(python3 -c "import kiss_slam, os; print(os.path.dirname(os.path.dirname(kiss_slam.__file__)))") && \
    patch -p1 --batch -d "$SITE" < viewer/kiss_slam_viewer_keep_running.patch && \
    grep -q "self.viewer_closed = True" "$SITE/kiss_slam/tools/visualizer.py" && \
    ! grep -qE "^\s*os\._exit\(" "$SITE/kiss_slam/tools/visualizer.py" && \
    python3 -m py_compile "$SITE/kiss_slam/tools/visualizer.py"

COPY ./overlay/scripts/ ./scripts/

# The final guards fail the image build loudly: first every shared library the
# CPU module of open3d needs (all missing ones listed at once; the optional
# CUDA build and PyTorch/TensorFlow ops are not loaded and not checked), then
# the imports.
RUN O3D=$(python3 -c "import importlib.util, os; print(os.path.dirname(importlib.util.find_spec('open3d').origin))") && \
    missing=$(find "$O3D/cpu" -maxdepth 1 -name "pybind*.so" -exec ldd {} \; 2>/dev/null | grep "not found" | sort -u) && \
    if [ -n "$missing" ]; then echo "[build] open3d needs missing libraries:"; echo "$missing"; exit 1; fi && \
    python3 -c "import kiss_icp, kiss_slam, map_closures, open3d, laspy, lazrs, rosbags, matplotlib; \
from kiss_icp.datasets import dataset_factory; from kiss_icp.datasets.rosbag import RosbagDataset; \
from kiss_slam.pipeline import SlamPipeline; print('[build] KISS-SLAM, dataloaders and converter deps OK')" && \
    test -f /kiss_ws/scripts/kiss_slam_bench.py && test -f /kiss_ws/scripts/kiss_slam_to_hdmapping.py

CMD ["bash"]
