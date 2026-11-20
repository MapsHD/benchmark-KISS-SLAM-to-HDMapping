## Hint

Please change branch to [Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-KISS-SLAM-to-HDMapping/tree/Bunker-DVI-Dataset-reg-1) for quick experiment.

## Example Dataset:

Download the dataset from [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset/)

# benchmark-KISS-SLAM-to-HDMapping

Runs [KISS-SLAM](https://github.com/PRBonn/kiss-slam) on a rosbag and
converts the output to an [HDMapping](https://github.com/MapsHD/HDMapping)
session.

KISS-SLAM (T. Guadagnino et al., "KISS-SLAM: A Simple, Robust, and Accurate
3D LiDAR SLAM System With Enhanced Generalization Capabilities", IROS 2025) is
a LiDAR-only SLAM. KISS-ICP odometry builds local maps; MapClosures detects
loops between them, and a g2o pose graph over the local maps corrects the
trajectory. A final pose graph over all scans gives one pose per scan.

The submodule `src/KISS-SLAM` is pinned to commit `3d4d1fa` (2025-12-05), the
newest on KISS-SLAM's main branch. The authors keep the code of the paper
results under the tag `IROS25`.

## Prerequisites

- Docker
- A ROS 1 bag file or a ROS 2 bag directory with a `sensor_msgs/PointCloud2`
  topic. A per-point time field (`t`, `timestamp` or `time`) lets KISS-SLAM
  deskew the scans; without it they are used as they are.

KISS-SLAM needs no ROS: it reads the bag through the
[rosbags](https://pypi.org/project/rosbags/) library. It cannot read Livox
`CustomMsg`; convert such a bag to `PointCloud2` first with
[livox_bag_aggregate](https://github.com/MapsHD/livox_bag_aggregate).

## Step 1 — Clone with submodules

```bash
git clone https://github.com/MapsHD/benchmark-KISS-SLAM-to-HDMapping.git --recursive
cd benchmark-KISS-SLAM-to-HDMapping
```

## Step 2 — Build the Docker image

```bash
docker build -t kiss-slam_standalone .
```

This installs:
- Ubuntu 22.04, Python 3.10, Eigen, SuiteSparse
- KISS-ICP 1.2.3, MapClosures 2.0.2 and Open3D 0.19.0 from PyPI, the newest
  releases at the pinned KISS-SLAM commit
- KISS-SLAM from the submodule; g2o and SuiteSparse for the pose graph are
  built by KISS-SLAM's own CMake
- rosbags, laspy and the HDMapping converter

## Step 3 — Run the pipeline

```bash
chmod +x docker_session_run-kiss-slam.sh
./docker_session_run-kiss-slam.sh /path/to/input.bag /path/to/output/dir
```

The input is a ROS 1 `.bag` file or a ROS 2 bag directory. With no arguments, a
GUI file selector is used (requires `zenity`).

| Variable | Default | Meaning |
|----------|---------|---------|
| `CONFIG_FILE` | none (author's defaults) | KISS-SLAM parameter file, see below |
| `TOPIC` | `/livox/pointcloud` | `PointCloud2` topic in the bag |
| `VIS` | `1` | KISS-SLAM's Open3D live viewer (off without a `DISPLAY`) |
| `LIBGL_SW` | `1` | Mesa software rendering for the viewer |
| `POINT_SKIP` | `1` | the converter keeps every Nth point |
| `CONVERT_RESULTS_DIR` | none | an existing results directory in the container, e.g. `/output/kiss-slam-results/<timestamp>`: skip SLAM, only convert |

**Parameters.** Without `CONFIG_FILE`, KISS-SLAM runs with its defaults
(`odometry.preprocessing.max_range` 100 m, `local_mapper.splitting_distance`
100 m, ...). To change them, write the defaults to a file with KISS-SLAM's own
tool and edit it:

```bash
docker run --rm -u 1000:1000 -v "$PWD":/work -w /work kiss-slam_standalone kiss_slam_dump_config
CONFIG_FILE=kiss_slam.yaml ./docker_session_run-kiss-slam.sh /path/to/input.bag /path/to/output/dir
```

KISS-SLAM's README suggests for indoor data `max_range` 50 (which sets the
voxel size to 0.5 m) and a smaller `splitting_distance`.

Before anything starts, the script checks that the topic is in the bag as
`PointCloud2` with `x`, `y`, `z` fields.

**What happens:**

KISS-SLAM runs offline on the CPU: it reads the bag as fast as it processes the
scans and exits when the bag is done. A run can take longer than the bag, but
no scan is dropped. There is no playback and no recording.

With `VIS=1`, KISS-SLAM's Open3D viewer shows the run live:
- yellow: the current local map;
- green: the pose of every scan;
- blue: the key poses of the local maps and the edges between them.

The camera is free: rotate with the left mouse button, pan with Ctrl + left
button, zoom with the wheel. `C` fits the view to everything drawn, and `SPACE`
pauses. Closing the viewer (or `ESC`) does not stop KISS-SLAM.

When KISS-SLAM has finished, the same container converts its output into the
HDMapping session format.

## Step 4 — Open in HDMapping

Output files appear in `<output_dir>/output_hdmapping-kiss-slam/`:

```
lio_initial_poses.reg
poses.reg
scan_lio_0.laz
...
session.json
trajectory_lio_0.csv
...
```

Open `session.json` with the
[multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping)
application.

KISS-SLAM's own results stay in `<output_dir>/kiss-slam-results/<timestamp>/`
(`latest` links to the newest run): `<sequence>_poses_tum.txt`,
`trajectory.g2o`, `trajectory.png` with the loop closures, the local maps and
`config.yml`. The log is in `<output_dir>/logs/kiss-slam.log`.

## Notes on KISS-SLAM

**Output.** KISS-SLAM writes one pose per scan, in scan order, in TUM format
(`timestamp tx ty tz qx qy qz qw`), stamped with the bag time of the scan. The
poses are the final ones after loop closing and the pose-graph optimisation.
The converter places every scan of the bag with its pose (scan `i` with pose
`i`, checked by time; the nearest pose in time otherwise) and writes the
chunked session. Points closer than 0.5 m to the sensor are left out, as in
the other converters.

**Timestamps.** The benchmark does not change KISS-SLAM's clock. The
trajectory times are the bag times of the scans.

**Viewer patch.** Upstream, `ESC` or `Q` in the viewer ends the whole process
with `os._exit(0)` before KISS-SLAM writes any result. Closing the window while
the viewer is paused leaves the run waiting forever. The Docker build applies
[overlay/viewer/kiss_slam_viewer_keep_running.patch](overlay/viewer/kiss_slam_viewer_keep_running.patch),
which only closes the window in both cases. The run goes on, and the estimate
is unchanged. The run script also starts the viewer playing, where upstream it
starts paused until `SPACE`.

**Optional outputs.** The run script calls KISS-SLAM's pipeline stage by stage.
SLAM and the pose file are required. The closure plot, the local maps, the g2o
graph and the logs are each optional: a failure in one of them is reported and
does not cost the run.

## Contact

januszbedkowski@gmail.com
