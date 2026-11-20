#!/usr/bin/env python3
"""Runs KISS-SLAM on a bag through its own Python API and converts the result
to an HDMapping session.

Stages, each reported on its own:
  1. preflight  the point cloud topic must be in the bag as sensor_msgs/PointCloud2
                with x, y, z fields (exit code 42 if not: nothing is run)
  2. SLAM       kiss_slam.pipeline.SlamPipeline on KISS-ICP's rosbag dataloader,
                with the author's default parameters or the given config file;
                the results (poses in TUM format, local maps, closures, config)
                land in <out-root>/<timestamp>/ with a 'latest' link
  3. convert    kiss_slam_to_hdmapping.convert() on those poses and the bag

The exit code is 42 for a preflight failure, 1 for any other failure.
"""

import argparse
import os
import sys
import traceback
from pathlib import Path

PREFLIGHT_FAILED = 42
TIME_FIELDS = ("t", "timestamp", "time")   # the names KISS-ICP's reader accepts


class PreflightError(Exception):
    pass


def preflight(bag, topic):
    from rosbags.highlevel import AnyReader

    try:
        with AnyReader([Path(bag)]) as reader:
            connections = [c for c in reader.connections if c.topic == topic]
            livox = sorted({c.topic for c in reader.connections if c.msgtype.endswith("/CustomMsg")})
            livox_hint = (f" The bag has the Livox CustomMsg topic(s) {', '.join(livox)}, which KISS-SLAM "
                          "cannot read: convert the bag to PointCloud2 with livox_bag_aggregate "
                          "(https://github.com/MapsHD/livox_bag_aggregate) and pass the resulting "
                          "<bag>-pc.bag, e.g. reg-1.bag-pc.bag." if livox else "")
            if not connections:
                clouds = [c.topic for c in reader.connections if c.msgtype == "sensor_msgs/msg/PointCloud2"]
                raise PreflightError(f"topic {topic} is not in {bag}; "
                                     f"its PointCloud2 topics: {', '.join(clouds) or '(none)'}. "
                                     "Set TOPIC to the LiDAR one." + livox_hint)
            msgtype = connections[0].msgtype
            if msgtype != "sensor_msgs/msg/PointCloud2":
                raise PreflightError(f"{topic} is {msgtype}; KISS-SLAM reads sensor_msgs/msg/PointCloud2."
                                     + livox_hint)
            count = sum(c.msgcount for c in connections)
            if count == 0:
                raise PreflightError(f"{topic} has no messages in {bag}")
            _, _, rawdata = next(reader.messages(connections=connections))
            msg = reader.deserialize(rawdata, msgtype)
    except PreflightError:
        raise
    except Exception as e:
        raise PreflightError(f"cannot read {bag}: {type(e).__name__}: {e}") from e

    names = [f.name for f in msg.fields]
    missing = [k for k in ("x", "y", "z") if k not in names]
    if missing:
        raise PreflightError(f"{topic} lacks the point fields {missing} (fields: {names})")
    time_field = next((n for n in names if n in TIME_FIELDS), None)
    print(f"[preflight] {topic}: {count} PointCloud2 messages, fields {names}")
    # KISS-SLAM's front end is KISS-ICP, so the deskewing and the accepted
    # time field names are KISS-ICP's.
    if time_field:
        print(f"[preflight] per-point time field '{time_field}': KISS-SLAM deskews each scan with it")
    else:
        print("[preflight] WARNING: no per-point time field (t/timestamp/time): KISS-SLAM cannot deskew the scans")


def run_slam(bag, topic, out_root, config_file, visualize):
    from kiss_icp.datasets import dataset_factory
    from kiss_slam.pipeline import SlamPipeline

    dataset = dataset_factory(dataloader="rosbag", data_dir=Path(bag), sequence=None, topic=topic, meta=None)
    pipeline = SlamPipeline(
        dataset=dataset,
        config_file=Path(config_file) if config_file else None,
        visualize=visualize,
        n_scans=-1,
        jump=0,
        refuse_scans=False,
    )
    # Results go next to the HDMapping session instead of ./slam_output.
    pipeline.slam_config.out_dir = out_root
    pipeline.config.out_dir = out_root
    if visualize:
        # The viewer starts paused until SPACE is pressed; the benchmark runs unattended.
        pipeline.visualizer.play_crun = True
    print(f"[kiss-slam] config: {'author defaults' if not config_file else config_file}")
    print(f"[kiss-slam] {len(dataset)} scans on {topic}; results -> {out_root}")

    # The stages of SlamPipeline.run(), called one by one: SLAM and the pose
    # file are required; every other output is optional and only reported if
    # it fails, so it cannot cost a finished run. The ground-truth poses and
    # the global map (refuse_scans) are not produced: the bag has no ground
    # truth and refuse_scans is off by default.
    pipeline._run_pipeline()
    pipeline._create_output_dir()
    pipeline._write_result_poses()
    results_dir = pipeline.results_dir
    print(f"[kiss-slam] poses written to {results_dir}")

    optional = [
        ("evaluation", pipeline._run_evaluation),
        ("closure count", pipeline._evaluate_closures),
        ("config.yml", pipeline._write_cfg),
        ("result_metrics.log", pipeline._write_log),
        ("trajectory.g2o", pipeline._write_graph),
        ("trajectory.png (closure plot)", pipeline._write_closures),
        ("local maps", pipeline._write_local_maps),
        ("summary", pipeline.results.print),
    ]
    for name, stage in optional:
        try:
            stage()
        except Exception as e:
            print(f"[kiss-slam] WARNING: optional output '{name}' failed ({type(e).__name__}: {e}); "
                  "the poses are not affected")
    return results_dir


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="input bag (ROS 1 .bag file or ROS 2 bag directory)")
    ap.add_argument("--topic", required=True, help="PointCloud2 topic")
    ap.add_argument("--out-root", required=True, help="directory for KISS-SLAM's own results")
    ap.add_argument("--session", required=True, help="output HDMapping session directory")
    ap.add_argument("--config", default="", help="KISS-SLAM parameter file (default: the author's defaults)")
    ap.add_argument("--visualize", action="store_true", help="open KISS-SLAM's Open3D viewer")
    ap.add_argument("--point-skip", type=int, default=1, help="converter keeps every Nth point")
    ap.add_argument("--results-dir", default="",
                    help="skip SLAM and convert this existing results directory")
    args = ap.parse_args()

    # Only savefig is used (trajectory.png); never a window.
    os.environ.setdefault("MPLBACKEND", "Agg")

    try:
        preflight(args.bag, args.topic)
    except PreflightError as e:
        print(f"[preflight] ERROR: {e}", file=sys.stderr)
        sys.exit(PREFLIGHT_FAILED)
    except Exception as e:  # a bug in the check must not start a run blind
        print(f"[preflight] ERROR: the check itself failed ({type(e).__name__}: {e})", file=sys.stderr)
        sys.exit(PREFLIGHT_FAILED)

    if args.results_dir:
        results_dir = args.results_dir
        print(f"[kiss-slam] SLAM skipped, converting {results_dir}")
    else:
        try:
            results_dir = run_slam(args.bag, args.topic, args.out_root, args.config, args.visualize)
        except Exception:
            print("[kiss-slam] ERROR: KISS-SLAM failed:", file=sys.stderr)
            traceback.print_exc()
            sys.exit(1)

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    try:
        import kiss_slam_to_hdmapping
    except Exception:
        print("[convert] ERROR: cannot load the converter:", file=sys.stderr)
        traceback.print_exc()
        print(f"[convert] KISS-SLAM's results are kept in {results_dir}", file=sys.stderr)
        sys.exit(1)

    try:
        kiss_slam_to_hdmapping.convert(args.bag, results_dir, args.topic, args.session, args.point_skip)
    except kiss_slam_to_hdmapping.ConversionError as e:
        print(f"[convert] ERROR: {e}", file=sys.stderr)
        print(f"[convert] KISS-SLAM's results are kept in {results_dir}", file=sys.stderr)
        sys.exit(1)
    except Exception:
        print("[convert] ERROR: the conversion failed unexpectedly:", file=sys.stderr)
        traceback.print_exc()
        print(f"[convert] KISS-SLAM's results are kept in {results_dir}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
