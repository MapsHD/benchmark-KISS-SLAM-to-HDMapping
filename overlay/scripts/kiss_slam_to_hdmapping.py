#!/usr/bin/env python3
"""Converts a KISS-SLAM run to an HDMapping session (chunked LAZ + trajectory
CSVs + poses.reg + session.json), the format the C++ *-to-hdmapping converters
of the other benchmarks produce.

KISS-SLAM runs offline on the bag and writes its trajectory as a TUM pose file
(<sequence>_poses_tum.txt: timestamp tx ty tz qx qy qz qw), one pose per
processed scan, in scan order, stamped with the bag time of the scan. The
world-frame cloud is rebuilt here: every scan of the input bag is placed with
its pose. Scan i is matched to pose i when their times agree, otherwise to the
pose nearest in time.

Each stage reports its own failures: a scan that cannot be read is skipped and
counted, a chunk that cannot be written ends the conversion with an error.
"""

import argparse
import glob
import json
import os
import sys
import traceback
from pathlib import Path

import numpy as np

CHUNK_SIZE = 2_000_000
INDEX_MATCH_TOLERANCE_NS = int(0.005 * 1e9)   # pose i belongs to scan i if within 5 ms
TIME_MATCH_TOLERANCE_NS = int(0.06 * 1e9)     # otherwise the nearest pose within 60 ms
MIN_RANGE_M = 0.5                             # blind-zone filter, as in the other converters


class ConversionError(Exception):
    """An error that makes the session incomplete; reported, conversion stops."""


def quat_to_rot(qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0:
        raise ConversionError("zero quaternion in the pose file")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def tait_bryan_from_matrix(m):
    # Same convention as pose_tait_bryan_from_affine_matrix in the C++ converters.
    if m[0, 2] < 1:
        if m[0, 2] > -1:
            fi = np.arcsin(m[0, 2])
            om = np.arctan2(-m[1, 2], m[2, 2])
            ka = np.arctan2(-m[0, 1], m[0, 0])
        else:
            fi = -np.pi / 2.0
            om = -np.arctan2(m[1, 0], m[1, 1])
            ka = 0.0
    else:
        fi = np.pi / 2.0
        om = np.arctan2(m[1, 0], m[1, 1])
        ka = 0.0
    return om, fi, ka


def find_pose_file(results_dir):
    """The pipeline writes <sequence>_poses_tum.txt (and <sequence>_gt_tum.txt with ground truth)."""
    candidates = sorted(glob.glob(os.path.join(results_dir, "*_poses_tum.txt")))
    if not candidates:
        raise ConversionError(f"no *_poses_tum.txt in {results_dir}; KISS-SLAM wrote no trajectory")
    if len(candidates) > 1:
        print(f"WARNING: several pose files in {results_dir}, using {candidates[0]}")
    return candidates[0]


def load_tum_poses(path):
    """Returns a list of (timestamp_ns, 4x4 pose) in file order."""
    try:
        data = np.loadtxt(path, comments="#", ndmin=2)
    except (OSError, ValueError) as e:
        raise ConversionError(f"cannot read the pose file {path}: {e}") from e
    if data.size == 0:
        raise ConversionError(f"the pose file {path} is empty; KISS-SLAM produced no poses")
    if data.shape[1] < 8:
        raise ConversionError(f"{path} has {data.shape[1]} columns, a TUM file has 8")
    poses = []
    for row in data:
        if not np.all(np.isfinite(row)):
            raise ConversionError(f"non-finite value in the pose file {path}: {row}")
        T = np.eye(4)
        T[:3, :3] = quat_to_rot(*row[4:8])
        T[:3, 3] = row[1:4]
        poses.append((int(round(row[0] * 1e9)), T))
    print(f"Using pose file: {path} ({len(poses)} poses)")
    return poses


def read_bag_clouds(bag_path, topic):
    from rosbags.highlevel import AnyReader

    with AnyReader([Path(bag_path)]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            available = "\n".join(f"  {c.topic}  {c.msgtype}" for c in reader.connections)
            raise ConversionError(f"topic {topic} not found in {bag_path}. Available:\n{available}")
        for connection, _, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            yield stamp_ns, msg


def parse_pointcloud2(msg):
    """Returns (N,3) float32 xyz and (N,) float32 intensity, or raises ValueError."""
    offsets = {f.name: (f.offset, f.datatype) for f in msg.fields}
    missing = [k for k in ("x", "y", "z") if k not in offsets]
    if missing:
        raise ValueError(f"point cloud lacks the fields {missing}")
    step = msg.point_step
    n = msg.width * msg.height
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)

    def field_f32(name):
        off = offsets[name][0]
        return buf[:, off:off + 4].copy().view(np.float32).reshape(n)

    xyz = np.stack([field_f32("x"), field_f32("y"), field_f32("z")], axis=1)
    intensity = field_f32("intensity") if "intensity" in offsets else np.zeros(n, np.float32)
    return xyz, intensity


def save_laz(path, xyz, intensity, ts_ns):
    import laspy

    header = laspy.LasHeader(version="1.2", point_format=1)
    header.scales = np.array([0.0001, 0.0001, 0.0001])
    header.offsets = np.array([0.0, 0.0, 0.0])
    las = laspy.LasData(header)
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    las.intensity = np.clip(intensity, 0, 65535).astype(np.uint16)
    las.gps_time = ts_ns.astype(np.float64)
    las.write(str(path))
    print(f"saved {path} ({xyz.shape[0]} points)")


def match_pose(frame_index, stamp_ns, poses, pose_ts):
    """Pose of scan frame_index: pose frame_index if its time agrees, else the nearest in time."""
    if frame_index < len(poses) and abs(int(pose_ts[frame_index]) - stamp_ns) <= INDEX_MATCH_TOLERANCE_NS:
        return frame_index
    idx = np.searchsorted(pose_ts, stamp_ns)
    best, best_dt = None, None
    for j in (idx - 1, idx):
        if 0 <= j < len(pose_ts):
            dt = abs(int(pose_ts[j]) - stamp_ns)
            if best_dt is None or dt < best_dt:
                best, best_dt = j, dt
    if best is None or best_dt > TIME_MATCH_TOLERANCE_NS:
        return None
    return best


def convert(bag, results_dir, topic, out, point_skip=1):
    poses = load_tum_poses(find_pose_file(results_dir))
    pose_ts = np.array([p[0] for p in poses], dtype=np.int64)
    if np.any(np.diff(pose_ts) < 0):
        print("WARNING: pose timestamps are not increasing; nearest-time matching may pick wrong poses")

    os.makedirs(out, exist_ok=True)

    # ── Rebuild the world-frame cloud, chunked ────────────────────────────────
    chunks = []          # dicts: xyz, intensity, ts, ts_min, ts_max
    cur_xyz, cur_int, cur_ts = [], [], []
    cur_count = 0
    matched_poses = []   # (bag_timestamp, pose) of every matched scan, in scan order
    total_frames = matched_frames = unmatched_frames = unreadable_frames = 0
    by_index = by_time = 0

    def flush_chunk():
        nonlocal cur_xyz, cur_int, cur_ts, cur_count
        if cur_count == 0:
            return
        xyz = np.concatenate(cur_xyz)
        inten = np.concatenate(cur_int)
        ts = np.concatenate(cur_ts)
        chunks.append({"xyz": xyz, "intensity": inten, "ts": ts,
                       "ts_min": int(ts.min()), "ts_max": int(ts.max())})
        cur_xyz, cur_int, cur_ts = [], [], []
        cur_count = 0

    print("Rebuilding the world-frame cloud from the bag and the KISS-SLAM poses...")
    for stamp_ns, msg in read_bag_clouds(bag, topic):
        frame_index = total_frames
        total_frames += 1
        best = match_pose(frame_index, stamp_ns, poses, pose_ts)
        if best is None:
            unmatched_frames += 1
            if unmatched_frames <= 5:
                print(f"WARNING: scan {frame_index} at {stamp_ns / 1e9:.3f} has no pose within "
                      f"{TIME_MATCH_TOLERANCE_NS / 1e6:.0f} ms, skipped")
            continue
        if best == frame_index:
            by_index += 1
        else:
            by_time += 1
        matched_frames += 1
        T = poses[best][1]
        matched_poses.append((stamp_ns, T))

        try:
            xyz, intensity = parse_pointcloud2(msg)
        except (ValueError, TypeError, AttributeError) as e:
            unreadable_frames += 1
            if unreadable_frames <= 5:
                print(f"WARNING: scan {frame_index} cannot be read ({e}), its pose is kept without points")
            continue
        finite = np.isfinite(xyz).all(axis=1)
        rng2 = (xyz * xyz).sum(axis=1)
        keep = finite & (rng2 > MIN_RANGE_M * MIN_RANGE_M)
        xyz, intensity = xyz[keep], intensity[keep]
        if point_skip > 1:
            xyz, intensity = xyz[::point_skip], intensity[::point_skip]
        if xyz.shape[0] == 0:
            continue

        world = xyz.astype(np.float64) @ T[:3, :3].T + T[:3, 3]
        cur_xyz.append(world)
        cur_int.append(intensity)
        cur_ts.append(np.full(world.shape[0], stamp_ns, dtype=np.int64))
        cur_count += world.shape[0]
        if cur_count > CHUNK_SIZE:
            flush_chunk()
    flush_chunk()  # keep any non-empty tail

    print(f"Scans: {total_frames}, matched to poses: {matched_frames} ({by_index} by index, {by_time} by time), "
          f"without pose: {unmatched_frames}, unreadable: {unreadable_frames}; chunks: {len(chunks)}")
    if total_frames == 0:
        raise ConversionError(f"no messages on {topic} in {bag}")
    if len(poses) != total_frames:
        print(f"WARNING: KISS-SLAM wrote {len(poses)} poses for {total_frames} scans")
    if unmatched_frames > 0.1 * total_frames:
        print(f"WARNING: {unmatched_frames} of {total_frames} scans have no pose; "
              "check that the SLAM run processed this bag and topic")
    if not chunks:
        raise ConversionError("no scan could be placed; the session would be empty")

    # ── Index the trajectory into chunks (first matching chunk, like the C++) ─
    chunks_traj = [[] for _ in chunks]
    for ts_ns, T in matched_poses:
        for j, ch in enumerate(chunks):
            if ch["ts_min"] <= ts_ns <= ch["ts_max"]:
                chunks_traj[j].append((ts_ns, T))
                break
    for j, trj in enumerate(chunks_traj):
        print(f"chunk {j}: {len(trj)} trajectory elements, {chunks[j]['xyz'].shape[0]} points")

    # ── Offset = mean of the trajectory positions in chunks ──────────────────
    all_pos = [T[:3, 3] for trj in chunks_traj for (_, T) in trj]
    offset = np.mean(all_pos, axis=0) if all_pos else np.zeros(3)

    # ── Save LAZ + trajectory CSVs ───────────────────────────────────────────
    m_poses, file_names = [], []
    csv_header = ("timestamp_nanoseconds pose00 pose01 pose02 pose03 "
                  "pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 "
                  "timestampUnix_nanoseconds om_rad fi_rad ka_rad")

    for i, (ch, trj) in enumerate(zip(chunks, chunks_traj)):
        if not trj:
            print(f"chunk {i}: no trajectory element, left out")
            continue
        first_T = trj[0][1]
        first_inv = np.linalg.inv(first_T)
        local = ch["xyz"] @ first_inv[:3, :3].T + first_inv[:3, 3]

        laz_name = f"scan_lio_{i}.laz"
        try:
            save_laz(os.path.join(out, laz_name), local, ch["intensity"], ch["ts"])
        except Exception as e:  # laspy/lazrs raise several types; any failure leaves a hole
            raise ConversionError(f"cannot write {laz_name}: {type(e).__name__}: {e}") from e
        file_names.append(laz_name)
        m_poses.append(first_T.copy())

        csv_path = os.path.join(out, f"trajectory_lio_{i}.csv")
        try:
            with open(csv_path, "w") as f:
                f.write(csv_header + "\n")
                for ts_ns, T in trj:
                    rel = first_inv @ T
                    om, fi, ka = tait_bryan_from_matrix(T)
                    vals = " ".join(f"{rel[r, c]:.10g}" for r in range(3) for c in range(4))
                    f.write(f"{ts_ns} {vals} {ts_ns} {om:.20g} {fi:.20g} {ka:.20g} \n")
        except OSError as e:
            raise ConversionError(f"cannot write {csv_path}: {e}") from e
        print(f"saved {csv_path}")

    if not file_names:
        raise ConversionError("no chunk has a trajectory element; the session would be empty")

    for T in m_poses:
        T[:3, 3] -= offset

    # ── poses.reg / lio_initial_poses.reg ────────────────────────────────────
    def save_poses_reg(path):
        with open(path, "w") as f:
            f.write(f"{len(m_poses)}\n")
            for name, T in zip(file_names, m_poses):
                f.write(name + "\n")
                for r in range(3):
                    f.write(" ".join(f"{T[r, c]:.6g}" for c in range(4)) + "\n")
                f.write("0 0 0 1\n")

    try:
        save_poses_reg(os.path.join(out, "lio_initial_poses.reg"))
        save_poses_reg(os.path.join(out, "poses.reg"))
    except OSError as e:
        raise ConversionError(f"cannot write the pose files in {out}: {e}") from e

    # ── session.json ─────────────────────────────────────────────────────────
    out_abs = os.path.abspath(out)
    session = {
        "Session Settings": {
            "folder_name": out_abs,
            "initial_poses_file_name": os.path.join(out_abs, "lio_initial_poses.reg"),
            "lidar_odometry_version": "HdMap",
            "offset_x": 0.0,
            "offset_y": 0.0,
            "offset_z": 0.0,
            "out_folder_name": out_abs,
            "out_poses_file_name": os.path.join(out_abs, "poses.reg"),
            "poses_file_name": os.path.join(out_abs, "poses.reg"),
        },
        "laz_file_names": [{"file_name": os.path.join(out_abs, n)} for n in file_names],
    }
    session_path = os.path.join(out_abs, "session.json")
    try:
        with open(session_path, "w") as f:
            json.dump(session, f, indent=2, sort_keys=True)
    except OSError as e:
        raise ConversionError(f"cannot write {session_path}: {e}") from e
    print(f"saving file: '{session_path}'")
    print("=== CONVERSION DONE ===")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="input bag (ROS 1 .bag file or ROS 2 bag directory)")
    ap.add_argument("--results-dir", required=True, help="KISS-SLAM results directory (holds <sequence>_poses_tum.txt)")
    ap.add_argument("--topic", required=True, help="point cloud topic in the bag")
    ap.add_argument("--out", required=True, help="output HDMapping session directory")
    ap.add_argument("--point-skip", type=int, default=1, help="keep every Nth point (default 1 = all)")
    args = ap.parse_args()
    try:
        convert(args.bag, args.results_dir, args.topic, args.out, args.point_skip)
    except ConversionError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception:
        print("ERROR: the conversion failed unexpectedly:", file=sys.stderr)
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
