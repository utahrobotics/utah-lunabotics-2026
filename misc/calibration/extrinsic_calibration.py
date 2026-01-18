from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple
from datafusion import col

import numpy as np
import rerun as rr
import argparse


def normalize_quat(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return normalize_quat(np.array([x, y, z, w]))


def pose_to_matrix(pose: "Pose") -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(pose.q_xyzw)
    T[:3, 3] = pose.t_xyz
    return T


def matrix_to_pose(T: np.ndarray, log_time: int = 0) -> "Pose":
    return Pose(
        t_xyz=T[:3, 3].copy(),
        q_xyzw=rot_to_quat(T[:3, :3]),
        log_time=log_time
    )


def invert_pose(pose: "Pose") -> "Pose":
    T = pose_to_matrix(pose)
    T_inv = np.linalg.inv(T)
    return matrix_to_pose(T_inv, pose.log_time)


def compose_poses(p1: "Pose", p2: "Pose") -> "Pose":
    T1 = pose_to_matrix(p1)
    T2 = pose_to_matrix(p2)
    return matrix_to_pose(T1 @ T2, p2.log_time)


def compute_relative_motion(pose_prev: "Pose", pose_curr: "Pose") -> "Pose":
    return compose_poses(invert_pose(pose_prev), pose_curr)


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def rot_to_axis_angle(R: np.ndarray) -> Tuple[np.ndarray, float]:
    angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if angle < 1e-10:
        return np.array([1.0, 0.0, 0.0]), 0.0
    if np.abs(angle - np.pi) < 1e-10:
        eigvals, eigvecs = np.linalg.eig(R)
        idx = np.argmin(np.abs(eigvals - 1))
        axis = np.real(eigvecs[:, idx])
        return axis / np.linalg.norm(axis), angle
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) / (2 * np.sin(angle))
    return axis, angle


def calibrate_hand_eye(poses_A: List["Pose"], poses_B: List["Pose"]) -> np.ndarray:
    n = len(poses_A)
    if n != len(poses_B) or n < 3:
        raise ValueError("Need at least 3 synchronized pose pairs")

    A_list = []
    B_list = []

    for i in range(n - 1):
        A_rel = compute_relative_motion(poses_A[i], poses_A[i + 1])
        B_rel = compute_relative_motion(poses_B[i], poses_B[i + 1])
        A_list.append(A_rel)
        B_list.append(B_rel)

    M = np.zeros((3 * len(A_list), 3))
    b = np.zeros(3 * len(A_list))

    valid_count = 0
    for i, (A_rel, B_rel) in enumerate(zip(A_list, B_list)):
        R_A = quat_to_rot(A_rel.q_xyzw)
        R_B = quat_to_rot(B_rel.q_xyzw)

        axis_A, angle_A = rot_to_axis_angle(R_A)
        axis_B, angle_B = rot_to_axis_angle(R_B)

        if angle_A < 0.01 or angle_B < 0.01:
            continue

        alpha = axis_A * angle_A
        beta = axis_B * angle_B

        M[3*valid_count:3*valid_count+3, :] = skew(alpha + beta)
        b[3*valid_count:3*valid_count+3] = beta - alpha
        valid_count += 1

    if valid_count < 2:
        raise ValueError("Not enough valid rotation samples for calibration")

    M = M[:3*valid_count]
    b = b[:3*valid_count]

    r_x, _, _, _ = np.linalg.lstsq(M, b, rcond=None)

    theta = np.linalg.norm(r_x)
    if theta < 1e-10:
        R_X = np.eye(3)
    else:
        k = r_x / theta
        R_X = np.eye(3) + np.sin(theta) * skew(k) + (1 - np.cos(theta)) * (skew(k) @ skew(k))

    C = np.zeros((3 * len(A_list), 3))
    d = np.zeros(3 * len(A_list))

    valid_count = 0
    for i, (A_rel, B_rel) in enumerate(zip(A_list, B_list)):
        R_A = quat_to_rot(A_rel.q_xyzw)
        t_A = A_rel.t_xyz
        t_B = B_rel.t_xyz

        C[3*valid_count:3*valid_count+3, :] = R_A - np.eye(3)
        d[3*valid_count:3*valid_count+3] = R_X @ t_B - t_A
        valid_count += 1

    C = C[:3*valid_count]
    d = d[:3*valid_count]

    t_X, _, _, _ = np.linalg.lstsq(C, d, rcond=None)

    X = np.eye(4)
    X[:3, :3] = R_X
    X[:3, 3] = t_X

    return X


def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    q0 = normalize_quat(q0)
    q1 = normalize_quat(q1)

    dot = np.dot(q0, q1)

    if dot < 0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return normalize_quat(result)

    theta_0 = np.arccos(dot)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    sin_theta_0 = np.sin(theta_0)

    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0

    return s0 * q0 + s1 * q1


def interpolate_pose(p0: "Pose", p1: "Pose", target_time: int) -> "Pose":
    t0, t1 = p0.log_time, p1.log_time
    if t1 == t0:
        return p0

    alpha = (target_time - t0) / (t1 - t0)

    t_interp = p0.t_xyz + alpha * (p1.t_xyz - p0.t_xyz)
    q_interp = slerp(p0.q_xyzw, p1.q_xyzw, alpha)

    return Pose(t_xyz=t_interp, q_xyzw=q_interp, log_time=target_time)


def interpolate_poses_to_timestamps(
    poses: List["Pose"], target_times: np.ndarray
) -> Tuple[List["Pose"], np.ndarray]:
    if not poses:
        return [], np.array([], dtype=bool)

    pose_times = np.array([p.log_time for p in poses])
    min_time, max_time = pose_times.min(), pose_times.max()

    interpolated = []
    valid_mask = []

    for target_time in target_times:
        if target_time < min_time or target_time > max_time:
            valid_mask.append(False)
            continue

        idx = np.searchsorted(pose_times, target_time)

        if idx == 0:
            interpolated.append(Pose(
                t_xyz=poses[0].t_xyz.copy(),
                q_xyzw=poses[0].q_xyzw.copy(),
                log_time=target_time
            ))
        elif idx >= len(poses):
            interpolated.append(Pose(
                t_xyz=poses[-1].t_xyz.copy(),
                q_xyzw=poses[-1].q_xyzw.copy(),
                log_time=target_time
            ))
        else:
            interpolated.append(interpolate_pose(poses[idx - 1], poses[idx], target_time))

        valid_mask.append(True)

    return interpolated, np.array(valid_mask, dtype=bool)


@dataclass(frozen=True)
class Pose:
    t_xyz: np.ndarray
    q_xyzw: np.ndarray
    log_time: int


def extract_nested_array(series, width: int) -> np.ndarray:
    result = []
    for val in series:
        if val is None or len(val) == 0:
            result.append(np.full(width, np.nan))
        else:
            result.append(np.asarray(val[0], dtype=np.float64))
    return np.array(result)


def extract_poses(dataset, entity_path: str) -> List[Pose]:
    t_col = f"{entity_path}:Transform3D:translation"
    q_col = f"{entity_path}:Transform3D:quaternion"

    view = dataset.filter_contents([entity_path])
    df = view.reader(index="log_time", fill_latest_at=True)
    df = df.select(col("log_time"), col(t_col), col(q_col)).sort(col("log_time"))

    pdf = df.to_pandas()

    times = pdf["log_time"].values.astype(np.int64)
    translations = extract_nested_array(pdf[t_col], 3)
    quaternions = extract_nested_array(pdf[q_col], 4)

    valid = ~(np.isnan(translations).any(axis=1) | np.isnan(quaternions).any(axis=1))

    poses: List[Pose] = []
    for i in np.where(valid)[0]:
        poses.append(Pose(
            t_xyz=translations[i],
            q_xyzw=quaternions[i],
            log_time=int(times[i])
        ))

    return poses


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("rrd_file")
    args = parser.parse_args()
    rrd_path = Path(args.rrd_file)

    with rr.server.Server(datasets={"dataset": [rrd_path]}) as server:
        dataset = server.client().get_dataset("dataset")

        kiss_icp_poses = extract_poses(dataset, "/kiss_icp/local")
        t265_poses = extract_poses(dataset, "/localizer/t265_raw")

    print(f"Extracted {len(kiss_icp_poses)} kiss-icp poses")
    print(f"Extracted {len(t265_poses)} t256 poses")

    kiss_times = np.array([p.log_time for p in kiss_icp_poses])
    t265_interp, valid_mask = interpolate_poses_to_timestamps(t265_poses, kiss_times)

    kiss_icp_aligned = [p for p, valid in zip(kiss_icp_poses, valid_mask) if valid]

    if len(t265_interp) < 3:
        print("ERROR: Need at least 3 aligned poses for calibration!")
        return

    X = calibrate_hand_eye(kiss_icp_aligned, t265_interp)

    R = X[:3, :3]
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0

    print("\nExtrinsic Calibration Results:")
    print("Transform from t256 --> kiss-icp")

    print("\n4x4 Transformation Matrix:")
    for row in X:
        print(f"  [{row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}, {row[3]:+.6f}]")

    print(f"\nTranslation (whatever units rerun uses):")
    print(f"  x: {X[0, 3]:+.6f}")
    print(f"  y: {X[1, 3]:+.6f}")
    print(f"  z: {X[2, 3]:+.6f}")

    print(f"\nRotation quaternion")
    q = rot_to_quat(R)
    print(f"  [{q[0]:+.6f}, {q[1]:+.6f}, {q[2]:+.6f}, {q[3]:+.6f}]")


if __name__ == "__main__":
    main()
