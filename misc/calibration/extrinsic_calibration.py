from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple
from datafusion import col
from scipy.spatial.transform import Rotation, Slerp

import numpy as np
import rerun as rr
import argparse


@dataclass(frozen=True)
class Pose:
    """
    A data class for a pose (a translation and a rotation), as well as a time step.
    """
    t_xyz: np.ndarray
    q_xyzw: np.ndarray
    log_time: int

    def to_matrix(self) -> np.ndarray:
        """
        Returns this pose as a homogenous transformation matrix.
    
        :rtype: ndarray
        """
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(self.q_xyzw).as_matrix()
        T[:3, 3] = self.t_xyz
        return T

    @staticmethod
    def from_matrix(T: np.ndarray, log_time: int = 0) -> "Pose":
        """
        Converts a homogenous transformation matrix and time step to a pose.
        
        :param T: The homogenous transformation matrix
        :type T: np.ndarray
        :param log_time: The time step
        :type log_time: int
        :return: A pose constructed from the transformation matrix and time step
        :rtype: Pose
        """
        return Pose(
            t_xyz=T[:3, 3].copy(),
            q_xyzw=Rotation.from_matrix(T[:3, :3]).as_quat(),
            log_time=log_time
        )

    def inverse(self) -> "Pose":
        """
        Returns the inverse of this pose, as defined by the inverse of the corresponding 
        homogenous transformation matrix.
        
        :rtype: Pose
        """
        T_inv = np.linalg.inv(self.to_matrix())
        return Pose.from_matrix(T_inv, self.log_time)

    def compose(self, other: "Pose") -> "Pose":
        """
        Composes two poses. If you have a pose representing the transformation from A to B, and one
        representing the transformation from B to C, composing those would give the transformation from
        A to C.
        
        :param other: The other pose to compose with
        :type other: "Pose"
        :rtype: Pose
        """
        return Pose.from_matrix(self.to_matrix() @ other.to_matrix(), other.log_time)

    def relative_to(self, other: "Pose") -> "Pose":
        """
        Returns the pose of `other` expressed in the coordinate frame of `self`. 
        
        :param other: The other pose to compare
        :type other: "Pose"
        :rtype: Pose
        """
        return self.inverse().compose(other)


def skew(v: np.ndarray) -> np.ndarray:
    """
    Returns the Skew-Symetric matrix of a vector.
    If the original vector is `a` and this matrix is `M`, performing matrix multiplication
    `Mb` on some other vector `b` is equivallent to the cross product `a x b`.
    
    :param v: The vector to construct the Skew-Symetric matrix from
    :type v: np.ndarray
    :return: A Skew-Symetric matrix from the provided vector
    :rtype: ndarray
    """
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def calibrate_hand_eye(poses_A: List[Pose], poses_B: List[Pose]) -> np.ndarray:
    """
    Calculates the homogenous transformation matrix `X` from sensor pose `A` to sensor pose `B`, satisfying the 
    linear equation `AX=XB`.
    
    :param poses_A: A list of aligned poses from sensor A. Each pose must be aligned by index to the poses given
                    for sensor B
    :type poses_A: List[Pose]
    :param poses_B: A list of aligned poses from sensor B. Each pose must be aligned by index to the poses given
                    for sensor A
    :type poses_B: List[Pose]
    :return: The homogenous transformation matrix `X` from given poses.
    :rtype: ndarray
    """
    n = len(poses_A)
    if n != len(poses_B) or n < 3:
        raise ValueError("Need at least 3 synchronized pose pairs")

    A_list = []
    B_list = []

    for i in range(n - 1):
        A_rel = poses_A[i].relative_to(poses_A[i + 1])
        B_rel = poses_B[i].relative_to(poses_B[i + 1])
        A_list.append(A_rel)
        B_list.append(B_rel)

    M = np.zeros((3 * len(A_list), 3))
    b = np.zeros(3 * len(A_list))

    valid_count = 0
    for A_rel, B_rel in zip(A_list, B_list):
        rotvec_A = Rotation.from_quat(A_rel.q_xyzw).as_rotvec()
        rotvec_B = Rotation.from_quat(B_rel.q_xyzw).as_rotvec()

        angle_A = np.linalg.norm(rotvec_A)
        angle_B = np.linalg.norm(rotvec_B)

        # If the angles are tiny, just skip them for the sake of numerical stability.
        # If somehow you don't have enough angles from the .rrd data, you need to pilot the robot
        # more aggresively.
        if angle_A < 0.01 or angle_B < 0.01:
            continue

        M[3*valid_count:3*valid_count+3, :] = skew(rotvec_A + rotvec_B)
        b[3*valid_count:3*valid_count+3] = rotvec_B - rotvec_A
        valid_count += 1

    if valid_count < 2:
        raise ValueError("Not enough valid rotation samples for calibration")

    M = M[:3*valid_count]
    b = b[:3*valid_count]

    r_x, _, _, _ = np.linalg.lstsq(M, b, rcond=None)

    R_X = Rotation.from_rotvec(r_x).as_matrix()

    C = np.zeros((3 * len(A_list), 3))
    d = np.zeros(3 * len(A_list))

    valid_count = 0
    for A_rel, B_rel in zip(A_list, B_list):
        R_A = Rotation.from_quat(A_rel.q_xyzw).as_matrix()
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


def interpolate_pose(p0: Pose, p1: Pose, target_time: int) -> Pose:
    """
    Interpolates between two consecutive poses for some target time. Linear interpolation is used for translation,
    while SLERP is used for rotation.
    
    :param p0: The first pose to interpolate between. This should be the closest available pose (by log_time)
               to the target_time that doesn't exceed it.
    :type p0: Pose
    :param p1: The second pose to interpolate between. This should be the closest available pose (by log_time)
               to the target_time that doesn't preceed it.
    :type p1: Pose
    :param target_time: The targeted time to interpolate to.
    :type target_time: int
    :return: A pose linearly interpolated between the two given poses, at the given target time.
    :rtype: Pose
    """
    # If poses are at same timestep, no need to interpolate.
    t0, t1 = p0.log_time, p1.log_time
    if t1 == t0:
        return p0

    # Linear interpolation for translation.
    alpha = (target_time - t0) / (t1 - t0)
    t_interp = p0.t_xyz + alpha * (p1.t_xyz - p0.t_xyz)

    # SLERP interpolation for rotation.
    rots = Rotation.from_quat([p0.q_xyzw, p1.q_xyzw])
    slerp = Slerp([0, 1], rots)
    q_interp = slerp(alpha).as_quat()

    return Pose(t_xyz=t_interp, q_xyzw=q_interp, log_time=target_time)


def interpolate_poses_to_timestamps(poses: List[Pose], target_times: np.ndarray) -> Tuple[List[Pose], np.ndarray]:
    """
    Takes in a list of poses and interpolates them to the given list of target times. 
    Since kiss-icp has a significantly lower frequency (about 90x less), passing in the 
    kiss-icp timestamps for target_times and interpolating the t265 to those times works fine.

    Additionally returns an array containing whether the pose at the given index is valid or not.
    An interpolated pose is considered invalid if the target time for that pose is outside the range of
    given poses to interpolate from. (e.g If you want to interpolate a pose at time t=500 but the smallest
    given pose is at t=600.)
    
    :param poses: The list of poses to interpolate.
    :type poses: List[Pose]
    :param target_times: The target pose times to intepolate to.
    :type target_times: np.ndarray
    :return: A list of interpolated poses, chosen from the given target_times, as well as an array containing
             whether the pose at the given index is valid or not.
    :rtype: Tuple[List[Pose], ndarray]
    """
    if not poses:
        return [], np.array([], dtype=bool)

    pose_times = np.array([p.log_time for p in poses])
    min_time, max_time = pose_times.min(), pose_times.max()

    interpolated = []
    valid_mask = []

    for target_time in target_times:
        # Don't interpolate if outside of given poses.
        if target_time < min_time or target_time > max_time:
            valid_mask.append(False)
            continue

        # Otherwise, interpolate between the two closes poses.
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


def extract_nested_array(series, width: int) -> np.ndarray:
    """
    Corrects nested arrays that comes from extracting vector data from pandas df.
    
    :param series: The pandas series containing the vector data
    :param width: The width of the extracted vector/matrix
    :type width: int
    :return: The extracted numpy array/matrix from given data
    :rtype: ndarray[_AnyShape, dtype[Any]]
    """
    result = []
    for val in series:
        if val is None or len(val) == 0:
            result.append(np.full(width, np.nan))
        else:
            result.append(np.asarray(val[0], dtype=np.float64))
    return np.array(result)


def extract_poses(dataset, entity_path: str) -> List[Pose]:
    """
    Extracts the poses from a given entity path in the .rcc dataset.
    
    :param dataset: The dataset extracted from the .rcc file
    :param entity_path: The entity path for a given sensor in the dataset
    :type entity_path: str
    :return: A list of poses from the given entity path
    :rtype: List[Pose]
    """
    t_col = f"{entity_path}:Transform3D:translation"
    q_col = f"{entity_path}:Transform3D:quaternion"

    view = dataset.filter_contents([entity_path])
    df = view.reader(index="log_time", fill_latest_at=True)
    df = df.select(col("log_time"), col(t_col), col(q_col)).sort(col("log_time"))

    pdf = df.to_pandas()

    # Extract the translations and quaternions across all time steps.
    times = pdf["log_time"].values.astype(np.int64)
    translations = extract_nested_array(pdf[t_col], 3)
    quaternions = extract_nested_array(pdf[q_col], 4)

    valid = ~(np.isnan(translations).any(axis=1) | np.isnan(quaternions).any(axis=1))

    # Collect all valid transforms into a list of poses.
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

    # Extract poses from .rrd file.
    with rr.server.Server(datasets={"dataset": [rrd_path]}) as server:
        dataset = server.client().get_dataset("dataset")

        kiss_icp_poses = extract_poses(dataset, "/kiss_icp/local")
        t265_poses = extract_poses(dataset, "/localizer/t265_raw")

    print(f"Extracted {len(kiss_icp_poses)} kiss-icp poses")
    print(f"Extracted {len(t265_poses)} t265 poses")

    # Interpolate t265 poses to match timestamp of kiss-icp poses.
    kiss_times = np.array([p.log_time for p in kiss_icp_poses])
    t265_interp, valid_mask = interpolate_poses_to_timestamps(t265_poses, kiss_times)

    kiss_icp_aligned = [p for p, valid in zip(kiss_icp_poses, valid_mask) if valid]

    # I can't imagine you'd have less than 3 poses from the data, if that happens it's indicative of some
    # deeper issue.
    if len(t265_interp) < 3:
        print("ERROR: Need at least 3 aligned poses for calibration!")
        return

    X = calibrate_hand_eye(kiss_icp_aligned, t265_interp)

    R = Rotation.from_matrix(X[:3, :3])
    q = R.as_quat()
    euler = R.as_euler('xyz', degrees=True)

    print("\nExtrinsic Calibration Results:")
    print("Transform from t265 --> kiss-icp")

    print("\n4x4 Transformation Matrix:")
    for row in X:
        print(f"  [{row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}, {row[3]:+.6f}]")

    print(f"\nTranslation:")
    print(f"  x: {X[0, 3]:+.6f}")
    print(f"  y: {X[1, 3]:+.6f}")
    print(f"  z: {X[2, 3]:+.6f}")

    print(f"\nRotation (quaternion [x, y, z, w]):")
    print(f"  [{q[0]:+.6f}, {q[1]:+.6f}, {q[2]:+.6f}, {q[3]:+.6f}]")

    print(f"\nRotation (euler xyz, degrees):")
    print(f"  roll:  {euler[0]:+.3f}")
    print(f"  pitch: {euler[1]:+.3f}")
    print(f"  yaw:   {euler[2]:+.3f}")


if __name__ == "__main__":
    main()
