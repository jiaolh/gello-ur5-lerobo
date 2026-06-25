import numpy as np


class SafetyChecker:
    def __init__(self, max_joint_delta: float):
        self.max_joint_delta = max_joint_delta

    def limit_step(self, current_joints: np.ndarray, target_joints: np.ndarray) -> np.ndarray:
        current_joints = np.asarray(current_joints, dtype=float)
        target_joints = np.asarray(target_joints, dtype=float)

        if current_joints.shape != (6,):
            raise ValueError(f"Expected current_joints shape (6,), got {current_joints.shape}")
        if target_joints.shape != (6,):
            raise ValueError(f"Expected target_joints shape (6,), got {target_joints.shape}")

        delta = target_joints - current_joints
        clipped_delta = np.clip(delta, -self.max_joint_delta, self.max_joint_delta)
        return current_joints + clipped_delta
