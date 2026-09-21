"""CUDA-resident trial state and the small per-tick control summary."""

from __future__ import annotations

import numpy as np
import warp as wp

from graspdatagen.geometry import FloatArray, matrix_poses
from graspdatagen.records import METRICS, STAGES
from graspdatagen.runtime import GraspScene
from graspdatagen.validation_kernels import Criteria, TrialState, observe_grasps


class DeviceTrial:
    """Keep acceptance state on CUDA; read only three control integers each tick."""

    def __init__(
        self,
        scene: GraspScene,
        initial_object: FloatArray,
        initial_joints: FloatArray,
        opening_invalid: np.ndarray,
    ) -> None:
        self.scene: GraspScene = scene
        self.criteria: Criteria = Criteria()
        p = self.criteria
        for name in (
            "steps_per_second",
            "minimum_contact_force_N",
            "max_translation_m",
            "max_rotation_rad",
            "max_linear_speed_m_s",
            "max_angular_speed_rad_s",
            "max_joint_speed_m_s",
            "joint_tolerance_m",
            "approach_translation_m",
            "approach_rotation_rad",
        ):
            setattr(p, name, float(getattr(scene.profile, name)))
        p.stable_steps = round(scene.profile.stable_window_s * scene.profile.steps_per_second)
        p.loss_steps = round(scene.profile.contact_loss_s * scene.profile.steps_per_second)
        calibration = scene.pair.definition["calibration"]
        p.active = calibration["active_index"]
        p.follower = calibration["follower_index"]
        p.gearing = calibration["mimic"]["gearing"]
        p.offset = calibration["mimic"]["offset"]
        com = np.asarray(scene.pair.object_manifest["physical"]["com_pose_xyzw"][:3])
        p.com = wp.vec3d(*com)
        tcp = matrix_poses(scene.pair.arrays["T_B_tcp"][None])[0]
        p.tcp = wp.transformd(wp.vec3d(*tcp[:3]), wp.quatd(*tcp[3:]))
        n, device = scene.count, scene.device
        self.state: TrialState = TrialState()
        s = self.state
        s.initial = wp.array(matrix_poses(initial_object), dtype=wp.transformd, device=device)
        s.limits = wp.array(scene.pair.arrays["joint_limits_m"], dtype=wp.float64, device=device)
        s.previous_com = wp.array(
            initial_object[:, :3, :3] @ com + initial_object[:, :3, 3],
            dtype=wp.vec3d,
            device=device,
        )
        s.previous_rotation = wp.array(initial_object[:, :3, :3], dtype=wp.mat33d, device=device)
        s.previous_joints = wp.array(initial_joints, dtype=wp.float64, device=device)
        s.baseline = wp.empty(n, dtype=wp.transformd, device=device)
        s.missing = wp.zeros(n, dtype=wp.int32, device=device)
        s.stable = wp.zeros(n, dtype=wp.int32, device=device)
        s.failure = wp.array(opening_invalid.astype(np.int64), dtype=wp.int64, device=device)
        status = np.zeros((n, len(STAGES)), dtype=np.int64)
        status[opening_invalid, 0] = -1
        s.status = wp.array(status, dtype=wp.int64, device=device)
        s.metrics = wp.zeros((n, len(STAGES), len(METRICS)), dtype=wp.float64, device=device)
        identity = np.zeros((n, 7))
        identity[:, -1] = 1
        s.actual_tcp = wp.array(identity, dtype=wp.transformd, device=device)
        s.actual_joints = wp.zeros(initial_joints.shape, dtype=wp.float64, device=device)
        # alive, newly failed, and invalid-state bit mask.
        s.summary = wp.zeros(3, dtype=wp.int32, device=device)
        self.host_summary: wp.array = wp.empty(3, dtype=wp.int32, device="cpu", pinned=True)
        self.alive: int = int((~opening_invalid).sum())
        self.newly_failed: int = 0

    def observe(self, values: wp.array, stage: int, holding: bool, settling: bool) -> None:
        self.state.summary.zero_()
        wp.launch(
            observe_grasps,
            dim=self.scene.count,
            inputs=[
                values,
                self.scene.dofs,
                len(self.scene.contact_links),
                self.criteria,
                self.state,
                stage,
                holding,
                settling,
            ],
            device=self.scene.device,
        )
        wp.copy(self.host_summary, self.state.summary)
        wp.synchronize_stream(wp.get_stream(self.scene.device))
        alive, failed, invalid = self.host_summary.numpy()
        if invalid & 1:
            raise RuntimeError("Nonfinite PhysX state: solver_invalid")
        if invalid & 2:
            raise RuntimeError("Cross-environment target contact: solver_invalid")
        if invalid & 4:
            raise ValueError("Pose quaternions must be normalized")
        self.alive, self.newly_failed = int(alive), int(failed)
