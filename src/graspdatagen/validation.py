"""Continuous free-space grasp protocol, shared unchanged by generation and replay."""

from __future__ import annotations

import time

import numpy as np
from scipy.spatial.transform import Rotation

from graspdatagen.config import ValidationProfile, digest
from graspdatagen.geometry import FloatArray, matrix_poses, pose_matrices
from graspdatagen.records import FAILURES, METRICS, STAGES, CandidateBatch, ValidationBatch
from graspdatagen.runtime import GraspScene


def smooth_motion(phase: float) -> float:
    return phase**3 * (10 - 15 * phase + 6 * phase**2)


def trial_conditions(
    batch: CandidateBatch, profile: ValidationProfile, seed: int
) -> tuple[np.ndarray, FloatArray]:
    seeds = np.array(
        [
            [int(digest([seed, int(i), t])[:16], 16) for t in range(profile.trials)]
            for i in batch.ids
        ],
        dtype=np.uint64,
    )
    accelerations = np.empty((len(batch), profile.trials, 6 + profile.random_directions, 3))
    for i in range(len(batch)):
        for t in range(profile.trials):
            rng = np.random.default_rng(seeds[i, t])
            random = rng.normal(size=(profile.random_directions, 3))
            random /= np.linalg.norm(random, axis=1)[:, None]
            accelerations[i, t] = np.vstack((np.eye(3), -np.eye(3), random)) * (
                profile.disturbance_acceleration_m_s2
            )
    return seeds, accelerations


def validate(
    scene: GraspScene,
    batch: CandidateBatch,
    initial_object: FloatArray,
    seeds: np.ndarray,
    accelerations: FloatArray,
) -> ValidationBatch:
    """Execute every trial from pregrasp; only reset writes the dynamic object pose.

    Explicit accelerations also serve the real late-disturbance counterexample:
    the audit changes its final pulse while using this same controller/checker.
    A failure is permanent within a trial, and all trials must pass to publish.
    """
    import warp as wp

    from graspdatagen.gpu_validation import DeviceTrial

    p = scene.profile
    n, trials = len(batch), p.trials
    if n != scene.count or initial_object.shape != (n, 4, 4):
        raise ValueError("Candidate and scene shapes disagree")
    if seeds.shape != (n, trials) or accelerations.shape != (n, trials, 6 + p.random_directions, 3):
        raise ValueError("Trial conditions have invalid shapes")
    if not np.isfinite(accelerations).all():
        raise ValueError("Nonfinite disturbance")
    failure = np.zeros((n, trials), dtype=np.int64)
    status = np.zeros((n, trials, len(STAGES)), dtype=np.int64)
    metrics = np.zeros((n, trials, len(STAGES), len(METRICS)))
    actual_tcp = np.tile(np.eye(4), (n, trials, 1, 1))
    actual_joints = np.zeros((n, trials, batch.pregrasp_joints.shape[1]))
    trace_frames: list[wp.array] = []
    trace_steps: list[np.ndarray] = []
    T_B_tcp = scene.pair.arrays["T_B_tcp"]
    target_base = initial_object @ batch.target @ np.linalg.inv(T_B_tcp)
    pre_base = initial_object @ batch.pregrasp @ np.linalg.inv(T_B_tcp)
    limits = scene.pair.arrays["joint_limits_m"]
    calibration = scene.pair.definition["calibration"]
    active = calibration["active_index"]
    zero_force = np.zeros((n, 3))
    step_number = 0

    def run_trial(trial: int) -> DeviceTrial:
        stage_time = np.zeros(len(STAGES), dtype=np.float64)
        stage_steps = np.zeros(len(STAGES), dtype=np.int64)
        stage_alive_start = np.full(len(STAGES), -1, dtype=np.int64)
        stage_alive_end = np.full(len(STAGES), -1, dtype=np.int64)

        def print_stage_profile() -> None:
            print(
                "P2 PROFILE trial="
                + str(trial)
                + " | "
                + " | ".join(
                    f"{STAGES[i]}: {stage_time[i]:.3f}s/"
                    f"{stage_steps[i]}steps/"
                    f"{stage_alive_start[i]}->{stage_alive_end[i]}alive"
                    for i in range(len(STAGES))
                    if stage_steps[i] > 0
                ),
                flush=True,
            )

        scene.reset(matrix_poses(initial_object), matrix_poses(pre_base), batch.pregrasp_joints)
        if scene.runtime.gui:
            positions = np.concatenate((initial_object[:, :3, 3], pre_base[:, :3, 3]))
            scene.runtime.frame_validation(positions, scene.visual_radius)
        commands = batch.pregrasp_joints.copy()
        opening_invalid = (
            (batch.opening <= batch.contact_width)
            | (commands < limits[:, 0]).any(axis=1)
            | (commands > limits[:, 1] + p.joint_tolerance_m).any(axis=1)
        )
        device_trial = DeviceTrial(scene, initial_object, batch.pregrasp_joints, opening_invalid)

        def observe(stage: int, holding: bool, settling: bool) -> None:
            nonlocal step_number
            observe_started = time.perf_counter()
            if stage_alive_start[stage] < 0:
                stage_alive_start[stage] = device_trial.alive
            scene.runtime.step()
            step_number += 1
            state = scene.capture()
            device_trial.observe(state, stage, holding, settling)
            if trial == 0 and (
                step_number == 1
                or step_number % max(1, p.steps_per_second // 20) == 0
                or settling
                or device_trial.newly_failed > 0
            ):
                # capture() reuses its buffer on the next tick. Preserve selected
                # frames on CUDA and decode them only after all trials finish.
                trace_frames.append(wp.clone(state))
                trace_steps.append(np.array([stage, step_number], dtype=np.int64))
            stage_time[stage] += time.perf_counter() - observe_started
            stage_steps[stage] += 1
            stage_alive_end[stage] = device_trial.alive

        observe(0, False, False)
        for step in range(round(p.approach_s * p.steps_per_second)):
            fraction = (step + 1) / round(p.approach_s * p.steps_per_second)
            blend = fraction * fraction * (3 - 2 * fraction)
            base = pre_base.copy()
            base[:, :3, 3] += blend * (target_base[:, :3, 3] - pre_base[:, :3, 3])
            scene.move_base(matrix_poses(base))
            observe(1, False, False)
            if device_trial.alive == 0:
                break
        if device_trial.alive == 0:
            print_stage_profile()
            return device_trial
        device_trial.state.stable.zero_()
        device_trial.state.missing.zero_()
        for step in range(round(p.close_s * p.steps_per_second)):
            commands[:, active] += np.clip(
                batch.close_command - commands[:, active],
                -p.command_speed_m_s / p.steps_per_second,
                p.command_speed_m_s / p.steps_per_second,
            )
            scene.command(commands)
            observe(2, False, step == round(p.close_s * p.steps_per_second) - 1)
        if device_trial.alive == 0:
            print_stage_profile()
            return device_trial
        scene.gravity(True)
        device_trial.state.stable.zero_()
        device_trial.state.missing.zero_()
        for step in range(round(p.hold_s * p.steps_per_second)):
            observe(3, True, step == round(p.hold_s * p.steps_per_second) - 1)
        if device_trial.alive == 0:
            print_stage_profile()
            return device_trial
        rotation_steps = round(p.rotation_s * p.steps_per_second)
        translation_steps = round(p.translation_s * p.steps_per_second)
        quarter = rotation_steps // 4
        half = rotation_steps // 2
        motion_steps = rotation_steps + translation_steps
        directions = accelerations.shape[2]
        pivot = (target_base @ T_B_tcp)[:, :3, 3]
        approach = (initial_object @ batch.target)[:, :3, :3] @ scene.pair.arrays[
            "approach_axis_tcp"
        ]
        lateral = np.column_stack((-approach[:, 1], approach[:, 0], np.zeros(n)))
        lateral[np.linalg.norm(lateral, axis=1) < 1e-8] = (1.0, 0.0, 0.0)
        lateral /= np.linalg.norm(lateral, axis=1)[:, None]
        base = target_base.copy()

        for step in range(rotation_steps):
            if step < quarter:
                phase = (step + 1) / quarter
                angle = -p.rotation_limit_rad * smooth_motion(phase)
            elif step < quarter + half:
                phase = (step - quarter + 1) / half
                angle = p.rotation_limit_rad * (-1 + 2 * smooth_motion(phase))
            else:
                phase = (step - quarter - half + 1) / quarter
                angle = p.rotation_limit_rad * (1 - smooth_motion(phase))
            rotation = Rotation.from_rotvec(approach * angle).as_matrix()
            base[:, :3, :3] = rotation @ target_base[:, :3, :3]
            base[:, :3, 3] = pivot + np.einsum(
                "nij,nj->ni", rotation, target_base[:, :3, 3] - pivot
            )
            scene.force(accelerations[:, trial, step * directions // motion_steps])
            scene.move_base(matrix_poses(base))
            observe(4, True, False)
            if device_trial.alive == 0:
                scene.force(zero_force)
                print_stage_profile()
                return device_trial

        half = translation_steps // 2
        base = target_base.copy()
        for step in range(translation_steps):
            phase = (step + 1) / half if step < half else (step - half + 1) / half
            blend = smooth_motion(phase)
            distance = p.translation_distance_m * (blend if step < half else 1 - blend)
            base[:, :3, 3] = target_base[:, :3, 3] + distance * lateral
            direction = (rotation_steps + step) * directions // motion_steps
            scene.force(accelerations[:, trial, direction])
            scene.move_base(matrix_poses(base))
            observe(5, True, False)
            if device_trial.alive == 0:
                scene.force(zero_force)
                print_stage_profile()
                return device_trial
        scene.force(zero_force)
        device_trial.state.stable.zero_()
        for step in range(round(p.final_hold_s * p.steps_per_second)):
            observe(6, True, step == round(p.final_hold_s * p.steps_per_second) - 1)
        scene.force(zero_force)

        print_stage_profile()
        return device_trial

    for trial in range(trials):
        trial_started = time.perf_counter()
        device_trial = run_trial(trial)
        failure[:, trial] = device_trial.state.failure.numpy()
        status[:, trial] = device_trial.state.status.numpy()
        metrics[:, trial] = device_trial.state.metrics.numpy()
        actual_tcp[:, trial] = pose_matrices(device_trial.state.actual_tcp.numpy())
        actual_joints[:, trial] = device_trial.state.actual_joints.numpy()
        print(
            f"P2 trial {trial}: {(status[:, trial] == 1).all(axis=1).sum()}/{n} passed; "
            f"failure_codes={np.bincount(failure[:, trial], minlength=len(FAILURES)).tolist()}; "
            f"elapsed_s={time.perf_counter() - trial_started:.2f}",
            flush=True,
        )
    trace_lists: dict[str, list[np.ndarray]] = {
        key: [] for key in ("object", "tcp", "joints", "contact", "velocity")
    }
    for frame in trace_frames:
        state = scene.decode(frame.numpy().astype(np.float64))
        for key in trace_lists:
            trace_lists[key].append(state["base"] @ T_B_tcp if key == "tcp" else state[key])
    trace = {key: np.asarray(value) for key, value in trace_lists.items()}
    trace["stage"] = np.asarray(trace_steps)
    return ValidationBatch(
        batch,
        failure,
        status,
        metrics,
        actual_tcp,
        actual_joints,
        initial_object,
        seeds,
        accelerations,
        trace,
    )
