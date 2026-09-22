"""One CUDA thread per environment for the continuous grasp acceptance protocol."""

# Warp checks its DSL annotations when compiling the real simulation workload.
# mypy: disable-error-code="valid-type,call-arg,call-overload,arg-type,attr-defined,index,operator"
# Warp uses explicit scalar constructors to mark mutable loop variables.
# ruff: noqa: UP018

import warp as wp

from graspdatagen.records import FAILURES, METRICS

GRAVITY_FAILURE = wp.constant(FAILURES.index("gravity_slip"))
ROTATION_FAILURE = wp.constant(FAILURES.index("rotation_slip"))
TRANSLATION_FAILURE = wp.constant(FAILURES.index("translation_slip"))
FINAL_HOLD_FAILURE = wp.constant(FAILURES.index("final_hold_slip"))
JOINT_FAILURE = wp.constant(FAILURES.index("joint_constraint_violation"))
METRIC_COUNT = wp.constant(len(METRICS))


@wp.struct
class Criteria:
    steps_per_second: wp.float64
    minimum_contact_force_N: wp.float64
    max_translation_m: wp.float64
    max_rotation_rad: wp.float64
    max_linear_speed_m_s: wp.float64
    max_angular_speed_rad_s: wp.float64
    max_joint_speed_m_s: wp.float64
    joint_tolerance_m: wp.float64
    approach_translation_m: wp.float64
    approach_rotation_rad: wp.float64
    stable_steps: int
    loss_steps: int
    active: int
    follower: int
    gearing: wp.float64
    offset: wp.float64
    com: wp.vec3d
    tcp: wp.transformd


@wp.struct
class TrialState:
    initial: wp.array(dtype=wp.transformd)
    limits: wp.array2d(dtype=wp.float64)
    previous_com: wp.array(dtype=wp.vec3d)
    previous_rotation: wp.array(dtype=wp.mat33d)
    previous_joints: wp.array2d(dtype=wp.float64)
    baseline: wp.array(dtype=wp.transformd)
    missing: wp.array(dtype=int)
    stable: wp.array(dtype=int)
    failure: wp.array(dtype=wp.int64)
    status: wp.array2d(dtype=wp.int64)
    metrics: wp.array3d(dtype=wp.float64)
    actual_tcp: wp.array(dtype=wp.transformd)
    actual_joints: wp.array2d(dtype=wp.float64)
    summary: wp.array(dtype=int)


@wp.func
def packed_pose(values: wp.array2d(dtype=float), env: int, offset: int) -> wp.transformd:
    position = wp.vec3d(
        wp.float64(values[env, offset]),
        wp.float64(values[env, offset + 1]),
        wp.float64(values[env, offset + 2]),
    )
    rotation = wp.quatd(
        wp.float64(values[env, offset + 3]),
        wp.float64(values[env, offset + 4]),
        wp.float64(values[env, offset + 5]),
        wp.float64(values[env, offset + 6]),
    )
    return wp.transformd(position, wp.normalize(rotation))


@wp.func
def pose_error(reference: wp.transformd, current: wp.transformd) -> wp.vec2d:
    delta = wp.transform_multiply(wp.transform_inverse(reference), current)
    q = wp.transform_get_rotation(delta)
    angle = wp.float64(2.0) * wp.atan2(wp.length(wp.vec3d(q[0], q[1], q[2])), wp.abs(q[3]))
    return wp.vec2d(wp.length(wp.transform_get_translation(delta)), angle)


# This row has the same order as records.METRICS, also used by the on-disk schema.
MetricRow = wp.types.vector(length=len(METRICS), dtype=wp.float64)


@wp.kernel
def observe_grasps(
    values: wp.array2d(dtype=float),
    dofs: int,
    contact_links: int,
    p: Criteria,
    s: TrialState,
    stage: int,
    holding: bool,
    settling: bool,
) -> None:
    env = wp.tid()
    contact_offset = 20 + 2 * dofs
    invalid = int(0)
    for j in range(values.shape[1]):
        if not wp.isfinite(values[env, j]):
            invalid = invalid | 1
    for j in range(contact_links):
        if values[env, contact_offset + contact_links + j] > 0.0:
            invalid = invalid | 2
    for offset in range(0, 14, 7):
        q = wp.quatd(
            wp.float64(values[env, offset + 3]),
            wp.float64(values[env, offset + 4]),
            wp.float64(values[env, offset + 5]),
            wp.float64(values[env, offset + 6]),
        )
        # Same tolerance as pose_matrices(): np.allclose(norm, 1, atol=1e-7).
        if wp.abs(wp.length(q) - wp.float64(1.0)) > wp.float64(1.01e-5):
            invalid = invalid | 4
    if invalid != 0:
        wp.atomic_or(s.summary, 2, invalid)
        return

    object_pose = packed_pose(values, env, 0)
    tcp = wp.transform_multiply(packed_pose(values, env, 7), p.tcp)
    relative = wp.transform_multiply(wp.transform_inverse(tcp), object_pose)
    if stage == 2 and settling:
        # Closure readback also belongs to already rejected environments, just
        # as in the host protocol. Its baseline is used starting on the next tick.
        s.baseline[env] = relative
        s.actual_tcp[env] = wp.transform_inverse(relative)
        for j in range(dofs):
            s.actual_joints[env, j] = wp.float64(values[env, 14 + j])
    if s.failure[env] != 0:
        return

    contacts = wp.float64(wp.min(values[env, contact_offset], values[env, contact_offset + 1]))
    palm_force = wp.float64(0.0)
    for j in range(2, contact_links):
        palm_force = wp.max(palm_force, wp.float64(values[env, contact_offset + j]))
    bilateral = contacts >= p.minimum_contact_force_N
    missing = int(0)
    if not bilateral:
        missing = s.missing[env] + 1
    s.missing[env] = missing
    solver_linear = wp.length(
        wp.vec3d(
            wp.float64(values[env, 14 + 2 * dofs]),
            wp.float64(values[env, 15 + 2 * dofs]),
            wp.float64(values[env, 16 + 2 * dofs]),
        )
    )
    solver_angular = wp.length(
        wp.vec3d(
            wp.float64(values[env, 17 + 2 * dofs]),
            wp.float64(values[env, 18 + 2 * dofs]),
            wp.float64(values[env, 19 + 2 * dofs]),
        )
    )
    # Keep resolved COM motion separate from raw solver velocities.
    com = wp.transform_point(object_pose, p.com)
    rotation = wp.quat_to_matrix(wp.transform_get_rotation(object_pose))
    delta_rotation = rotation * wp.transpose(s.previous_rotation[env])
    linear = wp.length(com - s.previous_com[env]) * p.steps_per_second
    angular = (
        wp.acos(
            wp.clamp(
                (wp.trace(delta_rotation) - wp.float64(1.0)) * wp.float64(0.5),
                wp.float64(-1.0),
                wp.float64(1.0),
            )
        )
        * p.steps_per_second
    )
    s.previous_com[env] = com
    s.previous_rotation[env] = rotation
    joint_speed = wp.float64(0.0)
    solver_joint_speed = wp.float64(0.0)
    limit_error = wp.float64(0.0)
    for j in range(dofs):
        joint = wp.float64(values[env, 14 + j])
        joint_speed = wp.max(joint_speed, wp.abs(joint - s.previous_joints[env, j]))
        s.previous_joints[env, j] = joint
        solver_joint_speed = wp.max(
            solver_joint_speed, wp.abs(wp.float64(values[env, 14 + dofs + j]))
        )
        limit_error = wp.max(limit_error, wp.max(s.limits[j, 0] - joint, joint - s.limits[j, 1]))
    joint_speed = joint_speed * p.steps_per_second
    slow = (
        linear < p.max_linear_speed_m_s
        and angular < p.max_angular_speed_rad_s
        and joint_speed < p.max_joint_speed_m_s
    )
    stable = int(0)
    if bilateral and slow:
        stable = s.stable[env] + 1
    s.stable[env] = stable
    drift = wp.vec2d(wp.float64(0.0))
    if holding:
        drift = pose_error(s.baseline[env], relative)
    mimic_error = wp.abs(
        wp.float64(values[env, 14 + p.follower])
        + p.gearing * wp.float64(values[env, 14 + p.active])
        + p.offset
    )
    current = MetricRow(
        contacts,
        drift[0],
        drift[1],
        wp.float64(missing) / p.steps_per_second,
        linear,
        angular,
        joint_speed,
        linear,
        angular,
        joint_speed,
        wp.float64(stable) / p.steps_per_second,
        solver_joint_speed,
        solver_joint_speed,
        mimic_error,
        limit_error,
        palm_force,
        solver_linear,
        solver_angular,
        solver_linear,
        solver_angular,
        wp.float64(1.0),
    )
    steps = s.metrics[env, stage, METRIC_COUNT - 1]
    if steps != wp.float64(0.0):
        current[0] = wp.min(s.metrics[env, stage, 0], contacts)
    for j in range(1, METRIC_COUNT - 1):
        if j < 7 or j == 11 or (j >= 13 and j < 18):
            current[j] = wp.max(s.metrics[env, stage, j], current[j])
    current[METRIC_COUNT - 1] = steps + wp.float64(1.0)
    for j in range(METRIC_COUNT):
        s.metrics[env, stage, j] = current[j]

    code = int(0)
    if wp.max(mimic_error, limit_error) > p.joint_tolerance_m:
        code = JOINT_FAILURE
    palm = palm_force >= p.minimum_contact_force_N
    hold_failure = FINAL_HOLD_FAILURE
    if stage == 3:
        hold_failure = GRAVITY_FAILURE
    elif stage == 4:
        hold_failure = ROTATION_FAILURE
    elif stage == 5:
        hold_failure = TRANSLATION_FAILURE
    if stage <= 1:
        moved = pose_error(s.initial[env], object_pose)
        collision = moved[0] > p.approach_translation_m or moved[1] > p.approach_rotation_rad
        for j in range(contact_links):
            if wp.float64(values[env, contact_offset + j]) >= p.minimum_contact_force_N:
                collision = True
        if collision:
            code = 2
    elif stage == 2:
        if palm:
            code = 4
    elif holding:
        if (
            drift[0] > p.max_translation_m
            or drift[1] > p.max_rotation_rad
            or missing > p.loss_steps
            or palm
        ):
            code = hold_failure
    if settling and stable < p.stable_steps:
        code = hold_failure
        if stage == 2:
            code = 3
            if bilateral:
                code = 4
    s.status[env, stage] = wp.int64(1)
    if code != 0:
        s.failure[env] = wp.int64(code)
        s.status[env, stage] = wp.int64(-1)
        wp.atomic_add(s.summary, 1, 1)
    else:
        wp.atomic_add(s.summary, 0, 1)
