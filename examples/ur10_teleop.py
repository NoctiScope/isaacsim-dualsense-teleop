import asyncio
import builtins
import sys
import types
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
from pxr import Usd, UsdGeom


# ---------------------------------------------------------
# PROJECT PATH
# ---------------------------------------------------------

# Replace this with the directory where you cloned this repository.
PROJECT_ROOT = Path(r"D:\isaacsim_dualsense_teleop")

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Stop whichever example was active before this script was run. The shared
# cleanup callback handles current versions; the attribute loop also cleans up
# subscriptions left behind by older versions of either example.
_previous_cleanup = getattr(builtins, "_dualsense_teleop_cleanup", None)
if callable(_previous_cleanup):
    _previous_cleanup("another teleop script started")
for _name in (
    "_jetbot_dualsense_subscription",
    "_jetbot_dualsense_timeline_subscription",
    "_jetbot_dualsense_stage_subscription",
    "_ur10_dualsense_subscription",
    "_ur10_dualsense_timeline_subscription",
    "_ur10_dualsense_stage_subscription",
):
    _old_subscription = getattr(builtins, _name, None)
    if _old_subscription is not None:
        _old_subscription.unsubscribe()
        setattr(builtins, _name, None)
for _name in ("_jetbot_dualsense_gamepad", "_ur10_dualsense_gamepad"):
    _old_gamepad = getattr(builtins, _name, None)
    if _old_gamepad is not None:
        _old_gamepad.close()
        setattr(builtins, _name, None)

# Read source directly on every run: bypass sys.modules and .pyc caches.
_input_path = PROJECT_ROOT / "input" / "dualsense_input.py"
_input_module = types.ModuleType("_ur10_dualsense_input")
_input_module.__file__ = str(_input_path)
exec(compile(_input_path.read_bytes(), str(_input_path), "exec"), _input_module.__dict__)
DualSenseInput = _input_module.DualSenseInput
print(f"[UR10 Teleop] Fresh source: {_input_path}")


# ---------------------------------------------------------
# Isaac Sim 6.x articulation API
# ---------------------------------------------------------

from isaacsim.core.experimental.prims import Articulation, RigidPrim


# =========================================================
# CONFIG
# =========================================================

UR10_PATH = "/World/ur10"
TCP_PATH = "/World/ur10/ee_link/tcp"
END_EFFECTOR_PATH = "/World/ur10/ee_link"
END_EFFECTOR_LINK = "ee_link"
HOME_NAMED_POSE_PATH = "/World/ur10/NamedPoses/pose_1"

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

MAX_LINEAR_SPEED = 0.15   # m/s
MAX_ANGULAR_SPEED = 0.75  # rad/s
MAX_JOINT_SPEED = 0.60    # rad/s
IK_GAIN = 0.25
POSITION_DEADBAND = 0.0008             # m
ORIENTATION_DEADBAND = np.deg2rad(0.3) # rad
R2_MODE_THRESHOLD = 0.50

ENABLE_ORIENTATION_CONTROL = True
MAX_POSITION_ERROR = 0.05
MAX_ORIENTATION_ERROR = np.deg2rad(10.0)

JOINT_LIMIT_MARGIN = np.deg2rad(8.0)
JOINT_LIMIT_SLOWDOWN_DISTANCE = np.deg2rad(12.0)
MAX_JOINT_TRACKING_ERROR = np.deg2rad(5.0)
NULLSPACE_CENTERING_GAIN = 0.05

IK_DAMPING_MIN = 0.03
IK_DAMPING_MAX = 0.25
SINGULARITY_DAMPING_START = 0.10
SINGULARITY_STRONG = 0.03
SINGULARITY_FREEZE = 0.01
DIAGNOSTIC_INTERVAL = 0.5

# Conservative world-frame workspace limits for the scene's base at the origin.
WORKSPACE_MIN = np.array([-1.20, -1.20, 0.05], dtype=np.float64)
WORKSPACE_MAX = np.array([1.20, 1.20, 1.60], dtype=np.float64)


# =========================================================
# HELPERS
# =========================================================

def quaternion_multiply(a, b):
    """Multiply wxyz quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def rotation_vector_to_quaternion(rotation_vector):
    """Convert a world-frame rotation vector to a wxyz quaternion."""
    angle = float(np.linalg.norm(rotation_vector))
    if angle < 1.0e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    axis = rotation_vector / angle
    half_angle = 0.5 * angle
    return np.concatenate(([np.cos(half_angle)], axis * np.sin(half_angle)))


def quaternion_error(target, current):
    """Return the shortest world-frame rotation vector from current to target."""
    current_conjugate = current * np.array([1.0, -1.0, -1.0, -1.0])
    error = quaternion_multiply(target, current_conjugate)
    if error[0] < 0.0:
        error = -error
    vector_length = float(np.linalg.norm(error[1:]))
    if vector_length < 1.0e-9:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * np.arctan2(vector_length, np.clip(error[0], -1.0, 1.0))
    return error[1:] * (angle / vector_length)


def rotate_vector(quaternion, vector):
    """Rotate a 3D vector with a wxyz quaternion."""
    vector_quaternion = np.concatenate(([0.0], vector))
    conjugate = quaternion * np.array([1.0, -1.0, -1.0, -1.0])
    return quaternion_multiply(
        quaternion_multiply(quaternion, vector_quaternion),
        conjugate,
    )[1:]


def skew(vector):
    """Return the cross-product matrix for a 3D vector."""
    x, y, z = vector
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )


def smoothstep(edge0, edge1, value):
    """Continuous 0-to-1 interpolation with zero slope at both ends."""
    t = np.clip((value - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def adaptive_ik_parameters(sigma_min):
    """Return continuous damping and Cartesian scaling near singularities."""
    damping_weight = 1.0 - smoothstep(
        SINGULARITY_STRONG,
        SINGULARITY_DAMPING_START,
        sigma_min,
    )
    damping = IK_DAMPING_MIN + damping_weight * (IK_DAMPING_MAX - IK_DAMPING_MIN)

    # Full Cartesian correction at sigma >= 0.03, frozen at sigma <= 0.01.
    cartesian_scale = smoothstep(SINGULARITY_FREEZE, SINGULARITY_STRONG, sigma_min)
    return float(damping), float(cartesian_scale)


def leash_position_target(target, current):
    """Keep the integrated position target close enough to the measured TCP."""
    error = target - current
    error_norm = float(np.linalg.norm(error))
    if error_norm > MAX_POSITION_ERROR:
        return current + error * (MAX_POSITION_ERROR / error_norm)
    return target


def leash_orientation_target(target, current):
    """Keep the integrated orientation target within a bounded angular error."""
    error = quaternion_error(target, current)
    error_norm = float(np.linalg.norm(error))
    if error_norm <= MAX_ORIENTATION_ERROR:
        return target
    limited_error = error * (MAX_ORIENTATION_ERROR / error_norm)
    limited_target = quaternion_multiply(
        rotation_vector_to_quaternion(limited_error),
        current,
    )
    return limited_target / np.linalg.norm(limited_target)


def protect_joint_command(joint_delta, measured_joint_positions):
    """Apply smooth soft-limit protection and a per-joint tracking leash."""
    protected_delta = joint_delta.copy()
    finite_indices = np.flatnonzero(finite_joint_limit_mask)

    for index in finite_indices:
        lower_distance = min(
            commanded_joint_positions[index] - joint_soft_lower_limits[index],
            measured_joint_positions[index] - joint_soft_lower_limits[index],
        )
        upper_distance = min(
            joint_soft_upper_limits[index] - commanded_joint_positions[index],
            joint_soft_upper_limits[index] - measured_joint_positions[index],
        )

        if protected_delta[index] < 0.0:
            protected_delta[index] *= smoothstep(
                0.0,
                JOINT_LIMIT_SLOWDOWN_DISTANCE,
                lower_distance,
            )
        elif protected_delta[index] > 0.0:
            protected_delta[index] *= smoothstep(
                0.0,
                JOINT_LIMIT_SLOWDOWN_DISTANCE,
                upper_distance,
            )

    candidate = commanded_joint_positions + protected_delta

    # Intersect each joint's tracking leash with its finite soft-limit range.
    # If measured state is already too far outside a soft limit for the two
    # constraints to overlap, move only one tracking-leash step toward safety.
    tracking_lower = measured_joint_positions - MAX_JOINT_TRACKING_ERROR
    tracking_upper = measured_joint_positions + MAX_JOINT_TRACKING_ERROR
    command_lower = tracking_lower.copy()
    command_upper = tracking_upper.copy()
    command_lower[finite_joint_limit_mask] = np.maximum(
        command_lower[finite_joint_limit_mask],
        joint_soft_lower_limits[finite_joint_limit_mask],
    )
    command_upper[finite_joint_limit_mask] = np.minimum(
        command_upper[finite_joint_limit_mask],
        joint_soft_upper_limits[finite_joint_limit_mask],
    )

    valid_intersection = command_lower <= command_upper
    candidate[valid_intersection] = np.clip(
        candidate[valid_intersection],
        command_lower[valid_intersection],
        command_upper[valid_intersection],
    )
    for index in np.flatnonzero(~valid_intersection):
        if measured_joint_positions[index] < joint_soft_lower_limits[index]:
            candidate[index] = tracking_upper[index]
        else:
            candidate[index] = tracking_lower[index]

    if finite_indices.size:
        distances = np.minimum(
            measured_joint_positions[finite_indices] - joint_soft_lower_limits[finite_indices],
            joint_soft_upper_limits[finite_indices] - measured_joint_positions[finite_indices],
        )
        closest_local_index = int(np.argmin(distances))
        closest_joint_index = int(finite_indices[closest_local_index])
        minimum_limit_distance = float(distances[closest_local_index])
        near_limit = minimum_limit_distance < JOINT_LIMIT_SLOWDOWN_DISTANCE
    else:
        closest_joint_index = None
        minimum_limit_distance = np.inf
        near_limit = False

    protection_changed_command = not np.allclose(
        candidate,
        commanded_joint_positions + joint_delta,
        rtol=0.0,
        atol=1.0e-12,
    )
    protection_active = near_limit or protection_changed_command
    return candidate, protection_active, near_limit, closest_joint_index, minimum_limit_distance


# =========================================================
# CHECK STAGE
# =========================================================

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("Open ur10_scene.usd before running ur10_teleop.py.")

ur10_prim = stage.GetPrimAtPath(UR10_PATH)
tcp_prim = stage.GetPrimAtPath(TCP_PATH)

if not ur10_prim.IsValid():
    raise RuntimeError(f"UR10 not found at {UR10_PATH}.")
if not tcp_prim.IsValid():
    raise RuntimeError(f"TCP not found at {TCP_PATH}.")
if "IsaacSiteAPI" not in tcp_prim.GetAppliedSchemas():
    raise RuntimeError(f"IsaacSiteAPI is not applied to {TCP_PATH}.")

tcp_local_transform = UsdGeom.Xformable(tcp_prim).GetLocalTransformation()
if isinstance(tcp_local_transform, tuple):
    tcp_local_transform = tcp_local_transform[0]
tcp_local_transform.Orthonormalize()
TCP_LOCAL_POSITION = np.asarray(tcp_local_transform.ExtractTranslation(), dtype=np.float64)
_tcp_local_quaternion = tcp_local_transform.ExtractRotationQuat()
TCP_LOCAL_ORIENTATION = np.array(
    [_tcp_local_quaternion.GetReal(), *_tcp_local_quaternion.GetImaginary()],
    dtype=np.float64,
)
TCP_LOCAL_ORIENTATION /= np.linalg.norm(TCP_LOCAL_ORIENTATION)


# =========================================================
# GAMEPAD AND ROBOT
# =========================================================

gamepad = DualSenseInput(deadzone=0.08)
builtins._ur10_dualsense_gamepad = gamepad

ur10 = Articulation(UR10_PATH)
end_effector = RigidPrim(END_EFFECTOR_PATH)
timeline = omni.timeline.get_timeline_interface()


# =========================================================
# STATE
# =========================================================

emergency_stop = False
input_failed = False
control_failed = False
control_initialized = False

arm_dof_indices = None
end_effector_link_index = None
joint_lower_limits = None
joint_upper_limits = None
joint_soft_lower_limits = None
joint_soft_upper_limits = None
finite_joint_limit_mask = None
target_position = None
target_orientation = None
commanded_joint_positions = None

diagnostic_elapsed = 0.0
singularity_warning_active = False
joint_limit_warning_active = False
subscription = None
timeline_subscription = None
stage_subscription = None
home_restore_task = None
shutting_down = False


def get_tcp_world_pose():
    """Compute TCP pose from the physics-backed ee_link pose and the Site offset."""
    positions, orientations = end_effector.get_world_poses()
    ee_position = positions.numpy()[0].astype(np.float64)
    ee_orientation = orientations.numpy()[0].astype(np.float64)
    ee_orientation /= np.linalg.norm(ee_orientation)

    tcp_offset_world = rotate_vector(ee_orientation, TCP_LOCAL_POSITION)
    tcp_position = ee_position + tcp_offset_world
    tcp_orientation = quaternion_multiply(ee_orientation, TCP_LOCAL_ORIENTATION)
    tcp_orientation /= np.linalg.norm(tcp_orientation)
    return tcp_position, tcp_orientation, tcp_offset_world


def read_home_named_pose():
    """Read the saved UR10 NamedPose and return it in ARM_JOINTS order, in radians."""
    pose_prim = stage.GetPrimAtPath(HOME_NAMED_POSE_PATH)
    if not pose_prim.IsValid():
        raise RuntimeError(f"Home NamedPose not found at {HOME_NAMED_POSE_PATH}.")

    valid_attribute = pose_prim.GetAttribute("isaac:robot:pose:valid")
    if valid_attribute and valid_attribute.HasAuthoredValueOpinion():
        if not bool(valid_attribute.Get()):
            raise RuntimeError(f"Home NamedPose is marked invalid: {HOME_NAMED_POSE_PATH}.")

    joints_relationship = pose_prim.GetRelationship("isaac:robot:pose:joints")
    values_attribute = pose_prim.GetAttribute("isaac:robot:pose:jointValues")
    joint_paths = list(joints_relationship.GetTargets()) if joints_relationship else []
    joint_values_degrees = values_attribute.Get() if values_attribute else None

    if joint_values_degrees is None or len(joint_paths) != len(joint_values_degrees):
        raise RuntimeError(
            f"Home NamedPose has mismatched joint paths and values: {HOME_NAMED_POSE_PATH}."
        )

    values_by_joint_name = {
        joint_path.name: float(value)
        for joint_path, value in zip(joint_paths, joint_values_degrees)
    }
    missing_joints = [name for name in ARM_JOINTS if name not in values_by_joint_name]
    if missing_joints:
        raise RuntimeError(
            f"Home NamedPose is missing UR10 joints: {', '.join(missing_joints)}."
        )

    # IsaacNamedPose stores revolute-joint values in degrees; the articulation
    # tensor API uses radians. Reorder by name instead of trusting relation order.
    home_positions = np.deg2rad(
        np.array([values_by_joint_name[name] for name in ARM_JOINTS], dtype=np.float64)
    )
    if not np.all(np.isfinite(home_positions)):
        raise RuntimeError(f"Home NamedPose contains non-finite values: {HOME_NAMED_POSE_PATH}.")
    return home_positions


def initialize_control():
    """Apply the saved home pose, then initialize teleop targets from that pose."""
    global arm_dof_indices, end_effector_link_index
    global joint_lower_limits, joint_upper_limits
    global joint_soft_lower_limits, joint_soft_upper_limits, finite_joint_limit_mask
    global target_position, target_orientation, commanded_joint_positions
    global control_initialized

    arm_dof_indices = ur10.get_dof_indices(ARM_JOINTS).numpy().tolist()
    end_effector_link_index = int(ur10.get_link_indices(END_EFFECTOR_LINK).numpy()[0])

    home_joint_positions = read_home_named_pose()
    # NamedPose is stored scene data and is not automatically applied by Play.
    # Set both physical state and drive target once so gravity or a stale saved
    # velocity cannot choose the teleop start pose before the controller starts.
    ur10.set_dof_positions(
        home_joint_positions.reshape(1, -1),
        dof_indices=arm_dof_indices,
    )
    ur10.set_dof_velocities(
        np.zeros((1, len(ARM_JOINTS)), dtype=np.float64),
        dof_indices=arm_dof_indices,
    )

    target_position, target_orientation, _ = get_tcp_world_pose()
    commanded_joint_positions = home_joint_positions.copy()
    send_commanded_joint_positions()

    lower_limits, upper_limits = ur10.get_dof_limits(dof_indices=arm_dof_indices)
    joint_lower_limits = lower_limits.numpy()[0].astype(np.float64)
    joint_upper_limits = upper_limits.numpy()[0].astype(np.float64)
    joint_ranges = joint_upper_limits - joint_lower_limits
    finite_joint_limit_mask = (
        np.isfinite(joint_lower_limits)
        & np.isfinite(joint_upper_limits)
        & (joint_ranges > 2.0 * JOINT_LIMIT_MARGIN)
        # Treat extremely large finite sentinel ranges as unbounded/continuous.
        & (joint_ranges < 100.0 * np.pi)
    )
    joint_soft_lower_limits = np.full(len(ARM_JOINTS), -np.inf, dtype=np.float64)
    joint_soft_upper_limits = np.full(len(ARM_JOINTS), np.inf, dtype=np.float64)
    joint_soft_lower_limits[finite_joint_limit_mask] = (
        joint_lower_limits[finite_joint_limit_mask] + JOINT_LIMIT_MARGIN
    )
    joint_soft_upper_limits[finite_joint_limit_mask] = (
        joint_upper_limits[finite_joint_limit_mask] - JOINT_LIMIT_MARGIN
    )

    stiffnesses, dampings = ur10.get_dof_gains(dof_indices=arm_dof_indices)
    max_efforts = ur10.get_dof_max_efforts(dof_indices=arm_dof_indices)
    control_initialized = True

    print(
        f"[UR10 Teleop] Applied home NamedPose: {HOME_NAMED_POSE_PATH} "
        f"{np.array2string(np.rad2deg(commanded_joint_positions), precision=1)} deg"
    )
    print(f"[UR10 Teleop] TCP start position: {target_position}")
    print(
        "[UR10 Teleop] drive stiffness: "
        f"{np.array2string(stiffnesses.numpy()[0], precision=3)}"
    )
    print(
        "[UR10 Teleop] drive damping: "
        f"{np.array2string(dampings.numpy()[0], precision=3)}"
    )
    print(
        "[UR10 Teleop] drive max effort: "
        f"{np.array2string(max_efforts.numpy()[0], precision=3)}"
    )
    print(
        "[UR10 Teleop] joint lower limits (deg): "
        f"{np.array2string(np.rad2deg(joint_lower_limits), precision=1)}"
    )
    print(
        "[UR10 Teleop] joint upper limits (deg): "
        f"{np.array2string(np.rad2deg(joint_upper_limits), precision=1)}"
    )


def sync_commanded_joints_to_current():
    """Reset the persistent joint command to the measured arm position."""
    global commanded_joint_positions
    if not control_initialized:
        return
    commanded_joint_positions = (
        ur10.get_dof_positions().numpy()[0, arm_dof_indices].astype(np.float64).copy()
    )


def send_commanded_joint_positions():
    """Keep sending the persistent position target so the drives hold the arm."""
    if commanded_joint_positions is None:
        return
    ur10.set_dof_position_targets(
        commanded_joint_positions.reshape(1, -1),
        dof_indices=arm_dof_indices,
    )


def restore_home_pose_after_stop():
    """Restore scene joint state/targets to pose_1 after the physics timeline stops."""
    global control_initialized, emergency_stop, control_failed
    global arm_dof_indices, end_effector_link_index
    global target_position, target_orientation, commanded_joint_positions
    global diagnostic_elapsed, singularity_warning_active, joint_limit_warning_active

    home_joint_positions = read_home_named_pose()

    # STOP invalidates the tensor articulation, so restore through USD. Author
    # the reset in the session layer: the next Play sees pose_1, while the USD
    # file itself is not dirtied by every teleop session.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for joint_name, position_radians in zip(ARM_JOINTS, home_joint_positions):
            joint_prim = stage.GetPrimAtPath(f"{UR10_PATH}/joints/{joint_name}")
            if not joint_prim.IsValid():
                raise RuntimeError(f"UR10 joint not found while restoring home: {joint_name}.")

            position_degrees = float(np.rad2deg(position_radians))
            joint_prim.GetAttribute("drive:angular:physics:targetPosition").Set(
                position_degrees
            )
            joint_prim.GetAttribute("state:angular:physics:position").Set(
                position_degrees
            )
            joint_prim.GetAttribute("state:angular:physics:velocity").Set(0.0)

    # Force complete reinitialization on the next Play. Tensor indices and
    # measured poses from the stopped physics scene must not be reused.
    control_initialized = False
    emergency_stop = False
    control_failed = False
    arm_dof_indices = None
    end_effector_link_index = None
    target_position = None
    target_orientation = None
    commanded_joint_positions = None
    diagnostic_elapsed = 0.0
    singularity_warning_active = False
    joint_limit_warning_active = False
    print(f"[UR10 Teleop] STOP - restored home NamedPose: {HOME_NAMED_POSE_PATH}")


# =========================================================
# LIFECYCLE
# =========================================================

async def restore_home_pose_on_next_update():
    """Wait for Isaac Sim's STOP cleanup, then author the home reset."""
    await omni.kit.app.get_app().next_update_async()
    if shutting_down or omni.usd.get_context().get_stage() is not stage:
        return
    try:
        restore_home_pose_after_stop()
    except Exception as exc:
        print(f"[UR10 Teleop] Failed to restore home pose after STOP: {exc}")


def stop_physics_subscription():
    global subscription
    if subscription is not None:
        subscription.unsubscribe()
        subscription = None
    builtins._ur10_dualsense_subscription = None


def start_physics_subscription():
    global subscription
    if subscription is None and not shutting_down:
        subscription = (
            omni.physx
            .get_physx_interface()
            .subscribe_physics_step_events(update)
        )
        builtins._ur10_dualsense_subscription = subscription


def cleanup(reason="cleanup"):
    global timeline_subscription, stage_subscription, home_restore_task, shutting_down
    if shutting_down:
        return
    shutting_down = True
    stop_physics_subscription()
    if home_restore_task is not None and not home_restore_task.done():
        home_restore_task.cancel()
    home_restore_task = None
    if timeline_subscription is not None:
        timeline_subscription.unsubscribe()
        timeline_subscription = None
    if stage_subscription is not None:
        stage_subscription.unsubscribe()
        stage_subscription = None
    gamepad.close()
    builtins._ur10_dualsense_timeline_subscription = None
    builtins._ur10_dualsense_stage_subscription = None
    builtins._ur10_dualsense_gamepad = None
    if getattr(builtins, "_dualsense_teleop_cleanup", None) is cleanup:
        builtins._dualsense_teleop_cleanup = None
    print(f"[UR10 Teleop] Closed: {reason}")


def on_timeline_event(event):
    global home_restore_task
    event_type = omni.timeline.TimelineEventType(event.type)
    if event_type == omni.timeline.TimelineEventType.STOP:
        # The articulation tensor becomes invalid during STOP. Remove the
        # physics callback immediately, then restore pose_1 after STOP cleanup.
        stop_physics_subscription()
        home_restore_task = asyncio.ensure_future(restore_home_pose_on_next_update())
    elif event_type == omni.timeline.TimelineEventType.PLAY:
        start_physics_subscription()


def on_stage_event(event):
    if event.type == int(omni.usd.StageEventType.CLOSING):
        cleanup("stage closing")


# =========================================================
# UPDATE
# =========================================================

def update(step):
    global emergency_stop, input_failed, control_failed
    global target_position, target_orientation, commanded_joint_positions
    global diagnostic_elapsed, singularity_warning_active, joint_limit_warning_active

    if not timeline.is_playing():
        return

    try:
        if not control_initialized:
            initialize_control()

        gamepad.update()

        # Cross always wins if Cross and Triangle are pressed together.
        if gamepad.button("cross"):
            if not emergency_stop:
                print("[UR10 Teleop] E-STOP")
                sync_commanded_joints_to_current()
            emergency_stop = True
            send_commanded_joint_positions()
            return

        if gamepad.pressed("triangle"):
            target_position, target_orientation, _ = get_tcp_world_pose()
            sync_commanded_joints_to_current()
            emergency_stop = False
            control_failed = False
            print("[UR10 Teleop] ENABLED")

        if emergency_stop:
            send_commanded_joint_positions()
            return

        dt = float(step)
        dt = np.clip(dt, 0.0, 0.05)

        # Read current TCP pose and the shifted TCP Jacobian before integrating targets.
        current_position, current_orientation, tcp_offset_world = get_tcp_world_pose()

        jacobians = ur10.get_jacobian_matrices().numpy()
        jacobian = jacobians[0, end_effector_link_index - 1][:, arm_dof_indices].astype(np.float64)
        jacobian[:3] -= skew(tcp_offset_world) @ jacobian[3:]

        # With orientation disabled this is a true 3x6 XYZ task. The remaining
        # three joint DOFs are available to the null-space secondary objective.
        if ENABLE_ORIENTATION_CONTROL:
            task_jacobian = jacobian
        else:
            task_jacobian = jacobian[:3, :]

        singular_values = np.linalg.svd(task_jacobian, compute_uv=False)
        sigma_min = float(singular_values[-1])
        damping, cartesian_scale = adaptive_ik_parameters(sigma_min)

        singularity_protection_active = cartesian_scale < 0.999
        if singularity_protection_active and not singularity_warning_active:
            print("[UR10 Teleop] Near singularity - Cartesian command scaled")
        singularity_warning_active = singularity_protection_active

        # R2 switches the left stick from XY translation to Pitch/Yaw rotation.
        rotation_mode = gamepad.trigger("r2") >= R2_MODE_THRESHOLD
        if rotation_mode:
            linear_velocity = np.zeros(3, dtype=np.float64)
        else:
            linear_velocity = np.array(
                [
                    gamepad.axis("left_x"),
                    -gamepad.axis("left_y"),
                    -gamepad.axis("right_y"),
                ],
                dtype=np.float64,
            ) * MAX_LINEAR_SPEED
        target_position += linear_velocity * dt
        target_position = np.clip(target_position, WORKSPACE_MIN, WORKSPACE_MAX)

        # World-frame roll, pitch, yaw.
        if ENABLE_ORIENTATION_CONTROL:
            if rotation_mode:
                # Mapping follows the requested X/Y order: X -> Pitch, Y -> Yaw.
                angular_velocity = np.array(
                    [
                        0.0,
                        gamepad.axis("left_x"),
                        -gamepad.axis("left_y"),
                    ],
                    dtype=np.float64,
                ) * MAX_ANGULAR_SPEED
            else:
                angular_velocity = np.array(
                    [
                        gamepad.axis("right_x"),
                        0.0,
                        0.0,
                    ],
                    dtype=np.float64,
                ) * MAX_ANGULAR_SPEED
            delta_orientation = rotation_vector_to_quaternion(angular_velocity * dt)
            target_orientation = quaternion_multiply(delta_orientation, target_orientation)
            target_orientation /= np.linalg.norm(target_orientation)

        # Leash accumulated targets to the current TCP pose. This is separate
        # from the coarse world-frame workspace box.
        target_position = leash_position_target(target_position, current_position)
        target_orientation = leash_orientation_target(target_orientation, current_orientation)

        position_error = target_position - current_position
        orientation_error = quaternion_error(target_orientation, current_orientation)
        position_error_norm = float(np.linalg.norm(position_error))
        orientation_error_norm = float(np.linalg.norm(orientation_error))
        if np.linalg.norm(position_error) < POSITION_DEADBAND:
            position_error[:] = 0.0
        if np.linalg.norm(orientation_error) < ORIENTATION_DEADBAND:
            orientation_error[:] = 0.0

        if ENABLE_ORIENTATION_CONTROL:
            task_error = np.concatenate([position_error, orientation_error])
        else:
            task_error = position_error

        # Apply singularity scaling only to the IK correction. Target integration
        # remains unscaled; the Cartesian target leash prevents unreachable wind-up.
        task_dim = task_jacobian.shape[0]
        damping_matrix = np.eye(task_dim, dtype=np.float64) * (damping**2)
        damped_inverse = np.linalg.solve(
            task_jacobian @ task_jacobian.T + damping_matrix,
            np.eye(task_dim, dtype=np.float64),
        )
        jacobian_pseudoinverse = task_jacobian.T @ damped_inverse
        joint_delta = jacobian_pseudoinverse @ (task_error * cartesian_scale)
        joint_delta *= IK_GAIN

        measured_joint_positions = (
            ur10.get_dof_positions().numpy()[0, arm_dof_indices].astype(np.float64)
        )

        if not ENABLE_ORIENTATION_CONTROL:
            # Use position-task redundancy to move finite-range joints gently
            # toward their midpoints without disturbing the XYZ primary task.
            centering_error = np.zeros(len(ARM_JOINTS), dtype=np.float64)
            finite_lower = joint_lower_limits[finite_joint_limit_mask]
            finite_upper = joint_upper_limits[finite_joint_limit_mask]
            joint_midpoints = 0.5 * (finite_lower + finite_upper)
            half_ranges = 0.5 * (finite_upper - finite_lower)
            centering_error[finite_joint_limit_mask] = (
                joint_midpoints
                - commanded_joint_positions[finite_joint_limit_mask]
            ) / half_ranges
            centering_delta = (
                NULLSPACE_CENTERING_GAIN * centering_error * dt * cartesian_scale
            )
            nullspace_projector = (
                np.eye(len(ARM_JOINTS), dtype=np.float64)
                - jacobian_pseudoinverse @ task_jacobian
            )
            joint_delta += nullspace_projector @ centering_delta

        max_joint_step = MAX_JOINT_SPEED * dt
        joint_delta = np.clip(joint_delta, -max_joint_step, max_joint_step)

        (
            commanded_joint_positions,
            joint_limit_protection_active,
            near_joint_limit,
            closest_joint_index,
            minimum_soft_limit_distance,
        ) = protect_joint_command(joint_delta, measured_joint_positions)

        if near_joint_limit and not joint_limit_warning_active:
            print(f"[UR10 Teleop] Near joint limit: {ARM_JOINTS[closest_joint_index]}")
        joint_limit_warning_active = near_joint_limit

        send_commanded_joint_positions()

        diagnostic_elapsed += dt
        if diagnostic_elapsed >= DIAGNOSTIC_INTERVAL:
            max_joint_delta = float(np.max(np.abs(joint_delta)))
            max_tracking_error = float(
                np.max(np.abs(commanded_joint_positions - measured_joint_positions))
            )
            if closest_joint_index is None:
                closest_joint_name = "none"
                soft_limit_distance_degrees = float("inf")
            else:
                closest_joint_name = ARM_JOINTS[closest_joint_index]
                soft_limit_distance_degrees = np.rad2deg(minimum_soft_limit_distance)
            print(
                "[UR10 Teleop] "
                f"sigma_min={sigma_min:.5f}, "
                f"position_error_norm={position_error_norm:.5f} m, "
                f"orientation_error_norm={orientation_error_norm:.5f} rad, "
                f"max(abs(joint_delta))={max_joint_delta:.5f} rad, "
                f"min_soft_limit_distance={soft_limit_distance_degrees:.2f} deg, "
                f"closest_joint={closest_joint_name}, "
                f"max_tracking_error={np.rad2deg(max_tracking_error):.2f} deg, "
                f"joint_limit_active={joint_limit_protection_active}"
            )
            diagnostic_elapsed = 0.0

        input_failed = False
        control_failed = False

    except Exception as exc:
        emergency_stop = True
        try:
            sync_commanded_joints_to_current()
            send_commanded_joint_positions()
        except Exception:
            pass
        if not input_failed and not control_failed:
            print(f"[UR10 Teleop] Control failed; stopped: {exc}")
        if getattr(gamepad, "connected", True):
            control_failed = True
        else:
            input_failed = True


# =========================================================
# SUBSCRIBE
# =========================================================

timeline_subscription = (
    timeline
    .get_timeline_event_stream()
    .create_subscription_to_pop(
        on_timeline_event,
        name="UR10 DualSense timeline lifecycle",
    )
)
stage_subscription = (
    omni.usd
    .get_context()
    .get_stage_event_stream()
    .create_subscription_to_pop(
        on_stage_event,
        name="UR10 DualSense stage lifecycle",
    )
)

if timeline.is_playing():
    start_physics_subscription()

builtins._ur10_dualsense_timeline_subscription = timeline_subscription
builtins._ur10_dualsense_stage_subscription = stage_subscription
builtins._dualsense_teleop_cleanup = cleanup


print("")
print("====================================")
print("DualSense UR10 TCP Teleop ACTIVE")
print("====================================")
print("Default:")
print("  Left Stick X/Y : TCP X/Y")
print("  Right Stick Y  : TCP Z")
print("  Right Stick X  : Roll")
print("Hold R2:")
print("  Left Stick X/Y : Pitch/Yaw")
print("Cross ❌        : Emergency Stop")
print("Triangle △      : Enable")
print("")
print(f"Max linear  : {MAX_LINEAR_SPEED:.2f} m/s")
print(f"Max angular : {MAX_ANGULAR_SPEED:.2f} rad/s")
print(f"Orientation control: {ENABLE_ORIENTATION_CONTROL}")
print("Press Play to begin controlling the UR10.")
print("====================================")
