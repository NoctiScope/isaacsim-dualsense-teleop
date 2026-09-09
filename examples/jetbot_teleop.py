import sys
import builtins
import types
from pathlib import Path

import numpy as np
import omni.usd
import omni.timeline
import omni.physx

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
if not _input_path.is_file():
    raise FileNotFoundError(
        f"Cannot find {_input_path}. "
        "Edit PROJECT_ROOT near the top of jetbot_teleop.py."
    )
_input_module = types.ModuleType("_jetbot_dualsense_input")
_input_module.__file__ = str(_input_path)
exec(compile(_input_path.read_bytes(), str(_input_path), "exec"), _input_module.__dict__)
DualSenseInput = _input_module.DualSenseInput
print(f"[JetBot Teleop] Fresh source: {_input_path}")


# ---------------------------------------------------------
# Isaac Sim 6.x wheeled robot API
# ---------------------------------------------------------

from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.experimental.wheeled_robots.controllers import (
    DifferentialController,
)


# =========================================================
# CONFIG
# =========================================================

JETBOT_PATH = "/World/jetbot"

LEFT_WHEEL_JOINT = "left_wheel_joint"
RIGHT_WHEEL_JOINT = "right_wheel_joint"

WHEEL_RADIUS = 0.03       # m
WHEEL_BASE = 0.1125       # m

MAX_LINEAR = 0.30         # m/s
MAX_ANGULAR = 1.00        # rad/s


# =========================================================
# CHECK STAGE
# =========================================================

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("Open the JetBot scene before running jetbot_teleop.py.")

jetbot_prim = stage.GetPrimAtPath(JETBOT_PATH)

if not jetbot_prim.IsValid():
    raise RuntimeError(
        f"JetBot not found at {JETBOT_PATH}. "
        "Check the prim path in the Stage."
    )


# =========================================================
# GAMEPAD AND ROBOT
# =========================================================

gamepad = DualSenseInput(deadzone=0.08)
builtins._jetbot_dualsense_gamepad = gamepad

jetbot = WheeledRobot(
    paths=JETBOT_PATH,
    wheel_dof_names=[
        LEFT_WHEEL_JOINT,
        RIGHT_WHEEL_JOINT,
    ],
)

controller = DifferentialController(
    wheel_radius=WHEEL_RADIUS,
    wheel_base=WHEEL_BASE,
    max_linear_speed=MAX_LINEAR,
    max_angular_speed=MAX_ANGULAR,
)


# =========================================================
# STATE
# =========================================================

emergency_stop = False
input_failed = False
timeline = omni.timeline.get_timeline_interface()
subscription = None
timeline_subscription = None
stage_subscription = None
shutting_down = False


# =========================================================
# UPDATE
# =========================================================

def update(event):
    global emergency_stop, input_failed

    if not timeline.is_playing():
        return

    # ----------------------------------------------
    # Read DualSense
    # ----------------------------------------------

    try:
        gamepad.update()
    except Exception as exc:
        emergency_stop = True
        try:
            jetbot.apply_wheel_actions(np.zeros(2, dtype=np.float32))
        except Exception:
            stop_physics_subscription()
        if not input_failed:
            print(f"[JetBot Teleop] Input failed; stopped: {exc}")
        input_failed = True
        return

    left_x = gamepad.axis("left_x")
    left_y = gamepad.axis("left_y")

    # ----------------------------------------------
    # Emergency stop
    # ----------------------------------------------

    if gamepad.button("cross"):
        if not emergency_stop:
            print("[JetBot Teleop] E-STOP")
        emergency_stop = True

    elif gamepad.pressed("triangle"):
        emergency_stop = False
        print("✅ JETBOT ENABLED")

    # ----------------------------------------------
    # Convert stick -> robot velocity
    # ----------------------------------------------

    if emergency_stop:
        linear = 0.0
        angular = 0.0

    else:
        # DualSense:
        # stick forward = -1
        #
        # Robot:
        # forward = positive
        linear = -left_y * MAX_LINEAR

        # DualSense:
        # stick right = +1
        #
        # Robot:
        # positive yaw = left turn
        angular = -left_x * MAX_ANGULAR

    command = np.array(
        [linear, angular],
        dtype=np.float32,
    )

    # ----------------------------------------------
    # Differential drive
    # ----------------------------------------------

    wheel_velocities = controller.forward(command)

    try:
        jetbot.apply_wheel_actions(wheel_velocities)
    except Exception as exc:
        # A stage can close between the timeline and physics callbacks. Stop
        # this callback immediately instead of reporting the same invalid prim
        # assertion on every physics frame.
        print(f"[JetBot Teleop] Robot became invalid; physics subscription stopped: {exc}")
        stop_physics_subscription()


# =========================================================
# LIFECYCLE
# =========================================================


def stop_physics_subscription():
    global subscription
    if subscription is not None:
        subscription.unsubscribe()
        subscription = None
    builtins._jetbot_dualsense_subscription = None


def start_physics_subscription():
    global subscription
    if subscription is None and not shutting_down:
        subscription = (
            omni.physx
            .get_physx_interface()
            .subscribe_physics_step_events(update)
        )
        builtins._jetbot_dualsense_subscription = subscription


def cleanup(reason="cleanup"):
    global timeline_subscription, stage_subscription, shutting_down
    if shutting_down:
        return
    shutting_down = True
    stop_physics_subscription()
    if timeline_subscription is not None:
        timeline_subscription.unsubscribe()
        timeline_subscription = None
    if stage_subscription is not None:
        stage_subscription.unsubscribe()
        stage_subscription = None
    gamepad.close()
    builtins._jetbot_dualsense_timeline_subscription = None
    builtins._jetbot_dualsense_stage_subscription = None
    builtins._jetbot_dualsense_gamepad = None
    if getattr(builtins, "_dualsense_teleop_cleanup", None) is cleanup:
        builtins._dualsense_teleop_cleanup = None
    print(f"[JetBot Teleop] Closed: {reason}")


def on_timeline_event(event):
    event_type = omni.timeline.TimelineEventType(event.type)
    if event_type == omni.timeline.TimelineEventType.STOP:
        stop_physics_subscription()
        print("[JetBot Teleop] STOP - physics subscription paused")
    elif event_type == omni.timeline.TimelineEventType.PLAY:
        start_physics_subscription()


def on_stage_event(event):
    if event.type == int(omni.usd.StageEventType.CLOSING):
        cleanup("stage closing")


# =========================================================
# SUBSCRIBE
# =========================================================

timeline_subscription = (
    timeline
    .get_timeline_event_stream()
    .create_subscription_to_pop(
        on_timeline_event,
        name="JetBot DualSense timeline lifecycle",
    )
)
stage_subscription = (
    omni.usd
    .get_context()
    .get_stage_event_stream()
    .create_subscription_to_pop(
        on_stage_event,
        name="JetBot DualSense stage lifecycle",
    )
)

if timeline.is_playing():
    start_physics_subscription()

builtins._jetbot_dualsense_timeline_subscription = timeline_subscription
builtins._jetbot_dualsense_stage_subscription = stage_subscription
builtins._dualsense_teleop_cleanup = cleanup


print("")
print("====================================")
print("🎮 DualSense JetBot Teleop ACTIVE")
print("====================================")
print("Left Stick Y : Forward / Backward")
print("Left Stick X : Turn")
print("Cross ❌      : Emergency Stop")
print("Triangle △   : Enable")
print("")
print(f"Max linear  : {MAX_LINEAR:.2f} m/s")
print(f"Max angular : {MAX_ANGULAR:.2f} rad/s")
print("====================================")
