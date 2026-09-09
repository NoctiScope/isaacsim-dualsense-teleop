# input/dualsense_input.py
from pathlib import Path
import json
import pygame


class DualSenseInput:
    """
    Generic DualSense input reader based on pygame.

    Responsibilities:
    - Read raw joystick axes/buttons
    - Convert physical indices to semantic names
    - Apply stick deadzone
    - Detect button pressed/released edges

    Does NOT contain any robot-specific logic.
    """

    def __init__(
        self,
        mapping_path="config/dualsense_mapping.json",
        deadzone=0.08,
    ):
        if not 0.0 <= deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1).")
        self.deadzone = deadzone

        # --------------------------------------------------
        # Load mapping
        # --------------------------------------------------

        mapping_path = Path(mapping_path)
        if not mapping_path.is_absolute():
            mapping_path = Path(__file__).resolve().parents[1] / mapping_path
        self.mapping_path = mapping_path.resolve()
        with self.mapping_path.open("r", encoding="utf-8") as f:
            mapping = json.load(f)
        print(f"[DualSense] Mapping: {self.mapping_path}")

        self.axis_map = mapping["axes"]
        self.button_map = mapping["buttons"]

        # --------------------------------------------------
        # Initialize pygame joystick
        # --------------------------------------------------

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No gamepad detected.")

        # For now use the first controller
        self.pad = pygame.joystick.Joystick(0)
        self.pad.init()
        self.instance_id = self.pad.get_instance_id()
        self.connected = True

        print(f"[DualSense] Connected: {self.pad.get_name()}")
        print(
            f"[DualSense] "
            f"Axes={self.pad.get_numaxes()}, "
            f"Buttons={self.pad.get_numbuttons()}"
        )

        # --------------------------------------------------
        # State
        # --------------------------------------------------

        self.axes = [0.0] * self.pad.get_numaxes()

        self.buttons = [0] * self.pad.get_numbuttons()
        self.prev_buttons = [0] * self.pad.get_numbuttons()

        # Read initial state immediately
        self.update()

    # ======================================================
    # Update
    # ======================================================

    def update(self):
        """
        Read the latest controller state.

        Call once every simulation/update frame.
        """

        for event in pygame.event.get([pygame.JOYDEVICEREMOVED]):
            if event.instance_id == self.instance_id:
                self.connected = False
        if not self.connected:
            self.axes = [0.0] * len(self.axes)
            self.buttons = [0] * len(self.buttons)
            raise RuntimeError(
                "Gamepad disconnected. Reconnect the gamepad and rerun the active teleop script."
            )

        # Save previous button state for edge detection
        self.prev_buttons = self.buttons.copy()

        # Axes
        for i in range(self.pad.get_numaxes()):
            self.axes[i] = self.pad.get_axis(i)

        # Buttons
        for i in range(self.pad.get_numbuttons()):
            self.buttons[i] = self.pad.get_button(i)

    # ======================================================
    # Axis
    # ======================================================

    def axis(self, name, apply_deadzone=True):
        """
        Get an axis by semantic name.

        Example:
            gamepad.axis("left_x")
            gamepad.axis("left_y")
        """

        index = self.axis_map[name]
        value = self.axes[index]

        # Triggers use -1 ~ +1, so don't apply stick deadzone to them
        if name in ("l2", "r2"):
            return value

        if apply_deadzone:
            value = self._apply_deadzone(value)

        return value

    def trigger(self, name):
        """
        Get L2 / R2 normalized to 0 ~ 1.

        pygame:
            released = -1
            pressed  = +1

        converted:
            released = 0
            pressed  = 1
        """

        if name not in ("l2", "r2"):
            raise ValueError("trigger() only supports 'l2' or 'r2'.")

        value = self.axis(name, apply_deadzone=False)

        return (value + 1.0) / 2.0

    # ======================================================
    # Buttons
    # ======================================================

    def button(self, name):
        """
        True while button is held.
        """

        index = self.button_map[name]
        return bool(self.buttons[index])

    def pressed(self, name):
        """
        True only on the frame the button is pressed.
        """

        index = self.button_map[name]

        return (
            self.buttons[index] == 1
            and self.prev_buttons[index] == 0
        )

    def released(self, name):
        """
        True only on the frame the button is released.
        """

        index = self.button_map[name]

        return (
            self.buttons[index] == 0
            and self.prev_buttons[index] == 1
        )

    # ======================================================
    # Helpers
    # ======================================================

    def _apply_deadzone(self, value):
        """
        Stick deadzone with rescaling.

        Example deadzone = 0.08:
            -0.05 -> 0
             0.05 -> 0
             1.00 -> 1
        """

        if abs(value) <= self.deadzone:
            return 0.0

        sign = 1.0 if value > 0 else -1.0

        magnitude = (
            abs(value) - self.deadzone
        ) / (
            1.0 - self.deadzone
        )

        return sign * magnitude

    def name(self):
        return self.pad.get_name()

    def close(self):
        self.connected = False
        self.pad.quit()
