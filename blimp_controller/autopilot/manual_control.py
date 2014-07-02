from blimp_controller.autopilot.base import BaseAutopilot
from blimp_controller.constants import Instructions


class ManualControl(BaseAutopilot):
    def __init__(self):
        self.current_instructions = []
        self.left_thumbstick_vertical = 0.0
        self.left_thumbstick_horizontal = 0.0
        self.right_thumbstick_vertical = 0.0
        self.right_thumbstick_horizontal = 0.0
        self.instruction_states = {
            Instructions.YAW_LEFT: 0.0,
            Instructions.YAW_RIGHT: 0.0,
            Instructions.PITCH_UP: 0.0,
            Instructions.PITCH_DOWN: 0.0,
            Instructions.SPEED: 0.0,
        }
        super(ManualControl, self).__init__()

    def _think_speed(self):
        value = self.left_thumbstick_vertical
        if value < 0:
            self.instruction_states[Instructions.SPEED] = abs(value)
            # self.instruction_states[Instructions.SPEED_UP] = abs(value)
            # self.instruction_states[Instructions.SPEED_DOWN] = 0.0
        elif value > 0:
            pass
        else:
            self.instruction_states[Instructions.SPEED] = 0.0

    def _think_pitch(self):
        value = self.right_thumbstick_vertical
        if value < 0:
            # pitch down
            self.instruction_states[Instructions.PITCH_DOWN] = abs(value)
            self.instruction_states[Instructions.PITCH_UP] = 0.0
        elif value > 0:
            # pitch up
            self.instruction_states[Instructions.PITCH_DOWN] = 0.0
            self.instruction_states[Instructions.PITCH_UP] = abs(value)
        else:
            self.instruction_states[Instructions.PITCH_DOWN] = 0.0
            self.instruction_states[Instructions.PITCH_UP] = 0.0

    def _avoid_joystick_sticking(self):
        sticking_value = 0.2  # the xbox won't necessarily change value to 0 here
        might_as_well_be_zero = 0.05
        if self.right_thumbstick_vertical <= sticking_value:
            self.right_thumbstick_vertical *= 0.9
        if self.right_thumbstick_horizontal <= sticking_value:
            self.right_thumbstick_horizontal *= 0.9
        if self.left_thumbstick_vertical <= sticking_value:
            self.left_thumbstick_vertical *= 0.9
        if self.left_thumbstick_horizontal <= sticking_value:
            self.left_thumbstick_horizontal *= 0.9

        if self.right_thumbstick_vertical <= might_as_well_be_zero:
            self.right_thumbstick_vertical = 0
        if self.right_thumbstick_horizontal <= might_as_well_be_zero:
            self.right_thumbstick_horizontal = 0
        if self.left_thumbstick_vertical <= might_as_well_be_zero:
            self.left_thumbstick_vertical = 0
        if self.left_thumbstick_horizontal <= might_as_well_be_zero:
            self.left_thumbstick_horizontal = 0

    def set_right_thumbstick_horizontal(self, value):
        self.right_thumbstick_horizontal = value

    def _think_yaw(self):
        value = self.right_thumbstick_horizontal
        if value < 0:
            self.instruction_states[Instructions.YAW_LEFT] = abs(value)
            self.instruction_states[Instructions.YAW_RIGHT] = 0.0
        elif value > 0:
            self.instruction_states[Instructions.YAW_LEFT] = 0.0
            self.instruction_states[Instructions.YAW_RIGHT] = abs(value)
        else:
            self.instruction_states[Instructions.YAW_LEFT] = 0.0
            self.instruction_states[Instructions.YAW_RIGHT] = 0.0

    def think(self):
        self._think_speed()
        self._think_pitch()
        self._think_yaw()

    def get_instructions(self):
        instructions = []
        for instruction, value in self.instruction_states.items():
            instructions.append((instruction, value,))
        return instructions

    def set_left_thumbstick_vertical(self, value):
        self.left_thumbstick_vertical = value

    def set_left_thumbstick_horizontal(self, value):
        self.left_thumbstick_horizontal = value

    def set_right_thumbstick_vertical(self, value):
        self.right_thumbstick_vertical = value

