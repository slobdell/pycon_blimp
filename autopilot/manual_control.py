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

    def _think_yaw(self):
        value = self.right_thumbstick_horizontal
        if value < 0:
            # yaw left
            self.instruction_states[Instructions.YAW_LEFT] = abs(value)
            self.instruction_states[Instructions.YAW_RIGHT] = 0.0
        elif value > 0:
            # yaw right
            self.instruction_states[Instructions.YAW_LEFT] = 0.0
            self.instruction_states[Instructions.YAW_RIGHT] = abs(value)
        else:
            self.instruction_states[Instructions.YAW_LEFT] = 0.0
            self.instruction_states[Instructions.YAW_RIGHT] = 0.0

    def think(self):
        # TODO: safety control logic would go here
        self._think_speed()
        self._think_pitch()
        self._think_yaw()

    def get_instructions(self):
        instructions = []
        sent_yaw = False  # must send these commands every time
        sent_pitch = False  # must send these commands every time
        sent_speed = False
        for instruction, value in self.instruction_states.items():
            if value != 0:
                if instruction in (Instructions.YAW_LEFT, Instructions.YAW_RIGHT,):
                    sent_yaw = True
                elif instruction in (Instructions.PITCH_UP, Instructions.PITCH_DOWN,):
                    sent_pitch = True
                elif instruction == Instructions.SPEED:
                    sent_speed = True
                instructions.append((instruction, value,))
        if not sent_yaw:
            instructions.append((Instructions.YAW_LEFT, 0, ))
        if not sent_pitch:
            instructions.append((Instructions.PITCH_DOWN, 0, ))
        if not sent_speed:
            instructions.append((Instructions.SPEED, 0, ))
        return instructions

    def set_left_thumbstick_vertical(self, value):
        self.left_thumbstick_vertical = value

    def set_left_thumbstick_horizontal(self, value):
        self.left_thumbstick_horizontal = value

    def set_right_thumbstick_vertical(self, value):
        self.right_thumbstick_vertical = value

    def set_right_thumbstick_horizontal(self, value):
        self.right_thumbstick_horizontal = value
