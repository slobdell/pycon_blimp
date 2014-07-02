from boost_roboard import RoboardManager
from blimp_controller.constants import Instructions
from blimp_controller.roboard.base_pwm_controller import BasePWMController
import time


class PWMMotors(object):
    RIGHT_MOTOR = 0
    LEFT_MOTOR = 1
    ELEVATOR = 2
    TAIL_RUDDER = 3
    GEAR = 4  # lights
    THRUST_VECTOR = 5

    THRUST_MOTORS = (RIGHT_MOTOR, LEFT_MOTOR,)


class BlimpCalibrations(object):
    MIN_PWM_VALUE = 800

    MIN_THRUST_VALUE = 800
    MAX_THRUST_VALUE = 1700
    THRUST_STEP = 2  # percent

    RUDDER_LEFT_MAX_DUTY = 1990
    RUDDER_NEUTRAL_DUTY = 1600
    RUDDER_RIGHT_MAX_DUTY = RUDDER_NEUTRAL_DUTY - (RUDDER_LEFT_MAX_DUTY - RUDDER_NEUTRAL_DUTY)
    MIN_RUDDER_ANGLE = -180  # right
    MAX_RUDDER_ANGLE = 180  # left

    MIN_THRUST_VECTOR = 800  # horizontal
    MAX_THRUST_VECTOR = 1130  # vertical
    THRUST_VECTOR_ANGLE_STEP = 3
    MIN_THRUST_VECTOR_ANGLE = -45
    MAX_THRUST_VECTOR_ANGLE = 45
    THRUST_VECTOR_NEUTRAL_ANGLE = (MAX_THRUST_VECTOR_ANGLE + MIN_THRUST_VECTOR_ANGLE) / 2

    ELEVATOR_MIN_DUTY = 800  # down
    ELEVATOR_MAX_DUTY = 1900  # up
    MIN_ELEVATOR_ANGLE = -180
    MAX_ELEVATOR_ANGLE = 180
    ELEVATOR_ANGLE_STEP = 15


class BlimpPWMController(BasePWMController):

    def __init__(self):
        self.killed = False

        self.roboard_manager = RoboardManager()
        self.roboard_manager.initialize()

        self.rotor_states = {}
        self.servo_states = {}
        self._init_internal_states()
        self._arm_rotors()

    def _init_internal_states(self):
        self.current_thrust_percent = 0.0
        self.current_thrust_vector_angle = BlimpCalibrations.THRUST_VECTOR_NEUTRAL_ANGLE
        self.current_rudder_angle = 0.0
        self.current_elevator_angle = 0.0

    def _arm_rotors(self):
        for channel in PWMMotors.THRUST_MOTORS:
            self.rotor_states[channel] = BlimpCalibrations.MIN_THRUST_VALUE
        self.rotor_states[PWMMotors.TAIL_RUDDER] = BlimpCalibrations.RUDDER_NEUTRAL_DUTY
        self._send_commands_to_rotors()
        time.sleep(3.5)

    def kill(self):
        for channel in PWMMotors.THRUST_MOTORS:
            self.rotor_states[channel] = BlimpCalibrations.MIN_THRUST_VALUE
        self.rotor_states[PWMMotors.ELEVATOR] = BlimpCalibrations.ELEVATOR_MIN_DUTY
        self.rotor_states[PWMMotors.TAIL_RUDDER] = BlimpCalibrations.RUDDER_NEUTRAL_DUTY
        self._send_commands_to_rotors()
        self.killed = True

    def unkill(self):
        self.killed = False

    def _tick_pitch(self):
        self._tick_thrust_vector()
        self._tick_elevator()

    def _tick_thrust_vector(self):
        '''
        Thrust vector should be in [-180...180]
        '''
        pwm_range = BlimpCalibrations.MAX_THRUST_VECTOR - BlimpCalibrations.MIN_THRUST_VECTOR
        angle_range = BlimpCalibrations.MAX_THRUST_VECTOR_ANGLE - BlimpCalibrations.MIN_THRUST_VECTOR_ANGLE

        # for logical purposes, negative angles correspond to moving the thrust
        # down, but for calculations, it needs to be inverted to correspond to
        # servo
        point_in_linear_range = self.current_thrust_vector_angle - BlimpCalibrations.MIN_THRUST_VECTOR_ANGLE
        percent_of_max = float(point_in_linear_range) / angle_range

        duty_value = percent_of_max * pwm_range + BlimpCalibrations.MIN_THRUST_VECTOR
        channel = PWMMotors.THRUST_VECTOR
        self.servo_states[channel] = duty_value
        self._send_command_to_servo(channel, duty_value)

    def _tick_elevator(self):
        pwm_range = BlimpCalibrations.ELEVATOR_MAX_DUTY - BlimpCalibrations.ELEVATOR_MIN_DUTY

        angle_range = BlimpCalibrations.MAX_ELEVATOR_ANGLE - BlimpCalibrations.MIN_ELEVATOR_ANGLE
        angle = self.current_elevator_angle
        point_in_linear_range = angle - BlimpCalibrations.MIN_ELEVATOR_ANGLE
        percent_of_max = float(point_in_linear_range) / angle_range

        duty_value = percent_of_max * pwm_range + BlimpCalibrations.ELEVATOR_MIN_DUTY
        channel = PWMMotors.ELEVATOR
        self.rotor_states[channel] = duty_value

    def _tick_thrust(self):
        pwm_range = BlimpCalibrations.MAX_THRUST_VALUE - BlimpCalibrations.MIN_THRUST_VALUE
        point_in_linear_range = self.current_thrust_percent * pwm_range
        duty_value = point_in_linear_range + BlimpCalibrations.MIN_THRUST_VALUE
        for channel in PWMMotors.THRUST_MOTORS:
            self.rotor_states[channel] = duty_value

    def __init__(self):
        self.INSTRUCTION_TO_FUNCTION = {
            Instructions.PITCH_UP: self._handle_pitch_up,
            Instructions.PITCH_DOWN: self._handle_pitch_down,
            Instructions.YAW_LEFT: self._handle_yaw_left,
            Instructions.YAW_RIGHT: self._handle_yaw_right,
            Instructions.HOVER: self._handle_hover,
            Instructions.SPEED: self._handle_speed,
        }

    def _handle_yaw_left(self, intensity):
        self.current_rudder_angle = intensity * BlimpCalibrations.MAX_RUDDER_ANGLE

    def _handle_yaw_right(self, intensity):
        self.current_rudder_angle = intensity * BlimpCalibrations.MIN_RUDDER_ANGLE

    def update_from_instructions(self, instructions):
        for instruction, intensity in instructions:
            func = self.INSTRUCTION_TO_FUNCTION[instruction]
            func(intensity)

    def _tick_yaw(self):
        pwm_range = BlimpCalibrations.RUDDER_LEFT_MAX_DUTY - BlimpCalibrations.RUDDER_RIGHT_MAX_DUTY
        angle_range = BlimpCalibrations.MAX_RUDDER_ANGLE - BlimpCalibrations.MIN_RUDDER_ANGLE
        point_in_linear_range = self.current_rudder_angle - BlimpCalibrations.MIN_RUDDER_ANGLE
        percent_of_max = float(point_in_linear_range) / angle_range
        duty_value = percent_of_max * pwm_range + BlimpCalibrations.RUDDER_RIGHT_MAX_DUTY
        channel = PWMMotors.TAIL_RUDDER
        self.rotor_states[channel] = duty_value

    def tick(self):
        self._tick_pitch()
        self._tick_thrust()
        self._tick_yaw()
        self._send_commands_to_rotors()

    def _send_commands_to_rotors(self):
        for channel, value in self.rotor_states.items():
            value = int(value)
            self.roboard_manager.send_continuous_pwm_to_channel(channel, value)


