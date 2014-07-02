from boost_roboard import RoboardManager
import time
'''
all_channels = [0, 1, 2, 3]
for channel in all_channels:
    manager.send_continuous_pwm_to_channel(channel, 1000)
time.sleep(1)
for channel in all_channels:
    manager.send_continuous_pwm_to_channel(channel, 1180)
time.sleep(30)
'''
DEBUG = False


class Rotors(object):
    FRONT_RIGHT = 0
    BACK_RIGHT = 1
    BACK_LEFT = 2
    FRONT_LEFT = 3

    counter_clockwise_rotors = (FRONT_RIGHT, BACK_LEFT, )
    clockwise_rotors = (BACK_RIGHT, FRONT_LEFT, )
    front_rotors = (FRONT_LEFT, FRONT_RIGHT, )
    back_rotors = (BACK_LEFT, BACK_RIGHT, )
    right_rotors = (FRONT_RIGHT, BACK_RIGHT, )
    left_rotors = (FRONT_LEFT, BACK_LEFT, )
    all_rotors = (FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, )


class RotorController(object):
    '''
    value for all of these functions is [-1.0...1.0]
    '''
    MAX_PWM_DUTY = 1900.0  # change back to 2000 later
    MIN_PWM_DUTY = 1000.0
    MAX_DIFFERENTIAL = 150.0
    CEILING_PWM_DUTY = MAX_PWM_DUTY - MAX_DIFFERENTIAL
    MAX_COLLECTIVE_STEP_SIZE = 50.0
    COLLECTIVE_SENSITIVITY = 0.01

    def __init__(self):
        self.roboard_manager = RoboardManager()
        self.roboard_manager.initialize()
        self._arm_rotors()
        self.rotor_states = {}
        self.killed = False
        for channel in Rotors.all_rotors:
            self.rotor_states[channel] = self.MIN_PWM_DUTY
        self.previous_collective = 0.0
        self.current_collective = 0.0
        self.current_yaw = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_jerk = 0.0

    def _arm_rotors(self):
        for channel in Rotors.all_rotors:
            self.roboard_manager.send_continuous_pwm_to_channel(channel, 1000)
        time.sleep(1)

    def kill(self):
        for channel in Rotors.all_rotors:
            self.rotor_states[channel] = self.MIN_PWM_DUTY
        self._send_commands_to_rotors()
        self.killed = True

    def _send_commands_to_rotors(self):
        if self.killed:
            return
        try:
            for channel, value in self.rotor_states.items():
                value = int(value)
                # print channel, value
                self.roboard_manager.send_continuous_pwm_to_channel(channel, value)
            # print "\n"
        except OverflowError:
            global DEBUG
            DEBUG = True
            print "WARNING!  Got overflow error sending commands to rotors"
            for channel, value in self.rotor_states.items():
                print channel, value
            print "current collective: %s" % self.current_collective
            print "Resetting to safe values..."
            for channel in Rotors.all_rotors:
                self.rotor_states[channel] = self.MIN_PWM_DUTY

    def _handle_collective(self):
        self.current_collective += self.current_jerk
        if self.current_collective < 0.0:
            self.current_collective = 0.0
        elif self.current_collective > 1.0:
            self.current_collective = 1.00

        pwm_range = self.CEILING_PWM_DUTY - self.MIN_PWM_DUTY
        base_pwm_value = self.MIN_PWM_DUTY + self.current_collective * pwm_range
        for channel in Rotors.all_rotors:
            self.rotor_states[channel] = base_pwm_value

    def _handle_yaw(self):
        yaw_pwm_magnitude = self.current_yaw * self.MAX_DIFFERENTIAL
        if DEBUG:
            print "yaw: %s" % yaw_pwm_magnitude
        for channel in Rotors.clockwise_rotors:
            self.rotor_states[channel] += yaw_pwm_magnitude
        for channel in Rotors.counter_clockwise_rotors:
            self.rotor_states[channel] -= yaw_pwm_magnitude

    def _handle_pitch(self):
        pitch_pwm_magnitude = self.current_pitch * self.MAX_DIFFERENTIAL
        if DEBUG:
            print "pitch: %s" % pitch_pwm_magnitude
        if pitch_pwm_magnitude > 0:
            # pitch backward
            for channel in Rotors.front_rotors:
                self.rotor_states[channel] += pitch_pwm_magnitude
        elif pitch_pwm_magnitude < 0:
            for channel in Rotors.back_rotors:
                self.rotor_states[channel] -= pitch_pwm_magnitude

    def _handle_roll(self):
        roll_pwm_magnitude = self.current_roll * self.MAX_DIFFERENTIAL
        if DEBUG:
            print "Roll: %s" % roll_pwm_magnitude
        if roll_pwm_magnitude > 0:
            # roll right
            for channel in Rotors.left_rotors:
                self.rotor_states[channel] += roll_pwm_magnitude
        elif roll_pwm_magnitude < 0:
            for channel in Rotors.right_rotors:
                self.rotor_states[channel] -= roll_pwm_magnitude

    def tick(self):
        self._handle_collective()
        self._handle_yaw()
        self._handle_pitch()
        self._handle_roll()
        self._send_commands_to_rotors()
        '''
        counter_clockwise_rotors = (FRONT_RIGHT, BACK_LEFT, )
        clockwise_rotors = (BACK_RIGHT, FRONT_LEFT, )
        front_rotors = (FRONT_LEFT, FRONT_RIGHT, )
        back_rotors = (BACK_LEFT, BACK_RIGHT, )
        right_rotors = (FRONT_RIGHT, BACK_RIGHT, )
        left_rotors = (FRONT_LEFT, BACK_LEFT, )
        '''
    def pitch(self, value):
        '''
        -1 means pitch forward, 1 means pitch backward
        '''
        self.current_pitch = value

    def roll(self, value):
        '''
        1 means roll right, -1 means roll left
        '''
        self.current_roll = value

    def yaw(self, value):
        '''
        1 means yaw right, -1 means yaw left
        '''
        self.current_yaw = value

    def collective(self, value):
        '''
        This exhibits different behavior than other commands.
        Whereas other thumbstick commands will immediately
        correspond to a value, this will increment or decrement
        the jerk which will be added to the overall collective
        force every iteration
        '''
        # if previous_collective and value are both pos
        # or both neg...
        if value != 0 and self.previous_collective / value > 0:
            if abs(value) < abs(self.previous_collective):
                self.previous_value = value
                return
        self.previous_collective = value

        # -1 is forward on the xbox thumbstick
        value *= -1 * self.COLLECTIVE_SENSITIVITY
        self.current_jerk = value
