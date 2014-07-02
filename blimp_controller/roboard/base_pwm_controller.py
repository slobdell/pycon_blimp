from abc import abstractmethod

DEBUG = False


class BasePWMController(object):

    @abstractmethod
    def update_from_instructions(instructions):
        '''
        A list of tuples of size 2.  Each tuple contains
        an instruction and an intensity.  Instructions are
        defined in constants.instructions
        '''
        pass

    @abstractmethod
    def kill(self):
        '''
        Shuts down all motors appropriately.  Called before
        shutting down the entire robot
        '''
        pass

    @abstractmethod
    def tick(self):
        '''
        Called every cycle to actually update all of the
        motor values
        '''
        pass

    def _send_commands_to_rotors(self):
        if self.killed:
            return
        global DEBUG
        try:
            for channel, value in self.rotor_states.items():
                value = int(value)

                if DEBUG:
                    print channel, value
                self.roboard_manager.send_continuous_pwm_to_channel(channel, value)
            if DEBUG:
                print "\n"
        except OverflowError:
            DEBUG = True
            print "WARNING!  Got overflow error sending commands to rotors"
            for channel, value in self.rotor_states.items():
                print channel, value

    def _send_command_to_servo(self, channel, pwm_value, count=1):
        self.roboard_manager.send_pwm_to_channel(channel, int(pwm_value), count)
