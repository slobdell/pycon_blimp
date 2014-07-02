import time
import datetime
import gevent
import os

from gevent import monkey

from blimp_controller.autopilot.blimp import BlimpAutopilot
from blimp_controller.autopilot.manual_control import ManualControl
from blimp_controller.camera.camera_manager import CameraManager
from blimp_controller.communication import Communicator
from blimp_controller.constants import Commands
from blimp_controller.constants import JPEG_FRAME, JPEG_HEADER
from blimp_controller.constants import XBoxButtons
from blimp_controller.constants import SensorData
from blimp_controller.roboard.blimp_pwm_controller import BlimpPWMController
# from blimp_controller.utils import blimp_log
from blimp_controller.sensors.sensor_manager import SensorManager

TIME_BETWEEN_KILL_COMMANDS = 1.0
SECONDS_WITHOUT_COMMS = 5.0


def get_functions_for_commands(manual_control_manager):
    command_to_function = {
        Commands.LEFT_THUMBSTICK_HORIZONTAL: manual_control_manager.set_left_thumbstick_horizontal,
        Commands.LEFT_THUMBSTICK_VERTICAL: manual_control_manager.set_left_thumbstick_vertical,
        Commands.RIGHT_THUMBSTICK_HORIZONTAL: manual_control_manager.set_right_thumbstick_horizontal,
        Commands.RIGHT_THUMBSTICK_VERTICAL: manual_control_manager.set_right_thumbstick_vertical
    }
    return command_to_function


class Controller(object):
    def __init__(self):
        self.autopilot = BlimpAutopilot()
        command_to_internal_functions = {
            Commands.BUTTON_PRESSED: self.handle_button_pressed,
            Commands.KEEP_ALIVE: self.keepalive,
            SensorData.LATITUDE: self.autopilot.add_waypoint_lat,
            SensorData.LONGITUDE: self.autopilot.add_waypoint_lon,
        }
        self.communicator = Communicator()
        self.pwm_controller = BlimpPWMController()
        self.sensor_manager = SensorManager()
        self.manual_control_manager = ManualControl()
        self.command_to_function = get_functions_for_commands(self.manual_control_manager)
        self.command_to_function.update(command_to_internal_functions)
        self.last_kill_command = None
        self.last_comms_time = datetime.datetime.now()
        self.using_autopilot = False

    def keepalive(self, junk_value):
        self.last_comms_time = datetime.datetime.now()
        self.pwm_controller.unkill()

    def keepalive_thread(self):
        while True:
            now = datetime.datetime.now()
            timedelta = now - self.last_comms_time
            if timedelta.seconds > SECONDS_WITHOUT_COMMS:
                print "KILLING MOTORS"
                self.pwm_controller.kill()
            time.sleep(1.0)

    def handle_button_pressed(self, value):
        if value == XBoxButtons.START:
            self.issue_kill_command()
        elif value == XBoxButtons.UP:
            self.autopilot.increase_target_altitude()
        elif value == XBoxButtons.DOWN:
            self.autopilot.decrease_target_altitude()
        elif value == XBoxButtons.SELECT:
            self.using_autopilot = not self.using_autopilot
        elif value == XBoxButtons.RIGHT:
            self.autopilot.increase_speed()
        elif value == XBoxButtons.LEFT:
            self.autopilot.decrease_speed()
        elif value == XBoxButtons.Y:
            self.autopilot.hover()
        elif value == XBoxButtons.A:
            self.sensor_manager.zeroize_gps_altitude()

    def issue_kill_command(self):
        now = datetime.datetime.now()
        if self.last_kill_command is None:
            self.last_kill_command = now
            return
        timedelta = now - self.last_kill_command
        seconds_elapsed = timedelta.seconds + timedelta.microseconds / 1000000.0
        if seconds_elapsed <= TIME_BETWEEN_KILL_COMMANDS:
            print "SHUTTING DOWN"
            self.pwm_controller.kill()
            os.popen("shutdown -h now")
        self.last_kill_command = now

    def sensor_read_thread(self):
        sleep_time = 0.1
        seconds_to_send_data = 2.0
        ticks_to_send_data = seconds_to_send_data / sleep_time
        ticks = 0
        while True:
            self.sensor_manager.tick()
            data = self.sensor_manager.get_data_packet()
            data.update({
                SensorData.SPEED: self.autopilot.speed,
                SensorData.TARGET_ALTITUDE: self.autopilot.target_altitude,
                SensorData.USING_AUTOPILOT: 1 if self.using_autopilot else 0,
            })
            self.autopilot.update_from_sensor_data(data)
            if ticks % ticks_to_send_data == 0:
                self.communicator.enqueue_commands(data)
            ticks += 1
            time.sleep(sleep_time)

    def pwm_control_thread(self):
        while True:
            self.pwm_controller.tick()
            time.sleep(0.05)

    def communicator_read(self):
        while True:
            while self.communicator.data_available():
                command, value = self.communicator.read_frame()
                if command in self.command_to_function:
                    func = self.command_to_function[command]
                    func(value)
            time.sleep(.01)

    def communicator_write(self):
        while True:
            self.communicator.send_command()
            time.sleep(.01)

    def autopilot_thread(self):
        sleep_time = 0.1
        while True:
            if not self.using_autopilot:
                time.sleep(sleep_time)
                continue
            self.autopilot.think()
            instructions = self.autopilot.get_instructions()
            self.pwm_controller.update_from_instructions(instructions)

            client_data = self.autopilot.get_data_for_client()
            self.communicator.enqueue_commands(client_data)
            time.sleep(sleep_time)

    def manual_control_thread(self):
        sleep_time = 0.1
        while True:
            if self.using_autopilot:
                time.sleep(sleep_time)
                continue
            self.manual_control_manager.think()
            instructions = self.manual_control_manager.get_instructions()
            self.pwm_controller.update_from_instructions(instructions)
            time.sleep(sleep_time)

    def live_view_thread(self):
        camera_manager = CameraManager()
        while True:
            jpeg_bytes = camera_manager.get_jpeg_frame()
            if jpeg_bytes is None:
                # this case occurs at camera init
                time.sleep(0.1)
                continue
            self.communicator.enqueue_command(JPEG_HEADER, len(jpeg_bytes))
            self.communicator.enqueue_command(JPEG_FRAME, jpeg_bytes)
            time.sleep(0.3)

    def main_loop(self):
        all_greenlets = []
        all_greenlets.append(gevent.spawn(self.communicator_read))
        all_greenlets.append(gevent.spawn(self.communicator_write))
        all_greenlets.append(gevent.spawn(self.pwm_control_thread))
        all_greenlets.append(gevent.spawn(self.sensor_read_thread))
        all_greenlets.append(gevent.spawn(self.autopilot_thread))
        all_greenlets.append(gevent.spawn(self.manual_control_thread))
        all_greenlets.append(gevent.spawn(self.keepalive_thread))
        # all_greenlets.append(gevent.spawn(self.live_view_thread))
        gevent.joinall(all_greenlets)

if __name__ == "__main__":
    monkey.patch_all()
    controller = Controller()
    controller.main_loop()
