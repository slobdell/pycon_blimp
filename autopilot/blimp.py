import datetime
import math

from blimp_controller.autopilot.base import BaseAutopilot
from blimp_controller.constants import Instructions
from blimp_controller.constants import Commands
from blimp_controller.utils import radians_to_degrees
from blimp_controller.utils import azimuth_from_gps_coords
from blimp_controller.utils import get_distance_meters
from blimp_controller.utils import get_delta_angle

TARGET_ALTITUDE_STEP = 5  # value to increase or decrease target altitude from client
SPEED_PERCENT_STEP = 5  # value to increase or decrease thrust vector magnitude from client
ACCEPTABLE_METERS_FROM_WAYPOINT = 10

MAX_ANGULAR_VELOCITY = 60.0  # 50 degrees/second, total shot in the dark right now

# ensure that these values remain floats
MAX_PITCH = 30.0
MIN_PITCH = -30.0
IDEAL_PITCH_OFFSET = 4.0  # blimp is slightly angled up in ideal scenario

# between target altitude and current altitude, we need to create an arbitrary
# distance from which we deem that the altitude should be reached.  We set this
# up for some basic trigonometry
DISTANCE_FT_TO_REACH_ALTITUDE = 50


class BlimpAutopilot(BaseAutopilot):
    def __init__(self):
        self.target_azimuth = None
        self.target_pitch = None
        self.target_thrust_vector = None
        self.target_lat = None
        self.target_lon = None
        self.target_angular_velocity = None
        self.hover = False
        super(BlimpAutopilot, self).__init__()
        self.INSTRUCTION_FUNCTIONS = (
            self._get_yaw_instruction,
            self._get_pitch_instruction,
            self._get_speed_instruction,
            self._get_hover_instruction,
        )
        self.client_commands = {}
        self.pitch_velocity = 0.0  # up is positive, down is negative
        self.yaw_velocity = 0.0  # right is positive, left is negative

        # self.previous_yaw_velocity = 0.0  # needed for acceleration
        self.previous_azimuth = None
        self.previous_pitch = None

        # self.yaw_acceleration = 0.0  # right is positive, left is negative
        self.last_time_read_yaw = None
        self.last_time_read_pitch = None

        # Base includes:
        # self.current_azimuth
        # self.current_pitch
        # self.current_roll
        # self.current_lat
        # self.current_lon
        # self.target_altitude
        # self.current_altitude
        # self.speed
        # all of the above are updated in the Base class

    def _compute_target_azimuth(self):
        from_coord = (self.current_lat, self.current_lon)
        to_coord = (self.target_lat, self.target_lon)
        self.target_azimuth = azimuth_from_gps_coords(from_coord, to_coord)

    def get_data_for_client(self):
        client_commands = self.client_commands
        self.client_commands = {}
        return client_commands

    def _update_target_coordinate(self):
        '''
        sets self.target_lat, self.target_lon to either
        first value in self.waypoints or None, None
        '''
        if self.target_lat is not None or self.target_lon is not None:
            return

        if len(self.waypoints) == 0:
            self.target_lat = None
            self.target_lon = None
            return
        self.target_lat, self.target_lon = self.waypoints.pop(0)

    def _update_current_waypoint(self):
        '''
        First function called before anything; determines
        if we need to update waypoint based on distance
        '''
        self._update_target_coordinate()
        if self.current_lat is None or self.current_lon is None or self.target_lat is None or self.target_lon is None:
            # can't do anything if None values are in place for any lat, lons
            return
        from_coord = (self.current_lat, self.current_lon)
        to_coord = (self.target_lat, self.target_lon)
        distance_meters = get_distance_meters(from_coord, to_coord)
        if distance_meters <= ACCEPTABLE_METERS_FROM_WAYPOINT:
            self.client_commands[Commands.POP_WAYPOINT] = 1
            self.target_lat = None
            self.target_lon = None
            self._update_target_coordinate()

    def _update_yaw_velocity(self):
        '''
        Still working this, but I don't think we need to account for
        acceleration because we're re-computing so often
        '''
        current_time = datetime.datetime.now()
        if self.last_time_read_yaw is None or self.previous_azimuth is None:
            self.last_time_read_yaw = current_time
            self.previous_azimuth = self.current_azimuth
            return
        timedelta = current_time - self.last_time_read_yaw
        seconds_elapsed = timedelta.seconds + timedelta.microseconds / 1000000.0

        delta_azimuth = get_delta_angle(self.previous_azimuth, self.current_azimuth)

        # self.previous_yaw_velocity = self.yaw_velocity
        self.yaw_velocity = float(delta_azimuth) / seconds_elapsed
        # self.yaw_acceleration = (self.yaw_velocity - self.previous_yaw_velocity) / seconds_elapsed
        self.last_time_read_yaw = current_time

    def _update_pitch_velocity(self):
        '''
        Unsure if we have any need for this, leaving in place for now
        '''
        current_time = datetime.datetime.now()
        if self.last_time_read_pitch is None or self.previous_pitch is None:
            self.last_time_read_pitch = current_time
            self.previous_pitch = self.current_pitch
            return
        timedelta = current_time - self.last_time_read_yaw
        seconds_elapsed = timedelta.seconds + timedelta.microseconds / 1000000.0

        delta_pitch = self.current_pitch - self.previous_pitch
        self.pitch_velocity = float(delta_pitch) / seconds_elapsed
        self.last_time_read_pitch = current_time

    def _think_yaw(self):
        self._update_current_waypoint()
        self._update_yaw_velocity()
        if self.target_lat is None or self.target_lon is None:
            self.hover = True
        else:
            self.hover = False
            self._compute_target_azimuth()

    def _think_pitch(self):
        '''
        Adjust pitch based on target elevation, current pitch,
        max pitch, min pitch
        '''
        if self.current_altitude is None or self.hover:
            delta_altitude = 0
        else:
            delta_altitude = self.target_altitude - self.current_altitude
        delta_ground_distance = DISTANCE_FT_TO_REACH_ALTITUDE
        target_pitch_radians = math.atan2(delta_altitude, delta_ground_distance)
        self.target_pitch = radians_to_degrees(target_pitch_radians)
        self.target_pitch += IDEAL_PITCH_OFFSET
        if self.target_pitch > MAX_PITCH:
            self.target_pitch = MAX_PITCH
        elif self.target_pitch < MIN_PITCH:
            self.target_pitch = MIN_PITCH

    def think(self):
        '''
        self.update_from_sensor_data from
        the base class will have been called
        '''
        self._think_yaw()
        self._think_pitch()

    def hover(self):
        self.hover = not self.hover

    def increase_target_altitude(self):
        self.target_altitude += TARGET_ALTITUDE_STEP

    def decrease_target_altitude(self):
        self.target_altitude -= TARGET_ALTITUDE_STEP

    def increase_speed(self):
        new_val = self.speed + SPEED_PERCENT_STEP
        if new_val > 100:
            self.speed = 100.0
        else:
            self.speed = new_val

    def decrease_speed(self):
        new_val = self.speed - SPEED_PERCENT_STEP
        if new_val < 0:
            self.speed = 0
        else:
            self.speed = new_val

    def _get_speed_instruction(self):
        if self.hover:
            max_delta = MAX_PITCH - MIN_PITCH
            if self.current_pitch:
                delta = self.current_pitch - self.target_pitch
            else:
                delta = 0
            percent_delta = float(delta) / max_delta
            if self.speed == 0:
                self.speed = 5
            self.speed -= self.speed * percent_delta
            self.speed = (max(self.speed, 0.0))
            self.speed = (min(self.speed, 100.0))
        return (Instructions.SPEED, self.speed / 100.0)

    def _get_hover_instruction(self):
        if self.hover:
            return (Instructions.HOVER, 0)
        return None

    def _get_pitch_instruction(self):
        if None in (self.current_pitch, self.target_pitch) or self.hover:
            return None
        if self.target_pitch > self.current_pitch:
            instruction = Instructions.PITCH_UP
        else:
            instruction = Instructions.PITCH_DOWN
        max_possible_delta = MAX_PITCH - MIN_PITCH
        intensity = abs(self.current_pitch - self.target_pitch) / max_possible_delta
        intensity *= 2.0
        if intensity > 1.0:
            intensity = 1.0
        return (instruction, intensity)

    def _get_distance_left(self, current, target):
        current = int(current)
        target = int(target)
        degrees_ticked = 0
        while current != target:
            current -= 1
            degrees_ticked += 1
            if current >= 360:
                current -= 360
            elif current < 0:
                current += 360
        return degrees_ticked

    def _get_distance_right(self, current, target):
        current = int(current)
        target = int(target)
        degrees_ticked = 0
        while current != target:
            current += 1
            degrees_ticked += 1
            if current >= 360:
                current -= 360
            elif current < 0:
                current += 360
        return degrees_ticked

    def _fix_values_if_necessary(self):
        '''
        Unsure if this method is necessary, but adding it to be doubly safe
        '''
        if self.current_azimuth >= 360:
            self.current_azimuth -= 360
        if self.target_azimuth >= 360:
            self.target_azimuth -= 360

        if self.current_azimuth < 0:
            self.current_azimuth += 360
        if self.target_azimuth < 0:
            self.target_azimuth += 360

    def _get_yaw_instruction(self):
        '''
        azimuths needs to be in [0...359], pretty sure they already are,
        but will double check
        '''
        if None in (self.current_azimuth, self.target_azimuth):
            return (Instructions.YAW_LEFT, 0)
        elif self.hover:
            return (Instructions.YAW_LEFT, 0)

        self._fix_values_if_necessary()

        delta_angle = get_delta_angle(self.current_azimuth, self.target_azimuth)
        angular_intensity = min(2 * abs(delta_angle) / 180.0, 1.0)
        target_velocity = angular_intensity * MAX_ANGULAR_VELOCITY

        # dist_left = self._get_distance_left(self.current_azimuth, self.target_azimuth)
        # dist_right = self._get_distance_right(self.current_azimuth, self.target_azimuth)
        # if min(dist_left, dist_right) == dist_left:
        if delta_angle < 0:
            # left
            target_velocity *= -1
        # else go right
        diff_target_current_velocity = target_velocity - self.yaw_velocity
        if diff_target_current_velocity <= 0:
            instruction = Instructions.YAW_LEFT
        else:
            instruction = Instructions.YAW_RIGHT
        intensity = min(abs(2.0 * diff_target_current_velocity / MAX_ANGULAR_VELOCITY), 1.0)
        return (instruction, intensity)

    def get_instructions(self):
        instructions = []
        for func in self.INSTRUCTION_FUNCTIONS:
            instructions_result = func()
            if instructions_result is None:
                continue
            if isinstance(instructions_result, list):
                instructions += instructions_result
            else:
                instructions.append(instructions_result)
        return instructions

    def set_target_gps_coordinate(self, lat, lon):
        self.target_lat = lat
        self.target_lon = lon
