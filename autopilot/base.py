from abc import abstractmethod

from blimp_controller.constants import SensorData


class BaseAutopilot(object):
    def __init__(self):
        self.current_azimuth = None
        self.current_pitch = None
        self.current_roll = None
        self.current_lat = None
        self.current_lon = None
        self.current_altitude = None
        self.target_altitude = 0
        self.waypoints = []
        self.new_waypoint = [None, None]
        self.speed = 0

    def update_from_sensor_data(self, data_packet_dict):
        self.current_azimuth = data_packet_dict.get(SensorData.AZIMUTH)
        self.current_pitch = data_packet_dict.get(SensorData.PITCH)
        self.current_roll = data_packet_dict.get(SensorData.ROLL)
        self.current_lat = data_packet_dict.get(SensorData.LATITUDE)
        self.current_lon = data_packet_dict.get(SensorData.LONGITUDE)
        self.current_altitude = data_packet_dict.get(SensorData.ALTITUDE)
        self.target_altitude = data_packet_dict.get(SensorData.TARGET_ALTITUDE)

    @abstractmethod
    def think(self):
        '''
        recompute once per cycle
        '''
        pass

    @abstractmethod
    def get_instructions(self):
        '''
        Return a series of enums with appropriate intensities
        i.e.:
            return [(Instructions.HOVER, 0), (Instructions.PITCH_DOWN, 30.0)]
        '''
        pass

    @abstractmethod
    def get_data_for_client(self):
        '''
        Surfaces information to send back to client,
        i.e. waypoint can be popped.  Should be computed
        during think()
        '''
        pass

    def _add_waypoint_if_ready(self):
        if self.new_waypoint[0] and self.new_waypoint[1]:
            self.waypoints.append(self.new_waypoint)
            self.new_waypoint = [None, None]

    def add_waypoint_lat(self, latitude):
        self.new_waypoint[0] = latitude
        self._add_waypoint_if_ready()

    def add_waypoint_lon(self, longitude):
        self.new_waypoint[1] = longitude
        self._add_waypoint_if_ready()
