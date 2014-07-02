import time

from blimp_controller.sensors.boost_accelerometer_sensor import AccelerometerSensor
from blimp_controller.sensors.boost_altimeter_sensor import AltimeterSensor
from blimp_controller.sensors.boost_compass_sensor import CompassSensor
from blimp_controller.sensors.gps_sensor import GPSSensor

from blimp_controller.constants import SensorData


class SensorManager(object):
    def __init__(self):
        self.compass_sensor = CompassSensor()
        self.accelerometer_sensor = AccelerometerSensor()
        self.altimeter_sensor = AltimeterSensor()
        self.gps_sensor = GPSSensor()
        self.update_this_iteration = False
        time.sleep(0.1)
        self.altimeter_sensor.zeroize_altitude()
        self._cached_data = {}
        self.current_lat = None
        self.current_lon = None
        self.current_altitude = None
        self.altitude_offset = 0.0

    def zeroize_gps_altitude(self):
        '''
        called the first time current altitude is not None
        '''
        self.altitude_offset = self.current_altitude

    def get_azimuth(self):
        cache_key = "azimuth"
        if cache_key in self._cached_data:
            return self._cached_data[cache_key]
        azimuth = self.compass_sensor.get_azimuth()
        azimuth = (azimuth + 360 - 90) % 360
        self._cached_data[cache_key] = azimuth
        return azimuth

    def get_altitude(self):
        '''
        Avoid this function in favor of GPS altitude
        '''
        cache_key = "altitude"
        if cache_key in self._cached_data:
            return self._cached_data[cache_key]
        altitude = self.altimeter_sensor.get_altitude()
        self._cached_data[cache_key] = altitude
        return altitude

    def get_lat_long_alt(self):
        cache_key = "lat_lon"
        if cache_key in self._cached_data:
            return self._cached_data[cache_key]
        (latitude, longitude, altitude) = self.gps_sensor.read()
        if latitude is None or longitude is None:
            return None, None, None
        old_altitude = self.current_altitude
        self.current_lat = latitude
        self.current_lon = longitude
        self.current_altitude = altitude
        if self.current_altitude and not old_altitude:
            self.zeroize_gps_altitude()
        self._cached_data[cache_key] = (self.current_lat, self.current_lon, self.current_altitude)
        return self.current_lat, self.current_lon, self.current_altitude

    def _update_accelerometer(self):
        if self.update_this_iteration:
            return
        self.accelerometer_sensor.read()
        self.updated_this_iteration = True

    def _clear_cache(self):
        self._cached_data = {}

    def get_pitch(self):
        cache_key = "pitch"
        if cache_key in self._cached_data:
            return self._cached_data[cache_key]
        self._update_accelerometer()
        pitch = self.accelerometer_sensor.get_pitch()
        self._cached_data[cache_key] = pitch
        return pitch

    def get_roll(self):
        '''
        Positive roll means roll left,
        negative roll means roll right
        '''
        cache_key = "roll"
        if cache_key in self._cached_data:
            return self._cached_data[cache_key]
        self._update_accelerometer()
        roll = self.accelerometer_sensor.get_roll()
        self._cached_data[cache_key] = roll
        return roll

    def tick(self):
        self._clear_cache()
        self.updated_this_iteration = False

    def get_data_packet(self):
        data = {
            SensorData.PITCH: int(self.get_pitch()),
            SensorData.ROLL: int(self.get_roll()),
            SensorData.AZIMUTH: int(self.get_azimuth()),
        }
        self.get_lat_long_alt()  # this can return None, None, but updates self.current_lat and lon
        lat = self.current_lat
        lon = self.current_lon
        if self.current_altitude:
            alt = self.current_altitude - self.altitude_offset
        else:
            alt = None
        if lat and lon and alt:
            data.update({
                SensorData.LATITUDE: lat,
                SensorData.LONGITUDE: lon,
                SensorData.ALTITUDE: int(alt)
            })
        return data
