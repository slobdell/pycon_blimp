import serial
import time
from blimp_controller.utils import find_tty_usb
# from blimp_controller.utils import blimp_log


class GPSSensor(object):

    def __init__(self):
        self.initialized = False
        serial_port_address = find_tty_usb("PL2303 Serial Port")
        if serial_port_address is None:
            return
        print "Opening %s for GPS Module" % serial_port_address
        self.serial_port = serial.Serial(
            port=serial_port_address,
            baudrate=4800,
            parity=serial.PARITY_NONE,
            bytesize=serial.EIGHTBITS,
            xonxoff=True
        )
        time.sleep(0.1)
        self.initialized = True
        print "Successfully connected to GPS Sensor"

    def __del__(self):
        if self.initialized:
            self.serial_port.close()

    def _convert_to_decimal(self, value):
        '''
        Variable names here are horrible.  Took some logic that
        I wrote a year ago and don't know exactly what's happening
        here.  Need to come back and fix.  Don't judge.
        '''
        value = float(value)
        degrees = int(value / 100.0)
        decimal_degrees = (value - (100 * degrees)) / 60.0
        full_value = degrees + decimal_degrees
        return full_value

    def _parse_data(self, data_from_serial):
        if not data_from_serial.startswith("$GPGGA"):
            return None, None, None
        tokens = data_from_serial.split(",")
        if len(tokens) <= 10:
            return None, None, None

        gps_fix = tokens[6]
        if gps_fix == '0':  # no satellites
            return None, None, None

        dlat = tokens[2]
        dlon = tokens[4]

        full_latitude = self._convert_to_decimal(dlat)
        full_longitude = self._convert_to_decimal(dlon)

        north_or_south = tokens[3][0]
        if north_or_south == 'S':
            full_latitude *= -1
        east_or_west = tokens[5][0]
        if east_or_west == 'W':
            full_longitude *= -1
        altitude = float(tokens[9])
        return full_latitude, full_longitude, altitude

    def read(self):
        '''
        Returns the latest GPS signal received given the buffered data
        in waiting.  Keep in mind that this might be None, None
        '''
        if not self.initialized:
            return None, None, None
        latitude, longitude, altitude = (None, None, None)
        while self.serial_port.inWaiting():
            # get the latest read, not just the first thing we see
            data = self.serial_port.readline()
            temp_latitude, temp_longitude, temp_altitude = self._parse_data(data)
            if temp_latitude is not None:
                latitude = temp_latitude
            if temp_longitude is not None:
                longitude = temp_longitude
            if temp_altitude is not None:
                altitude = temp_altitude
        return latitude, longitude, altitude
