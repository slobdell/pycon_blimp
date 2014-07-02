import os
import datetime
import math
from os.path import join

LOG_FILE = "blimp_logging.log"


# in hindsight, don't use this, just use struct.pack
def value_to_byte_val(value):
    byte_val = 0
    if value < 0:
        byte_val += 128
    value = abs(value)
    byte_val += int(value * 127)
    return byte_val


def byte_val_to_value(byte_val):
    is_negative = False
    if byte_val > 128:
        is_negative = True
        byte_val -= 128
    actual_val = float(byte_val) / 127
    if is_negative:
        actual_val *= -1
    return actual_val


def find_tty_usb(name_contains_str):
    linux_command = "lsusb | grep \"%s\"" % name_contains_str
    device_string = os.popen(linux_command).read()
    arr = device_string.split(":")
    if len(arr) < 2:
        return None
    idVendor = arr[1].split(" ")[-1]
    idProduct = arr[2].split(" ")[0]

    for dnbase in os.listdir('/sys/bus/usb/devices'):
        dn = join('/sys/bus/usb/devices', dnbase)
        if not os.path.exists(join(dn, 'idVendor')):
            continue
        idv = open(join(dn, 'idVendor')).read().strip()
        if idv != idVendor:
            continue
        idp = open(join(dn, 'idProduct')).read().strip()
        if idp != idProduct:
            continue
        for subdir in os.listdir(dn):
            if subdir.startswith(dnbase + ':'):
                for subsubdir in os.listdir(join(dn, subdir)):
                    if subsubdir.startswith('ttyUSB'):
                        return join('/dev', subsubdir)


def blimp_log(data):
    with open(LOG_FILE, "a+") as log_file:
        log_file.write("{%s: %s}" % (datetime.datetime.now(), data))
        log_file.write("\n")


def radians_to_degrees(radians):
    return radians * 180.0 / math.pi


def azimuth_from_gps_coords(from_coord, to_coord):
    '''
    returns 0...360
    '''
    if None in from_coord + to_coord:
        return None
    lat1 = from_coord[0]
    lat2 = to_coord[0]
    lon1 = from_coord[1]
    lon2 = to_coord[1]
    dy = lat2 - lat1
    dx = math.cos(math.pi / 180.0 * lat1) * (lon2 - lon1)
    angle = math.atan2(dy, dx)
    theta = radians_to_degrees(angle)
    degrees_north = 90.0 - theta
    if degrees_north < 0:
        degrees_north += 360
    return degrees_north


def degrees_to_radians(degrees):
    return degrees * math.pi / 180


def get_distance_meters(from_coord, to_coord):
    radius_of_earth_m = 6371 * 1000

    lat1 = from_coord[0]
    lat2 = to_coord[0]
    lon1 = from_coord[1]
    lon2 = to_coord[1]

    delta_lat = degrees_to_radians(lat2 - lat1)
    delta_lon = degrees_to_radians(lon2 - lon1)
    a = pow(math.sin(delta_lat / 2), 2) +\
        math.cos(degrees_to_radians(lat1)) * math.cos(degrees_to_radians(lat2)) *\
        pow(math.sin(delta_lon / 2), 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = radius_of_earth_m * c
    return d

def get_delta_angle(from_azimuth, to_azimuth):
    delta_azimuth = to_azimuth - from_azimuth
    if delta_azimuth > 180:
        delta_azimuth = delta_azimuth - 360
    elif delta_azimuth < -180:
        delta_azimuth = delta_azimuth + 360
    return delta_azimuth
