from blimp_controller.communication.xbee_manager import XBeeManager
from blimp_controller.constants import THUMBSTICK_COMMANDS
from blimp_controller.constants import JPEG_HEADER, JPEG_FRAME
from blimp_controller.constants import SensorData
from blimp_controller.utils import byte_val_to_value
# from blimp_controller.utils import integer_to_two_bytes, two_bytes_to_integer
import struct

HEADER_BYTES = (chr(254), chr(255),)
JPEG_COMMANDS = set([chr(JPEG_FRAME), chr(JPEG_HEADER)])


class Communicator(object):
    def __init__(self):
        self.xbee_module = XBeeManager()
        self.command_queue = []

    def enqueue_command(self, command, value):
        if command is None:
            return
        if isinstance(value, basestring):
            # special case for jpeg bytes (or possibly raw bytes)
            self.command_queue.append((chr(command), value))
            return

        # special case for lat and lon for floats
        if command in (SensorData.LATITUDE, SensorData.LONGITUDE):
            float_bytes = struct.pack('d', value)
            full_tuple = (chr(command),) + tuple(float_bytes)
            self.command_queue.append(full_tuple)
            return
        msb_byte, lsb_byte = struct.pack('h', value)
        self.command_queue.append((chr(command), msb_byte, lsb_byte, ))

    def enqueue_commands(self, dict_obj):
        for command, value in dict_obj.items():
            self.enqueue_command(command, value)

    def already_has_jpeg(self):
        for tuple_obj in self.command_queue:
            if tuple_obj[0] in JPEG_COMMANDS:
                return True
        return False

    def _trim_command_queue_by_latest(self):
        '''
        For the controller we're shaving off old
        jpeg bytes so we don't have a backfilled buffer
        '''
        newest_commands = {}
        for tuple_obj in self.command_queue:
            if ord(tuple_obj[0]) == JPEG_FRAME:
                newest_commands[tuple_obj[0]] = tuple_obj[1]
        self.command_queue = [t for t in self.command_queue if ord(t[0]) not in (JPEG_HEADER, JPEG_FRAME)]
        for tuple_obj in newest_commands.items():
            if ord(tuple_obj[0]) == JPEG_FRAME:
                jpeg_bytes = tuple_obj[1]
                self.enqueue_command(JPEG_HEADER, len(jpeg_bytes))
                self.command_queue.append(tuple_obj)
            else:
                self.command_queue.append(tuple_obj)

    def send_command(self):
        if len(self.command_queue) == 0:
            return
        while len(self.command_queue) > 0:
            for byte in HEADER_BYTES:
                self.xbee_module.write(byte)
            tuple_obj = self.command_queue.pop(0)
            if len(tuple_obj) == 2:
                # special case for JPEG bytes
                command, jpeg_bytes = tuple_obj
                self.xbee_module.write(command)
                self.xbee_module.write(jpeg_bytes)
                return
            command = tuple_obj[0]
            raw_bytes = tuple_obj[1:]
            self.xbee_module.write(command)
            for raw_byte in raw_bytes:
                self.xbee_module.write(raw_byte)

    def data_available(self):
        return  self.xbee_module.serial_port.inWaiting() > 0

    def _read_float_values(self):
        bytes_read = []
        for j in xrange(8):
            bytes_read.append(self.xbee_module.read())
        byte_string = "".join(bytes_read)
        return struct.unpack('d', byte_string)[0]

    def read_frame(self):
        while True:
            first_byte_read = self.xbee_module.read()
            while first_byte_read != HEADER_BYTES[0]:
                first_byte_read = self.xbee_module.read()
            if self.xbee_module.read() == HEADER_BYTES[1]:
                break
        command = ord(self.xbee_module.read())
        if command in (SensorData.LATITUDE, SensorData.LONGITUDE):
            float_value = self._read_float_values()
            return (command, float_value)
        msb_byte = self.xbee_module.read()
        lsb_byte = self.xbee_module.read()
        value = struct.unpack('h', "%s%s" % (msb_byte, lsb_byte))[0]
        if command in THUMBSTICK_COMMANDS:
            value = byte_val_to_value(value)
        return (command, value)
