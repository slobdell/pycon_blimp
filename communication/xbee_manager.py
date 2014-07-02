import serial
from blimp_controller.utils import find_tty_usb


class XBeeManager():
    def read(self):
        if not self.initialized:
            return None
        return self.serial_port.read()

    def write(self, value):
        if not self.initialized:
            return
        self.serial_port.write(value)

    def __init__(self):
        self.initialized = False
        serial_port_address = find_tty_usb("232 USB-Serial")
        if serial_port_address is None:
            return
        print "Opening %s for XBee Module" % serial_port_address
        self.serial_port = serial.Serial(
            port=serial_port_address,
            baudrate=57600,
            parity=serial.PARITY_NONE,
            bytesize=serial.EIGHTBITS,
            xonxoff=True
        )
        self.initialized = True
        print "Successfully opened XBee Module"

    def __del__(self):
        if self.initialized:
            self.serial_port.flushInput()
            self.serial_port.flushOutput()
            self.serial_port.close()
            print "Closing connection to serial port"
