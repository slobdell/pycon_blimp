
    import serial
    from blimp_controller.utils import find_tty_usb


    class XBeeManager():
        def read(self):
            return self.serial_port.read()

        def write(self, value):
            self.serial_port.write(value)

        def __init__(self):
            serial_port_address = find_tty_usb("232 USB-Serial")
            self.serial_port = serial.Serial(
                port=serial_port_address,
                baudrate=57600,
                parity=serial.PARITY_NONE,
                bytesize=serial.EIGHTBITS,
                xonxoff=True
            )

    def __del__(self):
        if self.initialized:
            self.serial_port.flushInput()
            self.serial_port.flushOutput()
            self.serial_port.close()
            print "Closing connection to serial port"
