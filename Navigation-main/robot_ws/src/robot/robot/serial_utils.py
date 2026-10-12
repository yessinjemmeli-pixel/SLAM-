from __future__ import print_function, division, absolute_import
import sys
import glob
import serial

#This was not written by us and was just used in serial.py


# From https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python
def get_serial_ports() -> list[str]:
    """
    Lists serial ports.
    :return: ([str]) A list of available serial ports
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    results = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            results.append(port)
        except (OSError, serial.SerialException):
            pass
    return results


def open_serial_port(serial_port: str | None = None,
                     baudrate: int = 115200,
                     timeout: float | None = 1,
                     write_timeout: float | None = 0):
    """
    Try to open serial port with Arduino
    If not port is specified, it will be automatically detected
    """
    if serial_port is None:
        try:
            serial_port = get_serial_ports()[0]
        except IndexError as e:
            print("Couldn't find a serial port.")
    # timeout=0 non-blocking mode, return immediately in any case, returning zero or more,
    # up to the requested number of bytes
    return serial.Serial(port=serial_port, baudrate=baudrate, timeout=timeout, write_timeout=write_timeout)
