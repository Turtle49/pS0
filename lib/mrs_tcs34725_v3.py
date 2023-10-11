
"""
`mrs_tcs34725`

Circuitpython module for the TCS34725 sensor

v2: adopted by MRS
v3: included 0x10 address in list of accepted devices to enable their use, current list: (0x44,0x4d, 0x10)

"""
import time

from adafruit_bus_device import i2c_device
from micropython import const

try:
    from typing import Tuple
    from busio import I2C
except ImportError:
    pass


_COMMAND_BIT = const(0x80)
_REGISTER_ENABLE = const(0x00)
_REGISTER_ATIME = const(0x01)
_REGISTER_AILT = const(0x04)
_REGISTER_AIHT = const(0x06)
_REGISTER_ID = const(0x12)
_REGISTER_APERS = const(0x0C)
_REGISTER_CONTROL = const(0x0F)
_REGISTER_SENSORID = const(0x12)
_REGISTER_STATUS = const(0x13)

_REGISTER_CDATAL = const(0x14)
_REGISTER_CDATAH = const(0x15)
_REGISTER_RDATAL = const(0x16)
_REGISTER_RDATAH = const(0x17)
_REGISTER_GDATAL = const(0x18)
_REGISTER_GDATAH = const(0x19)
_REGISTER_BDATAL = const(0x1A)
_REGISTER_BDATAH = const(0x1B)

_ENABLE_AIEN = const(0x10)
_ENABLE_WEN = const(0x08)
_ENABLE_AEN = const(0x02)
_ENABLE_PON = const(0x01)
_GAINS = (1, 4, 16, 60)
_CYCLES = (0, 1, 2, 3, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60)
_INTEGRATION_TIME_THRESHOLD_LOW = 2.4
_INTEGRATION_TIME_THRESHOLD_HIGH = 614.4


class TCS34725:

    # Class-level buffer for reading and writing data with the sensor.
    # This reduces memory allocations but means the code is not re-entrant or
    # thread safe!
    _BUFFER = bytearray(3)

    def __init__(self, i2c: I2C, address: int = 0x29):
        self._device = i2c_device.I2CDevice(i2c, address)
        self._active = False
        self.integration_time = 2.4
        # Check sensor ID is expectd value.
        sensor_id = self._read_u8(_REGISTER_SENSORID)
        if sensor_id not in (0x44,0x4d, 0x10):
            raise RuntimeError("Could not find sensor, check wiring!")
        self.active = True

    @property
    def active(self):
        """The active state of the sensor.  Boolean value that will
        enable/activate the sensor with a value of True and disable with a
        value of False.
        """
        return self._active

    @active.setter
    def active(self, val: bool):
        val = bool(val)
        if self._active == val:
            return
        self._active = val
        enable = self._read_u8(_REGISTER_ENABLE)
        if val:
            self._write_u8(_REGISTER_ENABLE, enable | _ENABLE_PON | _ENABLE_AEN)

    @property
    def integration_time(self):
        """The integration time of the sensor in milliseconds."""
        return self._integration_time

    @integration_time.setter
    def integration_time(self, val: float):
        if (
            not _INTEGRATION_TIME_THRESHOLD_LOW
            <= val
            <= _INTEGRATION_TIME_THRESHOLD_HIGH
        ):
            raise ValueError(
                "Integration Time must be between '{0}' and '{1}'".format(
                    _INTEGRATION_TIME_THRESHOLD_LOW, _INTEGRATION_TIME_THRESHOLD_HIGH
                )
            )
        cycles = int(val / 2.4)
        self._integration_time = (
            cycles * 2.4
        )  # pylint: disable=attribute-defined-outside-init
        self._write_u8(_REGISTER_ATIME, 256 - cycles)

    @property
    def gain(self):
        """The gain of the sensor.  Should be a value of 1, 4, 16,
        or 60.
        """
        return _GAINS[self._read_u8(_REGISTER_CONTROL)]

    @gain.setter
    def gain(self, val: int):
        if val not in _GAINS:
            raise ValueError(
                "Gain should be one of the following values: {0}".format(_GAINS)
            )
        self._write_u8(_REGISTER_CONTROL, _GAINS.index(val))

    @property
    def isready(self):
        """MRS: routine that checks whether the chip is ready to pass on data"""
        return bool(self._valid())

    @property
    def color_rgb_bytes(self):
        """Read the RGB color detected by the sensor.  Returns a 3-tuple of
        red, green, blue component values as bytes (0-255).
        """
        r, g, b, clear = self.color_raw

        # Avoid divide by zero errors ... if clear = 0 return black
        if clear == 0:
            return (0, 0, 0)

        # Each color value is normalized to clear, to obtain int values between 0 and 255.
        # A gamma correction of 2.5 is applied to each value as well, first dividing by 255,
        # since gamma is applied to values between 0 and 1
        red = int(pow((int((r / clear) * 256) / 255), 2.5) * 255)
        green = int(pow((int((g / clear) * 256) / 255), 2.5) * 255)
        blue = int(pow((int((b / clear) * 256) / 255), 2.5) * 255)

        # Handle possible 8-bit overflow
        red = min(red, 255)
        green = min(green, 255)
        blue = min(blue, 255)
        return (red, green, blue)

    @property
    def color_raw(self):
        """Read the raw RGBC color detected by the sensor.  Returns a 4-tuple of
        16-bit red, green, blue, clear component byte values (0-65535).
        """
        try:
            data = [
                self._read_u16(reg)
                for reg in (
                    _REGISTER_RDATAL,
                    _REGISTER_GDATAL,
                    _REGISTER_BDATAL,
#                    _REGISTER_CDATAL,
                    )
            ]
        except OSError:
            data = [0,0,0]
        return data


    @property
    def color_raw8(self):
        """Read the raw RGBC color detected by the sensor.
        """
        try:
            data = [
                self._read_u8(reg)
                for reg in (
                    _REGISTER_RDATAL,
                    _REGISTER_RDATAH,
                    _REGISTER_GDATAL,
                    _REGISTER_GDATAH,
                    _REGISTER_BDATAL,
                    _REGISTER_BDATAH,
                )
            ]
        except OSError:
            data = [0,0,0]
        return [data[1] << 8 | data[0], data[3] << 8 | data[2], data[5] << 8 | data[4]]




    def _valid(self) -> bool:
        # Check if the status bit is set and the chip is ready.
#Original line        return bool(self._read_u8(_REGISTER_STATUS) & 0x01) ##MU I think there's a typo here
        return bool(self._read_u8(_REGISTER_STATUS) & 0b00000001) ##MU I think there's a typo here

    def _read_u8(self, address: int) -> int:
        # Read an 8-bit unsigned value from the specified 8-bit address.
        with self._device as i2c:
            self._BUFFER[0] = (address | _COMMAND_BIT) & 0xFF
            i2c.write_then_readinto(self._BUFFER, self._BUFFER, out_end=1, in_end=1)
        return self._BUFFER[0]

    def _read_u16(self, address: int) -> int:
        # Read a 16-bit unsigned value from the specified 8-bit address.
        with self._device as i2c:
            self._BUFFER[0] = (address | _COMMAND_BIT) & 0xFF

            i2c.write_then_readinto(self._BUFFER, self._BUFFER, out_end=1, in_end=2)

            #i2c.readinto(self._BUFFER)

#        return (self._BUFFER[0] << 8) | self._BUFFER[1]
        return (self._BUFFER[1] << 8) | self._BUFFER[0]
#        return self._BUFFER[0]

    def _write_u8(self, address: int, val: int):
        # Write an 8-bit unsigned value to the specified 8-bit address.
        with self._device as i2c:
            self._BUFFER[0] = (address | _COMMAND_BIT) & 0xFF
            self._BUFFER[1] = val & 0xFF
            i2c.write(self._BUFFER, end=2)

    def _write_u16(self, address: int, val: int):
        # Write a 16-bit unsigned value to the specified 8-bit address.
        with self._device as i2c:
            self._BUFFER[0] = (address | _COMMAND_BIT) & 0xFF
            self._BUFFER[1] = val & 0xFF
            self._BUFFER[2] = (val >> 8) & 0xFF
            i2c.write(self._BUFFER)
