# Pi Pico MCP23S08 SPI Implementation
# 2022 Kyle Nahas
# Forked from jebentancour's version of Pi_MCP23S08
# https://github.com/jebentancour/Pi_MCP23S08
# Modified with the help of https://docs.micropython.org/en/latest/library/machine.SPI.html#machine-spi
from builtins import bytearray

import micropython
from micropython import const
from machine import Pin, SPI


class MCP23S08(object):
    """This class provides an abstraction of the GPIO expander MCP23S08."""

    DIR_INPUT = const(1)
    DIR_OUTPUT = const(0)
    PULLUP_ENABLED = const(1)
    PULLUP_DISABLED = const(0)
    LEVEL_LOW = const(0)
    LEVEL_HIGH = const(1)

    """Register addresses as documented in the technical data sheet at
    https://ww1.microchip.com/downloads/en/DeviceDoc/21919e.pdf
    """
    MCP23S08_IODIR = const(0x00)
    MCP23S08_IPOL = const(0x01)
    MCP23S08_GPIO = const(0x09)
    MCP23S08_OLAT = const(0x0A)
    MCP23S08_IOCON = const(0x05)
    MCP23S08_GPPU = const(0x06)

    """Bit field flags as documented in the technical data sheet at
    http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf

    bit 0 Unimplemented: Read as ‘0’

    bit 1 INTPOL: This bit sets the polarity of the INT output pin
        1 = Active-high
        0 = Active-low

    bit 2 ODR: Configures the INT pin as an open-drain output
        1 = Open-drain output (overrides the INTPOL bit.)
        0 = Active driver output (INTPOL bit sets the polarity.)

    bit 3 HAEN: Hardware Address Enable bit (MCP23S08 only) (Note 1)
        1 = Enables the MCP23S08 address pins.
        0 = Disables the MCP23S08 address pins.

    bit 4 DISSLW: Slew Rate control bit for SDA output
        1 = Slew rate disabled
        0 = Slew rate enabled

    bit 5 SEQOP: Sequential Operation mode bit
        1 = Sequential operation disabled, address pointer does not increment.
        0 = Sequential operation enabled, address pointer increments.

    bit 6 MIRROR: INT Pins Mirror bit
        1 = The INT pins are internally connected
        0 = The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB

    bit 7 BANK: Controls how the registers are addressed
        1 = The registers associated with each port are separated into different banks.
        0 = The registers are in the same bank (addresses are sequential).
    """

    IOCON_UNUSED = const(0x01)
    IOCON_INTPOL = const(0x02)
    IOCON_ODR = const(0x04)
    IOCON_HAEN = const(0x08)
    IOCON_DISSLW = const(0x10)
    IOCON_SEQOP = const(0x20)
    IOCON_UNUSED2 = const(0x40)
    IOCON_UNUSED3 = const(0x80)

    IOCON_INIT = const(
        0x00
    )  # IOCON_BANK_MODE = 0, IOCON_HAEN = 0 address pins disabled

    MCP23S08_CMD_WRITE = const(0x42)
    MCP23S08_CMD_READ = const(0x43)

    def __init__(
        self, spi=None, address=None, bus=0, speed=1000000, sck=2, mosi=3, miso=4, cs=5
    ):
        """
        Constructor

        :param address: The device ID of the component, i.e., the hardware address (default 0.0)
        :param spi: optional argument which passes pre-initialized SPI bus
        :param bus: The SPI Bus to Initialize
        :param speed: The speed to run the SPI bus at; default 1MHz
        :param sck: The pin number for the spi clock pin
        :param mosi: The pin number for the MOSI pin
        :param miso: The pin number for the MISO pin
        :param cs: The pin number for the chip-enable pin

        """
        if not spi:
            self.spi = SPI(
                bus,
                baudrate=speed,
                polarity=0,
                phase=0,
                bits=8,
                firstbit=SPI.MSB,
                sck=Pin(sck),
                mosi=Pin(mosi),
                miso=Pin(miso),
            )
        else:
            self.spi = spi

        self._ha_en = True
        if not address:
            self.device_address = 0x00
        else:
            self.device_address = address
            self._ha_en = True

        self.cs = Pin(cs, Pin.OUT, value=1)

        self._buf_rx_word = [0, 0]
        self._buf_tx_word = [0, 0]
        self._buf_rx = bytearray(3)
        self._buf_tx = bytearray(3)
        self._buf_register = 0
        self._buf_command = 0

        self._GPIO = 0x00
        self._IODIR = 0xFF
        self._GPPU = 0x00
        self.isInitialized = False

    def open(self):
        """Initializes the MCP23S08 with hardware-address access
        and sequential operations mode.
        """
        # self.spi.open(self.bus, self.ce)
        # self.spi.max_speed_hz = 10000000
        self.isInitialized = True

        config = MCP23S08.IOCON_INIT

        if self._ha_en:
            config |= MCP23S08.IOCON_HAEN

        self._writeRegister(MCP23S08.MCP23S08_IOCON, config)

    def close(self):
        """Closes the SPI connection that the MCP23S08 component is using."""
        # self.spi.close()
        self.isInitialized = False

    def setPullupMode(self, pin, mode):
        """
        Enables or disables the pull-up mode for input pins.

        :param pin: The pin index (0 - 15)
        :param mode: The pull-up mode (MCP23S08.PULLUP_ENABLED, MCP23S08.PULLUP_DISABLED)
        """
        assert pin < 8
        assert (mode == MCP23S08.PULLUP_ENABLED) or (mode == MCP23S08.PULLUP_DISABLED)
        assert self.isInitialized

        register = MCP23S08.MCP23S08_GPPU
        data = self._GPPU
        noshifts = pin

        if mode == MCP23S08.PULLUP_ENABLED:
            data |= 1 << noshifts
        else:
            data &= ~(1 << noshifts)

        self._writeRegister(register, data)

        self._GPPU = data

    def setPullup(self, data):
        assert self.isInitialized

        self._writeRegister(MCP23S08.MCP23S08_GPPU, data)
        self._GPPU = data

    def setDirection(self, pin, direction):
        """
        Sets the direction for a given pin.

        :param pin: The pin index (0 - 15)
        :param direction: The direction of the pin (MCP23S08.DIR_INPUT, MCP23S08.DIR_OUTPUT)
        """
        assert pin < 8
        assert (direction == MCP23S08.DIR_INPUT) or (direction == MCP23S08.DIR_OUTPUT)
        assert self.isInitialized

        register = MCP23S08.MCP23S08_IODIR
        data = self._IODIR
        noshifts = pin

        if direction == MCP23S08.DIR_INPUT:
            data |= 1 << noshifts
        else:
            data &= ~(1 << noshifts)

        self._writeRegister(register, data)

        self._IODIR = data

    def setDirPORT(self, data):
        """
        Sets the direction for the entire port in one shot.

        :param data: bitfield representing pin directions. Set (1) indicates input, Cleared (0) indicates output
        """
        assert self.isInitialized

        self._writeRegister(MCP23S08.MCP23S08_IODIR, data)
        self._IODIR = data

    def digitalRead(self, pin):
        """
        Reads the logical level of a given pin.

        :param pin: The pin index (0 - 7)
        :return: MCP23S08.LEVEL_LOW, if the logical level of the pin is low, otherwise MCP23S08.LEVEL_HIGH.
        """
        assert self.isInitialized
        assert pin < 8

        self._GPIO = self._readRegister(MCP23S08.MCP23S08_GPIO)
        if (self._GPIO & (1 << pin)) != 0:
            return MCP23S08.LEVEL_HIGH
        else:
            return MCP23S08.LEVEL_LOW

    def readGPIO(self):
        assert self.isInitialized
        # It is unclear why we are only reading the second byte.
        data = self._readRegister(MCP23S08.MCP23S08_GPIO)
        self._GPIO = data[2]
        return data[2]

    def digitalWrite(self, pin, level):
        """
        Sets the level of a given pin.

        :param pin: The pin index (0 - 7)
        :param level: The logical level to be set (MCP23S08.LEVEL_LOW, MCP23S08.LEVEL_HIGH)
        """
        assert self.isInitialized
        assert pin < 8
        assert (level == MCP23S08.LEVEL_HIGH) or (level == MCP23S08.LEVEL_LOW)

        register = MCP23S08.MCP23S08_GPIO
        data = self._GPIO
        num_shifts = pin

        if level == MCP23S08.LEVEL_HIGH:
            data |= 1 << num_shifts
        else:
            data &= ~(1 << num_shifts)

        self._writeRegister(register, data)

        self._GPIO = data

    def writeGPIO(self, data):
        assert self.isInitialized

        self._writeRegister(MCP23S08.MCP23S08_GPIO, data)
        self._GPIO = data

    def writeNibble(self, data, half):
        """
        Writes to half of (a nibble) the pins

            :param data: The data to be written. Must be positive and less than or equal to 0xF (0b1111)
            :param half: Low or High, indicating which nibble to be written. Accepts LEVEL_LOW or LEVEL_HIGH
        """
        assert self.isInitialized
        assert 0 <= data <= 0xF
        assert (half == MCP23S08.LEVEL_HIGH) or (half == MCP23S08.LEVEL_LOW)
        if half is MCP23S08.LEVEL_LOW:
            self._GPIO &= 0xF0
            self._GPIO |= data
        else:
            self._GPIO &= 0xF
            self._GPIO |= data << 4
        self._writeRegister(MCP23S08.MCP23S08_GPIO, self._GPIO)

    @micropython.native
    def _writeRegister(self, register, value):
        # assert self.isInitialized

        cmd = MCP23S08.MCP23S08_CMD_WRITE | ((self.device_address) << 1)
        tx = bytearray([cmd, register, value])
        # self.spi.xfer2([command, register, value])
        try:
            self.cs.value(0)
            self.spi.write(tx)
        finally:
            self.cs.value(1)

    @micropython.native
    def _readRegister(self, register):
        # assert self.isInitialized

        cmd = MCP23S08.MCP23S08_CMD_READ | ((self.device_address) << 1)
        tx = bytearray([cmd, register, 0])
        rx = bytearray(3)
        # data = self.spi.xfer2([command, register, 0])
        # return data[2]
        try:
            self.cs.value(0)
            self.spi.write_readinto(tx, rx)
        finally:
            self.cs.value(1)
            return rx
