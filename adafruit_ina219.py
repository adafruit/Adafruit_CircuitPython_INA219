# SPDX-FileCopyrightText: 2017 Dean Miller for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_ina219`
====================================================

CircuitPython driver for the INA219 current sensor.

* Author(s): Dean Miller

Implementation Notes
--------------------

**Hardware:**

* `Adafruit INA219 High Side DC Current Sensor Breakout <https://www.adafruit.com/product/904>`_

* `Adafruit INA219 FeatherWing <https://www.adafruit.com/product/3650>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware (2.2.0+) for the ESP8622 and M0-based boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice

from adafruit_register.i2c_struct import ROUnaryStruct, UnaryStruct
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_bit import ROBit

try:
    import typing  # pylint: disable=unused-import
    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_INA219.git"

# Bits
# pylint: disable=too-few-public-methods

# Config Register (R/W)
_REG_CONFIG = const(0x00)


class BusVoltageRange:
    """Constants for ``bus_voltage_range``"""

    RANGE_16V = 0x00  # set bus voltage range to 16V
    RANGE_32V = 0x01  # set bus voltage range to 32V (default)


class Gain:
    """Constants for ``gain``"""

    DIV_1_40MV = 0x00  # shunt prog. gain set to  1, 40 mV range
    DIV_2_80MV = 0x01  # shunt prog. gain set to /2, 80 mV range
    DIV_4_160MV = 0x02  # shunt prog. gain set to /4, 160 mV range
    DIV_8_320MV = 0x03  # shunt prog. gain set to /8, 320 mV range


class ADCResolution:
    """Constants for ``bus_adc_resolution`` or ``shunt_adc_resolution``"""

    ADCRES_9BIT_1S = 0x00  #  9bit,   1 sample,     84us
    ADCRES_10BIT_1S = 0x01  # 10bit,   1 sample,    148us
    ADCRES_11BIT_1S = 0x02  # 11 bit,  1 sample,    276us
    ADCRES_12BIT_1S = 0x03  # 12 bit,  1 sample,    532us
    ADCRES_12BIT_2S = 0x09  # 12 bit,  2 samples,  1.06ms
    ADCRES_12BIT_4S = 0x0A  # 12 bit,  4 samples,  2.13ms
    ADCRES_12BIT_8S = 0x0B  # 12bit,   8 samples,  4.26ms
    ADCRES_12BIT_16S = 0x0C  # 12bit,  16 samples,  8.51ms
    ADCRES_12BIT_32S = 0x0D  # 12bit,  32 samples, 17.02ms
    ADCRES_12BIT_64S = 0x0E  # 12bit,  64 samples, 34.05ms
    ADCRES_12BIT_128S = 0x0F  # 12bit, 128 samples, 68.10ms


class Mode:
    """Constants for ``mode``"""

    POWERDOWN = 0x00  # power down
    SVOLT_TRIGGERED = 0x01  # shunt voltage triggered
    BVOLT_TRIGGERED = 0x02  # bus voltage triggered
    SANDBVOLT_TRIGGERED = 0x03  # shunt and bus voltage triggered
    ADCOFF = 0x04  # ADC off
    SVOLT_CONTINUOUS = 0x05  # shunt voltage continuous
    BVOLT_CONTINUOUS = 0x06  # bus voltage continuous
    SANDBVOLT_CONTINUOUS = 0x07  # shunt and bus voltage continuous


# SHUNT VOLTAGE REGISTER (R)
_REG_SHUNTVOLTAGE = const(0x01)

# BUS VOLTAGE REGISTER (R)
_REG_BUSVOLTAGE = const(0x02)

# POWER REGISTER (R)
_REG_POWER = const(0x03)

# CURRENT REGISTER (R)
_REG_CURRENT = const(0x04)

# CALIBRATION REGISTER (R/W)
_REG_CALIBRATION = const(0x05)
# pylint: enable=too-few-public-methods


def _to_signed(num: int) -> int:
    if num > 0x7FFF:
        num -= 0x10000
    return num


class INA219: # pylint: disable=too-many-instance-attributes
    """Driver for the INA219 current sensor"""

    # Basic API:

    # INA219( i2c_bus, addr)  Create instance of INA219 sensor
    #    :param i2c_bus          The I2C bus the INA219is connected to
    #    :param addr (0x40)      Address of the INA219 on the bus (default 0x40)

    # shunt_voltage               RO : shunt voltage scaled to Volts
    # bus_voltage                 RO : bus voltage (V- to GND) scaled to volts (==load voltage)
    # current                     RO : current through shunt, scaled to mA
    # power                       RO : power consumption of the load, scaled to Watt
    # set_calibration_32V_2A()    Initialize chip for 32V max and up to 2A (default)
    # set_calibration_32V_1A()    Initialize chip for 32V max and up to 1A
    # set_calibration_16V_400mA() Initialize chip for 16V max and up to 400mA

    # Advanced API:
    # config register break-up
    #   reset                     WO : Write Reset.RESET to reset the chip (must recalibrate)
    #   bus_voltage_range         RW : Bus Voltage Range field (use BusVoltageRange.XXX constants)
    #   gain                      RW : Programmable Gain field (use Gain.XXX constants)
    #   bus_adc_resolution        RW : Bus ADC resolution and averaging modes (ADCResolution.XXX)
    #   shunt_adc_resolution      RW : Shunt ADC resolution and averaging modes (ADCResolution.XXX)
    #   mode                      RW : operating modes in config register (use Mode.XXX constants)

    # raw_shunt_voltage           RO : Shunt Voltage register (not scaled)
    # raw_bus_voltage             RO : Bus Voltage field in Bus Voltage register (not scaled)
    # conversion_ready            RO : Conversion Ready bit in Bus Voltage register
    # overflow                    RO : Math Overflow bit in Bus Voltage register
    # raw_power                   RO : Power register (not scaled)
    # raw_current                 RO : Current register (not scaled)
    # calibration                 RW : calibration register (note: value is cached)

    def __init__(self, i2c_bus: I2C, addr: int = 0x40) -> None:
        self.i2c_device = I2CDevice(i2c_bus, addr)
        self.i2c_addr = addr

        # Set chip to known config values to start
        self._cal_value = 0
        self._current_lsb = 0
        self._power_lsb = 0
        self.set_calibration_32V_2A()

    # config register break-up
    reset = RWBits(1, _REG_CONFIG, 15, 2, False)
    bus_voltage_range = RWBits(1, _REG_CONFIG, 13, 2, False)
    gain = RWBits(2, _REG_CONFIG, 11, 2, False)
    bus_adc_resolution = RWBits(4, _REG_CONFIG, 7, 2, False)
    shunt_adc_resolution = RWBits(4, _REG_CONFIG, 3, 2, False)
    mode = RWBits(3, _REG_CONFIG, 0, 2, False)

    # shunt voltage register
    raw_shunt_voltage = ROUnaryStruct(_REG_SHUNTVOLTAGE, ">h")

    # bus voltage register
    raw_bus_voltage = ROBits(13, _REG_BUSVOLTAGE, 3, 2, False)
    conversion_ready = ROBit(_REG_BUSVOLTAGE, 1, 2, False)
    overflow = ROBit(_REG_BUSVOLTAGE, 0, 2, False)

    # power and current registers
    raw_power = ROUnaryStruct(_REG_POWER, ">H")
    raw_current = ROUnaryStruct(_REG_CURRENT, ">h")

    # calibration register
    _raw_calibration = UnaryStruct(_REG_CALIBRATION, ">H")

    @property
    def calibration(self) -> int:
        """Calibration register (cached value)"""
        return self._cal_value  # return cached value

    @calibration.setter
    def calibration(self, cal_value: int) -> None:
        self._cal_value = (
            cal_value  # value is cached for ``current`` and ``power`` properties
        )
        self._raw_calibration = self._cal_value

    @property
    def shunt_voltage(self) -> float:
        """The shunt voltage (between V+ and V-) in Volts (so +-.327V)"""
        # The least signficant bit is 10uV which is 0.00001 volts
        return self.raw_shunt_voltage * 0.00001

    @property
    def bus_voltage(self) -> float:
        """The bus voltage (between V- and GND) in Volts"""
        # Shift to the right 3 to drop CNVR and OVF and multiply by LSB
        # Each least signficant bit is 4mV
        return self.raw_bus_voltage * 0.004

    @property
    def current(self) -> float:
        """The current through the shunt resistor in milliamps."""
        # Sometimes a sharp load will reset the INA219, which will
        # reset the cal register, meaning CURRENT and POWER will
        # not be available ... always setting a cal
        # value even if it's an unfortunate extra step
        self._raw_calibration = self._cal_value
        # Now we can safely read the CURRENT register!
        return self.raw_current * self._current_lsb

    @property
    def power(self) -> float:
        """The power through the load in Watt."""
        # Sometimes a sharp load will reset the INA219, which will
        # reset the cal register, meaning CURRENT and POWER will
        # not be available ... always setting a cal
        # value even if it's an unfortunate extra step
        self._raw_calibration = self._cal_value
        # Now we can safely read the CURRENT register!
        return self.raw_power * self._power_lsb

    def set_calibration_32V_2A(self) -> None:  # pylint: disable=invalid-name
        """Configures to INA219 to be able to measure up to 32V and 2A of current.
        Actual max current: 3.2 A.

        .. note:: These calculations assume a 0.1 shunt ohm resistor is present
        """
        # 1. Determine max possible bus voltage, 16 or 32 V
        #self.bus_voltage_range = BusVoltageRange.RANGE_16V
        self.bus_voltage_range = BusVoltageRange.RANGE_32V

        # 2. Determine the installed shunt resistor value
        # By default, a 0.1 Ohm resistor is installed
        rshunt = 0.1  # (Resistor value in ohms)

        # 2. Estimate the max expected current
        # MaxExpected_I = 2 A

        # 3. Calculate maximum possible current for each gain value
        # MaxI_gain1_40mV = 0.04 / rshunt = 0.4 A
        # MaxI_gain2_80mV = 0.08 / rshunt = 0.8 A
        # MaxI_gain4_160mV = 0.16 / rshunt = 1.6 A
        # MaxI_gain8_320mV = 0.32 / rshunt = 3.2 A

        # 4. Evaluate whether to replace the shunt resistor
        #
        # If MaxExpected_I << MaxI_gain1_40mV, expect poor resolution.
        # If a good resolution is important for you, consider de-soldering the 0.1 Ohm shunt
        # resistor and soldering another one with a higher resistance.
        #
        # If MaxExpected_I > MaxI_gain8_320mV, consider soldering a shunt resistor with a smaller
        # resistance.
        # Either replacing the one currently in place or soldering another one on top (in parallel)
        # of the current one.
        # Remember that the maximum voltage across the shunt resistor that the INA219 chip can
        # stand is 26 V

        # 5. Select a gain for which MaxI_gainX_XXmV > MaxExpected_I
        #self.gain = Gain.DIV_1_40MV   # For 0 < MaxExpected_I < MaxI_gain1_40mV
        #self.gain = Gain.DIV_2_80MV   # For MaxI_gain1_40mV < MaxExpected_I < MaxI_gain2_80mV
        #self.gain = Gain.DIV_4_160MV  # For MaxI_gain2_80mV < MaxExpected_I < MaxI_gain4_160mV
        self.gain = Gain.DIV_8_320MV   # For MaxI_gain4_160mV < MaxExpected_I < MaxI_gain8_320mV

        # 6. Select a calibration value
        # Values below 4096 will harm the resolution
        #
        # Too high values will limit the maximum measurable current without any advantage (causing
        # an overflow to happen earlier)
        # (above 32768 for gain 1, above 16384 for gain 2, above 8192 for gain 4, above 4096 for
        # gain 8)
        #
        # Use a value different from 4096 only if you are actually calibrating the board versus a
        # reliable current measured with a better equipment.
        self.calibration = 4096

        # 7. Calculate the current LSB (least significant bit) value in mA
        # Current_LSB = 0.04096 / (calibration * rshunt) = 0.04096 / (4096 * 0.1) = 0.0001 A
        # "1000*" is for having the output in milliAmps
        self._current_lsb = 1000 * 0.04096 / (self.calibration * rshunt)

        # 8. Calculate the power LSB in W
        # Power_LSB = 20 * CurrentLSB in A = 20 * 0.0001 = 0.002 (2 mW per bit)
        # "/1000" is for converting mA to A
        self._power_lsb = 20 * self._current_lsb / 1000  # in Watts

        # 9. Compute the Maximum Power
        # Multiplying the maximum possible bus voltage (16 or 32 V) by the maximum current for the
        # chosen gain and resistor:
        # MaximumPower = 32 V * MaxI_gain8_320mV = 32 V * 3.2 A = 102.4 W

        # 10. Select the resolution
        # Increasing the bits will increase the measurement time but will give better resolution
        # Increasing the samples to be averaged will further increase the measurement time
        # resulting in less noisy measurements
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_1S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_1S

        # 11. Select the operation mode
        # With continuous mode, the a new reading will be performed as soon as the previous one
        # ended
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        # With triggered mode, a new measurement is performed each time the triggered mode is
        # configured (the following line works both as configuration and as trigger)
        #self.mode = Mode.SANDBVOLT_TRIGGERED
        # In order to know if the triggered measurement is complete, the status of
        # conversion_ready can be checked

    def set_calibration_32V_1A(self) -> None:  # pylint: disable=invalid-name
        """Configures to INA219 to be able to measure up to 32V and 1A of current.
        Actual max current: 1.6 A.

        .. note:: These calculations assume a 0.1 ohm shunt resistor is present"""
        # 1. Determine max possible bus voltage, 16 or 32 V
        #self.bus_voltage_range = BusVoltageRange.RANGE_16V
        self.bus_voltage_range = BusVoltageRange.RANGE_32V

        # 2. Determine the installed shunt resistor value
        # By default, a 0.1 Ohm resistor is installed
        rshunt = 0.1  # (Resistor value in ohms)

        # 2. Estimate the max expected current
        # MaxExpected_I = 1 A

        # 3. Calculate maximum possible current for each gain value
        # MaxI_gain1_40mV = 0.04 / rshunt = 0.4 A
        # MaxI_gain2_80mV = 0.08 / rshunt = 0.8 A
        # MaxI_gain4_160mV = 0.16 / rshunt = 1.6 A
        # MaxI_gain8_320mV = 0.32 / rshunt = 3.2 A

        # 4. Evaluate whether to replace the shunt resistor
        #
        # If MaxExpected_I << MaxI_gain1_40mV, expect poor resolution.
        # If a good resolution is important for you, consider de-soldering the 0.1 Ohm shunt
        # resistor and soldering another one with a higher resistance.
        #
        # If MaxExpected_I > MaxI_gain8_320mV, consider soldering a shunt resistor with a smaller
        # resistance.
        # Either replacing the one currently in place or soldering another one on top (in parallel)
        # of the current one.
        # Remember that the maximum voltage across the shunt resistor that the INA219 chip can
        # stand is 26 V

        # 5. Select a gain for which MaxI_gainX_XXmV > MaxExpected_I
        #self.gain = Gain.DIV_1_40MV   # For 0 < MaxExpected_I < MaxI_gain1_40mV
        #self.gain = Gain.DIV_2_80MV   # For MaxI_gain1_40mV < MaxExpected_I < MaxI_gain2_80mV
        self.gain = Gain.DIV_4_160MV   # For MaxI_gain2_80mV < MaxExpected_I < MaxI_gain4_160mV
        #self.gain = Gain.DIV_8_320MV  # For MaxI_gain4_160mV < MaxExpected_I < MaxI_gain8_320mV

        # 6. Select a calibration value
        # Values below 4096 will harm the resolution
        #
        # Too high values will limit the maximum measurable current without any advantage (causing
        # an overflow to happen earlier)
        # (above 32768 for gain 1, above 16384 for gain 2, above 8192 for gain 4, above 4096 for
        # gain 8)
        #
        # Use a value different from 4096 only if you are actually calibrating the board versus a
        # reliable current measured with a better equipment.
        self.calibration = 4096

        # 7. Calculate the current LSB (least significant bit) value in mA
        # Current_LSB = 0.04096 / (calibration * rshunt) = 0.04096 / (4096 * 0.1) = 0.0001 A
        # "1000*" is for having the output in milliAmps
        self._current_lsb = 1000 * 0.04096 / (self.calibration * rshunt)

        # 8. Calculate the power LSB in W
        # Power_LSB = 20 * CurrentLSB in A = 20 * 0.0001 = 0.002 (2 mW per bit)
        # "/1000" is for converting mA to A
        self._power_lsb = 20 * self._current_lsb / 1000  # in Watts

        # 9. Compute the Maximum Power
        # Multiplying the maximum possible bus voltage (16 or 32 V) by the maximum current for the
        # chosen gain and resistor:
        # MaximumPower = 32 V * MaxI_gain4_160mV = 32 V * 1.6 A = 51.2 W

        # 10. Select the resolution
        # Increasing the bits will increase the measurement time but will give better resolution
        # Increasing the samples to be averaged will further increase the measurement time
        # resulting in less noisy measurements
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_1S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_1S

        # 11. Select the operation mode
        # With continuous mode, the a new reading will be performed as soon as the previous one
        # ended
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        # With triggered mode, a new measurement is performed each time the triggered mode is
        # configured (the following line works both as configuration and as trigger)
        #self.mode = Mode.SANDBVOLT_TRIGGERED
        # In order to know if the triggered measurement is complete, the status of
        # conversion_ready can be checked

    def set_calibration_16V_400mA(self) -> None:  # pylint: disable=invalid-name
        """Configures to INA219 to be able to measure up to 16V and 400mA of current.

        .. note:: These calculations assume a 0.1 ohm shunt resistor is present"""
        # 1. Determine max possible bus voltage, 16 or 32 V
        self.bus_voltage_range = BusVoltageRange.RANGE_16V
        #self.bus_voltage_range = BusVoltageRange.RANGE_32V

        # 2. Determine the installed shunt resistor value
        # By default, a 0.1 Ohm resistor is installed
        rshunt = 0.1  # (Resistor value in ohms)

        # 2. Estimate the max expected current
        # MaxExpected_I = 0.4 A

        # 3. Calculate maximum possible current for each gain value
        # MaxI_gain1_40mV = 0.04 / rshunt = 0.4 A
        # MaxI_gain2_80mV = 0.08 / rshunt = 0.8 A
        # MaxI_gain4_160mV = 0.16 / rshunt = 1.6 A
        # MaxI_gain8_320mV = 0.32 / rshunt = 3.2 A

        # 4. Evaluate whether to replace the shunt resistor
        #
        # If MaxExpected_I << MaxI_gain1_40mV, expect poor resolution.
        # If a good resolution is important for you, consider de-soldering the 0.1 Ohm shunt
        # resistor and soldering another one with a higher resistance.
        #
        # If MaxExpected_I > MaxI_gain8_320mV, consider soldering a shunt resistor with a smaller
        # resistance.
        # Either replacing the one currently in place or soldering another one on top (in parallel)
        # of the current one.
        # Remember that the maximum voltage across the shunt resistor that the INA219 chip can
        # stand is 26 V

        # 5. Select a gain for which MaxI_gainX_XXmV > MaxExpected_I
        self.gain = Gain.DIV_1_40MV    # For 0 < MaxExpected_I < MaxI_gain1_40mV
        #self.gain = Gain.DIV_2_80MV   # For MaxI_gain1_40mV < MaxExpected_I < MaxI_gain2_80mV
        #self.gain = Gain.DIV_4_160MV  # For MaxI_gain2_80mV < MaxExpected_I < MaxI_gain4_160mV
        #self.gain = Gain.DIV_8_320MV  # For MaxI_gain4_160mV < MaxExpected_I < MaxI_gain8_320mV

        # 6. Select a calibration value
        # Values below 4096 will harm the resolution
        #
        # Too high values will limit the maximum measurable current without any advantage (causing
        # an overflow to happen earlier)
        # (above 32768 for gain 1, above 16384 for gain 2, above 8192 for gain 4, above 4096 for
        # gain 8)
        #
        # Use a value different from 4096 only if you are actually calibrating the board versus a
        # reliable current measured with a better equipment.
        self.calibration = 4096

        # 7. Calculate the current LSB (least significant bit) value in mA
        # Current_LSB = 0.04096 / (calibration * rshunt) = 0.04096 / (4096 * 0.1) = 0.0001 A
        # "1000*" is for having the output in milliAmps
        self._current_lsb = 1000 * 0.04096 / (self.calibration * rshunt)

        # 8. Calculate the power LSB in W
        # Power_LSB = 20 * CurrentLSB in A = 20 * 0.0001 = 0.002 (2 mW per bit)
        # "/1000" is for converting mA to A
        self._power_lsb = 20 * self._current_lsb / 1000  # in Watts

        # 9. Compute the Maximum Power
        # Multiplying the maximum possible bus voltage (16 or 32 V) by the maximum current for
        # the chosen gain and resistor:
        # MaximumPower = 16 V * MaxI_gain1_40mV = 16 V * 0.4 A = 6.4 W

        # 10. Select the resolution
        # Increasing the bits will increase the measurement time but will give better resolution
        # Increasing the samples to be averaged will further increase the measurement time
        # resulting in less noisy measurements
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_1S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_1S

        # 11. Select the operation mode
        # With continuous mode, the a new reading will be performed as soon as the previous one
        # ended
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        # With triggered mode, a new measurement is performed each time the triggered mode is
        # configured (the following line works both as configuration and as trigger)
        #self.mode = Mode.SANDBVOLT_TRIGGERED
        # In order to know if the triggered measurement is complete, the status of
        # conversion_ready can be checked

    def set_calibration_16V_5A(self) -> None:  # pylint: disable=invalid-name
        """Configures to INA219 to be able to measure up to 16V and 5000mA of current.
        Actual max current: 8 A.

        .. note:: These calculations assume a 0.02 ohm shunt resistor is present"""
        # 1. Determine max possible bus voltage, 16 or 32 V
        self.bus_voltage_range = BusVoltageRange.RANGE_16V
        #self.bus_voltage_range = BusVoltageRange.RANGE_32V

        # 2. Determine the installed shunt resistor value
        # By default, a 0.1 Ohm resistor is installed
        rshunt = 0.02  # (Resistor value in ohms)

        # 2. Estimate the max expected current
        # MaxExpected_I = 5 A

        # 3. Calculate maximum possible current for each gain value
        # MaxI_gain1_40mV = 0.04 / rshunt = 2 A
        # MaxI_gain2_80mV = 0.08 / rshunt = 4 A
        # MaxI_gain4_160mV = 0.16 / rshunt = 8 A
        # MaxI_gain8_320mV = 0.32 / rshunt = 16 A

        # 4. Evaluate whether to replace the shunt resistor
        #
        # If MaxExpected_I << MaxI_gain1_40mV, expect poor resolution.
        # If a good resolution is important for you, consider de-soldering the 0.1 Ohm shunt
        # resistor and soldering another one with a higher resistance.
        #
        # If MaxExpected_I > MaxI_gain8_320mV, consider soldering a shunt resistor with a smaller
        # resistance.
        # Either replacing the one currently in place or soldering another one on top (in parallel)
        # of the current one.
        # Remember that the maximum voltage across the shunt resistor that the INA219 chip can
        # stand is 26 V

        # 5. Select a gain for which MaxI_gainX_XXmV > MaxExpected_I
        #self.gain = Gain.DIV_1_40MV   # For 0 < MaxExpected_I < MaxI_gain1_40mV
        #self.gain = Gain.DIV_2_80MV   # For MaxI_gain1_40mV < MaxExpected_I < MaxI_gain2_80mV
        self.gain = Gain.DIV_4_160MV   # For MaxI_gain2_80mV < MaxExpected_I < MaxI_gain4_160mV
        #self.gain = Gain.DIV_8_320MV  # For MaxI_gain4_160mV < MaxExpected_I < MaxI_gain8_320mV

        # 6. Select a calibration value
        # Values below 4096 will harm the resolution
        #
        # Too high values will limit the maximum measurable current without any advantage (causing
        # an overflow to happen earlier)
        # (above 32768 for gain 1, above 16384 for gain 2, above 8192 for gain 4, above 4096 for
        # gain 8)
        #
        # Use a value different from 4096 only if you are actually calibrating the board versus a
        # reliable current measured with a better equipment.
        self.calibration = 4096

        # 7. Calculate the current LSB (least significant bit) value in mA
        # Current_LSB = 0.04096 / (calibration * rshunt) = 0.04096 / (4096 * 0.02) = 0.0005 A
        # "1000*" is for having the output in milliAmps
        self._current_lsb = 1000 * 0.04096 / (self.calibration * rshunt)

        # 8. Calculate the power LSB in W
        # Power_LSB = 20 * CurrentLSB in A = 20 * 0.0005 = 0.01 (10 mW per bit)
        # "/1000" is for converting mA to A
        self._power_lsb = 20 * self._current_lsb / 1000  # in Watts

        # 9. Compute the Maximum Power
        # Multiplying the maximum possible bus voltage (16 or 32 V) by the maximum current for
        # the chosen gain and resistor:
        # MaximumPower = 16 V * MaxI_gain4_160mV = 16 V * 8 A = 128 W

        # 10. Select the resolution
        # Increasing the bits will increase the measurement time but will give better resolution
        # Increasing the samples to be averaged will further increase the measurement time
        # resulting in less noisy measurements
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_1S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_1S

        # 11. Select the operation mode
        # With continuous mode, the a new reading will be performed as soon as the previous one
        # ended
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        # With triggered mode, a new measurement is performed each time the triggered mode is
        # configured (the following line works both as configuration and as trigger)
        #self.mode = Mode.SANDBVOLT_TRIGGERED
        # In order to know if the triggered measurement is complete, the status of
        # conversion_ready can be checked

    def set_calibration_16V_80mA(self) -> None:  # pylint: disable=invalid-name
        """Configures to INA219 to be able to measure up to 16V and 80mA of current.

        .. note:: These calculations assume a 1 ohm shunt resistor is present"""
        # 1. Determine max possible bus voltage, 16 or 32 V
        self.bus_voltage_range = BusVoltageRange.RANGE_16V
        #self.bus_voltage_range = BusVoltageRange.RANGE_32V

        # 2. Determine the installed shunt resistor value
        # By default, a 0.1 Ohm resistor is installed
        rshunt = 1  # (Resistor value in ohms)

        # 2. Estimate the max expected current
        # MaxExpected_I = 0.08 A

        # 3. Calculate maximum possible current for each gain value
        # MaxI_gain1_40mV = 0.04 / rshunt = 0.04 A
        # MaxI_gain2_80mV = 0.08 / rshunt = 0.08 A
        # MaxI_gain4_160mV = 0.16 / rshunt = 0.16 A
        # MaxI_gain8_320mV = 0.32 / rshunt = 0.32 A

        # 4. Evaluate whether to replace the shunt resistor
        #
        # If MaxExpected_I << MaxI_gain1_40mV, expect poor resolution.
        # If a good resolution is important for you, consider de-soldering the 0.1 Ohm shunt
        # resistor and soldering another one with a higher resistance.
        #
        # If MaxExpected_I > MaxI_gain8_320mV, consider soldering a shunt resistor with a smaller
        # resistance.
        # Either replacing the one currently in place or soldering another one on top (in parallel)
        #  of the current one.
        # Remember that the maximum voltage across the shunt resistor that the INA219 chip can
        # stand is 26 V

        # 5. Select a gain for which MaxI_gainX_XXmV > MaxExpected_I
        #self.gain = Gain.DIV_1_40MV    # For 0 < MaxExpected_I < MaxI_gain1_40mV
        self.gain = Gain.DIV_2_80MV   # For MaxI_gain1_40mV < MaxExpected_I < MaxI_gain2_80mV
        #self.gain = Gain.DIV_4_160MV  # For MaxI_gain2_80mV < MaxExpected_I < MaxI_gain4_160mV
        #self.gain = Gain.DIV_8_320MV  # For MaxI_gain4_160mV < MaxExpected_I < MaxI_gain8_320mV

        # 6. Select a calibration value
        # Values below 4096 will harm the resolution
        #
        # Too high values will limit the maximum measurable current without any advantage (causing
        # an overflow to happen earlier)
        # (above 32768 for gain 1, above 16384 for gain 2, above 8192 for gain 4, above 4096 for
        # gain 8)
        #
        # Use a value different from 4096 only if you are actually calibrating the board versus a
        # reliable current measured with a better equipment.
        self.calibration = 4096

        # 7. Calculate the current LSB (least significant bit) value in mA
        # Current_LSB = 0.04096 / (calibration * rshunt) = 0.04096 / (4096 * 1) = 0.00001 A
        # "1000*" is for having the output in milliAmps
        self._current_lsb = 1000 * 0.04096 / (self.calibration * rshunt)

        # 8. Calculate the power LSB in W
        # Power_LSB = 20 * Current_LSB in A = 20 * 0.00001 = 0.0002 W (0.2 mW per bit)
        # "/1000" is for converting mA to A
        self._power_lsb = 20 * self._current_lsb / 1000  # in Watts

        # 9. Compute the Maximum Power
        # Multiplying the maximum possible bus voltage (16 or 32 V) by the maximum current for
        # the chosen gain and resistor:
        # MaximumPower = 16 V * MaxI_gain2_80mV = 16 V * 0.08 A = 1.28 W

        # 10. Select the resolution
        # Increasing the bits will increase the measurement time but will give better resolution
        # Increasing the samples to be averaged will further increase the measurement time
        # resulting in less noisy measurements
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_1S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_1S

        # 11. Select the operation mode
        # With continuous mode, the a new reading will be performed as soon as the previous one
        # ended
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        # With triggered mode, a new measurement is performed each time the triggered mode is
        # configured (the following line works both as configuration and as trigger)
        #self.mode = Mode.SANDBVOLT_TRIGGERED
        # In order to know if the triggered measurement is complete, the status of
        # conversion_ready can be checked

    def set_calibration_16V_4mA(self) -> None:  # pylint: disable=invalid-name
        """Configures to INA219 to be able to measure up to 16V and 4mA of current.

        .. note:: These calculations assume a 10 ohm shunt resistor is present"""
        # 1. Determine max possible bus voltage, 16 or 32 V
        self.bus_voltage_range = BusVoltageRange.RANGE_16V
        #self.bus_voltage_range = BusVoltageRange.RANGE_32V

        # 2. Determine the installed shunt resistor value
        # By default, a 0.1 Ohm resistor is installed
        rshunt = 10  # (Resistor value in ohms)

        # 2. Estimate the max expected current
        # MaxExpected_I = 0.004 A

        # 3. Calculate maximum possible current for each gain value
        # MaxI_gain1_40mV = 0.04 / rshunt = 0.004 A
        # MaxI_gain2_80mV = 0.08 / rshunt = 0.008 A
        # MaxI_gain4_160mV = 0.16 / rshunt = 0.016 A
        # MaxI_gain8_320mV = 0.32 / rshunt = 0.032 A

        # 4. Evaluate whether to replace the shunt resistor
        #
        # If MaxExpected_I << MaxI_gain1_40mV, expect poor resolution.
        # If a good resolution is important for you, consider de-soldering the 0.1 Ohm shunt
        # resistor and soldering another one with a higher resistance.
        #
        # If MaxExpected_I > MaxI_gain8_320mV, consider soldering a shunt resistor with a smaller
        # resistance.
        # Either replacing the one currently in place or soldering another one on top (in parallel)
        # of the current one.
        # Remember that the maximum voltage across the shunt resistor that the INA219 chip can
        # stand is 26 V

        # 5. Select a gain for which MaxI_gainX_XXmV > MaxExpected_I
        self.gain = Gain.DIV_1_40MV    # For 0 < MaxExpected_I < MaxI_gain1_40mV
        #self.gain = Gain.DIV_2_80MV   # For MaxI_gain1_40mV < MaxExpected_I < MaxI_gain2_80mV
        #self.gain = Gain.DIV_4_160MV  # For MaxI_gain2_80mV < MaxExpected_I < MaxI_gain4_160mV
        #self.gain = Gain.DIV_8_320MV  # For MaxI_gain4_160mV < MaxExpected_I < MaxI_gain8_320mV

        # 6. Select a calibration value
        # Values below 4096 will harm the resolution
        #
        # Too high values will limit the maximum measurable current without any advantage (causing
        # an overflow to happen earlier)
        # (above 32768 for gain 1, above 16384 for gain 2, above 8192 for gain 4, above 4096 for
        # gain 8)
        #
        # Use a value different from 4096 only if you are actually calibrating the board versus a
        # reliable current measured with a better equipment.
        self.calibration = 4096

        # 7. Calculate the current LSB (least significant bit) value in mA
        # Current_LSB = 0.04096 / (calibration * rshunt) = 0.04096 / (4096 * 10) = 0.000001 A
        # "1000*" is for having the output in milliAmps
        self._current_lsb = 1000 * 0.04096 / (self.calibration * rshunt)

        # 8. Calculate the power LSB in W
        # Power_LSB = 20 * Current_LSB in A = 20 * 0.000001 = 0.00002 W (0.02 mW per bit)
        # "/1000" is for converting mA to A
        self._power_lsb = 20 * self._current_lsb / 1000  # in Watts

        # 9. Compute the Maximum Power
        # Multiplying the maximum possible bus voltage (16 or 32 V) by the maximum current for
        # the chosen gain and resistor:
        # MaximumPower = 16 V * MaxI_gain1_40mV = 16 V * 0.004 A = 0.064 W

        # 10. Select the resolution
        # Increasing the bits will increase the measurement time but will give better resolution
        # Increasing the samples to be averaged will further increase the measurement time
        # resulting in less noisy measurements
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_1S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_1S

        # 11. Select the operation mode
        # With continuous mode, the a new reading will be performed as soon as the previous one
        # ended
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        # With triggered mode, a new measurement is performed each time the triggered mode is
        # configured (the following line works both as configuration and as trigger)
        #self.mode = Mode.SANDBVOLT_TRIGGERED
        # In order to know if the triggered measurement is complete, the status of
        # conversion_ready can be checked
