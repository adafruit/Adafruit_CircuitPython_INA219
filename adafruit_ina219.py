# The MIT License (MIT)
#
# Copyright (c) 2017 Dean Miller for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_ina219`
====================================================

TODO(description)

* Author(s): Dean Miller
"""

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice

"""
BITS
"""
INA219_READ                            = const(0x01)


"""
CONFIG REGISTER (R/W)
"""
INA219_REG_CONFIG                      = const(0x00)
INA219_CONFIG_RESET                    = const(0x8000)  # Reset Bit

INA219_CONFIG_BVOLTAGERANGE_MASK       = const(0x2000)  # Bus Voltage Range Mask
INA219_CONFIG_BVOLTAGERANGE_16V        = const(0x0000)  # 0-16V Range
INA219_CONFIG_BVOLTAGERANGE_32V        = const(0x2000)  # 0-32V Range

INA219_CONFIG_GAIN_MASK                = const(0x1800)  # Gain Mask
INA219_CONFIG_GAIN_1_40MV              = const(0x0000)  # Gain 1, 40mV Range
INA219_CONFIG_GAIN_2_80MV              = const(0x0800)  # Gain 2, 80mV Range
INA219_CONFIG_GAIN_4_160MV             = const(0x1000)  # Gain 4, 160mV Range
INA219_CONFIG_GAIN_8_320MV             = const(0x1800)  # Gain 8, 320mV Range

INA219_CONFIG_BADCRES_MASK             = const(0x0780)  # Bus ADC Resolution Mask
INA219_CONFIG_BADCRES_9BIT             = const(0x0080)  # 9-bit bus res = 0..511
INA219_CONFIG_BADCRES_10BIT            = const(0x0100)  # 10-bit bus res = 0..1023
INA219_CONFIG_BADCRES_11BIT            = const(0x0200)  # 11-bit bus res = 0..2047
INA219_CONFIG_BADCRES_12BIT            = const(0x0400)  # 12-bit bus res = 0..4097

INA219_CONFIG_SADCRES_MASK             = const(0x0078)  # Shunt ADC Resolution and Averaging Mask
INA219_CONFIG_SADCRES_9BIT_1S_84US     = const(0x0000)  # 1 x 9-bit shunt sample
INA219_CONFIG_SADCRES_10BIT_1S_148US   = const(0x0008)  # 1 x 10-bit shunt sample
INA219_CONFIG_SADCRES_11BIT_1S_276US   = const(0x0010)  # 1 x 11-bit shunt sample
INA219_CONFIG_SADCRES_12BIT_1S_532US   = const(0x0018)  # 1 x 12-bit shunt sample
INA219_CONFIG_SADCRES_12BIT_2S_1060US  = const(0x0048)	 # 2 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_4S_2130US  = const(0x0050)  # 4 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_8S_4260US  = const(0x0058)  # 8 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_16S_8510US = const(0x0060)  # 16 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_32S_17MS   = const(0x0068)  # 32 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_64S_34MS   = const(0x0070)  # 64 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_128S_69MS  = const(0x0078)  # 128 x 12-bit shunt samples averaged together

INA219_CONFIG_MODE_MASK                = const(0x0007)  # Operating Mode Mask
INA219_CONFIG_MODE_POWERDOWN           = const(0x0000)
INA219_CONFIG_MODE_SVOLT_TRIGGERED     = const(0x0001)
INA219_CONFIG_MODE_BVOLT_TRIGGERED     = const(0x0002)
INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED = const(0x0003)
INA219_CONFIG_MODE_ADCOFF              = const(0x0004)
INA219_CONFIG_MODE_SVOLT_CONTINUOUS    = const(0x0005)
INA219_CONFIG_MODE_BVOLT_CONTINUOUS    = const(0x0006)
INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS = const(0x0007)	


"""
SHUNT VOLTAGE REGISTER (R)
"""
INA219_REG_SHUNTVOLTAGE                = const(0x01)


"""
BUS VOLTAGE REGISTER (R)
"""
INA219_REG_BUSVOLTAGE                  = const(0x02)


"""
POWER REGISTER (R)
"""
INA219_REG_POWER                       = const(0x03)


"""
CURRENT REGISTER (R)
"""
INA219_REG_CURRENT                     = const(0x04)


"""
CALIBRATION REGISTER (R/W)
"""
INA219_REG_CALIBRATION                 = const(0x05)


class INA219:

	def __init__(self, i2c, addr=0x40):
		self.i2c_device = I2CDevice(i2c, addr)


		self.i2c_addr = addr
		self.current_divider_mA = 0
		self.power_divider_mW = 0

		# Set chip to known config values to start
	  	self.set_calibration_32V_2A()


	def write_register (self, reg, value):
		seq = bytearray([reg, (value >> 8) & 0xFF, value & 0xFF])
		with self.i2c_device as i2c:
			self.i2c_device.write(seq)

	def read_register(self, reg):
		buf = bytearray(3)
		buf[0] = reg
		with self.i2c_device as i2c:
			i2c.write(buf, end=1, stop=False)
			i2c.read_into(buf, start=1)

	  	value = (buf[1] << 8) | (buf[2])
	  	return value

	"""
	 
		@brief  Configures to INA219 to be able to measure up to 32V and 2A
				of current.  Each unit of current corresponds to 100uA, and
				each unit of power corresponds to 2mW. Counter overflow
				occurs at 3.2A.
				
		@note   These calculations assume a 0.1 ohm resistor is present
	
	"""
	def set_calibration_32V_2A(self):
		# By default we use a pretty huge range for the input voltage,
		# which probably isn't the most appropriate choice for system
		# that don't use a lot of power.  But all of the calculations
		# are shown below if you want to change the settings.  You will
		# also need to change any relevant register settings, such as
		# setting the VBUS_MAX to 16V instead of 32V, etc.

		# VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
		# VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
		# RSHUNT = 0.1               (Resistor value in ohms)

		# 1. Determine max possible current
		# MaxPossible_I = VSHUNT_MAX / RSHUNT
		# MaxPossible_I = 3.2A

		# 2. Determine max expected current
		# MaxExpected_I = 2.0A

		# 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
		# MinimumLSB = MaxExpected_I/32767
		# MinimumLSB = 0.000061              (61uA per bit)
		# MaximumLSB = MaxExpected_I/4096
		# MaximumLSB = 0,000488              (488uA per bit)

		# 4. Choose an LSB between the min and max values
		#    (Preferrably a roundish number close to MinLSB)
		# CurrentLSB = 0.0001 (100uA per bit)

		# 5. Compute the calibration register
		# Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
		# Cal = 4096 (0x1000)

		self.cal_value = 4096

		# 6. Calculate the power LSB
		# PowerLSB = 20 * CurrentLSB
		# PowerLSB = 0.002 (2mW per bit)

		# 7. Compute the maximum current and shunt voltage values before overflow
		#
		# Max_Current = Current_LSB * 32767
		# Max_Current = 3.2767A before overflow
		#
		# If Max_Current > Max_Possible_I then
		#    Max_Current_Before_Overflow = MaxPossible_I
		# Else
		#    Max_Current_Before_Overflow = Max_Current
		# End If
		#
		# Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
		# Max_ShuntVoltage = 0.32V
		#
		# If Max_ShuntVoltage >= VSHUNT_MAX
		#    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
		# Else
		#    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
		# End If

		# 8. Compute the Maximum Power
		# MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
		# MaximumPower = 3.2 * 32V
		# MaximumPower = 102.4W

		# Set multipliers to convert raw current/power values
		self.current_divider_mA = 10  # Current LSB = 100uA per bit (1000/100 = 10)
		self.power_divider_mW = 2     # Power LSB = 1mW per bit (2/1)

		# Set Calibration register to 'Cal' calculated above	
		self.write_register(INA219_REG_CALIBRATION, self.cal_value)

		# Set Config register to take into account the settings above
		config = INA219_CONFIG_BVOLTAGERANGE_32V | \
			INA219_CONFIG_GAIN_8_320MV | \
			INA219_CONFIG_BADCRES_12BIT | \
			INA219_CONFIG_SADCRES_12BIT_1S_532US | \
			INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS
		self.write_register(INA219_REG_CONFIG, config)


	"""
	 
		@brief  Configures to INA219 to be able to measure up to 32V and 1A
				of current.  Each unit of current corresponds to 40uA, and each
				unit of power corresponds to 800�W. Counter overflow occurs at
				1.3A.
				
		@note   These calculations assume a 0.1 ohm resistor is present
	
	"""
	def set_calibration_32V_1A(self):
	  # By default we use a pretty huge range for the input voltage,
	  # which probably isn't the most appropriate choice for system
	  # that don't use a lot of power.  But all of the calculations
	  # are shown below if you want to change the settings.  You will
	  # also need to change any relevant register settings, such as
	  # setting the VBUS_MAX to 16V instead of 32V, etc.

	  # VBUS_MAX = 32V		(Assumes 32V, can also be set to 16V)
	  # VSHUNT_MAX = 0.32	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
	  # RSHUNT = 0.1			(Resistor value in ohms)

	  # 1. Determine max possible current
	  # MaxPossible_I = VSHUNT_MAX / RSHUNT
	  # MaxPossible_I = 3.2A

	  # 2. Determine max expected current
	  # MaxExpected_I = 1.0A

	  # 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	  # MinimumLSB = MaxExpected_I/32767
	  # MinimumLSB = 0.0000305             (30.5�A per bit)
	  # MaximumLSB = MaxExpected_I/4096
	  # MaximumLSB = 0.000244              (244�A per bit)

	  # 4. Choose an LSB between the min and max values
	  #    (Preferrably a roundish number close to MinLSB)
	  # CurrentLSB = 0.0000400 (40�A per bit)

	  # 5. Compute the calibration register
	  # Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
	  # Cal = 10240 (0x2800)

	  self.cal_value = 10240
	  
	  # 6. Calculate the power LSB
	  # PowerLSB = 20 * CurrentLSB
	  # PowerLSB = 0.0008 (800�W per bit)

	  # 7. Compute the maximum current and shunt voltage values before overflow
	  #
	  # Max_Current = Current_LSB * 32767
	  # Max_Current = 1.31068A before overflow
	  #
	  # If Max_Current > Max_Possible_I then
	  #    Max_Current_Before_Overflow = MaxPossible_I
	  # Else
	  #    Max_Current_Before_Overflow = Max_Current
	  # End If
	  #
	  # ... In this case, we're good though since Max_Current is less than MaxPossible_I
	  #
	  # Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	  # Max_ShuntVoltage = 0.131068V
	  #
	  # If Max_ShuntVoltage >= VSHUNT_MAX
	  #    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	  # Else
	  #    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	  # End If

	  # 8. Compute the Maximum Power
	  # MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	  # MaximumPower = 1.31068 * 32V
	  # MaximumPower = 41.94176W

	  # Set multipliers to convert raw current/power values
	  self.current_divider_mA = 25      # Current LSB = 40uA per bit (1000/40 = 25)
	  self.power_divider_mW = 1         # Power LSB = 800�W per bit

	  # Set Calibration register to 'Cal' calculated above	
	  self.write_register(INA219_REG_CALIBRATION, self.cal_value)

	  # Set Config register to take into account the settings above
	  config = INA219_CONFIG_BVOLTAGERANGE_32V | \
						INA219_CONFIG_GAIN_8_320MV | \
						INA219_CONFIG_BADCRES_12BIT | \
						INA219_CONFIG_SADCRES_12BIT_1S_532US | \
						INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS
	  self.write_register(INA219_REG_CONFIG, config)


	def set_calibration_16V_400mA(self):
	  
	  # Calibration which uses the highest precision for 
	  # current measurement (0.1mA), at the expense of 
	  # only supporting 16V at 400mA max.

	  # VBUS_MAX = 16V
	  # VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
	  # RSHUNT = 0.1               (Resistor value in ohms)
	  
	  # 1. Determine max possible current
	  # MaxPossible_I = VSHUNT_MAX / RSHUNT
	  # MaxPossible_I = 0.4A

	  # 2. Determine max expected current
	  # MaxExpected_I = 0.4A
	  
	  # 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	  # MinimumLSB = MaxExpected_I/32767
	  # MinimumLSB = 0.0000122              (12uA per bit)
	  # MaximumLSB = MaxExpected_I/4096
	  # MaximumLSB = 0.0000977              (98uA per bit)
	  
	  # 4. Choose an LSB between the min and max values
	  #    (Preferrably a roundish number close to MinLSB)
	  # CurrentLSB = 0.00005 (50uA per bit)
	  
	  # 5. Compute the calibration register
	  # Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
	  # Cal = 8192 (0x2000)

	  self.cal_value = 8192

	  # 6. Calculate the power LSB
	  # PowerLSB = 20 * CurrentLSB
	  # PowerLSB = 0.001 (1mW per bit)
	  
	  # 7. Compute the maximum current and shunt voltage values before overflow
	  #
	  # Max_Current = Current_LSB * 32767
	  # Max_Current = 1.63835A before overflow
	  #
	  # If Max_Current > Max_Possible_I then
	  #    Max_Current_Before_Overflow = MaxPossible_I
	  # Else
	  #    Max_Current_Before_Overflow = Max_Current
	  # End If
	  #
	  # Max_Current_Before_Overflow = MaxPossible_I
	  # Max_Current_Before_Overflow = 0.4
	  #
	  # Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	  # Max_ShuntVoltage = 0.04V
	  #
	  # If Max_ShuntVoltage >= VSHUNT_MAX
	  #    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	  # Else
	  #    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	  # End If
	  #
	  # Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	  # Max_ShuntVoltage_Before_Overflow = 0.04V
	  
	  # 8. Compute the Maximum Power
	  # MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	  # MaximumPower = 0.4 * 16V
	  # MaximumPower = 6.4W
	  
	  # Set multipliers to convert raw current/power values
	  self.current_divider_mA = 20  # Current LSB = 50uA per bit (1000/50 = 20)
	  self.power_divider_mW = 1     # Power LSB = 1mW per bit

	  # Set Calibration register to 'Cal' calculated above 
	  self.write_register(INA219_REG_CALIBRATION, self.cal_value)
	  
	  # Set Config register to take into account the settings above
	  config = INA219_CONFIG_BVOLTAGERANGE_16V | \
						INA219_CONFIG_GAIN_1_40MV | \
						INA219_CONFIG_BADCRES_12BIT | \
						INA219_CONFIG_SADCRES_12BIT_1S_532US | \
						INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS
	  self.write_register(INA219_REG_CONFIG, config) 


	"""
	 
		@brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
	
	"""
	def get_bus_voltage_raw(self):
	  value = self.read_register(INA219_REG_BUSVOLTAGE)

	  # Shift to the right 3 to drop CNVR and OVF and multiply by LSB
	  value = (self.to_signed(value >> 3) * 4)
	  return value


	"""
	 
		@brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
	
	"""
	def get_shunt_voltage_raw(self):
	  return self.to_signed(self.read_register(INA219_REG_SHUNTVOLTAGE))


	"""
	 
		@brief  Gets the raw current value (16-bit signed integer, so +-32767)
	
	"""
	def get_current_raw(self):

	  # Sometimes a sharp load will reset the INA219, which will
	  # reset the cal register, meaning CURRENT and POWER will
	  # not be available ... athis by always setting a cal
	  # value even if it's an unfortunate extra step
	  self.write_register(INA219_REG_CALIBRATION, self.cal_value)

	  # Now we can safely read the CURRENT register!
	  return self.to_signed(self.read_register(INA219_REG_CURRENT))

	 
	"""
	 
		@brief  Gets the shunt voltage in mV (so +-327mV)
	
	"""
	def get_shunt_voltage_mV(self):
	  value = self.get_shunt_voltage_raw()
	  return value * 0.01


	"""
	 
		@brief  Gets the shunt voltage in volts
	
	"""
	def get_bus_voltage_V(self):
	  value = self.get_bus_voltage_raw()
	  return value * 0.001


	"""
	 
		@brief  Gets the current value in mA, taking into account the
				config settings and current LSB
	
	"""
	def get_current_mA(self):
	  valueDec = self.get_current_raw()
	  valueDec /= self.current_divider_mA
	  return valueDec

	def to_signed(self, num):
		v = num
		if(num > 0x7FFF):
			v -= 0x10000
		return v
