# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Sample code and test for adafruit_ina219"""

import time

import board

from adafruit_ina219 import INA219, ADCResolution, BusVoltageRange

i2c_bus = board.I2C()  # uses board.SCL and board.SDA
# i2c_bus = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

ina219 = INA219(i2c_bus)

print("ina219 test")

# display some of the advanced field (just to test)
print("Config register:")
print("  bus_voltage_range:    0x%1X" % ina219.bus_voltage_range)
print("  gain:                 0x%1X" % ina219.gain)
print("  bus_adc_resolution:   0x%1X" % ina219.bus_adc_resolution)
print("  shunt_adc_resolution: 0x%1X" % ina219.shunt_adc_resolution)
print("  mode:                 0x%1X" % ina219.mode)
print("")

# optional : change configuration to use 32 samples averaging for both bus voltage and shunt voltage
ina219.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
ina219.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
# optional : change voltage range to 16V
ina219.bus_voltage_range = BusVoltageRange.RANGE_16V

# measure and display loop
while True:
    bus_voltage = ina219.bus_voltage  # voltage on V- (load side)
    shunt_voltage = ina219.shunt_voltage  # voltage between V+ and V- across the shunt
    current = ina219.current  # current in mA
    power = ina219.power  # power in watts

    # INA219 measure bus voltage on the load side. So PSU voltage = bus_voltage + shunt_voltage
    print(f"Voltage (VIN+) : {bus_voltage + shunt_voltage:6.3f}   V")
    print(f"Voltage (VIN-) : {bus_voltage:6.3f}   V")
    print(f"Shunt Voltage  : {shunt_voltage:8.5f} V")
    print(f"Shunt Current  : {current / 1000:7.4f}  A")
    print(f"Power Calc.    : {bus_voltage * (current / 1000):8.5f} W")
    print(f"Power Register : {power:6.3f}   W")
    print("")

    # Check internal calculations haven't overflowed (doesn't detect ADC overflows)
    if ina219.overflow:
        print("Internal Math Overflow Detected!")
        print("")

    time.sleep(2)
