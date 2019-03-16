import time

from board import SCL, SDA
import busio

import adafruit_ina219

i2c_bus = busio.I2C(SCL, SDA)

ina219 = adafruit_ina219.INA219(i2c_bus)

# change configuration to use 32 samples averaging for both bus voltage and shunt voltage
ina219.bus_adc_res = 0x0D       # BADCRES_12BIT_32S_17MS
ina219.shunt_adc_res = 0x0D     # SADCRES_12BIT_32S_17MS
ina219.bus_voltage_range = 0    # BVOLTAGERANGE_16V

print("ina219 test")

while True:
    bus_voltage = ina219.bus_voltage
    shunt_voltage = ina219.shunt_voltage
    current = ina219.current

    # INA219 measure bus voltage on the load side. So power supply voltage = busVoltage+shuntVoltage
    print("PSU Voltage:   {:6.3f} V".format(bus_voltage + shunt_voltage))
    print("Shunt Voltage: {:9.6f} V".format(shunt_voltage))
    print("Load Voltage:  {:6.3f} V".format(bus_voltage))
    print("Current:       {:9.6f} A".format(current/1000))
    print("")

    time.sleep(2)
