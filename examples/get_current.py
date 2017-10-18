from board import *
import time
import busio

import adafruit_ina219

myI2C = busio.I2C(SCL, SDA)

ina219 = adafruit_ina219.INA219(myI2C)

print("ina219 test")

while(1):
	shuntvoltage = 0
	busvoltage = 0
	current_mA = 0
	loadvoltage = 0

	shuntvoltage = ina219.get_shunt_voltage_mV()
	busvoltage = ina219.get_bus_voltage_V()
	current_mA = ina219.get_current_mA()
	loadvoltage = busvoltage + (shuntvoltage / 1000)

	print("Bus Voltage:   " + str(busvoltage) + " V")
	print("Shunt Voltage: " + str(shuntvoltage) + " mV")
	print("Load Voltage:  " + str(loadvoltage) + " V")
	print("Current:       " + str(current_mA) + " mA")
	print("")

	time.sleep(2)