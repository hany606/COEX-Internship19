import VL53L1X
from time import sleep

tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open() # Initialise the i2c bus and configure the sensor
tof.start_ranging(2) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
while(True):
	distance_in_mm = tof.get_distance() # Grab the range in mm
	print(distance_in_mm)
	sleep(0.5)
tof.stop_ranging() # Stop ranging
