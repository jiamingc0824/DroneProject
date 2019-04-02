#!/usr/bin/env python
from dronekit import connect, VehicleMode, LocationGlobalRelative, Battery
from pymavlink import mavutil
import time

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default="com4")
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=False)
vehicle.wait_ready(True, timeout=240)


while True:
    print "Battery:", vehicle.battery
    print "Home location:", vehicle.home_location
    time.sleep(1)


vehicle.close()
