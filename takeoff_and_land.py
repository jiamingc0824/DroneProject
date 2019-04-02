#!/usr/bin/env python

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default="com4")
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=False)
vehicle.wait_ready(True, timeout=300)


# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):
    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print "Is Vehicle Armable? ", vehicle.is_armable
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print "Vehicle is arming"
        time.sleep(1)

    print "Taking off to y=", aTargetAltitude, "meter"
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    # Check that vehicle has reached takeoff altitude
    while True:
        print "Current Altitude: ", vehicle.location.global_relative_frame.alt
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print "Reached target altitude of ", aTargetAltitude, "meter"
            break
        time.sleep(1)


# Initialize the takeoff sequence to 1m
arm_and_takeoff(1)

print("Take off complete")

# Hover for 10 seconds
time.sleep(10)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()
