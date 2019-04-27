#!/usr/bin/env python

from dronekit import connect, VehicleMode, LocationGlobalRelative, Battery
from pymavlink import mavutil
from twisted.internet import reactor, protocol, task
from twisted.internet.protocol import Factory
import time
import keyboard
import argparse
import math
import threading
import socket
import datetime

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default="/dev/ttyUSB0")
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=False)
vehicle.wait_ready(True, timeout=240)
MSG = vehicle.message_factory.set_position_target_local_ned_encode(
    0,
    0, 0,
    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
    0b0000111111000111,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0)
loop = True
stop = True


def ArmAndTakeoff(aTargetAltitude):
    """
    Function to arm and then takeoff to a user specified altitude
    ArmAndTakeoff(aTargetAltitude)

    :param aTargetAltitude : Target altitude of the drone after taking off
    """
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


def UpdateVelocity(velocity_x, velocity_y, velocity_z, duration=1):
    """
    Function to send velocity component to the drone
    UpdateVelocity(velocity_x, velocity_y, velocity_z, duration)

    :param velocity_x  : Positive = Front, Negative = Back
    :param velocity_y  : Positive = Left, Negative = Right
    :param velocity_z  : Positive = Down, Negative = Up
    :param duration    : Duration of the message
    """
    global MSG
    MSG = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame, currently set to vehicle heading
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    for i in range(duration):
        vehicle.send_mavlink(MSG)
        print "Heading:", vehicle.heading
        print "Local:", vehicle.location.local_frame
        print "Global ", vehicle.location.global_relative_frame
        time.sleep(1)


def SendLocation(front, right, down):
    """
    Function to send displacement component to the drone
    SendLocation(front, right, down)

    :param front  : Positive = Front, Negative = Back
    :param right   : Positive = Right, Negative = Left
    :param down   : Positive = Down, Negative = Up
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame, currently set to vehicle heading
        0b0000111111111000,  # type_mask (only position enabled)
        front, right, down,  # x, y, z positions (not used)
        0, 0, 0,  # x, y, z velocity in m/s (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)


def ConditionYaw(_heading, rotation=1, relative=False):
    """
    Function to update the yaw of the drone based on the relative parameter
    ConditionYaw(_heading, rotation, relative)

    :param _heading  : Depends on relative, if relative = True, Take Current Heading of Drone + _heading,
                       else, absolute angle of cardinal direction, i.e. North = 0, East = 90
    :param rotation  : CW = 1, CCW = 0 (default = 1)
    :param relative  : Relative = False, Absolute = True (default False)
    """
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    _yaw = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        _heading,  # param 1, yaw in degrees
        0,  # yaw speed deg/s, not implemented in mavlink
        rotation,  # Direction: -1 ccw, 1 cw
        is_relative,  # Relative offset 1, absolute angle 0
        0, 0, 0)  # Not Used
    vehicle.send_mavlink(_yaw)
    while not (abs(vehicle.heading - _heading) <= 0.05):
        # Check if vehicle heading is target heading before moving on
        print "Heading:", vehicle.heading
        time.sleep(1)


def InitializeLanding(*args):
    """
    Emergency Landing Function
    """
    global loop
    loop = False
    UpdateVelocity(0, 0, 0)
    vehicle.mode = VehicleMode("LAND")
    vehicle.send_mavlink(MSG)
    time.sleep(10)


def StopMovement(*args):
    """
    Stop the Drone From Moving
    """
    global stop
    stop = False
    UpdateVelocity(0, 0, 0)
    vehicle.send_mavlink(MSG)


def WriteData():
    f = open("YawandBattery.txt", "a+")
    currentDT = datetime.datetime.now()
    f.write(str(currentDT) + "\n")
    f.write("Heading: " + str(vehicle.heading) + "\n")
    f.write("Battery: " + str(vehicle.battery) + "\n")


ArmAndTakeoff(2)
# Initialize the takeoff sequence to 2m
time.sleep(5)
# Hover for 5 seconds
UpdateVelocity(0, 0, 0)
vehicle.send_mavlink(MSG)
if vehicle.heading > 180:
    ConditionYaw(180, -1, True)
elif vehicle.heading <180:
    ConditionYaw(180, 1, True)
time.sleep(5)

for x in range(10):
    WriteData()
    print "Heading", vehicle.heading
    print vehicle.battery
    time.sleep(1)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")
time.sleep(10)
vehicle.close
