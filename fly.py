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

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default="com4")
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=False)
vehicle.wait_ready(True, timeout=240)
Target_Heading = 75  # Target heading of drone
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
examplecoordinate = [2.1, 234.3336]
PreviousPosition = [0, 0]
distance = []
heading = []
ORB = True
ORBComplete = False
vehicle.parameters['WPNAV_SPEED'] = 100


def readmission(aFileName):
    """
    Load a mission from a file into a list.

    :param aFileName : Filename of the waypoints
    """
    print "Reading mission from file: %s\n" % aFileName
    missionlist = []
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i == 0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray = line.split('\t')
                lat = float(linearray[8])  # Latitude of Waypoint
                lon = float(linearray[9])  # Longitude of Waypoint
                missionlist.append("{}@{}".format(lat, lon))
    return missionlist


def HeadingParser(string):
    return map(int, string)


def PathParser(string):
    """
    Parse path data from service layer
    """
    stringarray = []
    distance = []
    heading = []
    bearing = []
    stringarray = string.split('@')
    altitude = map(int, stringarray[0])
    distance = map(float, stringarray[1][1:len(stringarray[1]) - 1].split('|'))
    heading = map(int, stringarray[2][1:len(stringarray[2]) - 1].split('|'))
    bearing = stringarray[3][1:len(stringarray[3]) - 1].split('|')
    return altitude, distance, heading, bearing


def HeaderExtractor(string):
    """
    Extract message header obtained from TCP
    """
    stringarray = []
    stringarray = string.split("://")
    return stringarray[0], stringarray[1]


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


def SendLocation(front, left, down):
    """
    Function to send displacement component to the drone
    SendLocation(front, left, down)

    :param front  : Positive = Front, Negative = Back
    :param left   : Positive = Left, Negative = Right
    :param down   : Positive = Down, Negative = Up
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame, currently set to vehicle heading
        0b0000111111111000,  # type_mask (only position enabled)
        front, left, down,  # x, y, z positions (not used)
        0, 0, 0,  # x, y, z velocity in m/s (not used)
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used)
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
    while not (abs(vehicle.heading - _heading) <= 1):
        # Check if vehicle heading is target heading before moving on
        print "Heading:", vehicle.heading
        time.sleep(1)


def CaluculateRemainingDistance(CurrentN, CurrentE, Target, PreviousN=0, PreviousE=0):
    """
    Calculate the remaining distance that the drone needs to travel

    :param CurrentN   : Current Location of the drone in the North direction relative to home (Negative = South)
    :param CurrentE   : Current Location of the drone in the East direction relative to home (Negative = West)
    :param Target     : Target distance that the drone needs to travel relative to the heading
    :param PreviousN  : Previous Location of the drone in the North direction relative to home (Negative = South)
    :param PreviousE  : Previous Location of the drone in the East direction relative to home (Negative = West)
    """
    return Target - math.sqrt(abs(((CurrentN - PreviousN) ** 2) + ((CurrentE - PreviousE) ** 2)))


def GetStraight(Current, Target):
    Angle = abs(Current - Target)
    if Angle < 3 or Angle > 357:
        return True
    else:
        return False


def GetLeftOrRight(Current, Target):
    Left = Current - 90
    if Left < 0:
        Left += 360
    Right = Current + 90
    if Right > 360:
        Right -= 360
    if abs(Left - Target) < 5:
        return True
    elif abs(Right - Target) < 5:
        return False


def InitializeLanding(*args):
    """
    Emergency Landing Function
    """
    global loop
    loop = False
    UpdateVelocity(0, 0, 0)
    vehicle.send_mavlink(MSG)
    vehicle.mode = VehicleMode("LAND")
    time.sleep(10)


def InitializeORBSLAM(*args):
    """
    Initialize ORB Slam
    """
    global ORBComplete
    global loop
    while True:
        SendLocation(0, -1, 0)
        time.sleep(4)
        SendLocation(0, 1, 0)
        time.sleep(4)
        if not ORB or not loop:
            print vehicle.location.local_frame
            ORBComplete = True
            break


def StopORBSlam(*args):
    """
    Stop ORB Slam Initialization process
    """
    global ORB
    ORB = False


def ThreadingSendLocation(altitude, path, heading, bearing):
    """
    Handles sending drone displacement component on another thread to prevent server from being
    blocked on the main thread
    ThreadingSendLocation(altitude, path, heading, bearing)

    :param altitude : Array of Altitude
    :param path     : Array of Path
    :param heading  : Array of Heading
    :param bearing  : Array of GPS coordinated seperated by @
    """
    global loop
    PreE = 0
    PreN = 0
    iterator = 0
    Echo.Block = False
    for x in path:
        if not loop:
            break
        if abs(heading[iterator] - vehicle.heading) < 10:
            SendLocation(x, 0, 0)
        elif abs(heading[iterator] - vehicle.heading) > 10:
            SendLocation(0, x, 0)
        while CaluculateRemainingDistance(vehicle.location.local_frame.north, vehicle.location.local_frame.east, x, PreN, PreE) > 1:
            print CaluculateRemainingDistance(vehicle.location.local_frame.north, vehicle.location.local_frame.east, x, PreN, PreE)
            print vehicle.heading
            print "Local:", vehicle.location.local_frame
            print "Air Speed: {} Ground Speed:{}".format(vehicle.airspeed, vehicle.groundspeed)
            print "Vehicle Travelled: ", vehicle.location.local_frame.distance_home()
            time.sleep(1)
            if not loop:
                break
        time.sleep(1)
        PreE = vehicle.location.local_frame.east
        PreN = vehicle.location.local_frame.north
    Echo.Block = True


def ShutDown(*args):
    global loop
    loop = False
    reactor.stop()
    keyboard.unhook('l')


class Echo(protocol.Protocol):
    threads = []
    Block = True
    waypoint = []
    PathCompletedCheck = 0

    def connectionMade(self):
        self.waypoint = readmission("test.waypoints")

        def broadcast_msg():
            if self.PathCompletedCheck == 5:
                if self.Block:
                    self.transport.write("PATHEND://")
                self.PathCompletedCheck = 0
            else:
                self.transport.write("LOC://{}@{}@{}@{}".format(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt, vehicle.heading))
            self.PathCompletedCheck += 1
        self.looping_call = task.LoopingCall(broadcast_msg)
        self.looping_call.start(1)

    def dataReceived(self, data):
        header, info = HeaderExtractor(data)
        global ORB
        global ORBComplete
        if header == "shutdown":
            self.transport.write("Server Shutdown Initiated")
            reactor.stop()
        elif header == "KEYFRAME" or header == "LOC":
            self.transport.write("KEYFRAME://{}@{}@{},{}@{}@{},{}".format(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt, vehicle.location.local_frame.north, vehicle.location.local_frame.east, vehicle.location.local_frame.down, vehicle.heading))
        elif header == "land":
            self.transport.write("Landing Initialized")
            InitializeLanding()
        elif header == "TURN":
            pass
        elif header == "PATH" and self.Block and ORBComplete:
            alt, path, heading, bearing = PathParser(info)
            print path
            Movement = threading.Thread(target=ThreadingSendLocation, args=(alt, path, heading, bearing))
            self.threads.append(Movement)
            Movement.start()
        elif header == "INIT":
            self.transport.write("WP://" + "{}@{}".format(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon) + ",".join(str(x) for x in self.waypoint))
            ORB = False
        else:
            self.transport.write("Invalid")


ArmAndTakeoff(2)
# Initialize the takeoff sequence to 2m
print "Take off complete"
time.sleep(5)
# Hover for 5 seconds
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready(True, timeout=180)
    if not vehicle.home_location:
        print "Home Location Not Obtained Yet\n"

print "Obtained Home Location, Initializing Heading Correction"
print "Home:", vehicle.home_location
# A movement command must be sent before updating the drone Yaw angle
UpdateVelocity(0, 0, 0)
vehicle.send_mavlink(MSG)
ConditionYaw(Target_Heading, False)
time.sleep(2)
print "Heading Corrected, Initializing Echo Server"
# A movement command must be sent before updating the drone Yaw angle
# while not vehicle.heading == Target_Heading:
#     # Check if vehicle heading is target heading before moving on
#     if vehicle.heading > Target_Heading:
#         if abs(vehicle.heading - Target_Heading) > 180:
#             ConditionYaw(1, 1, True)
#         else:
#             ConditionYaw(1, -1, True)
#     elif vehicle.heading < Target_Heading:
#         if abs(vehicle.heading - Target_Heading) > 180:
#             ConditionYaw(1, -1, True)
#         else:
#             ConditionYaw(1, 1, True)
#     print "Heading ", vehicle.heading
#     time.sleep(1)
keyboard.hook_key('w', InitializeLanding)
keyboard.hook_key('i', InitializeORBSLAM)
keyboard.hook_key('l', StopORBSlam)
while loop:
    # print vehicle.location.global_frame.lat
    # print vehicle.location.global_frame.lon
    # print "Global ", vehicle.location.global_relative_frame
    print "Heading", vehicle.heading
    print "Local", vehicle.location.local_frame
    print "Battery:", vehicle.battery
    print "\n"
    time.sleep(1)


print("Now let's land")
vehicle.mode = VehicleMode("LAND")
# Wait for 10 seconds after landing to make sure everything is fine
time.sleep(10)
print "Local:", vehicle.location.local_frame
print "Heading:", vehicle.heading
# Close vehicle object
vehicle.close()