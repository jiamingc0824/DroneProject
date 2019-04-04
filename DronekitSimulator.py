from dronekit import connect, VehicleMode, LocationGlobalRelative
from twisted.internet import reactor, protocol, task
from twisted.internet.protocol import Factory
from pymavlink import mavutil
from array import *
import threading
import time
import dronekit_sitl
import keyboard
import math
import socket

print "Start simulator (SITL)"
sitl = dronekit_sitl.start_default(2.9423677, 101.8729529)
connection_string = sitl.connection_string()
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=False)
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
ORB = True
ORBComplete = False
Target_Heading = 135  # Target heading of drone
CURRENTLAYER = 2
vehicle.parameters['WPNAV_SPEED'] = 50
vehicle.parameters['ACRO_YAW_P'] = 1
vehicle.parameters['ATC_ACCEL_Y_MAX'] = 9000
vehicle.parameters['WP_YAW_BEHAVIOR'] = 0


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
            elif i == 1:
                print "First Coordinates suppose to be home location"
            else:
                linearray = line.split('\t')
                lat = float(linearray[8])  # Latitude of Waypoint
                lon = float(linearray[9])  # Longitude of Waypoint
                missionlist.append("{}@{}".format(lat, lon))
    return missionlist


def HeadingParser(string):
    """
    Parse heading data from service layer
    """
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
    print " Waiting for vehicle to initialise..."
    while not vehicle.is_armable:
        print "Is Vehicle Armable?"
        print vehicle.is_armable
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


def UpdateVelocity(velocity_x, velocity_y, velocity_z):
    """
    Function to send velocity component to the drone
    UpdateVelocity(velocity_x, velocity_y, velocity_z, duration)

    :param velocity_x  : Positive = Front, Negative = Back
    :param velocity_y  : Positive = Left, Negative = Right
    :param velocity_z  : Positive = Down, Negative = Up
    """
    global MSG
    # Update the mavlink message based on the velocity component
    MSG = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame, currently set to vehicle's heading
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


def UpdateVelocity2(velocity_x, velocity_y, velocity_z):
    global MSG
    # Update the mavlink message based on the velocity component
    MSG = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame, currently set to vehicle's EKF origin
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


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
    UpdateVelocity2(0, 0, 0)
    vehicle.mode = VehicleMode("LAND")
    vehicle.send_mavlink(MSG)
    time.sleep(10)


def InitializeORBSLAM(*args):
    """
    Initialize ORB Slam
    """
    global ORBComplete
    global loop
    while True:
        SendLocation(0, -1, 0)
        time.sleep(2)
        SendLocation(0, 1, 0)
        time.sleep(2)
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


def ThreadingConditionYaw(rotation):
    """
    Handles  drone yaw movement on another thread to prevent server from being blocked
    on the main thread
    ThreadingConditionYaw(rotation)

    :param rotation : Absolute angle of drone rotation
    """
    global Target_Heading
    Target_Heading = rotation
    Echo.blocked = False
    ConditionYaw(rotation)
    Echo.blocked = True


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
    global CURRENTLAYER
    global Target_Heading
    PreE = 0
    PreN = 0
    iterator = 0
    Echo.Block = False
    Coordinates = []
    for x in path:
        Coordinates = map(float, bearing[iterator].split(','))
        PreE = vehicle.location.local_frame.east
        PreN = vehicle.location.local_frame.north
        if not loop:
            break
        if abs(Target_Heading - vehicle.heading) > 1 and abs(Target_Heading - vehicle.heading) < 359:
            ConditionYaw(Target_Heading, False)
        if (CURRENTLAYER != altitude):
            if CURRENTLAYER > altitude:
                SendLocation(0, 0, 1)
            elif CURRENTLAYER < altitude:
                SendLocation(0, 0, -1)
            time.sleep(5)
            CURRENTLAYER = altitude
        if GetStraight(vehicle.heading, heading[iterator]):
            SendLocation(x, 0, 0)
            print "Moving Forward"
        elif not GetStraight(vehicle.heading, heading[iterator]):
            if GetLeftOrRight(vehicle.heading, heading[iterator]):
                print "Going Right"
                if (x > 0):
                    SendLocation(0, x * -1, 0)
                elif (x < 0):
                    SendLocation(0, x, 0)
            elif not GetLeftOrRight(vehicle.heading, heading[iterator]):
                print "Going Left"
                SendLocation(0, x, 0)
        while CaluculateRemainingDistance(vehicle.location.local_frame.north, vehicle.location.local_frame.east, x, PreN, PreE) > 1:
            print "Remaining Distance: ", CaluculateRemainingDistance(vehicle.location.local_frame.north, vehicle.location.local_frame.east, x, PreN, PreE)
            print "Heading: ", vehicle.heading
            print "Local:", vehicle.location.local_frame
            print "GPS: ", vehicle.location.global_frame
            print "Lat:{}, Lon:{}".format(Coordinates[0], Coordinates[1])
            print "Air Speed: {} Ground Speed:{}".format(vehicle.airspeed, vehicle.groundspeed)
            print "Vehicle Distance From Home: ", vehicle.location.local_frame.distance_home()
            time.sleep(1)
            if not loop:
                break
        time.sleep(1)
        Coordinates = []
        iterator += 1
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
        self.transport.write("WP://" + "{}@{}".format(vehicle.home_location.lat, vehicle.home_location.lon) + ",".join(str(x) for x in self.waypoint))

        def broadcast_msg():
            if self.PathCompletedCheck == 3:
                if self.Block:
                    self.transport.write("PATHEND://")
                self.PathCompletedCheck = 0
            else:
                self.transport.write("LOC://{}@{}@{}@{}@{}".format(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt, vehicle.heading, CURRENTLAYER))
            self.PathCompletedCheck += 1
        self.looping_call = task.LoopingCall(broadcast_msg)
        self.looping_call.start(1)

    def dataReceived(self, data):
        print data
        header, info = HeaderExtractor(data)
        global ORB
        global ORBComplete
        global CURRENTLAYER
        if header == "shutdown":
            self.transport.write("Server Shutdown Initiated")
            reactor.stop()
        elif header == "KEYFRAME" or header == "LOC":
            self.transport.write("KEYFRAME://{}@{}@{},{}@{}@{},{}".format(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt, vehicle.location.local_frame.north, vehicle.location.local_frame.east, vehicle.location.local_frame.down, vehicle.heading))
        elif header == "land":
            self.transport.write("Landing Initialized")
            InitializeLanding()
        elif header == "TURN" and self.Block and ORBComplete:
            rotation = HeadingParser(info)
            Heading = threading.Thread(target=ThreadingConditionYaw, args=(rotation))
            self.threads.append(Heading)
            Heading.start()
        elif header == "PATH" and self.Block and ORBComplete:
            alt, path, heading, bearing = PathParser(info)
            Movement = threading.Thread(target=ThreadingSendLocation, args=(alt, path, heading, bearing))
            self.threads.append(Movement)
            Movement.start()
        elif header == "INIT":
            # if info == "SERV@KEYFRAME":
                # ORB = False
            self.transport.write("{}@{}".format(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon))
        else:
            self.transport.write("Invalid")


ArmAndTakeoff(1)
# Initialize the takeoff sequence to 1m
print "Take off complete"
time.sleep(1)
# Hover for 5 seconds
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print "Home Location Not Obtained Yet"

print "Obtained Home Location, Initializing Heading Correction"
print "Home:", vehicle.home_location
# A movement command must be sent before updating the drone Yaw angle
UpdateVelocity2(0, 0, 0)
vehicle.send_mavlink(MSG)
ConditionYaw(Target_Heading, False)
time.sleep(1)
print "Heading Corrected, Initializing Echo Server"
keyboard.hook_key('w', InitializeLanding)
factory = protocol.ServerFactory()
factory.protocol = Echo
ORBInit = threading.Thread(target=InitializeORBSLAM)
ORBInit.start()
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    sock.connect(('localhost', 8080))
    print "Drone Init Completed, Notifying Service Layer"
    data = "droneserver"
    sock.sendall(data)
    sock.close()
except Exception as e:
    print e
reactor.listenTCP(8081, factory)
keyboard.hook_key('l', ShutDown)
reactor.run()

# while loop:
#     print vehicle.location.global_frame.lat
#     print vehicle.location.global_frame.lon
#     print "Global ", vehicle.location.global_frame
#     print "Heading", vehicle.heading
#     print vehicle.battery
#     print "Local:", vehicle.location.local_frame
#     print "Air Speed: {} Ground Speed:{}".format(vehicle.airspeed, vehicle.groundspeed)
#     print "Vehicle Travelled: ", vehicle.location.local_frame.distance_home()
#     print "Remaining Distance: ", CaluculateRemainingDistance(vehicle.location.local_frame.north, vehicle.location.local_frame.east, 80)
#     time.sleep(1)

vehicle.mode = VehicleMode("LAND")
# Close vehicle object before exiting script
vehicle.close()
# Shut down simulator
sitl.stop()
print("Completed")
