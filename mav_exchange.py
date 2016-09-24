import time, logging
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil

#Import custom modules
from params import Params
import geo_tools as geo
from drone_network import Networking
from collision_avoidance import CollisionThread

"""
------------------------------------------------------------------------------------------------------
---------------------------------------- Simulation---------------------------------------------------
------------------------------------------------------------------------------------------------------
"""
logging.basicConfig(filename="testing.log", filemode="w", level=logging.INFO)

#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(
    description='Control Copter and send commands in GUIDED mode ')
parser.add_argument(
    '--connect',
    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

#Start SITL if no connection string specified
if not args.connect:
    logging.info("Starting copter simulator (SITL)")
    from dronekit_sitl import SITL
    sitl = SITL()
    sitl.download('copter', '3.3', verbose=True)
    sitl_args = ['-I0', '--model', 'quad',
                 '--home=-35.363261,149.165230,584,353']
    sitl.launch(sitl_args, await_ready=True, restart=True)
    connection_string = 'tcp:127.0.0.1:5760'

# Connect to the Vehicle
logging.info('Connecting to vehicle on: %s', connection_string)
vehicle = connect(connection_string, wait_ready=True)

#Create the interface with UDP broadcast sockets
address = ("192.168.1.255", 54545)
network = Networking(address, "UDP_BROADCAST", vehicle)

#Add collision avoidance algorithm
t_collision = CollisionThread(network, 'priorities')
"""---------------------------------------------- TESTING THE INTERFACE --------------------------------------- """
"""----------------------------------------------------------------------------------------------------------------
--------------------------------------------------Basic Mission----------------------------------------------------
----------------------------------------------------------------------------------------------------------------"""


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    logging.info("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        logging.info(" Waiting for vehicle to initialise...")
        time.sleep(1)

    logging.info("Arming motors") 
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        logging.info(" Waiting for arming...")
        time.sleep(1)

    logging.info("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        logging.info(" Altitude: %s", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            logging.info("Reached target altitude")
            break
        time.sleep(1)


"""
arm_and_takeoff(10)

print "Set default/target airspeed to 3"
vehicle.airspeed = 3

print "Going towards first point for 30 seconds ..."
point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
vehicle.simple_goto(point1)
time.sleep(30)


print "Going towards second point for 30 seconds (groundspeed set to 10 m/s) ..."
point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
vehicle.simple_goto(point2, groundspeed=10)
time.sleep(10)

print "Returning to Launch"
vehicle.mode = VehicleMode("RTL")
time.sleep(10)

vehicle.mode = VehicleMode("LAND")
time.sleep(10)
"""
"""-----------------------------------------------------------------------------------
---------------------------------------Mission Upload---------------------------------
-----------------------------------------------------------------------------------"""


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()  # wait until download is complete.


def adds_square_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """

    cmds = vehicle.commands

    logging.info("Clear any existing commands")
    cmds.clear()

    logging.info("Define/add new commands")
    # Add new commands. The meaning/order of the parameters is documented in the Command class.

    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0,
                10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    lat = aLocation.lat
    lon = aLocation.lon
    alt = aLocation.alt
    point1 = geo.get_location_metres(lat, lon, alt, aSize, -aSize)
    point2 = geo.get_location_metres(lat, lon, alt, aSize, aSize)
    point3 = geo.get_location_metres(lat, lon, alt, -aSize, aSize)
    point4 = geo.get_location_metres(lat, lon, alt, -aSize, -aSize)

    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                point1.lat, point1.lon, 11))
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                point2.lat, point2.lon, 12))
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                point3.lat, point3.lon, 13))
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                point4.lat, point4.lon, 14))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                point4.lat, point4.lon, 14))

    logging.info("Upload new commands to vehicle")
    cmds.upload()


logging.info("Initializing interface")
network.run()

logging.info("Starting collision avoidance scheme")
t_collision.start()

logging.info('Create a new mission (for current location)')
adds_square_mission(vehicle.location.global_frame, 50)

# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
arm_and_takeoff(10)

logging.info("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next = 0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.

while True:
    try:
        nextwaypoint = vehicle.commands.next
        if vehicle.commands.count != 0:
            logging.info('Distance to waypoint (%s): %s', 
                nextwaypoint, 
                geo.distance_to_current_waypoint(vehicle))

            if nextwaypoint == 3:  #Skip to next waypoint
                logging.info('Skipping to Waypoint 5 when reach waypoint 3')
                vehicle.commands.next = 5
            if nextwaypoint == 5:  #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
                logging.info("Exit 'standard' mission when start heading to final waypoint (5)")
                #vehicle.mode = VehicleMode('RTL')
                break

        time.sleep(1)
    except KeyboardInterrupt:
        break

#Close broadcast thread and socket
logging.info("Close sockets")
network.stop()

#Get collision avoidance timing
t_collision.get_timing()

#Close vehicle object before exiting script
logging.info("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
