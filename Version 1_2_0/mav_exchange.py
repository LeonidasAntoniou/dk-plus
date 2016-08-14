"""
Version 1.2
-listen_task is done in a different thread
-more defensive broad() function
"""
import time, math, sys, socket, threading, select, rpdb2
from collections import namedtuple
from params import Params
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil 
import cPickle as pickle


MAX_STAY = 5 #seconds until entry is removed from structure
Geo = namedtuple("Geo", "lat lon")
simple_msg = namedtuple("simple_msg", "ID text")

"""
------------------------------------------------------------------------------------------------------
---------------------------------------- Simulation---------------------------------------------------
------------------------------------------------------------------------------------------------------
"""
#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not args.connect:
    print "Starting copter simulator (SITL)"
    from dronekit_sitl import SITL
    sitl = SITL()
    sitl.download('copter', '3.3', verbose=True)
    sitl_args = ['-I0', '--model', 'quad', '--home=-35.363261,149.165230,584,353']
    sitl.launch(sitl_args, await_ready=True, restart=True)
    connection_string='tcp:127.0.0.1:5760'


# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)




"""
--------------------------------------------------------------------------------------------------
---------------------------------Parameters (check Params class)----------------------------------
--------------------------------------------------------------------------------------------------
"""

#Parameter dictionary 
self_params = Params(vehicle=vehicle)
params = []

def update_params(message):
	global params
	found = False

	#First element
	if len(params) == 0:
		params.append(message)

	#Update values if message comes from the same sender or insert new sender
	else:
		for i in range(0, len(params)):
			if (params[i]).ID == message.ID:
				#Registered entry
				params[i] = message
				found = True
				break
		if found == False:
			#New entry
			params.append(message)

	#Remove entries that have not been updated the last four seconds
	params = [item for item in params if (time.time() - item.last_recv <= MAX_STAY)]




"""
--------------------------------------------------------------------------------------
---------------------------------- Broadcast process ---------------------------------
--------------------------------------------------------------------------------------
"""

#Address is set to broadcast, port chosen arbitrarily
#Used both for sending and receiving
address = ("192.168.1.255", 54545)

#Setting up a UDP socket for sending broadcast messages
sock_broad = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock_broad.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

#Setting up a UDP socket for receiving broadcast messages
#Set to non-blocking because we have select() for that reason
sock_listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock_listen.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock_listen.setblocking(0)
sock_listen.bind(address)

#These functions will be running concurrently until the end of the main process 
#This is achieved by creating a daemon thread for each one of the broad and listen functions
def broad():

	while True:
		try:
			if type(self_params)!='string':
				data = pickle.dumps(self_params, pickle.HIGHEST_PROTOCOL)
			else:
				data = self_params
		except Exception, e:
			data = " "
			print "Unpickling Error: ", e
		
		try:
			sock_broad.sendto(data, address)
		except Exception, e:
			print "Failed to broadcast: ", e

		time.sleep(1) #broadcast every 1s

def listen():
	while True:
		ready = select.select([sock_listen], [], [], 2.0) #wait until a message is received - timeout 1s
		if ready[0]:
			d = sock_listen.recvfrom(4096)
			raw_msg = d[0]
			sender_addr = d[1]
			sender_ip = (d[1])[0]

			
			#Ignore messages from drones in distance more than SAFETY_ZONE

			#Drone parameters are sent in pickle format. 
			#If message is not pickled then we are free to just read it
			try:
				msg = pickle.loads(raw_msg)
				if msg.ID == self_params.ID: #Ignore messages from self. ID is given based on uuid at initialization time
					pass

				else:
					if isinstance(msg, simple_msg): #A simple message structure is defined just in case
						print "Received from:", sender_addr, "Message: ", msg.text

					else:
						#If drone parameters are received, another function is called since 
						#we want the thread dedicated to receiving messages. 
						print "Received drone info" 
						task = threading.Thread(target=listen_task, args=(msg, ))
						task.start()
						
			except pickle.UnpicklingError:
				msg = raw_msg
				print "Received from:", sender_addr, "Message: ", msg

"""
Calculates distance from drone and assigns a timestamp
Afterwards the whole drone info is passed to the drone structure through the updater
!!!Need to calculate processing time here since the value can be fatally outdated!!!
"""
def listen_task(message):
	#Calculate distance
	self_coord = Geo(self_params.global_lat, self_params.global_lon)
	msg_coord = Geo(message.global_lat, message.global_lon)
	message.distance_from_self = get_distance_metres(self_coord, msg_coord)

	#Add timestamp
	message.last_recv = time.time()

	#Go to updater
	update_params(message)

"""
-----------------------------------------------------------------------------------
-----------------------Add observers and keep parameter list updated --------------
-----------------------------------------------------------------------------------
"""

#State of System (Initializing, Emergency, etc.)
@vehicle.on_attribute('system_status')
def decorated_system_status_callback(self, 	attr_name, value):
	self_params.system_status = value.state
	print 'System status changed to: ', self_params.system_status

#Battery information
@vehicle.on_attribute('battery')
def decorated_battery_callback(self, attr_name, value):
	if self_params.battery_level == value.level: 
		pass
	else:
		self_params.battery_level = value.level
		#print 'Battery level: ', self_params.battery_level


#Velocity information (m/s)
#return velocity in all three axis
@vehicle.on_attribute('velocity')
def decorated_velocity_callback(self, attr_name, value):
	if self_params.velocity == value:
		pass
	else:
		self_params.velocity = value
		#print 'Velocity changed to:\n', self_params.velocity, ' m/s'

"""	
	Airspeed and groundspeed are exactly the same in the simulation but 
	this is not applicable in real-life scenarios.
	Tolerance is added to cm scale
	Return: speed (m/s)
"""
@vehicle.on_attribute('airspeed')
def decorated_airspeed_callback(self, attr_name, value):
	if self_params.airspeed == round(value, 2):
		pass
	else:
		self_params.airspeed = round(value, 2)
		#print 'Airspeed changed to: ', self_params.airspeed, ' m/s'

@vehicle.on_attribute('groundspeed')
def decorated_groundspeed_callback(self, attr_name, value):
	if self_params.groundspeed == round(value, 2):
		pass
	else:
		self_params.groundspeed = round(value, 2)
		#print 'Groundspeed changed to: ', self_params.groundspeed, ' m/s'


#State of EKF
#return: True/False
@vehicle.on_attribute('vehicle.ekf_ok')   
def decorated_ekf_ok_callback(self, attr_name, value):
	self_params.ekf_ok = value
	print 'EKF availability changed to: ', self_params.ekf_ok

#GPS-related info 
#return: .eph (HDOP) .epv (VDOP) .fix_type .satellites_visible
@vehicle.on_attribute('vehicle.gps_0')   
def decorated_gps_callback(self, attr_name, value):
	self_params.gps_fix = value.fix_type
	self_params.gps_sat = value.satellites_visible
	self_params.gps_eph = value.eph
	self_params.gps_epv = value.epv
	print 'GPSInfo changed to:\nFix:', self_params.gps_fix, \
			'\nSatellites:', self_params.gps_sat, '\nEPH:', self_params.gps_eph, \
			'\nEPV: ', self_params.gps_epv

#Set altitude offboard
#return: True/False
@vehicle.on_attribute('set_altitude_target_global_int')
def decorated_set_global_altitude_callback(self, attr_name, value):
	self_params.set_global_alt = value
	print 'Ability to set global altitude changed to: ', self_params.set_global_alt

#Set attitude offboard
#return: True/False
@vehicle.on_attribute('set_attitude_target')
def decorated_set_attitude_callback(self, attr_name, value):
	self_params.set_attitude = value
	print 'Ability to set attitude changed to: ', self_params.set_attitude

#Flying mode
@vehicle.on_attribute('mode')
def decorated_mode_callback(self, attr_name, value):
	self_params.mode = value.name
	print 'Mode changed to: ', self_params.mode

"""	
	A precision of 7 decimal digits in lat/lon degrees is satisfactory.
	Tolerance of 7 decimal digits in degrees equals 11 milimetres
	http://gis.stackexchange.com/questions/8650/how-to-measure-the-accuracy-of-latitude-and-longitude
	Returns: 	altitude (metres)
				longitude (degrees)
				latitude (degrees)
"""
@vehicle.on_attribute('location.global_relative_frame')
def decorated_global_relative_frame_callback(self, attr_name, value):
	if self_params.global_alt == round(value.alt, 2) and \
		self_params.global_lat == round(value.lat, 7) and \
		self_params.global_lon == round(value.lon, 7):
		pass

	else:
		self_params.global_alt = round(value.alt, 2)
		self_params.global_lat = round(value.lat, 7)
		self_params.global_lon = round(value.lon, 7)
		#print 'Location changed to:\nAlt:', self_params.global_alt, \
		#		'\nLat:', self_params.global_lat, '\nLon:', self_params.global_lon

"""	
	Drone 360-degree heading, 0 is North. 
	Added a tolerance of +-1 degree
"""
@vehicle.on_attribute('heading')
def decorated_heading_callback(self, attr_name, value):
	if self_params.heading == value or \
		self_params.heading == (value + 1) or \
		self_params.heading == (value - 1):
		pass
	else:
		self_params.heading = value
		#print 'Heading changed to: ', self_params.heading


#Updates the next waypoint in case of mission
@vehicle.on_message('MISSION_CURRENT')	
def message_listener(self, name, message):
	
	try:
		if self_params.next_wp == message.seq:
			return
		else:
			print 'Next waypoint changed'
			self_params.next_wp = message.seq

			cmd = vehicle.commands
			if cmd.count == 0:
				print 'No waypoints found'

			else:
				print 'Waypoint' , cmd.next, ' out of ', cmd.count, ':'
				pos = cmd.next-1

				print 'Frame, Lat, Lon, Alt:',  cmd[pos].frame, cmd[pos].x, cmd[pos].y, cmd[pos].z
				self_params.next_wp_lat = cmd[pos].x
				self_params.next_wp_lon = cmd[pos].y
				self_params.next_wp_alt = cmd[pos].z
	except Exception, e:
		print "Error: ", e

"""
	Updates the other drones' parameters, called by listen_task()
"""
def update_params(message):
	global params
	found = False

	#First element
	if len(params) == 0:
		params.append(message)

	#Update values if message comes from the same sender or insert new sender
	else:
		for i in range(0, len(params)):
			if (params[i]).ID == message.ID:
				params[i] = message
				#params[i].print_all()
				found = True
				break
		if found == False:
			params.append(message)
			#params[i+1].print_all()

	#Remove entries that have not been updated the last MAX_STAY seconds
	params = [item for item in params if time.time() - item.last_recv <= MAX_STAY]
			


"""---------------------------------------------- TESTING THE INTERFACE --------------------------------------- """





"""----------------------------------------------------------------------------------------------------------------
--------------------------------------------------Basic Mission----------------------------------------------------
----------------------------------------------------------------------------------------------------------------"""
	

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

        
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt 
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            print "Reached target altitude"
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

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
	"""
	Returns the ground distance in metres between two LocationGlobal objects.

	This method is an approximation, and will not be accurate over large distances and close to the 
	earth's poles. It comes from the ArduPilot test code: 
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	"""
	dlat = 0
	dlong = 0

	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
		

	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.



def adds_square_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = vehicle.commands

    print " Clear any existing commands"
    cmds.clear() 
    
    print " Define/add new commands."
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)

    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    

    print " Upload new commands to vehicle"
    cmds.upload()


print 'Create a new mission (for current location)'
adds_square_mission(vehicle.location.global_frame,50)

#Creating and starting the broadcast and listen threads
t_broad = threading.Thread(target=broad)
t_listen = threading.Thread(target=listen)

t_listen.daemon = True
t_broad.daemon = True

t_listen.start()
t_broad.start()

# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
arm_and_takeoff(10)

print "Starting mission"
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")


# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.

while True:
	try:
	    nextwaypoint=vehicle.commands.next
	    print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
	  
	    if nextwaypoint==3: #Skip to next waypoint
	        print 'Skipping to Waypoint 5 when reach waypoint 3'
	        vehicle.commands.next = 5
	    if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
	        print "Exit 'standard' mission when start heading to final waypoint (5)"
	        break;
	    time.sleep(1)
	except KeyboardInterrupt:
		break;

for param in params:
	print param.last_recv
print time.time()

#Close broadcast thread and socket
print "Close sockets"
sock_broad.close()
sock_listen.close()

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()


# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()




	
