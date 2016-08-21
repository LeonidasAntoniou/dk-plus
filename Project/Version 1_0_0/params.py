"""
The parameters class. It is initialized with the vehicle's attributes at time of construction 
but it is constantly updated through attribute listeners on main program.

These parameters can provide the basic info for a future collision avoidance scheme. 
Any functions that can refer to the parameters can be written here.

Version 1.0
-Basic parameters chosen
-Initialization of attributes
-Print_all functionality
"""
from dronekit import connect, Command, VehicleMode, LocationGlobalRelative, LocationGlobal, socket
import uuid

class Params:
	def __init__(self, vehicle):
		self.ID = uuid.uuid4().int #Random UUID
		self.last_recv = None
		self.version = vehicle.version.release_version()
		self.ekf_ok = vehicle.ekf_ok
		self.gps_fix = vehicle.gps_0.fix_type
		self.gps_sat = vehicle.gps_0.satellites_visible
		self.gps_eph = vehicle.gps_0.eph
		self.gps_epv = vehicle.gps_0.epv
		self.set_global_alt = vehicle.capabilities.set_altitude_target_global_int
		self.set_attitude = vehicle.capabilities.set_attitude_target
		self.mode = vehicle.mode.name
		self.global_alt = vehicle.location.global_relative_frame.alt
		self.global_lat = vehicle.location.global_relative_frame.lat
		self.global_lon = vehicle.location.global_relative_frame.lon
		self.distance_from_self = None
		self.heading = vehicle.heading 					#degrees
		self.next_wp = None
		self.next_wp_lat = None
		self.next_wp_lon = None
		self.next_wp_alt = None
		self.battery_level = vehicle.battery.level 		#percentage
		self.velocity = vehicle.velocity				#m/s, airspeed
		self.groundspeed = vehicle.groundspeed			#m/s
		self.airspeed = vehicle.airspeed				#m/s
		self.system_status = vehicle.system_status.state

	def print_all(self):
		print "Printing parameter set of drone with ID:", self.ID
		print "Version:\t\t\t", self.version 
		print "Last Received:\t\t\t", self.last_recv 
		print "EKF_OK:\t\t\t\t", self.ekf_ok 
		print "GPS fix:\t\t\t", self.gps_fix 
		print "GPS No. satellites:\t\t", self.gps_sat 
		print "GPS EPH:\t\t\t", self.gps_eph
		print "GPS EPV:\t\t\t", self.gps_epv 
		print "Global altitude settable:\t", self.set_global_alt 
		print "Global attitude settable:\t", self.set_attitude 
		print "Distance from self:\t\t", self.distance_from_self
		print "Vehicle mode:\t\t\t", self.mode 
		print "Global altitude:\t\t", self.global_alt 
		print "Global latitude:\t\t", self.global_lat 
		print "Global longitude:\t\t", self.global_lon 
		print "Heading (degrees):\t\t", self.heading 
		print "Next waypoint number:\t\t", self.next_wp 
		print "Next waypoint latitude:\t\t", self.next_wp_lat 
		print "Next waypoint longitude:\t", self.next_wp_lon 
		print "Next waypoint altitude:\t\t", self.next_wp_alt 
		print "Battery level (%):\t\t", self.battery_level 
		print "Velocity (airspeed m/s):\t", self.velocity 
		print "Groundspeed (m/s):\t\t", self.groundspeed 
		print "Airspeed (m/s):\t\t\t", self.airspeed 
		print "System status:\t\t\t", self.system_status 
		print "\n\n"

