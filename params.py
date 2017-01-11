"""
The parameters class. It is initialized with the vehicle's attributes at time of construction 
but it is constantly updated through attribute listeners after calling the add_listeners() function

These parameters can provide the basic info for a future collision avoidance scheme. 
Any functions that can refer to the parameters can be written here.

Added support for "dummy" initialization for experimental purposes
"""
from dronekit import connect, Command, VehicleMode, LocationGlobalRelative, LocationGlobal, socket
import uuid, random, time, logging


class Params:
    def __init__(self, network=None, vehicle=None, dummy=False):

        if dummy:
            self.ID = random.randint(1000, 9999)
            self.last_recv = time.time()
            self.version = 1
            self.ekf_ok = False
            self.gps_fix = 3
            self.gps_sat = 10
            self.gps_eph = 100
            self.gps_epv = 200
            self.set_global_alt = True
            self.set_attitude = True
            self.mode = "AUTO"
            self.global_alt = 10
            self.global_lat = -35.3632086902
            self.global_lon = 149.165274916
            self.distance_from_self = None
            self.mission_importance = 0
            self.heading = 300  #degrees
            self.next_wp = None
            self.next_wp_lat = None
            self.next_wp_lon = None
            self.next_wp_alt = None
            self.battery_level = 100  #percentage
            self.velocity = [0.5, -3.1, 0.7]  #m/s, airspeed
            self.groundspeed = 3.46  #m/s
            self.airspeed = 3.46  #m/s
            self.system_status = "OK"

        else:
            self.ID = vehicle.parameters['SYSID_THISMAV'] # Need to specify different ID parameters on APM/PIX advanced
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
            self.mission_importance = 0  #default, for hobbyists and recreation
            self.heading = vehicle.heading  #degrees
            self.next_wp = None
            self.next_wp_lat = None
            self.next_wp_lon = None
            self.next_wp_alt = None
            self.battery_level = vehicle.battery.level  #percentage
            self.velocity = vehicle.velocity  #m/s, airspeed
            self.groundspeed = vehicle.groundspeed  #m/s
            self.airspeed = vehicle.airspeed  #m/s
            self.system_status = vehicle.system_status.state

        self.add_listeners(network, vehicle)

    def add_listeners(self, network, vehicle):
        """
			The function to observe updated values. These values must be contained in the params class
			and a networking scheme (through drone_network) must be active. 

			Object vehicle can be accesed through network.vehicle but it is an input for 
			the correct syntax of python's decorator functions.

			Any observers here are implemented based on the tutorial found in: 
			http://python.dronekit.io/automodule.html#dronekit.Locations.on_attribute

			Some of the values pass through thresholding so as to limit writes. 
			Thresholding is done based on experience and needs
		"""

        if network == None:
            logging.warning("No listeners added due to unknown network")
            return

        #State of System (Initializing, Emergency, etc.)
        @vehicle.on_attribute('system_status')
        def decorated_system_status_callback(self, attr_name, value):
            network.vehicle_params.system_status = value.state
            logging.info('System status changed to: %s', network.vehicle_params.system_status)

        #Battery information
        @vehicle.on_attribute('battery')
        def decorated_battery_callback(self, attr_name, value):
            if network.vehicle_params.battery_level == value.level:
                pass
            else:
                network.vehicle_params.battery_level = value.level
                #logging.info('Battery level: %s', network.vehicle_params.battery_level)

            #Velocity information (m/s)
            #return velocity in all three axis
        @vehicle.on_attribute('velocity')
        def decorated_velocity_callback(self, attr_name, value):
            if network.vehicle_params.velocity == value:
                pass
            else:
                network.vehicle_params.velocity = value
                #logging.info('Velocity changed to: %s m/s', network.vehicle_params.velocity)

        """	
			Airspeed and groundspeed are exactly the same in the simulation but 
			this is not applicable in real-life scenarios.
			Tolerance is added to cm scale
			Return: speed (m/s)
		"""

        @vehicle.on_attribute('airspeed')
        def decorated_airspeed_callback(self, attr_name, value):
            if network.vehicle_params.airspeed == round(value, 2):
                pass
            else:
                network.vehicle_params.airspeed = round(value, 2)
                #logging.info('Airspeed changed to: %s m/s', network.vehicle_params.airspeed)

        @vehicle.on_attribute('groundspeed')
        def decorated_groundspeed_callback(self, attr_name, value):
            if network.vehicle_params.groundspeed == round(value, 2):
                pass
            else:
                network.vehicle_params.groundspeed = round(value, 2)
                #logging.info('Groundspeed changed to: %s m/s', network.vehicle_params.groundspeed)

            #State of EKF
            #return: True/False
        @vehicle.on_attribute('vehicle.ekf_ok')
        def decorated_ekf_ok_callback(self, attr_name, value):
            network.vehicle_params.ekf_ok = value
            logging.info('EKF availability changed to: %s', network.vehicle_params.ekf_ok)

        #GPS-related info 
        #return: .eph (HDOP) .epv (VDOP) .fix_type .satellites_visible
        @vehicle.on_attribute('vehicle.gps_0')
        def decorated_gps_callback(self, attr_name, value):
            network.vehicle_params.gps_fix = value.fix_type
            network.vehicle_params.gps_sat = value.satellites_visible
            network.vehicle_params.gps_eph = value.eph
            network.vehicle_params.gps_epv = value.epv
            logging.info('GPSInfo changed to:\nFix: %s\nSatellites: %s\nEPH: %s\nEPV: %s', 
                network.vehicle_params.gps_fix, 
                network.vehicle_params.gps_sat, 
                network.vehicle_params.gps_eph, 
                network.vehicle_params.gps_epv)

        #Set altitude offboard
        #return: True/False
        @vehicle.on_attribute('set_altitude_target_global_int')
        def decorated_set_global_altitude_callback(self, attr_name, value):
            network.vehicle_params.set_global_alt = value
            logging.info('Ability to set global altitude changed to: %s', network.vehicle_params.set_global_alt)

        #Set attitude offboard
        #return: True/False
        @vehicle.on_attribute('set_attitude_target')
        def decorated_set_attitude_callback(self, attr_name, value):
            network.vehicle_params.set_attitude = value
            logging.info('Ability to set attitude changed to: %s ', network.vehicle_params.set_attitude)

        #Flying mode
        @vehicle.on_attribute('mode')
        def decorated_mode_callback(self, attr_name, value):
            network.vehicle_params.mode = value.name
            logging.info('Mode changed to: %s', network.vehicle_params.mode)

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
            if network.vehicle_params.global_alt == round(value.alt, 2) and \
             network.vehicle_params.global_lat == round(value.lat, 7) and \
             network.vehicle_params.global_lon == round(value.lon, 7):
                pass

            else:
                network.vehicle_params.global_alt = round(value.alt, 2)
                network.vehicle_params.global_lat = round(value.lat, 7)
                network.vehicle_params.global_lon = round(value.lon, 7)
                #logging.info('Location changed to:\nAlt: %s\nLat: %s\nLon: %s', 
                 #   network.vehicle_params.global_alt, 
                  #  network.vehicle_params.global_lat,
                   # network.vehicle_params.global_lon)

        """	
			Drone 360-degree heading, 0 is North. 
			Added a tolerance of +-1 degree
		"""

        @vehicle.on_attribute('heading')
        def decorated_heading_callback(self, attr_name, value):
            if network.vehicle_params.heading == value or \
             network.vehicle_params.heading == (value + 1) or \
             network.vehicle_params.heading == (value - 1):
                pass
            else:
                network.vehicle_params.heading = value
                #logging.info('Heading changed to: %s', network.vehicle_params.heading)

            #Updates the next waypoint in case of mission
        @vehicle.on_message('MISSION_CURRENT')
        def message_listener(self, name, message):

            try:
                if network.vehicle_params.next_wp == message.seq:
                    return
                else:
                    logging.info('Next waypoint changed')
                    network.vehicle_params.next_wp = message.seq

                    cmd = vehicle.commands
                    if cmd.count == 0:
                        logging.info('No waypoints found')

                    else:
                        logging.info('Waypoint %s out of %s:', cmd.next, cmd.count)
                        pos = cmd.next - 1

                        logging.info('Frame, Lat, Lon, Alt: %s, %s, %s, %s', cmd[pos].frame, cmd[
                            pos].x, cmd[pos].y, cmd[pos].z)
                        network.vehicle_params.next_wp_lat = cmd[pos].x
                        network.vehicle_params.next_wp_lon = cmd[pos].y
                        network.vehicle_params.next_wp_alt = cmd[pos].z
            except Exception, e:
                print "Error: ", e

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
        print "Mission Importance:\t\t", self._mission_importance
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

    """
	*EXTREMELY HACKABLE*

	Level 0: Default 	(e.g. for hobbyists, recreation, entertainment)
	Level 1: Important 	(e.g. for businesses, industry-related activity)
	Level 2: Top 		(e.g. for search and rescue, security activity)

	In order to elevate mission privileges, a special request must be made to the according authorities
	Currently not supported
	"""

    @property
    def mission_importance(self):
        return self._mission_importance

    @mission_importance.setter
    def mission_importance(self, level):
        #Need to add all kinds of security here
        self._mission_importance = 0
        if level == 1 | level == 2:
            logging.info("You need to ask permission from authoritative personel")
            #if request_successful: self._mission_importance = level
            #else: print "You don't have the rights."
