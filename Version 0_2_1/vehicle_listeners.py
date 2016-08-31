"""

"""

def add_listeners(network, vehicle):

	#State of System (Initializing, Emergency, etc.)
	@vehicle.on_attribute('system_status')
	def decorated_system_status_callback(self, 	attr_name, value):
		network.vehicle_params.system_status = value.state
		print 'System status changed to: ', network.vehicle_params.system_status

	#Battery information
	@vehicle.on_attribute('battery')
	def decorated_battery_callback(self, attr_name, value):
		if network.vehicle_params.battery_level == value.level: 
			pass
		else:
			network.vehicle_params.battery_level = value.level
			#print 'Battery level: ', network.vehicle_params.battery_level


	#Velocity information (m/s)
	#return velocity in all three axis
	@vehicle.on_attribute('velocity')
	def decorated_velocity_callback(self, attr_name, value):
		if network.vehicle_params.velocity == value:
			pass
		else:
			network.vehicle_params.velocity = value
			#print 'Velocity changed to:\n', network.vehicle_params.velocity, ' m/s'

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
			#print 'Airspeed changed to: ', network.vehicle_params.airspeed, ' m/s'

	@vehicle.on_attribute('groundspeed')
	def decorated_groundspeed_callback(self, attr_name, value):
		if network.vehicle_params.groundspeed == round(value, 2):
			pass
		else:
			network.vehicle_params.groundspeed = round(value, 2)
			#print 'Groundspeed changed to: ', network.vehicle_params.groundspeed, ' m/s'


	#State of EKF
	#return: True/False
	@vehicle.on_attribute('vehicle.ekf_ok')   
	def decorated_ekf_ok_callback(self, attr_name, value):
		network.vehicle_params.ekf_ok = value
		print 'EKF availability changed to: ', network.vehicle_params.ekf_ok

	#GPS-related info 
	#return: .eph (HDOP) .epv (VDOP) .fix_type .satellites_visible
	@vehicle.on_attribute('vehicle.gps_0')   
	def decorated_gps_callback(self, attr_name, value):
		network.vehicle_params.gps_fix = value.fix_type
		network.vehicle_params.gps_sat = value.satellites_visible
		network.vehicle_params.gps_eph = value.eph
		network.vehicle_params.gps_epv = value.epv
		print 'GPSInfo changed to:\nFix:', network.vehicle_params.gps_fix, \
				'\nSatellites:', network.vehicle_params.gps_sat, '\nEPH:', network.vehicle_params.gps_eph, \
				'\nEPV: ', network.vehicle_params.gps_epv

	#Set altitude offboard
	#return: True/False
	@vehicle.on_attribute('set_altitude_target_global_int')
	def decorated_set_global_altitude_callback(self, attr_name, value):
		network.vehicle_params.set_global_alt = value
		print 'Ability to set global altitude changed to: ', network.vehicle_params.set_global_alt

	#Set attitude offboard
	#return: True/False
	@vehicle.on_attribute('set_attitude_target')
	def decorated_set_attitude_callback(self, attr_name, value):
		network.vehicle_params.set_attitude = value
		print 'Ability to set attitude changed to: ', network.vehicle_params.set_attitude

	#Flying mode
	@vehicle.on_attribute('mode')
	def decorated_mode_callback(self, attr_name, value):
		network.vehicle_params.mode = value.name
		print 'Mode changed to: ', network.vehicle_params.mode

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
			#print 'Location changed to:\nAlt:', network.vehicle_params.global_alt, \
			#		'\nLat:', network.vehicle_params.global_lat, '\nLon:', network.vehicle_params.global_lon

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
			#print 'Heading changed to: ', network.vehicle_params.heading


	#Updates the next waypoint in case of mission
	@vehicle.on_message('MISSION_CURRENT')	
	def message_listener(self, name, message):
		
		try:
			if network.vehicle_params.next_wp == message.seq:
				return
			else:
				print 'Next waypoint changed'
				network.vehicle_params.next_wp = message.seq

				cmd = vehicle.commands
				if cmd.count == 0:
					print 'No waypoints found'

				else:
					print 'Waypoint' , cmd.next, ' out of ', cmd.count, ':'
					pos = cmd.next-1

					print 'Frame, Lat, Lon, Alt:',  cmd[pos].frame, cmd[pos].x, cmd[pos].y, cmd[pos].z
					network.vehicle_params.next_wp_lat = cmd[pos].x
					network.vehicle_params.next_wp_lon = cmd[pos].y
					network.vehicle_params.next_wp_alt = cmd[pos].z
		except Exception, e:
			print "Error: ", e