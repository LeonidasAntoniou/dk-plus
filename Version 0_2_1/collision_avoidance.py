
"""
Handler for the collision avoidance part of the module.
A specific drone_network.Networking object needs to be running
Custom algorithms can be specified
Spawns a thread by instantiation

Author: Leonidas Antoniou 
mail: leonidas.antoniou@gmail.com
"""
import threading, time, itertools
import geo_tools as geo
from dronekit import VehicleMode
from operator import attrgetter



class CollisionThread(threading.Thread):
	def __init__(self, network, algorithm=None):
		threading.Thread.__init__(self)
		self.daemon = True
		self.network = network
		self.near = []
		self.critical = []
		self.algorithm = algorithm
		self.control_taken = False
		self.firstTime = True
		self.in_session = False

	def run(self):
		#Deploy your collision avoidance algorithm here
		while True:			
			if self.algorithm == None:
				self.no_protocol()

			elif self.algorithm == 'priorities':
				self.priorities_protocol()
				
			else:
				pass

			time.sleep(1)				





	"""A Collision Avoidance API"""

	def no_protocol(self):
		#What to do if no protocol is specified
		#Currently it just outputs the drones in vicinity 
		self.update_drone_list()
		self.print_drones_in_vicinity()

	def priorities_protocol(self):
		#A naive protocol based in priorities:
		#Stops every drone whose priority is other than '1'
		#Drone with priority '1' continues mission/movement
		#Currently works for AUTO mode

		#Give priorities 
		self.give_priorities()
		self.update_drone_list()
		self.print_drones_in_vicinity()

		if len(self.near) == 0 and not self.in_session:
			return

		#Get priority number
		priority_num = None
		for drone in self.network.drones:
			if drone.ID == self.network.vehicle_params.ID:
				priority_num = drone.priority
				break

		print "Flying drone's priority number is: ", priority_num

		#Perform actions
		self.in_session = True
		if priority_num == 1:
			self.give_control()

		else:
			#Keep loitering (if GPS is faulty then drone will "toilet-bowl")
			self.take_control()


	def give_priorities(self):
		"""
		-If drone is grounded then its priority is low, exempting mode 'ARMED'
		-System status needs to be checked first, if it's critical it has the greatest capability of all
		-----If status is OK then highest priority is given to highest mission importance first
		---------Among same mission importances, highest priority is given to the ones with less capabilities
		------------If they have the same status, mission importance, capabilities fine-tune with battery level
		"""

		#Temporary priority lists
		top = []
		high = []
		medium = []
		low = []

		#Depending the autopilot. The following modes are found in ardupilot
		grounded_state = ['UNINIT', 'BOOT', 'POWEROFF', 'STANDBY', 'CALIBRATING', 'LOCKED']
		auto_modes = ['RTL', 'LOITER', 'AUTO', 'GUIDED', 'CIRCLE', 'LAND', 'BRAKE', 'LIFTOFF']
		manual_modes = ['ALT_HOLD', 'STABILIZE', 'MANUAL', 'ACRO', 'SPORT', 'DRIFT', 'POSHOLD', 'SIMPLE', 'SUPER_SIMPLE']
		

		#Assign each drone to its priority list
		priority_num = 1
		for drone in self.network.drones:
			has_capabilities = drone.set_global_alt or drone.set_attitude
			has_mayday = (drone.system_status == 'CRITICAL') or (drone.system_status == 'EMERGENCY')

			#Manual flying drones without capabilities 
			#Drones in Emergency/Critical state
			if (drone.mode in manual_modes and not has_capabilities and drone.system_status not in grounded_state) or has_mayday:
				top.append(drone)

			#Top importance drones in flying state or ready to fly				
			elif drone.mission_importance == 2 and drone.system_status not in grounded_state or\
				drone.mission_importance == 2 and drone.system_status not in grounded_state:
				high.append(drone)

			#Drones not in level-2 importance
			#Drones in flying state or armed with remote capabilities 
			#Drones flying or armed in one of the automatic flying modes
			elif (drone.mode in auto_modes and drone.system_status not in grounded_state ) or \
				(drone.mode in manual_modes and has_capabilities and drone.system_status not in grounded_state):
				medium.append(drone)

			#Grounded drones with low importance
			elif drone.system_status in grounded_state:
				low.append(drone)

			#Assign priority number, redundant because list is sorted 
			#based on priorities (top to lowest) but you never know...
			drone.priority = priority_num
			priority_num = priority_num + 1


		#Sort each sublist based on mission importance (top and high priority lists don't need that)
		medium.sort(key=attrgetter('mission_importance'), reverse=True)
		low.sort(key=attrgetter('mission_importance'), reverse=True)

		#Fine-tune sorting with battery level
		top.sort(key=attrgetter('battery_level'))
		high.sort(key=attrgetter('battery_level'))
		medium.sort(key=attrgetter('battery_level'))
		low.sort(key=attrgetter('battery_level'))

		#Combine everything back to drones list
		drones = [top, high, medium, low]
		self.network.drones = list(itertools.chain.from_iterable(drones))

		"""
		#Print priorities
		print "Printing Priorities:"
		for drone in self.network.drones:
			print "ID:", drone.ID, " Priority: ", drone.priority
		"""

	def take_control(self):
		#Change groundspeed to zero

		"""
		#The fun way to do it
		msg = self.network.vehicle.message_factory.mav_cmd_do_change_speed_encode(
			0, 0,    # target system, target component
    		mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, #command
    		0, #confirmation
    		0, #speed type, ignore on ArduCopter
		    0, # speed
		    0, 0, 0, 0, 0 #ignore other parameters
		    )

		self.network.vehicle.send_mavlink(msg)
		"""

		#First time in the session
		if self.firstTime:
			self.network.context.append(round(self.network.vehicle_params.groundspeed))
			self.firstTime = False

		#Change mode to GUIDED and assert
		self.network.vehicle.mode = VehicleMode("GUIDED")
		while self.network.vehicle.mode.name != "GUIDED":
			pass

		#In GUIDED mode we are free to change the speed to zero
		self.network.vehicle.groundspeed = 0.0
		print "Control taken!"

	def give_control(self):

		#Change mode to AUTO and assert
		self.network.vehicle.mode = VehicleMode("AUTO")
		while self.network.vehicle.mode.name != "AUTO":
			pass

		#Continue with pre-collision speed and end session
		#self.network.vehicle.groundspeed = self.network.context[0]
		self.firstTime = True
		self.in_session = False
		print "Control given!"

	def save_context(self):
		"""Currently keeping information about mode, altitude and mission"""

		#Clear previous entry
		self.network.context = []

		#Save context: mode, mission
		self.network.context.append(self.network.vehicle_params.mode)
		self.network.context.append(self.current_mission())
		self.network.context.append(self.network.vehicle.commands.next)
		self.network.context.append(self.network.vehicle_params.groundspeed)

		"""
		#Print context
		print "Context saved:"
		print "Mode: ", self.network.context[0]
		print "Waypoints:"
		for wp in self.network.context[1]:
			print wp
		print "Next waypoint:", self.network.context[2]
		"""

	def restore_context(self):
		"""Returns to the state before collision avoidance handling"""

		#Clear collision avoidance actions
		commands = self.current_mission()
		commands.clear()

		#Upload all pre-collision waypoints
		for cmd in self.network.context[1]:
			commands.add(cmd)

		commands.upload()
		self.network.vehicle.next = self.network.context[2]
		self.network.vehicle.mode = VehicleMode(self.network.context[0])
		self.network.vehicle.groundspeed = self.network.context[3]
		print "Context restored"

	def update_drone_list(self):
		self.near = []
		self.critical = []

		#1.Remove entries that have not been updated for the last MAX_STAY seconds
		self.network.drones = [item for item in self.network.drones if \
			(item.last_recv == None)\
			or (time.time() - item.last_recv <= self.network.MAX_STAY)]

		#2.Update own vehicle parameter entry in the right position
		for i in range(0, len(self.network.drones)):
			if self.network.drones[i].ID == self.network.vehicle_params.ID:
				self.network.drones[i] = self.network.vehicle_params
				break	

		#3.Update near-zone and critical-zone lists
		drone_list = self.network.drones

		#Only our drone is in the list
		if len(drone_list) == 1:
			pass
		
		else: 
			#This value is slightly not concurrent 
			own = self.network.vehicle_params

			#From the detected drones, add any within a SAFETY-to-CRITICAL-metre range
			self.near = [item for item in drone_list if \
				(item.ID != own.ID)
				&(geo.get_distance_metres(own.global_lat, own.global_lon, item.global_lat, item.global_lon) <= self.network.SAFETY_ZONE)\
				&(geo.get_distance_metres(own.global_lat, own.global_lon, item.global_lat, item.global_lon) > self.network.CRITICAL_ZONE)\
				& (abs(own.global_alt - item.global_alt) <= self.network.SAFETY_ZONE)]


			#From the near drones, add any within a CRITICAL-metre range 
			self.critical = [item for item in drone_list if \
				(item.ID != own.ID) \
				&(geo.get_distance_metres (own.global_lat, own.global_lon, item.global_lat, item.global_lon) <= self.network.CRITICAL_ZONE) \
				& (abs(own.global_alt - item.global_alt) <= self.network.CRITICAL_ZONE)]

	def print_drones_in_vicinity(self):
		#Print drone IDs that are in close and critical range
		#Inform if no such drones are found
		if len(self.near)==0 and len(self.critical)==0:
			print "No dangerous drones found"

		else:
			for drone in self.near:
				print "Drone approaching! ID: ", drone.ID

			for drone in self.critical:
				print "Drone too close!!!! ID: ", drone.ID

	def current_mission(self):
		#Retrieves current mission of vehicle
		cmds = self.network.vehicle.commands
		cmds.download
		cmds.wait_ready() #may lock
		return cmds


