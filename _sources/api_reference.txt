API Reference
*************

.. _networking:

.. class:: drone_network.Networking(address, protocol, vehicle)


	.. tip::

		In this class you can add your own networking protocol method. See :any:`Networking.create_udp_broadcast` for an example.


	The necessary class to interface your drone with the outer world. 

	- Responsible for opening/closing sockets according to a chosen protocol (currently only broadcast messaging through UDP). 

	- Invokes the send/receive/task threads.

	- Contains the vehicle's drone list and message queue.

	- Invokes the necessary parameter observers/listeners 


	:param tuple address: The network address in ('address', 'port') format. 

	.. _protocol:

	:param str protocol: The desired network protocol as implemeted inside the class.

	:param dronekit.Vehicle vehicle: 

		The object `vehicle <http://python.dronekit.io/automodule.html#dronekit.Vehicle>`_ as returned from `dronekit.connect() <http://python.dronekit.io/automodule.html#dronekit.connect>`_.

	.. _poll-rate:

	.. data:: POLL_RATE

		Set to 0.5 seconds. How often to send the messages (in seconds). 

	.. _max-stay:

	.. data:: MAX_STAY

		Set to 5 seconds. How long should messages be kept for without update (in seconds).

	.. _safety-zone:

	.. data:: SAFETY_ZONE

		Set to 40 metres. The radius in metres around the drone that is considered the border of the safety zone.

	.. _critical-zone:

	.. data:: CRITICAL_ZONE

		Set to 10 metres. The radius in metres around the drone that is considered the border of the critical zone.

	.. _msg-queue:

	.. attribute:: msg_queue

		A :obj:`Queue.Queue` object for storing received messages. 
		Can be changed directly.

	.. _vehicle-params:

	.. attribute:: vehicle_params

		A :obj:`params.Params` object to save the wanted parameters.
		It is not advised to modify it directly since the structure is updated automatically.

	.. _drones-list:

	.. attribute:: drones

		A :obj:`list` that saves the drone parameters of incoming drone messages.
		It is not advised to modify it directly since the structure is updated automatically.

	.. attribute:: context

		A :obj:`dictionary` that helps save important information just before any collision avoidance action.

		.. note::

			This attribute is not used in current version. You can use :ref:`context <context>` instead.

	.. attribute:: priority

		An :obj:`int` that indicates the drone's priority number in case of a collision avoidance scheme. It is currently not used but may prove helpful for future versions.

	.. attribute:: sock_send

		A :obj:`socket.socket` object used for sending messages.

	.. attribute:: sock_receive

		A :obj:`socket.socket` object used for receiving messages.

	.. attribute:: t_send

		A :ref:`SendThread <send-thread>` object that initializes the :obj:`threading` responsible for sending the drone parameters.

	.. attribute:: t_receive

		A :ref:`ReceiveThread <receive-thread>` object that initializes the :obj:`threading` responsible for receiving the other drones' parameters and adding them to :ref:`msg_queue <msg-queue>`.

	.. attribute:: t_task

		A :ref:`ReceiveTaskThread <receive-task-thread>` object that initializes the :obj:`threading` responsible for processing the drone parameters received.

	.. method:: run()

		- Opens the network sockets depending the explicitly stated :ref:`protocol <protocol>`
		- Starts the threads responsible for sending (:any:`t_send`) and receiving messages (:any:`t_receive`, :any:`t_task`).

	.. method:: stop()

		Closes the :obj:`socket` objects, opened by :obj:`Networking.run()`.

	.. method:: create_udp_broadcast(address)

		Opens two UDP sockets, one for sending and one for receiving messages. The sockets are set for broadcast messages by invoking the :obj:`socket.SO_BROADCAST` flag.

		:param tuple address: The network address in ('address', 'port') format. 
		:return: exit_status = '1' if the method performs succesfully all the tasks, '0' otherwise.
		:rtype: int
		:raises socket.error: 

			When failing to run any :any:`socket` commands
	
	.. method:: populate_drones(scenario)

		A method that populates the :ref:`drones <drones-list>` list for testing purposes.

		:param str scenario: A string specifying the wanted scenario. Must be implemented inside the method.


.. _send-thread:

.. class:: send_thread.SendThread(threading.Thread, network)

	A subclass of :obj:`threading.Thread`. A thread that when it is started, is responsible for sending the drone's vehicle parameters at the address specified when creating a :ref:`Networking <networking>` instance. It is self invoked inside the :ref:`Networking <networking>` class. 

	The class needs a :obj:`socket` to be open for sending messages.

	:params:

		network: A :ref:`Networking <networking>` object.

	.. data:: daemon = True 

		It is set to True so that the thread can keep running until the main thread finishes

	.. method:: run()

		An infinite `while <https://docs.python.org/2/reference/compound_stmts.html#while>`_ loop that pickles the :ref:`vehicle_params <vehicle-params>` attribute and sends it as a message to the specified socket. :obj:`time.sleep` is invoked with the :ref:`POLL_RATE <poll-rate>` parameter in order to set the frequency of those messages.

		.. warning::

			Be careful when setting the frequency too high as this may flood the network and reduce performance. High frequency means small :ref:`POLL_RATE <poll-rate>` number.

		:raises exception: When failing to send the message.


.. _receive-thread:

.. class:: receive_thread.ReceiveThread(threading.Thread, network)

	A subclass of :obj:`threading.Thread`. A thread that when it is started, is responsible for receiving any incoming messages from the address specified when creating a :ref:`Networking <networking>` instance. It is self invoked inside the :ref:`Networking <networking>` class. 

	The class needs a bound :obj:`socket` to be open for receiving incoming messages.

	:params:

		network: A :ref:`Networking <networking>` object.

	.. data:: daemon = True 

		It is set to True so that the thread can keep running until the main thread finishes

	.. method:: run()

		An infinite `while <https://docs.python.org/2/reference/compound_stmts.html#while>`_ loop that awaits for incoming messages with the `select() <https://docs.python.org/2/library/select.html#select.select>`_ function.
		If the received message is coming from a drone, then it adds it to the :ref:`msg_queue <msg-queue>` queue for further processing.

		:raises pickle.UnpicklingError: if the receiving message is not pickled.
		:raises exception: if there is a problem with the sockets.


.. _receive-task-thread:

.. class:: receive_task_thread.ReceiveTaskThread

	A subclass of :obj:`threading.Thread`. A thread that when it is started, is responsible for processing any messages in :ref:`msg_queue <msg-queue>` queue. It is self invoked inside the :ref:`Networking <networking>` class. 

	:params:

		network: A :ref:`Networking <networking>` object.

	.. data:: daemon = True 

		It is set to True so that the thread can keep running until the main thread finishes

	.. method:: run()

		An infinite `while <https://docs.python.org/2/reference/compound_stmts.html#while>`_ loop that processes the received messages one by one. Processing goes as follows:

			- Unpickle the message

			- Add a timestamp 

			- Calculate the distance between the two drones by calling :obj:`geo_tools.get_distance_metres`

			- Reject message if distance is beyond the :ref:`SAFETY_ZONE <safety-zone>` limit.

			- Add to :ref:`drones <drones-list>` list for further processing by the :ref:`Collision Avoidance <collision-thread>` thread.

			.. note::

				:obj:`Queue.Queue` is thread-safe, so after finishing the task on the message, :obj:`Queue.Queue.task_done` has to be called.

		:raises Queue.Empty: If no messages are inside the :ref:`msg_queue <msg-queue>`.


.. _collision-thread:

.. class:: collision_avoidance.CollisionThread(threading.Thread)

	Handler for the collision avoidance part of the module. A specific :ref:`Networking <networking>` object needs to be running. Here is the whole API for collision avoidance matters like context switching, prioritization and control. 

	.. tip::

		This is the class where you can deploy your own cooperative collision avoidance algorithm and test it. Check out this :ref:`Guide <deploy-protocol>`.

	.. attribute:: near

		A :obj:`list` that stores all the drones in the :ref:`drones <drones-list>` list that belong inside the :ref:`SAFETY_ZONE <safety-zone>` but out of the :ref:`CRITICAL_ZONE <critical-zone>`.

	.. attribute:: critical

		A :obj:`list` that stores all the drones in the :ref:`drones <drones-list>` list that belong inside the :ref:`CRITICAL_ZONE <critical-zone>` only.

	.. attribute:: in_session

		It is :obj:`True` when a collision avoidance scheme is in action and :obj:`False` otherwise.

	.. _context:

	.. attribute:: context

		A :obj:`collections.namedtuple` to save pre-collision avoidance information. Currently what is saved is `mode <http://python.dronekit.io/automodule.html#dronekit.Vehicle.mode>`_, `mission <http://python.dronekit.io/automodule.html#dronekit.Vehicle.commands>`_ and `next waypoint <http://python.dronekit.io/automodule.html#dronekit.CommandSequence.next>`_.

	.. method:: run()

		Based on the protocol that is used during the instantiation of a :ref:`Networking <networking>` object, it calls repeatedly the according method. It is like a switch-case piece of code for routing to the desired protocol.

		.. note::

			Currently the only supported protocol is the 'naive priorities' one. If you want to experiment with other protocols, they must be implemented as a method inside this class.

		.. warning::

			The :obj:`time.sleep` function must be called so that the main thread can acquire the lock, too.

	.. method:: no_protocol()

		If no protocol is used then there is a simple :obj:`near` and :obj:`critical` lists update. Then the output of the nearby drones is printed on stdout by calling :ref:`print_drones_in_vicinity() <print-drones>`.

	.. _priorities-protocol:

	.. method:: priorities_protocol()

		Based on this protocol, whenever there are nearby drones, each drone acquires explicitly a priority number. Every drone waits until its priority number becomes '1', so they can start taking actions. Waiting means keeping the same position in all three axes (lat, lon, altitude). 

		.. warning::

			Because of the high latency observed, a drone demonstrates a "momentum". This is troublesome because the drone does not freeze instantly. Instead, it flies some metres away (depending the groundpseed) before it gradually stops. For this reason you have to be EXTRA CAREFUL when setting the :ref:`SAFETY_ZONE <safety-zone>` attribute. 

		The protocol goes as follows:

		- :obj:`give_priorities`

		- Update :obj:`near` and :obj:`critical` lists

		- :obj:`give_control` or :obj:`take_control` depending the priority number

		.. note::

			:ref:`CRITICAL_ZONE <critical-zone>` is not utilised so far but maybe it can prove useful in the future.

	.. method:: give_priorities()

		The algorithm to spread the priority numbers to drones. Since each drone must hold the same algorithm, the prioritization is global. 

		.. note::

			Some might naturally think that this algorithm has flaws. It has been tested under worst-case scenarios and has given good results. A possible worst-case scenario is when there are three drones in line but not all drones see all. You can easily see that this "chained-prioritization" works just fine with the 'priorities' protocol.

		Priorities are given based on the following rules:

		* If drone is grounded then its priority is low, exempting mode 'ARMED'

		* Then, if system status needs to be checked first, if it's critical it has the highest priority of all

		* Then, if status is OK then highest priority is given to highest mission importance first

		* Then, among same mission importances, highest priority is given to the ones with less capabilities

		* Then, if they have the same status, mission importance, capabilities fine-tune with battery level

		:return: Updates the :ref:`drones <drones-list>` list with the correct priorities.

	.. method:: take_control()

		Takes control from flight controller by changing the mode to POSHOLD and overriding the RC3 channel (throttle). If control is being taken for first time, then the state of the drone is first saved to :ref:`context <context>`.

		.. warning::

			- FIXME: The "momentum" until the drone finally stops lasts for about 3-4 seconds. During this time, if initial speed is 4-6 m/s, the drone can stop more than 10 metres off the initial :obj:`take_control` call.

			- FIXME: If previous mode was POSHOLD then context saving will not happen. POSHOLD is a manual mode though, so most probably there will be no commands to be executed.

		.. note::

			Defensive coding has to be performed for the change of mode since commands by the flight controller are dropped silently. 

	.. method:: give_control()

		Gives control to flight controller by restoring the drone's previous state and cancelling the RC3 channel override. The :obj:`in_session` variable is returned to :obj:`False` to signal the end of the collision avoidance actions. 

		Use case: If the mode before was 'AUTO' with loaded waypoints, then the drone will continue its mission from where it stopped. 

	.. method:: save_context()

		Invoked at the first time of :obj:`take_control()` call. All information is saved in the :ref:`context <context>` class attribute.

		What is currently saved is:

			- `Mode <http://python.dronekit.io/automodule.html#dronekit.Vehicle.mode>`_

			- `Mission (loaded waypoints) <http://python.dronekit.io/automodule.html#dronekit.Vehicle.commands>`_ 

			- `Next waypoint <http://python.dronekit.io/automodule.html#dronekit.CommandSequence.next>`_

		.. tip::

			You are free to change the information changed according to your own collision avoidance protocol. You can see all the parameters that can be saved at the `DK-Python API reference <http://python.dronekit.io/automodule.html>`_ and the `MAVLink messages (from line 2095) <https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml>`_.

	.. method:: restore_context()

		Restores drone's saved state. 

		.. tip::

			In order to load the saved commands, the vehicle has to be in `GUIDED <http://python.dronekit.io/guide/copter/guided_mode.html>`_ mode. 

		The procedure goes as follows:

			- Change to `GUIDED <http://python.dronekit.io/guide/copter/guided_mode.html>`_ mode

			- Clear all the uploaded commands

			- Upload the previous commands

			- Change mode and next_waypoint 

		.. note::

			Saving/Restoring the context does not have any meaning in the 'priorities' protocol, since no extra actions are given to the controller, just an RC override. These functions are useful if you want to set a different flight plan for your collision avoidance protocol. They are invoked inside the 'priorities' protocol for you to see that they are functional. 

	.. method:: update_drone_list()

		This function is responsible for:

			- Cleaning-up drones that have not given a heartbeat the last :ref:`MAX_STAY <max-stay>` seconds.

			- Updating the lists (quite pythonically) containing the drones that are inside the :ref:`SAFETY_ZONE <safety-zone>` and the :ref:`CRITICAL_ZONE <critical-zone>`

	.. _print-drones:

	.. method:: print_drones_in_vicinity()

		A simple function that prints to stdout the drones in the :obj:`near` and :obj:`critical` lists.

	.. method:: current_mission()

		A simple function that retrieves the commands uploaded to the controller. See more information `here <http://python.dronekit.io/guide/auto_mode.html>`_.

		:return: The mission object. See :obj:`restore_context()` for an example of how to access its variables.
		:rtype: `CommandSequence <http://python.dronekit.io/automodule.html#dronekit.CommandSequence>`_

	.. method:: get_priority_num()

		Accesses the :ref:`drones <drones-list>` list (should be called after the :obj:`give_priorities()` method) in order to get the vehicle's priority number. 

		:return: The priority number of the vehicle.
		:rtype: int

.. _params:

.. class:: params.Params([network=None, vehicle=None, dummy=False])

	This class provides all the necessary vehicle attributes for the collision avoidance handling. All these parameters are updated automatically according to a threshold that is defined separately for every parameter. The parameters are sent every :ref:`POLL_RATE <poll-rate>` seconds if a :ref:`SendThread <send-thread>` object is running.

	:param Networking network: The :ref:`Networking <networking>` object.
	:param Vehicle vehicle: The dronekit.Vehicle object.
	:param bool dummy: If set to True then dummy parameters are added, for testing purposes.

	.. attribute:: ID

		A random ID, unique for every drone running the code. Uniqueness is guaranteed by running the :obj:`uuid.uuid4` function. Please do :strong:`not` change this value by yourself. 

	.. attribute:: last_recv

		A timestamp that is given upon receiving the specific drone's parameters by :ref:`ReceiveTaskThread <receive-task-thread>`.

	.. attribute:: version

		The release version of dronekit that runs the vehicle.

	.. attribute:: ekf_ok

		:obj:`bool` value that indicates EKF functionality.

	.. attribute:: gps_fix

		An :obj:`int` value that indicates the vehicle's GPS fix. Must be '3' in order to work properly.

	.. attribute:: gps_sat

		:obj:`int` value that specifies the number of satellites the GPS receives from (ideally 4 or more).

	.. attribute:: gps_eph

		The EPH (horizontal accuracy).

	.. attribute:: gps_epv

		The EPV (vertical accuracy).

	.. attribute:: set_global_alt

		:obj:`bool` value that indicates if altitude in the global frame can be changed offboard.

	.. attribute:: set_attitude

		:obj:`bool` value that indicates if attitude (pitch, yaw, roll) can be changed offboard.

	.. attribute:: mode

		Current mode of vehicle. Access mode.name for an :obj:`str` representation.

	.. attribute:: global_alt

		Current altitude in the global frame.

	.. attribute:: global_lat

		Current latitude in the global frame.

	.. attribute:: global_lon

		Current longititude in the global frame.

	.. attribute:: distance_from_self

		How many metres away is the received drone from this vehicle. Set upon receiving the specific drone's parameters by :ref:`ReceiveTaskThread <receive-task-thread>`.

	.. attribute:: mission_importance

		Default value is 0. It is changed depending the use (check :ref:`Mission Importance Setter <importance-setter>` for more information)			

		- Level 0: for everyday users and hobbyists

		- Level 1: for commercial users/companies

		- Level 2: governmental missions, search&rescue, national security

	.. attribute:: heading

		Where the drone is heading, in degrees, with 0 degrees being the North.

	.. attribute:: next_wp

		Next waypoint number as obtained from uploaded commands.

	.. attribute:: next_wp_lat

		Next waypoint's global latitude.

	.. attribute:: next_wp_lon

		Next waypoint's global longitude.

	.. attribute:: next_wp_alt

		Next waypoint's global altitude.

	.. attribute:: battery_level

		Battery level in percentage (100% is full).

	.. attribute:: velocity

		3-axis velocity in m/s.

	.. attribute:: groundspeed

		Groundspeed in m/s.

	.. attribute:: airspeed

		Airspeed in m/s. It can be different from groundspeed because of location's air speed etc.

	.. attribute:: system_status

		Must be 'OK' to be able to fly. There are other statuses that indicate that something is wrong like 'EMERGENCY', 'CRITICAL' etc.

	.. tip::

		Some attributes might not be readily available just by accesing the dronekit.Vehicle object. One example is the signal that the :emphasis:`next` waypoint has changed. If `DK-Python <http://python.dronekit.io/automodule.html>`_ does not mention the attribute you want to observe/send, you will have to check the `MAVLink message list <https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml>`_ and use the @vehicle.on_message decorator. 

		:strong:`Example:`
		Checking if the next waypoint has changed. If it has changed print the destination coordinates and assign them to the vehicle's parameters for easier access`::

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
							print 'Waypoint', cmd.next, ' out of ', cmd.count, ':'
							pos = cmd.next - 1

							print 'Frame, Lat, Lon, Alt:', cmd[pos].frame, cmd[
							pos].x, cmd[pos].y, cmd[pos].z
							network.vehicle_params.next_wp_lat = cmd[pos].x
							network.vehicle_params.next_wp_lon = cmd[pos].y
							network.vehicle_params.next_wp_alt = cmd[pos].z
				except Exception, e:
					print "Error: ", e

	.. _add-listeners:

	.. method:: add_listeners(network, vehicle)

		Uses function decorators to observe the attributes in the :ref:`Params <params>` class. See the source code or read this :ref:`Guide <add-params>` for further information. Note that changes are thresholded in order to minimize writes. Thresholding has been done based on own experience, observations and for certain purposes.

		.. warning::

			If you add a new attribute in this class then you will have to add its observer in this function, too. Otherwise the value will not be updated and will send wrong information about it.

	.. _print-params-all:

	.. method:: print_all()

		Prints to stdout all the attributes observed in the :ref:`Params <params>` class.

	.. _importance-setter:

	.. method:: mission_importance(level)

		A setter function. This attribute must not be able to be changed by the user for safety reasons.
		Mission importance must have the following semantics:

			- Level 0: for everyday users and hobbyists

			- Level 1: for commercial users/companies

			- Level 2: governmental missions, search&rescue, national security

		.. warning::

			Since the language is Python and the script can be changed by anyone, it is not safe to assume that the mission_importance attribute performs its role. Server-based authentication has to be deployed in order to verify the change in mission_importance. Do :strong:`NOT` utilise it as it is now.


.. class:: geo_tools()

	A set of functions with code from here and there for geographical transformations. 

	.. method:: get_location_metres(lat, lon, alt, dNorth, dEast)
	
		Returns the latitude/longitude `dNorth` and `dEast` metres from the specified original location. The returned location has the same `alt` value as the original.

		The function is useful when you want to move the vehicle around specifying locations relative to 
		the current vehicle position.
		The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
		For more information see:
		http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters

		.. data:: earth_radius=6378137.0 

			According to theory.


		:param float lat: Current global latitude in decimal degrees
		:param float lon: Current global longitude in decimal degrees.
		:param float alt: Current relative altitude in decimal degrees.
		:param int dNorth: North offset in metres.
		:param int dEast: East offset in metres.
		:return: LocationGlobal: The new offset coordinates in decimal degrees.
		:rtype: tuple(lat, lon, alt)

	.. method:: get_distance_metres(lat1, lon1, lat2, lon2)

		Returns the ground distance in metres between two coordinate sets.
		This method is an approximation, and will not be accurate over large distances and close to the earth's poles. It comes from the ArduPilot test code: 
		https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py

		:param float lat1: First latitude in global frame in decimal degrees.
		:param float lon1: First longitude in global frame in decimal degrees.
		:param float lat2: Second latitude in global frame in decimal degrees.
		:param float lon2: Second longitude in global frame in decimal degrees.
		:return: Ground distance between coordinates in metres
		:rtype: float

	.. method:: distance_to_current_waypoint(vehicle)

	    Gets distance in metres to the current waypoint. 
	    It returns None for the first waypoint (Home location).

	    :param Vehicle vehicle: The dronekit.Vehicle object for which we seek distance to current waypoint.
	    :return: Distance to current waypoint in metres.
	    :rtype: float


Index
===============
:ref:`genindex`
