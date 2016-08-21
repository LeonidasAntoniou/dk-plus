Guide
*****

.. _how-to-broadcast:

Broadcasting a message
======================

In order to send a message to another drone, a specific sending address needs to be stated. Currently, the drone network uses the :strong:`192.168.2.255` address for sending/receiving messages, meaning the broadcast address of the :strong:`192.168.2.x` network.

.. note::

	In order to be able to send/receive messages at the 192.168.2.x address your system needs to be connected in this network and as such hold an IP address of the :strong:`192.168.2.x` form (e.g. 192.168.2.5). This network can be an :strong:`ad-hoc` network that creates your system and any devices with the necessary credentials can join. If you are running a Linux distribution, check out the :ref:`Quickstart <wireless-interface>`. You could also "hard-code" the IP addresses of your drones for testing purposes. 

.. note::

	Do not forget to ensure that your firewall rules accept incoming/outgoing packets of your specified network protocol. For UDP broadcast, check the :ref:`Quickstart <firewall>`.


1. Firstly, you will have to initialize your `Vehicle <http://python.dronekit.io/automodule.html#dronekit.Vehicle>`_ object as stated in :ref:`Quickstart <first-example>`. 

2. Before running your :ref:`Networking <networking>` thread, make sure you have implemented the networking protocol of your liking. If you are ok with the UDP broadcast then just run your :ref:`Networking <networking>` thread.

3. Your code will start sending your vehicle's parameters immediately! 

.. tip::

	If you want to check the outcome you can use a network protocol analyzer like `WireShark <https://wireshark.org>`_. You can also open another terminal and check if you receive any messages (more info on that on :ref:`Receiving a message <how-to-receive>`).


.. _how-to-receive:

Receiving a message
===================

The message receiver is also embedded in the :ref:`Networking <networking>` class. If you have followed the guide on :ref:`Broadcasting a message <how-to-broadcast>`, then your code is already in for accepting any incoming messages. 

.. _add-params:

Watching and exchanging new parameters
======================================

If you want to watch :strong:`and` share any new parameters other than the ones stated in the :ref:`Params <params>` class, you will have to add them to that class explicitly. The reason you want to do that is because whatever is written in the :ref:`Params <params>` class is being correctly updated and sent to the other drones automatically. 

.. note::

	You can always use a parameter by accessing it from your `Vehicle <http://python.dronekit.io/automodule.html#dronekit.Vehicle>`_ object, but that way you will use it only for that moment and only for a certain reason.

Let's suppose that `heading <http://python.dronekit.io/automodule.html#dronekit.Vehicle.heading>`_ was not in your watchlist and you want to add it:

	1. In the :ref:`Params <params>` class constructor, the according attribute is added::

		self.heading = vehicle.heading

	2. In the constructor, the attribute is initialized with the vehicle's current value. In order to keep it updated, you need to add a listener. In the :ref:`add_listeners() <add-listeners>` method write::

		#Decorator on vehicle's attribute 'heading'
		@vehicle.on_attribute('heading')		

		#State the function. value parameter is the value of 'heading' because this is the attribute we are creating the callback function for.					
		def decorated_heading_callback(self, attr_name, value):	

		#Add a thresholding: Do not overwrite if value is the same or has a difference of +-1 degree. The sensor is very sensitive so this is necessary.	
		if network.vehicle_params.heading == value or \
			network.vehicle_params.heading == (value + 1) or \
			network.vehicle_params.heading == (value - 1):
			pass

		#If the value is worth being overwritten, do so.
		else:
			network.vehicle_params.heading = value
			#print 'Heading changed to: ', network.vehicle_params.heading

	3. If you run a :ref:`Networking <networking>` instance in your code, then the new attribute will be shared among the drones.


.. _deploy-protocol:

Deploying and running a new collision avoidance protocol
========================================================

Any collision avoidance protocols are implemented in the :ref:`CollisionThread <collision-thread>` class. This allows for automatically handling the threading procedures and using a convenient API that deals with advanced control actions. 

Currently, the only protocol implemented is the :ref:`'priorities' protocol <priorities-protocol>` which is quite naive. You can edit the :obj:`collision_avoidance.py` file to add your own algorithms and methods.

There are some methods that can help towards your goal to cooperative collision avoidance, like saving/restoring state and prioritization. See the :ref:`API reference <collision-thread>` for a full list.

.. tip::

	If you have an active :ref:`Network <networking>` then simply instantiate and run the :ref:`Collision <collision-thread>` module to test your algorithm. 

Testing along with an `RPi <https://www.raspberrypi.org/documentation/>`_
=========================================================================

.. tip::

	The RaspberryPi 3 model B contains an embedded wireless modem and antenna. If you have earlier versions you can use a usb wireless module.

The RPi has got an ARM-based SoC and as such it does not support `dronekit-sitl <http://python.dronekit.io/develop/sitl_setup.html>`_. Do not forget though that the RPi will be the actual companion computer for your drone, so no worries! 

Let's assume that you want to see an actual message exchange between two systems under your established ad-hoc network. Because the RPi cannot be a simulated vehicle, you will have create a script that sends and receives messages, a beacon.

.. note::

	You will have to take care of the interfacing of your RPi as it will not be able to connect to your network otherwise. This :ref:`Guide <wireless-interface>` might give you a hint. 

Creating a beacon in RPi
------------------------

1. This beacon opens two UDP sockets to send/receive messages from the broadcast address. One thread is used for receiving and another for sending messages. 
Paste the following code to a new python file, e.g beacon.py::

	"""
	A simple program that sends/listens broadcast packets through UDP socket
	Used to test the system if it is able to send/receive packets
	"""
	import time, math, sys, socket, threading, select, uuid
	from collections import namedtuple
	import cPickle as pickle
	from params import Params

	POLL_RATE = 0.5
	MAX_STAY = 5 #seconds until entry is removed from structure
	Geo = namedtuple("Geo", "lat lon")
	simple_msg = namedtuple("simple_msg", "ID text")

	count = 0
	self_params = Params(dummy=True)
	self_params.global_lat = -35.362856
	self_params.global_lon = 149.164745

	# Set the socket parameters
	address = ('192.168.2.255', 54545)  # host, port
	sock_broad = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
	sock_broad.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

	# Create socket and bind to address
	sock_listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock_listen.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
	sock_listen.bind(address)

	def broad():
		global count
		while True:
			#msg = simple_msg(self_id,"I am here")
			msg = self_params
			if sock_broad.sendto(pickle.dumps(msg), address):
				count = count + 1
				pass
			else:
				print "Message failed to send"
			time.sleep(POLL_RATE)

	def listen():
		print "Waiting for message"
		while True:
			try:
				ready = select.select([sock_listen], [], [], 1.0)
				if ready[0]:
					d = sock_listen.recvfrom(4096)
					raw_msg = d[0]

					try:
						msg = pickle.loads(raw_msg)
						if msg.ID == self_params.ID:
							pass
						else:
							print "From addr: '%s', msg: '%s'" % (d[1], msg)
							#msg.print_all()
							pass

					except Exception, e:
						print "Error in receiving: ", e

			except socket.timeout:
				print "Reached timeout. Closing..."
				t_listen.cancel()
				sock_listen.close()

	t_listen = threading.Thread(target=listen)
	t_broad = threading.Thread(target=broad)

	t_listen.daemon = True
	t_broad.daemon = True

	print "Beacon ID is: ", self_params.ID

	t_listen.start()
	t_broad.start()

	time.sleep(100) #test for 100s
	print "Messages sent: ", count
	print "Closing beacon"

2. Before running your script in the RPi console, you might need to install the following packages::

	sudo apt-get update && sudo apt-get upgrade
	sudo apt-get install python-pip python-dev build-essential
	sudo pip install --upgrade pip  
	sudo pip install dronekit 

3. Do not forget to transfer your :obj:`params.py` file to the same directory where your :obj:`beacon.py` is so that `pickle <https://docs.python.org/2/library/pickle.html#module-cPickle>`_ can deserialize your message.

.. tip::

	If you don't know how to connect to your RPi, check out these: `1.Finding the IP address <https://www.raspberrypi.org/documentation/remote-access/ip-address.md>`_, `2.Connecting through SSH <https://www.raspberrypi.org/documentation/remote-access/ssh/README.md>`_

	For remote file access to your RPi you may find useful the remote file transfer program, `FileZilla <https://filezilla-project.org>`_::

		sudo apt-get install filezilla*


.. tip::

	The program ends after 100 seconds. For convenience you can :obj:`Ctrl-C` to end it earlier.


Watching the action
-------------------

1. Power-up your RPi and wait untill it is booted.

2. Connect both your systems in the same network.

3. Open a command window and ssh to your RPi, e.g::

	ssh pi@192.168.2.1

4. Open another command window, navigate to your code and run it.

5. On the RPi command window, navigate to your beacon.py code and run it.

6. Watch the action. On the RPi window you should see something like::

	Using MAVLink 1.0
	Beacon ID is:  8056
	Waiting for message
	From addr: '('192.168.2.7', 55486)', msg: '<params.Params instance at 0x75a5dbe8>'
	From addr: '('192.168.2.7', 55486)', msg: '<params.Params instance at 0x75a5dbe8>'
	From addr: '('192.168.2.7', 55486)', msg: '<params.Params instance at 0x75a5dbe8>'
	From addr: '('192.168.2.7', 55486)', msg: '<params.Params instance at 0x75a5dbe8>'
	From addr: '('192.168.2.7', 55486)', msg: '<params.Params instance at 0x75a5dbe8>'
	From addr: '('192.168.2.7', 55486)', msg: '<params.Params instance at 0x75a5dbe8>'
	From addr: '('192.168.2.7', 55486)', msg: '<params.Params instance at 0x75a5dbe8>'
	From addr: '('192.168.2.7', 55486)', msg: '<params.Params instance at 0x75a5dbe8>'
	...

.. tip::
	
	- You can change the code in order to add a timestamp if you like, or print specific attributes or all, through the :ref:`print_all() <print-params-all>` method.

	- The most "clean" way of shutting down your board is by typing::

		sudo halt

	and then remove from power after the green light stops emitting. 

