"""
This piece of code is DEPRECATED
"""
"""
I will try to create a UDP socket for broadcasting messages.
In the same time, there is going to be a listener for messages as well.
"""

import socket 

port = 53005  # where do you expect to get a msg?
bufferSize = 4096 # whatever you need

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

for i in 0 to 10:
	s.sendto("Hey worldie", ('<broadcast>', port))
	

