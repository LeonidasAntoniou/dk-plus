"""
Experimenting with timestamps
"""
import time

timestamps = []

for i in range(0, 10):
	timestamps.append((time.time(), i))
	time.sleep(1)

print timestamps
timestamps = [times for times in timestamps if time.time() - times[0] <= 4]
print "The time now is: ", time.time()
print timestamps