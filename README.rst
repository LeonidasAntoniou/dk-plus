In this branch there is a try to convert the collision_avoidance module to a process, in order to take advantage of multi-core systems. 

Current problems: dronekit.vehicle object needs to be accessed and partly modified by the collision_avoidance module. This object though is mutable/unpicklable and cannot be sent through a pipe, thus making it really unconvenient. 



Please visit the documentation page for all the information concerning the project:

https://leonidasantoniou.github.io/dk-plus/
