# main idea 
The main idea is to generate a path that meets the following requirements: 
(1)the vehicle moves at a steady velocity 
(2)the vehicle accelerates/decelerates slowly
(3)the path is drivable
(4)allow vehicle to pass a slow moving vehicle ahead 
(5)avoid vehicle collision 

## vechicle velocity and acceleration/deceleration 
To ensure the vehicle moves at a steady pace, a ref_vel is set. The waypoints interval space is set according to this ref_vel.
To ensure smooth acceleration/deceleration, velocity is added or substracted a small amount (.224) if needed. 

## drivable path 
To ensure the path is smooth and drivable, anchor points are chosen at far distances (30m, 60m and 90m).
Then spine library is used to fit a smooth spine funciton. The waypoints are sampled based on the fitted spine function using again
the ref_vel and the target distance (30m). 

## allow vehicle to pass a slow moving vehicle ahead 
A finite state machine is used to allow state transitiones. Such finite states include "Keep Lane", "Lane Change Left" 
and "Lane Change Right". The cost of state transition is based on the cost funciton which penalizes lane change and short distances
between the ego vehicle's location and predicted other vehicles' locations on the target lane. 

## avoid vehicle collision 
To avoid collision, if the vehicle is within a warning_gap distance of a vehicle ahead on the same lane, the ego vehicle will
try to slow down and swith to other lanes if it is safe. 
