# Introduction
This ROS package implements a state dependent LQR controller for high accuracy trajectory tracking. It was tested with PX4_SITL and Gazebo before being flown in an agile racing quad with a Raspberry Pi 3B companion computer.  

# Commands
Runs PX4, VRPN (VICON mocap), and relay pose data.
```
roslaunch lqr_controller mavros_vrpn.launch
```
Runs main controller and loads param.yaml file.
```
roslaunch lqr_controller lqr_euler.launch
```

# Future Plans
Working on developing a learning algorithm for inflight thrust mapping. Strain gauges are attached to a quadrotor to collect thrust measurements in flight and how the mapping of normalized motor input [0,1] to thrust changes as a result of voltage decrement, ground effects, and propeller downwash. 
## Videos:
<a href="https://www.youtube.com/watch?v=MLneSW5LoOI" target="_blank"><img src="https://img.youtube.com/vi/MLneSW5LoOI/hqdefault.jpg" 
alt="VINS" width="320" height="240" border="10" /></a>

<a href="https://youtu.be/oX6Q640imiw" target="_blank"><img src="https://img.youtube.com/vi/oX6Q640imiw/hqdefault.jpg" 
alt="VINS" width="320" height="240" border="10" /></a>

<a href="https://youtu.be/GEzpsT8T1fg" target="_blank"><img src="https://img.youtube.com/vi/GEzpsT8T1fg/hqdefault.jpg" 
alt="VINS" width="320" height="240" border="10" /></a>

<a href="https://youtu.be/aMhNqe3vxsc" target="_blank"><img src="https://img.youtube.com/vi/aMhNqe3vxsc/hqdefault.jpg" 
alt="VINS" width="320" height="240" border="10" /></a>

<a href="https://www.youtube.com/watch?v=phKAAQSkkjc" target="_blank"><img src="https://img.youtube.com/vi/phKAAQSkkjc/hqdefault.jpg" 
alt="VINS" width="320" height="240" border="10" /></a>

<a href="https://www.youtube.com/watch?v=Arlwatxqgfo" target="_blank"><img src="https://img.youtube.com/vi/Arlwatxqgfo/hqdefault.jpg" 
alt="VINS" width="320" height="240" border="10" /></a>


## Acknowledgements:
This state dependent LQR controller is an extension of [1] for euler angles and quaternions.
## References:
[1] Foehn, Philipp & Scaramuzza, Davide. (2018). Onboard State Dependent LQR for Agile Quadrotors. 10.1109/ICRA.2018.8460885. 
