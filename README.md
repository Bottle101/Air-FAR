Air-FAR is a robust framework for 3D path planning that leverages a visibility graph updated dynamically for real-time replanning (<30ms for 300m trajectory). This planner represents the environment using 3D polyhedrons and employs a novel heuristic, multi-layer visibility graph construction algorithm to sidestep dimensional catastrophes. Its path planning guarantees asymptotic optimality through a combination of divide-and-conquer based path pruning and heuristic random sampling techniques. Full technical details will be disclosed in an upcoming paper.


<p align="center">
  <img src="img/factory.gif" alt="Demo" width="70%"/>
</p>

## Usage

The repository has been tested in Ubuntu 20.04 with ROS Noetic. Follow instructions in [Autonomous Exploration Development Environment](https://www.far-planner.com/) to setup the development environment. Make sure to checkout the branch that matches the computer setup, compile, and download the simulation environments.

To setup FAR Planner, clone the repository.
```
git clone https://github.com/Bottle101/Air-FAR.git
```
In a terminal, go to the folder, checkout the 'noetic' branch, and compile.
```
catkin_make
```
To run the code, go to the development environment folder in a terminal, source the ROS workspace, and launch.
```
source devel/setup.sh
roslaunch vehicle_simulator system_unity.launch
```
In another terminal, go to the FAR Planner folder, source the ROS workspace, and launch.
```
source devel/setup.sh
roslaunch airfar_planner airfar.launch
```
Now, users can send a waypoint by 3 step: 1. click the 'Goalpoint3D' button in RVIZ; 2. click a point in the map, hold the left mouse button and scroll the mouse wheel to adjust the altitude;  3. release the mouse. The vehicle will navigate to the goal and build a visibility graph (in cyan) along the way. Areas covered by the visibility graph become free space. When navigating in free space, the planner uses the built visibility graph, and when navigating in unknown space, the planner attempts to discover a way to the goal. By pressing the 'Reset Visibility Graph' button, the planner will reinitialize the visibility graph. 

<p align="center">
  <img src="img/campus.gif" alt="Method" width="70%"/>
</p>

Anytime during the navigation, users can use the control panel to navigate the vehicle by clicking the in the black box. The system will switch to *smart joystick* mode - the vehicle tries to follow the virtual joystick command and avoid collisions at the same time. To resume FAR Planner navigation, press the 'Resume Navigation to Goal' button or use the 'Goalpoint3D' button to set a new goal. Note that users can also use a PS3/4 or Xbox controller to repleace the virtual joystick.

<p align="center">
  <img src="img/control_panel.png" alt="ControlPanel" width="23%"/>
  &nbsp;&nbsp;&nbsp;
  <img src="img/ps3_controller.jpg" alt="PS3 Controller" width="55%"/>
</p>

## Change Configuration
The default configuration works well in most cases, if you are using **outdoor** environments and the planner runs too **slow** to fir your runtime requirement for your task, please use our configuration for large-scale complex outdoor environments. In `/airfar_planner/launch/airfar.launch`, change the `Line 7` :    
```
<arg name="config_file" default="outdoor"/>
```

## Author

[Botao He](https://github.com/Bottle101) (hebotao101@gmail.com)
