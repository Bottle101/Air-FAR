# README #

This README would normally document whatever steps are necessary to get your application up and running.

## What is this repository for?

Dynamic Routing Planner

## How to get the planner runing on your workspace 

Firstly, You may follow the instruction here to get the simulation workspace running.
[tutorials](https://www.cmu-exploration.com/)

## How to install and launch dynamic routing planner

#### Copy this planner workspace (Ask repo access from Fan Yang: fanyang2@alumni.cmu.edu)
```bash
git clone https://github.com/MichaelFYang/route_planner_ws.git
cd route_planner_ws/
catkin build
source devel/setup.bash
```

#### Run Autonomous Simulation (Change the ENV with the environment you want to run with.)
```bash
roslaunch vehicle_simulator system_ENV.launch
```

#### Launch dynamic routing planner
##### Notes: Go change simuluation flag to true if you run the planner in simualtion enviroment

```bash
cd <<YOUR WORKSPACE>>/src/dynamic_route_planner/config/
gedit default.yaml
"CHANGE is_simulation := true"
```
Launch the planner
```bash
roslaunch dynamic_planner dynamic_planner.launch
```

Change different configuration files (r.g. matterport environment)
```bash
roslaunch dynamic_planner dynamic_planner.launch config_file:=matterport
```

## Who do I talk to? 

Fan Yang
(fanyang2@alumni.cmu.edu)
