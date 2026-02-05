# Autoware-CARLA-Agent-Controller
This package is a ROS bridge to control CARLA agent(Ego & NPC) from Autoware.
## Overview
<p float="left">
  <img src="https://github.com/user-attachments/assets/890121b4-56c4-4106-b80d-4e91d3d10ae4" width="49%" />
  <img src="https://github.com/user-attachments/assets/79a2e4ae-f6a9-4df3-99e6-2a8c55a91558" width="49%" /> 
</p>

## How to use
### requirement
Tested on
* **OS:** Ubuntu 22.04 LTS
* **GPU:** NVIDIA GeForce RTX 3060 Mobile
* **CPU:** 12th Gen Intel(R) Core(TM) i7-12700H
* **Middleware:** ROS 2 Humble

### 1．Download CARLA Simulator 
Download CARLA Simulator from [here](https://github.com/carla-simulator/carla/releases/tag/0.9.15/) and check if the simulator launches correctly. 
```
~/<path_to_CARLA>/CarlaUE4.sh
```
[NOTE]\
・0.9.15 ver is tested \
・For problems launching the simulator, please ask on the official CARLA community.


### 2．Install Autoware universe package
Install Autoware-Universe from [here](https://autowarefoundation.github.io/autoware-documentation/main/installation/)
and check if the Autoware planning simulator launches correctly. 
```
source ~/<autoware_path>/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=<map_path> vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```
[NOTE]\
・For more details about Autoware_planning_simulator, please check [here](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/). \
・For problems launching the Autoware-Universe, please ask on the official Autoware foundation community or Autoware-Universe github.
### 3. Download CARLA Map
Install CARLA lanelet2 and PCD Map([here](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/))
and please check CARLA map is displayed in Rviz with Autoware planning simulator.
```
source ~/<autoware_path>/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=<CARLA_map_path> vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```
### 4. Install This repo & build
```
mkdir -p ~/ros2_ws/src/
git clone https://github.com/tier4/carla-agent-controller.git ~/ros2_ws/src/carla-agent-controller
cd ~/ros2_ws
pip3 install carla==0.9.15 # recommend to install the same version as the CARLA simulator. 
colcon build
```

### 5. launch related packages
```
### Terminal 1: Launch CARLA simulator.
~/<path_to_CARLA>/CarlaUE4.sh
```
```
### Terminal 2: Launch Autoware planning simulator.
source ~/<autoware_path>/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=<your_CARLA_map_path> vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```
```
### Terminal 3: Launch this repo.
source ~/<autoware_path>/install/setup.bash
source ~/ros2_ws/install/setup.ash
ros2 launch carla_agent_controller carla_agent_controller_launch.py
```

### 6. Set Ego pose and put dummy agents
Put the Ego vehicle and other agents in Autoware planning simulator\
[NOTE]\
・Place the agent in autoware plannign simulator. Details is [here](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/placing-objects/#placing-interactive-dummy-objects) 
・prediction.launch.xml →　default = simpl
## Inputs
| Name                                | Type                                            | Description                           |
| ----------------------------------- | ----------------------------------------------- | ------------------------------------- |
| `/simulation/debug/localization/pose_estimator/pose_with_covariance`          | `geometry_msgs::msg::PoseWithCovarianceStamped` | Ego pose                          |
| `/perception/object_recognition/objects` | `autoware_perception_msgs::msg::PredictedObjects`   | Objects Pose |


## Future Plans
・Support pedestrian agent\
・Synchronize traffic lights between Autoware and Carla\
・Support Lanelet2 maps defined by MGRS.
