# Team PEGASUS : Self-Driving Car Nanodegree Capstone Project

## Team
## Anshoo Mehra
https://www.linkedin.com/in/anshoomehra
Contributions: Waypoint Publisher, Traffic Light Detection, Twist Controller, Traffic Light Detection using Google Object Detection API 

## Sumit Chhabra
https://www.linkedin.com/in/sumit-chhabra-577a568
Contributions: Waypoint Publisher, Traffic Light Detection, Twist Controller, Traffic Light Detection using Google Object Detection API, ROS Performance Tuning

## Xinghou Liu
## Korhan Mutludoğan
## Jerry Hu

# Objective 
Capstone project is the final project of Udacity Self-Driving Nanodegree. 

The goal of this project is to write ROS nodes implementing core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. As successful outcome car should drive itself on simulation software & Udacity's self-driving car CARLA.

Technolgies in play: [ROS] (www.ros.org), [Autoware] (www.ros.org https://github.com/CPFL/Autoware), Python, C++

### Architecture

Architecture is broadly segmented as below mentioned ROS Nodes, ROS Topics & Vehicle Subsystems.

![ROS Architecture](./imgs/ros-architecture.png)

High level overview of Vehicle Subsystems, rest of the components are sort of self explanatory.

**Perception Module:** Is the first sub-system in pipeline, with primary purpose sensing the environment to perceive traffic lights, and to be later improvised for any obstacles, traffic hazards,road signs etc.

**Planning Module:** Is the second sub-system in pipeline, with primary purpose as route planning, in this project, we have to publish the waypoints with target vecolocities. Velocities are determined based on feedback from perception module, example, if we have traffic light(red or yello signal) detected 100 meters ahead, we must gradually decrease speed to have vehicle have full stop at traffic stop line, else must gradually attain maximum speed defined. In actual implementation, this module will have additional maps data &localization information, along with perception module for precise route planning.

**Control Module:** Is the last sub-system in pipeline, with primary purpose finalizing trajectory computing & controlling yaw, throttle & brake.

### ROS Nodes Description

#### Waypoint Updater Node [ /waypoint_updater ] 

This node does four key things:
- Receives base wayp points (published only once during life cycle) by subscribing to /base_waypoints topic.
- Compute closest waypoint, by comparing euclidean distance between current pose (received by subscribing /current_pose topic) and /base_waypoints. Further computing ahead 40 waypoints.
- Receive real-time traffic light data by subscribing to /traffic_waypoint topic, this data would help compute the velocities of each waypoint ahead. (Note, thus far we have implemented obstacle detetion). To elaborate, /traffic_waypoint publishes stop line waypoint to the closest traffic_light, and it publishes this data if traffic light is in red or yellow state. else otherwise it will publish -1; in essence logic keep observing this data everytime pose of car is published, and compute either acceleration / deceleration based on this information. Stopping vehicle smoothly is tricky, it took us while to perfect this step, key is to keep consistency with number of waypoints across the board, we defined search_index hyper parameter to keep it consistent across the board. However, this step made us debug every possible data and subsequettly we learned overall architetcure much better. 
- Lastly, publish waypoints to /final_waypoints topic such that further susbsystems in pipeline can perform trajectory decisions.

#### DBW Node [ /dbw_node ]

Primary function of this node is to optimally compute vehicle's throttle, steering(yaw), brake, further publishing these to following topics,            /steering_cmd, /brake_cmd & throttle_cmd.  

This node subscribes to the following topics:

- **dbw_enabled**: Indicates if the car is under dbw or driver control.
- **current_velocity**: To receive the current velocity of the vehicle.
- **twist_cmd**: Target vehicle linear and angular velocities in the form of twist commands are published to this topic.

This node use twist_controller to compute Throttle, Yaw and Brake. We made use of basic PID & Yaw controller, given implementations gave us good results without any chnages as such. This was also a good refresher of old projects when we implemented PID Controller. PID controller receives current speed, target speed and delta time to compute rate of change for accceleration & deceleration. 

`Yaw Controller` controls the steering angle based on the current linear velocity and the target linear and angular velocity.

Brake value computed by PID are further improvised with other factors like vehicle mass, wheel radius, fuel ratio to compute effective torque needed in N/m.

In ideal scenario, other facrors like air pressure, weight of onboarded passengers, road traction, road slope will all impact braking and needs to be considered while computation.

#### Traffic Light Detection Node [ /tl_detector ]

Closest traffic light stop line is found using similar logic as waypoint_updater node to find closest waypoint. We have /current_pose of the vehcile & /vehicle/traffic_lights provides us with the location of the traffic light in 3D map space. Using these data sets we compute the closest traffic stop line to the car, in simulation mode, we also receive traffic ligh state along with the traffic light position from simulator, this state can be used without traning system to detect traffic light state, however, in real run on Carla, this state will not be avaible and traffic light state detetcion has to be trained. We planned using Google Object detection API, with model "Single Shot Multibox Detector (SSD) with MobileNet" for performance reasons. The other reason use Google API to reduce complexity with training, bounding boxes logic & most importantly get hands-on expereince of 3rd party API's as these are becoming very popular in real-life use-cases across industry. However, at the time of indiviual submitions, our code make use of simulator fed traffic light state instead of trained model.

TL Detector, only publishes closest stop line waypoint if traffic light state is RED or YELLOW else it will publish -1. This is key dependency for waypoint_updater to process acceleration and deceleratin logic.

<Google Object Detetcion API details to be added later ..>

### Youtube Link: <>

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 127.0.0.1:4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
