# Collaborative Object Transport
## 1. Repository overview
* `multi_robot_controller`: Controller and mathematics for formation control
* `multi_robot_msgs`: Message definitions for formation control
* `transport_controller`: Implementation of top level system control of a collaborative object transport
* `transport_launcher`: Launchfiles for launching the different elements within a collaborative object transport

## 2. Installation
Start by changeing directory to your catkin workspace!
### Clone package
```
git clone https://github.com/matchRos/Collaborative_Transport.git src
```

### Install dependencies
Download and install the helper tools from https://github.com/ibMH/Helper_Tools manually.
Afterwards install missing dependencies by using rosdep (ROS_DISTRO has to be defined or substituted by your ROS destribution)
```
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
### Build packages
Use your standard build tools to build the downloaded packages e.g. : 
```
catkin_make
```
or
```
catkin build
```

