# Collaborative Object Transport
## 1. Repository overview
* `multi_robot_controller`: Controller and mathematics for formation control
* `multi_robot_msgs`: Message definitions for formation control
* `transport_controller`: Implementation of top level system control of a collaborative object transport
* `transport_launcher`: Launchfiles for launching the different elements within a collaborative object transport

## 2. Installation
### Install dependencies
```
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
### Build packages
Use your standard build system to build the downloaded packages
