# Project thesis 
For my master thesis project, which is a continuation of this project, see 
[here](github.com/pbullhove/master_thesis).

### Repository structure

## Installation

### Prerequesites
- Ubuntu 16.04


## Install ROS and Gazebo 
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Gazebo 7.0.0 comes with full installation of ROS Kinetic

## Clone this repository. 
```
git clone git@github.com:pbullhove/project_thesis
```
Clone tom13133's darknet_ros recursive inside /src
```
cd ~/project_thesis/catkin_ws/src
git clone --recursive git@github.com/tom13133/darknet_ros
```
This requires that you have set up your ssh keys for the recursive cloning to work. For a guide on how to do that see
[here](https://docs.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).

Build the project with
```
cd ~/project_thesis/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```
If that fails you might check and install your ROS dependencies, then try to build again. 
```
sudo apt-get install python-rosdep 
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```


## Add the nescessary Gazebo models
Add models to the hidden folder .gazebo/models
- Landing platform (helipad)
- ReVolt (revolt)

