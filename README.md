S-PTAM is a Stereo SLAM system able to compute the camera trajectory in real-time. It heavily exploits the parallel nature of the SLAM problem, separating the time-constrained pose estimation from less pressing matters such as map building and refinement tasks. On the other hand, the stereo setting allows to reconstruct a metric 3D map for each frame of stereo images, improving the accuracy of the mapping process with respect to monocular SLAM and avoiding the well-known bootstrapping problem. Also, the real scale of the environment is an essential feature for robots which have to interact with their surrounding workspace.

## Video
<a href="https://youtu.be/kq9DG5PQ2k8" target="_blank">
  <img src="https://github-production-user-asset-6210df.s3.amazonaws.com/6648400/260780126-f6a9ebab-d2f6-4a49-9111-d64355f408e4.png" alt="GNSS-Stereo-Inertial Fusion" width="700" />
</a>

## Related Publications:
[1] Taihú Pire, Thomas Fischer, Javier Civera, Pablo De Cristóforis and Julio Jacobo Berlles.  
**Stereo Parallel Tracking and Mapping for Robot Localization**  
Proc. of The International Conference on Intelligent Robots and Systems (IROS) (Accepted), Hamburg, Germany, 2015.

**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [License](#)
- [Disclaimer](#)
- [Prerequisites (dependencies)](#)
	- [ROS](#)
	- [g2o](#)
	- [PCL](#)
- [Installation](#)
- [Compilation](#)
	- [CMAKE flags](#)
- [ROS Package](#)
- [Tutorials](#)
	- [KITTI dataset](#)
	- [MIT Stata Center dataset](#)
	- [Indoor Level 7 S-Block dataset](#)
	- [Node Information](#)
		- [Subscribed Topics](#)
		- [Published Topics](#)
		- [Parameters](#)

# License

S-PTAM is released under GPLv3 license.

# Disclaimer
This site and the code provided here are under active development. Even though we try to only release working high quality code, this version might still contain some issues. Please use it with caution.

# Prerequisites (dependencies)

## ROS

We have tested S-PTAM in Ubuntu 16.04 with ROS Kinetic.

To install ROS (kinetic) use the following command:

`sudo apt-get install ros-kinetic-desktop`


## ros-utils

Install our ros-utils library from the source code provided in

`git clone git@github.com:lrse/ros-utils.git`




## g2o

Install [g2o](https://openslam.org/g2o.html) using the following command: 

`sudo apt install ros-kinetic-libg2o`  


## PCL

Install PCL using the following command: 

`sudo apt install ros-kinetic-pcl-ros`

# Installation

`https://github.com/CIFASIS/distributed-sptam`


# ROS Package

## Compilation

`catkin_make --pkg sptam -DSHOW_TRACKED_FRAMES=ON`

## CMAKE flags

SHOW_TRACKED_FRAMES=([ON|OFF], default: OFF)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Show the tracked frames by S-PTAM. Set it OFF to improve S-PTAM performance.

SHOW_PROFILING=([ON|OFF], default: ON)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Log in /tmp folder. Set it OFF to improve S-PTAM performance.

DSHOW_PRINTS=([ON|OFF], default: OFF)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Show info on screen. Set it OFF to improve S-PTAM performance.


# ROS Package

# Tutorials

We provide some examples of how to run with the most popular stereo datasets

## KITTI dataset

1. Download the KITTI rosbag [kitti_00.bag](http://www6.in.tum.de/~kloses/rvc/kitti_bags/kitti_00.bag) provided in   [KITTI rosbag files](http://www6.in.tum.de/~kloses/rvc/kitti_bags/)  

2. Uncompress the dataset  

	`rosbag decompress kitti_00.bag`  

3. Set use_sim_time ros variable true  

    `rosparam set use_sim_time true`  

4.  Play the dataset  

	`rosbag play kitti_00.bag`  
	
	(When S-PTAM run with the flag SHOW_TRACKED_FRAMES=ON the performance is reduced notoriusly).


## EuRoC dataset

1. Download the EuRoC [Machine Hall](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag) 01 rosbag provided in [EuRoC rosbag files](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)  

2. Uncompress the dataset  

	`rosbag decompress MH_01_easy.bag`  

3. Set use_sim_time ros variable true  

    `rosparam set use_sim_time true`  

4.  Play the dataset  

	`rosbag play MH_01_easy.bag`  
	
	(When S-PTAM run with the flag SHOW_TRACKED_FRAMES=ON the performance is reduced notoriusly).

## Execution 

Examples for KITTI dataset:

### Tracker & Mapper in same PC
	roslaunch sptam dsptam_kitti.launch

### Tracker only 
	roslaunch sptam dsptam_kitti-tracker.launch

### Distributed S-PTAM

#### Prerequisites 


In PC 1 (master):

	export ROS_IP=192.168.1.1


In PC 2 (client):

	export ROS_IP=192.168.1.2

	export ROS_MASTER_URI=http://192.168.1.1:11311

IP master PC and port 11311.

#### Launch files
In PC 1:

	roslaunch sptam dsptam_kitti-tracker.launch

In PC 2:

	roslaunch sptam dsptam_kitti-mapper.launch
