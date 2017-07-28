Author: Kurt Meister   
Course: ECE 573 Spring 2017  
Team: meister  

# License
Copyright (c) 2017, Kurt Meister  
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

# About
## Overview
This package contains the meister team final release of the meister team ECE 573 Spring 2017 project. The objectives of this project are to create a package for use with the catvehicle testbed which will
consume data from as few CAT Vehicle sensors as possible in order to be able to publish polygons to the /detections topic
which represent the objects in the CAT Vehicle's current world as accurately as possible. Upon detecting that the
CAT Vehicle has come to a stop this package will create a gazebo world file containing models representing the objects detected
during the test.

## Nodes Contained in this Package
#### Lidar Filter Node
The lidar filter node is responsible for taking the point cloud produced by the CAT Vehicle's velodyne lidar sensor which is published to the "/catvehicle/lidar_points" topic and filtering out those points that do not represent the objects in the world around the CAT Vehicle. Points filtered out will be those detecting the hood of the car, the ground, and the false detections at the extreme of the lidar sensor's range. The filtered dataset is then published to the "/meister/filtered_lidar_points" topic.

#### Lidar Filter Node Parameters  
The following parameters can be used to alter the behavior of the Lidar Filter Node. Default values are in parentheses.
* **/lidar\_filter\_node/min\_height** (0.2) - sets the minimum height below which points should be excluded from the filtered dataset. Used to exclude the ground points.
* **/lidar_filter_node/min_distance** (10) - sets the minimum distance inside of which points should be excluded from the filtered dataset. Used to exclude the hood of the CAT Vehicle.
* **/lidar_filter_node/max_distance** (35) - sets the maximum distance beyond which point should be excluded from the filtered dataset. Used to exclude the points returned from the extents of the lidar sensor's range

#### State Update Node
The State Update Node is responsible for transforming the point cloud produced by the Lidar Filter Node into the fixed global coordinate system for the simulation. It publishes the transformed Point Cloud to the "/meister/transformed_lidar_points" topic.

#### State Update Node Parameters
none

#### Vehicle Status Monitor Node
The Vehicle Status Monitor Node is responsible for detecting when the CAT Vehicle model has started publising data, when the CAT Vehicle has started moving, and when it has stopped. It subscribes to the "/catvehicle/vel"
topic, and publishes vehicle status to the "/meister/vehicle_status" topic in the form of strings. Vehicle status is only published when it transitions to a new state. Possible states are "started," "in_motion," and "stopped."

#### Vehicle Status Monitor Node Parameters
The following parameters can be used to alter the behavior of the Vehicle State Monitor Node. Default values are in parentheses.
* **/vehicle\_status\_monitor/in\_motion\_threshold** (0.75) - sets the velocity value above which the CAT Vehicle will be considered to be "in_motion"
* **/vehicle\_status\_monitor/stopped\_threshold** (0.1) - when velocity falls below this value after having been in motion the catvehicel will be considered to be "stopped"

#### Obstacle Detector Node
The Obstacle Detector Node is resposible for processing the output of the State Update Node to detect the locations of objects in the CAT Vehicle's world. It publishes this data to the "/detections" topic in the form of polygons representing the location and area (in the horizontal plane) of the detected objects. The Obstacle Detector Node also monitors the output of the Vehicle Status Monitor, and when the status transitions to "stoped" within 60 seconds it will produce a world file containing rectangular prisms which represent each object it has detected up to that point.

#### Obstacle Detector Node Parameters
The following parameters can be used to modify the behavior of the Obstacle Detector Node.
* **/obstacle\_detector\_node/world\_template\_filename** - full path to a world file which will form the basis for the world file produced when the vehicle stops. Typically, this should be worlds/empty.world. If this is not provided the world file will not be created when the CAT Vehicle comes to a stop.
* **/obstacle\_detector\_node/world\_output\_filename** - full path to which the output worldfile should be written. if this is not provided a world file will not be created when the CAT Vehicle comes to a stop.

#### Laser Data Node
The Lidar data leaves a data deadzone near the front of the CAT Vehicle due to the lidar detection of the hood of the vehicle. As a result, it would be better to do close range (less than 10m) detection using the lasers on the front of the CAT Vehicle.  It should be possible to reduce the laser detection messages to a point cloud, which could be combined with the lidar data and processed using the Obstacle Detector Node Developed for the Alpha release.

#### Laser Data Node Parameters
The following parameters can be used to modify the behavior of the Laser Data Node. Defualt values are in parentheses.
* **/laser_data/max_range** (45) - Laser Scan points beyond this distance will be excluded from the point cloud produced by the Laser Data Node
## Requirements
The following requirements have been implemented by this package. Instructions for how to verify each requirement can be found in docs/meister-RequirementsVerification.pdf. A or B baseline status is denoted in parentheses

### 1. Lidar Filter Node
1.1. (B) The Lidar Filter Node shall produce a filtered dataset from the /catvehicle/lidar_points topic  
  1.1.1 (B) A distance threshold filter shall be used to exclude objects less than 1.5 m from the velodyne sensor in the horizontal plan in order to exclude the hood of the car  
  1.1.2 (B) A distance threshold filter shall be used to exclude the points returned from the extents of the /catvehicle/lidar_points topic.  
  1.1.3 (B) A height threshold filter shall be used to exclude the ground or near ground (< 0.2m) detections.    

### 2. State Update Node
2.1 (B) The State Update Node shall transform the position of the points in the point cloud output from the Lidar Filter into the /catvehicle/odom coordinate system.  
2.2 (B) The State Update Node shall publish the transformed data to the /meister/transformed_lidar_points topic.

### 3. Vehicle Status Monitor Node
3.1 (B) The vehicle status monitor shall publish "in_motion" to the "/meister/vehicle_status" topic when CAT Vehicle linear velocity exceeds a parameterized value  
3.2 (B) The vehicle status monitor shall publish "stopped" to the "/meister/vehicle_status" topic when CAT Vehicle linear velocity falls below a parameterized value after having been in motion.  
3.3 (B) The vehicle status monitor shall publish "started" to the "meister/vehicle_status" topic upon first receiving a message from the "/catvehicle/vel" topic.  

### 4. Obstacle Detector Node
4.1 (B) The Obstacle Detector Node shall consume the filtered and transformed point cloud from the State Update Model to identify objects within the arc of the CAT Vehicle’s Lidar sensor which are taller than 0.2 Meters.    
  4.1.1 (B) The Obstacle Detector Node shall publish data to the “/detections” topic containing polygons encompassing the footprint of detected objects.  
  4.1.2 (A) The Obstacle Detector Node shall combine lidar points within 0.5 m of each other  in the X/Y plane into a single object.   
  4.1.3 (B) Polygons shall be axis aligned (catvehicle/odom coordinate system) bounding rectangles.  
4.2 (B) (B) The Obstacle Detector shall be capable of producing a gazebo world file within 60 seconds of receiving a message indicating the vehicle has come to rest after having previously been in motion.  
  4.2.1 (B) The world file shall contain a single axis aligned (in the fixed global coordinate system) bounding volume for each object detected by the Obstacle Detector Node.  
  4.2.2 (B) Bounding volume height shall be the maximum height of any lidar or laser detections used by the Obstacle Detector Node to produce a “/detections” message for that object.  
4.3 (A) The obstacle detector node shall be capable of utilizing both the data from the State Update Node and the Laser Data Node for producing obstacle detections.

### 5. Laser Data Node
5.1 (A) The Laser Data Node shall be responsible for generating a point cloud from the laser detections from the “/catvehicle/front_laser_points” topic in the “/catvehicle/odom” coordinate system.

# Dependencies
* Ubuntu 14.04
* ROS Indigo
* Gazebo  
* catvehicle  
* obstaclestopper  
* catvehicle_hoffmanfollower*
* catvehicle_simulink
 * stopafterdistance*
* GoogleTest

\*Both the catvehicle_hoffmanfollower and the stopafterdistance packages are generated simulink code. as a result, you
should follow the following tutorials when setting them up.

[catvehicle_hoffmanfollower demo](http://cps-vo.org/node/31850)  
[stopafterdistance demo](http://cps-vo.org/node/34101)
# Video Demos
Final Release Demo Video can be found at:[https://youtu.be/2xX1goVbfcU](https://youtu.be/2xX1goVbfcU)  
Alpha Release Demo Video can be found at [https://youtu.be/-3saktAf2Gk](https://youtu.be/-3saktAf2Gk)  
Beta Release Demo Video can be found at: [https://youtu.be/MIEZXIWmMOI](https://youtu.be/MIEZXIWmMOI)  

# Setup
It is recommended that you start with the virtual machine provided by the catvehicle challenge [here](https://cps-vo.org/node/26585).
### Option 1: Cloning from github/gitlab (shown in beta release demo video)
*note: you'll need permission to access the ece573-2017S-meister/meister repo on git.engr.arizona.edu for this method*
1. open a terminal and enter the following
```
cd ~
mkdir -p catvehicle_ws/src
cd catvehicle_ws/src
catkin_init_workspace
git clone https://github.com/sprinkjm/catvehicle.git
git clone https://github.com/sprinkjm/obstaclestopper.git
git clone https://github.com/sprinkjm/catvehicle_simulink
git clone https://git.engr.arizona.edu/ece573-2017S-meister/meister.git
cd ..
```
2. follow the directions from the CAT Vehicle hoffman follower demo above to generate the catvehicle_hoffmanfollower package  
3. follow the directions from the stopafterdistance demo above to generate the stopafterdistance package
4. in the terminal from step 1 enter the following
```
catkin_make
catkin_make tests
source devel/setup.bash
```

### Option 2: Cloning from github and using the cvchallenge_final.tgz submitted to D2L
1. Open a terminal and enter the following
```
cd ~
mkdir -p catvehicle_ws/src
cd catvehicle_ws/src
catkin_init_workspace
git clone https://github.com/sprinkjm/catvehicle.git
git clone https://github.com/sprinkjm/obstaclestopper.git
```
2. Download and save the cvchallenge_final.tgz to ~/catvehicle_ws/src
3. In the terminal from step 1, do the following
```
tar zxf cvchallenge_final.tgz
cd ..
catkin_make
catkin_make tests
source devel/setup.bash
```
### Instructions For Setting Up GoogleTest
Open a terminal and enter the following
```
sudo apt-get install libgtest-dev
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
#copy or symlink libgtest.a and ligtest_main.a to /usr/lib folder
sudo cp *.a /usr/lib
```
# Running This Project
See the video demo for examples of this in action.
#### Launch files under meister/launch
* #### meister_only.launch
This launch file will launch only the nodes included in this package. You will have to manually launch the catvehicle, and any method you choose of controlling catvehicle velocity

To use this launch file, open a terminal and enter the following:
```
cd ~/catvehicle_ws
source devel/setup.bash
roslaunch meister meister_only.launch
```

* #### meister.launch
This launch file assumes all of the dependencies above are pressent. It will launch the CAT Vehicle, the Hoffman Follower, stopafterdistance, the nodes included in this package and a recorder instance which will record a bagfile containing everything needed by the "plot_obstacle_detector_data" function discussed below, and save it to ~/catvehicle_ws/src/meister/bagfiles/meister.bag. You can open the file directly to see which arguments it supports. Note that it defaults to launching the CAT Vehicle in the meister/worlds/world1.world worldfile, but you can override that using the worldfile argument.  

To use this launch file, open a terminal and enter the following:
```
cd ~/catvehicle_ws
source devel/setup.bash
roslaunch meister meister.launch gui:=true worldfile:=world1.world
```

#### Worldfile Options
The following is a list of worldfiles which can be used with the "meister" launch file above that takes the "worldfile"
argument. Alternatively, you can copy worlds from the catvehicle, task2 seed, or task3 seed into the meister/worlds
directory and
* world1.world
* world2.world

# Verifying Requirements
There is a complete plan for veryifying all of the requirements listed above: *docs/meister-RequirmentsVerification.pdf*. It describes the test setup required to verify all of the above requirements, each of the requirements, the test(s)s needed to verify that requirment, how to execute that test and what the expected output of the test should be.

In addition, I've provided two launch files which can be used for testing  and verifying this package:

### launch/verification.test
This launch file will execute all of the tests described in the verification plan assuming you've followed the directions
in the verification plan test setup to create the *bagfiles/system_verification.bag* file.

##### To Run launch/verification.test
open a terminal and enter the following (takes approx. 3 minutes to run)
```
cd ~/catvehicle_ws
source devel/setup.bash
rostest meister verification.test
```

### launch/run_all_tests.test
This launch file not only executes all of the tests in the test plan, but additional regression tests, alternate versions of the tests from the test plan and unit tests created during development that were not included in the final verification plan

##### To Run launch/run_all_tests.test
open a terminal and enter the following (takes approx. 3 minutes to run)
```
cd ~/catvehicle_ws
source devel/setup.bash
rostest meister run_all_tests.test
```

# Reviewing Recorded Data
In the meister/matlab directory, there is a matlab function called "plot_obstacle_detector_data" which can be used to plot the lidar data used in creating the /detection messages, the raw lidar points, the filtered lidar points, the transformed lidar points and the detections will all be plotted.

It takes two arguments.
1. the path to a bagfile containing recordings of the following topics
  * /catvehicle/lidar_points  
  * /meister/transformed_lidar_points  
  * /meister/filtered_lidar_points  
  * /detections  
2. an offset time in seconds from the start of the bagfile  

for example, if you wanted to plot data from 3 seconds into a bagfile named /some/really/cool/bagfile.bag you'd enter the following in matlab (you only need the addpath command one time):

```
addpath ~/catvehicle_ws/src/meister/matlab
plot_obstacle_detector_data('/some/really/cool/bagfile.bag', 3);

```

# Reviewing Generated Gazebo Worldfiles
If you'd like to view the worldfile created by running this package, open a terminal and enter the following commands:
```
cd ~/catvehicle_ws
source devel/setup.bash
gazebo src/meister/worlds/detections.world
```
