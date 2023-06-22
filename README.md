# Home Robot Project

The home robot project. The goal is to integrate multiple components into one project. The key components are:
1. Manipulation : to pick and place objects
2. Navigation: to move inside a "home" environment
3. Perception: For object detection and related modules
4. (Optional) : Coppeliasim for simulation

The goal is to run different modules in their own docker container.  

As of now, you need to build all 3 modules, but perhaps these dependencies can be managed better later.

All docker images are in `src/docker/` directory. The different docker images are :

1. `base_image` : contains a base installation of ros-melodic, which all other containers are built from.
2. `navigation` : contains installations required for navigation : primarily mapping (with slam_karto), localization (with amcl) and navigation (with move_base)
3. `perception` : contains a yolo detector docker image.
4. `tidy_module` : a simple ros-melodic image with dependencies for house tidying modules
5. `pipeline` : support for behavior trees and other messages
6. `coppeliasim` : running simulation

For `manipulation`, refer to Jiaming's docker image.

Each docker container is also accompanied by a `catkin build` command that builds the respective ros packages needed for the module.

## Instructions to build
Navigate to each directory inside the docker directory, and run `bash build.sh` to build the different docker images.
To run, some tools are provided in `docker/dockerlogin` directory.
1. `bash cogrob_docker_create.sh <cont_name> <image_name>` to create a container
2. `bash cogrob_docker_exec.sh <cont_name>` to enter a created container.


The different modules in detail are : 

# Fetch
Contains the default fetch modules with the controller and its application packages. The packages of interest for this project are:
1. fetch_moveit_config : that loads the controllers and `move_group` needed for manipulation
2. fetch_navigation: that launches `move_base` node for navigation.
3. fetch_description : contains the URDF of the robot.

# Navigation
Navigation package is responsible for localization and navigation. The different submodules of interest are:

### Fetch Navigation

This package requires extensive tuning.
Found inside `fetch/fetch_ros/fetch_navigation` package. The main file of interest is [`src/fetch/fetch_ros/fetch_navigation/launch/fetch_nav.launch`](src/fetch/fetch_ros/fetch_navigation/launch/fetch_nav.launch)`
This has been modified from the default package, and we run two planners. 
1. Navfn global planner, that runs in the `move_base_planner_navfn` namespace. This is the standard global planner, that takes any free location on the map as a goal and navigates towards it. This is used for room-room navigation, or a default location navigation. The config files are named just `*move_base*`
2. Carrot global planner that runs in the `move_base_planner_carrot` namespace. This planner can plan a path only in straight lines, and is useful when the goal location is on an obstacle. This is good for receptacle navigation, where the goal point is on the receptacle, and we navigate in a straight line to a point as close as possible to the receptacle. The config files are named `*move_base_carrot*`

Some values to start tuning are:
1. Local planner tolerance values in `config/move_base.yaml` and/or `config/move_base_carrot.yaml`
2. Velocity and acceleration proviles in `config/fetch/move_base.yaml` and/or `config/fetch/move_base_carrot.yaml`

### Local path planner.
Talks with `move_base` node of `fetch_navigation`. It receives a Pose2D as a goal, and sends a Pose2D goal either to Navfn or Carrot planner depending on the input. It runs an ActionServer with the "move_fetch_robot_base" namespace.

### Semantic Localization
Contains services that can perform:
1. map-to-pixel and pixel-to-map conversions, to go from a pixel map to a metric 2D pose (x,y) in map frame and vice-versa. (This should probably be moved to a package called map_utils)
2. Semantic map modules. Loads an npy file containing semantic annotations of a map, along with its label.txt giving number->category conversion. It offers two services:
    1. "semantic_localize" - to provide the "room" and Pose2D of the robot, i.e. which room it is in, and the exact coordinates in map frame.
    2. "semantic_location_to_pose" - given a room, provides the Pose2D center of the room.

Details can be found in `semantic_localization/scripts/localize.py`

### Room Graph Navigator
Contains a module that navigates to a room on request.
Relevant file : (src/navigation/room_graph_navigator/scripts/object_room_navigator.py)[src/navigation/room_graph_navigator/scripts/object_room_navigator.py]. 
Action namespace: "object_room_navigator"
Input : room (string)
Output: None
Effect: Navigates to the room, by calling "semantic_location_to_pose" from the Semantic Localization package to get the Pose2D of the room, and navigates to it.

### Receptacle Navigator
Contains a module that navigates to receptacles. Contains an action to navigate, and 2 services.
1. available_receptacles service - given a list of candidate receptacles, returns the receptacles that are found in the scene and their locations
Service namespace: available_receptacles
SRV name: GetReceptacleLocations
Input: list of receptacles to search for (list of strings)
Output: NamedLocation[] of receptacle name with its 2D pose.

2. receptor_approach_pose service - given a receptacle with its 2D pose, find a free location that is closest to it.
Service namespace: receptor_approach_pose
SRV name: GetGoalPoseForReceptacle
Input: NamedLocation of Receptacle with its 2D pose
Output: 2DPose to navigate to.

3. receptacle_navigator action - navigates to a receptacle
Given a receptacle name, calls receptor_approach_pose service to get a 2D location, and calls the carrot planner to drive in a straight line to it.
Action namespace: receptacle_navigator
Action name: NavigateToReceptacleAction
Input: 2D pose
Output: Success or Failure

### Navigation bringup
A package to launch the different navigation packages

# Perception
Responsible for creating ros services/actions for all perception related modules. The main ones are:
## vision_msgs
Standard ROS message formats for object detection messages for 2D and 3D. Important messages are: Detection2D, Detection2DArray

## Object detector
A server for detecting objects on demand.
Service namespace: detector_2d
SRV name: detect2DObject
Input: None
Output: Detection2DArray of 2D bounding boxes of detected objects in the scene

## Receptacle detector
Not done yet, but should be similar

## yolov5_ros
It is a constant publisher to the /yolov5/detections topic. This needs to run in Python3, so this directory is copied to the perception docker container when it is being created.

# Manipulation.
Refer to Jiaming's modules

# Tidy Module
Contains services to detect objects out of place, and a service to get the correct placements. Services are:
### objects_out_of_place_service
Service namespace: objects_out_of_place_service
SRV name: IdentifyMisplacedObjects
Input: None (implicitly should call camera)
Output: [(obj, room, recep)], i.e. list of (obj,room,recep) pairs of object that are out of place from the given camera view.

It calls semantic_localize first to get its current location and the room it is in.
Then needs to call receptacle detector to get the receptacle it is looking at.
Then calls object detector to get the objects on the receptacle.
This information of (obj, room, recep) of all objects found in the scene on receptacles is sent to the internal tidy module.
Finally outputs the objects that are out of place

#TODO
Split into two files. One containing just the tidy components, and the other with the ROS related things to be able to call semantic localize only when needed

### correct_object_placement_service
Service namespace: correct_object_placement_service
SRV name: GetCorrectPlacements
Input: (obj, room, recep) pair
Output: [room, [receptacles]]

Given an object, and its current location (room, receptacle), gets the candidates of placements it can go to.

# Main pipeline - service_robot_pipeline

Runs all the different parts in a sequence. There are two ways to do this.
1. Standard python script with instructions one after another
2. Behavior Trees

## Standard script
Found in (src/service_robot_pipeline/scripts/main_pipeline.py)[src/service_robot_pipeline/scripts/main_pipeline.py]. NOT COMPLETE YET. Just calls one module after another and takes care of message passing.

## Behavior Trees
Adds everything in a behavior tree. Individual behaviors are listed in (src/service_robot_pipeline/scripts/behaviors.py)[src/service_robot_pipeline/scripts/behaviors.py]


# Instructions to run
Open 6 terminals.

In each terminal, create an instance of a docker image

## Terminal 0 (Optional simulation)

```
bash src/docker/dockerlogin/cogrob_docker_create.sh homerobot_coppeliasim coppeliasim:melodic
source devel/setup.bash
roslaunch fetch_coppeliasim launch_simulation.launch
```


## Terminal 1

```
bash src/docker/dockerlogin/cogrob_docker_create.sh homerobot_yolo ros_yolo_homerobot:noetic
source devel/setup.bash
roslaunch yolov5_ros yolov5.launch
```

## Terminal 2

```
bash src/docker/dockerlogin/cogrob_docker_create.sh homerobot_navigation ros_navigation_homerobot:melodic
source devel/setup.bash
roslaunch navigation_bringup navigation_launch.launch
```

## Terminal 3

Replace with whatever Jiaming says
```
bash src/docker/dockerlogin/cogrob_docker_create.sh ros_sr_pipeline homerobot_pipeline:melodic
source devel/setup.bash
roslaunch manipulation_bringup manipulation_launch.launch
```

## Terminal 4
```
bash src/docker/dockerlogin/cogrob_docker_create.sh homerobot_tidymodule homerobot_tidy_services:melodic
source devel/setup.bash
roslaunch tidy_services tidy_services.launch
```

## Terminal 5
Start the object detector modules
```
bash src/docker/dockerlogin/cogrob_docker_create.sh homerobot_pipeline homerobot_pipeline:melodic
source devel/setup.bash
roslaunch service_robot_pipeline service_robot_launch.launch
```

## Terminal 6
```
bash src/docker/dockerlogin/cogrob_docker_exec.sh homerobot_pipeline
source devel/setup.bash
rosrun service_robot_pipeline main_pipeline(_bt).py
```

Everything should run