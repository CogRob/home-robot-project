# Navigation module

This module is responsible for high level and low level navigation planning.

The packages that have been written are :
1. `local_path_planner`
2. `semantic_localization`
3. `room_graph_navigator`
4. `receptacle_navigator`
5. `navigation_bringup`
6. `object_goal_navigation` (not complete, and not even close to completion, it is not needed for the project now)

The other packages have been cloned, and can probably be installed with `apt`

## Local path planner
Simply makes a call to `move_base`. A wrapper to select which global planner is used. Can probably do away with this later.

## Semantic_localization
This package is responsible for 
1. providing the current room where the robot is.
2. Providing the room, given 2D coordinates
3. Get the 2D coordinates of a room (center of the room)

## Room Graph Navigator
Calls the global planner (navfn) to navigate to a room. Calls `semantic_localization` to get the 2D pose of a room and then calls `local_path_planner` to navigate to the room location

## Receptacle Navigator
Has two functions.
1. A service that returns all the receptacles with their 2D locations in the current room by looking around and calling receptacle detector.
2. Navigate to a receptacle (2D location) by calling `local_path_planner` with carrot planner

## Navigation bringup
Starts all the navigation servers