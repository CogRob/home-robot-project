# Receptacle Navigator

This package navigates to a receptacle (perhaps from the center of the room) to place the object. It receives the a list of names of potential receptacles and navigates towards them.

Exposes 2 services and an action.

## Services

### GetReceptacles

Given a list of candidate receptacles, returns the subset of receptacles found in the room, along with their locations in the map.
Input: string of receptacles
output: NamedLocation() of receptacles (name + 2D pose) of receptacles

#### Working
1. It calls `move_head` to turn the head in different directions (0, -45, -90, + 45, + 90) degrees.
2. At each angle, calls `detect_receptacles` to get a list of receptacles in the scene with their 3D location (centroid of the receptacle) in camera frame
3. For all detected receptacles, if it is in the given set of candidate receptacles, it finds the location of the receptacle in the map frame.

### GetGoalPoseForReceptacle
Finds the goal pose for a receptacle. (A redundant? function). Takes the 2D pose of the receptacle in the map frame, and finds the final orientation angle by drawing a straight line from current location of robot to the 2D goal location.
Input: NamedLocation() of receptacle
Output: 2D Pose to navigate to.

## NavigateToReceptacle Action

Navigates to a receptacle
Input: NamedLocation of receptacle
Output: success or failure

Calls `GetGoalPoseForReceptacle` to get the 2D pose to navigate to, and navigates to the pose.

## Some important execution details
1. Since it looks only at (0, -45, -90, + 45, + 90) angles, if a receptacle is behind the robot, it may not find it. So might have to implement a `turn()` function to turn in place to get a full scan. Or probably call this service continuously while navigating to the room maybe?
2. The receptacle detector cannot detect countertop, so not sure what to do here
3. For `GetReceptacles`, it maintains a dictionary of seen receptacles. So if it finds a receptacle it found previously (maybe due to overlap), it will just rewrite the dictionary, which is probably bad. Maybe better to do filtering, or return a probability score from receptacle detector and choose the highest one?