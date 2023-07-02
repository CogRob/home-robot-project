# Semantic Localization

Responsible for semantic annotations of map to 2D Pose conversions, all in `map` frame.
Requires the semantic map inside the `semantic_map` directory, with 2 files
1. `<mapname>.npy` - an image of `map.pgm` dimensions, with semantic label for each pixel (int64)
2. `<class_names.txt>` - the mapping of a semantic label number to the room name. 

Exposes 3 services

## SemanticLocalizer

Returns the name of the room that the robot is currently in.
Input : None
Output: Name of room

## PoseToSemanticLocation

Given a 2D location, return the room where the 2D location belongs to.
Input: Pose2D
Output: Name of room

## SemanticLocationToPose

Given the name of a room, return the 2D pose of the center. It gets the center of all pixels belonging to the room, and returns the 2D location of the room
Input: Name of room
Output: Pose2D of center 