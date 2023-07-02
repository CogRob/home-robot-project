# Room Graph Navigator

This module takes in the string of the room to navigate to, and navigates to it. Internally, it makes a call to the map to look up the coordinates of the room, and navigates to it.
It exposes an action, NavigateToRoom:

Internally, first calls `SemanticLocationToPose` to get 2D pose of the room, and then calls `local_path_planner` to move to the 2D pose using the `navfn` planner.

