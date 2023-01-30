# home-robot-project

The home robot project.

To run it locally:
1. change line 65 in src/coppeliasim/coppeliasim_run/src/start_coppeliasim.cpp to provide the Coppeliasim path.
2. ```catkin build -j8``` (or whatever number of processors)
3. ```roslaunch fetch_coppeliasim simulation.launch```
