# coppeliasim_run
This package runs CoppeliaSim as a ROS node using a `system(..)` call.

### Usage

```
# to launch CoppeliaSim as a ROS node from a terminal 
rosrun coppeliasim_run start_coppeliasim [arguments]
```

#### Node Arguments: 
```
-h: runs CoppeliaSim in headless mode (i.e. without any GUI)
-sXXX: automatically start the simulation. XXX is simulation time in msec [optional]
-q: automatically quits CoppeliaSim after the first simulation run ended
-XXX.ttt: loads a CoppeliaSim scene
-XXX.ttm: loads a CoppeliaSim model
-XXX.brs: loads an XReality scene
-XXX.brm: loads an XReality model
```
