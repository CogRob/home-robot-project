# coppeliasim_msgs_srvs
This package is an update of the [vrep_common] package -by [Jonathan Cacace][]- which contains the definitation of the required msgs and srvs that enable ros to communicate with the coppeliasim simulator. this package is one of 5 ros packages to have a fully ros-integration of Coppeliasim.

### Building 
The following instructions assume that a catkin workspace has been created at `$HOME/catkin_ws` and Coppeliasim directory is placed at the Home directory `$HOME/CoppeliaSim`. you always can update the paths based on your machine setup.

```bash
# change to the src directory in your Catkin workspace
cd $HOME/catkin_ws/src

# Clone coppeliasim_msgs_srvs pkg 
git clone https://github.com/mahmoud-a-ali/coppeliasim_msgs_srvs

# change to the main Catkin workspace
cd ..

# build the workspace (using catkin_tools)
 catkin build
 
 # activate your workspace
 source $HOME/catkin_ws/devel/setup.bash
```
This package does not generate any excutables, only header files for different messages and services will be generated.

### Usage
This package contains 8 msgs and 99 srvs definitions files. Here is an explanation of few msgs/srvs defined in this package:
- `simRosStartSimulation.srv`: a ros service to start the simulation in coppeliasim. Request part of the service is empty,  The response is an `int` indicates if the simulation is successfuly started or not.For more information check the  [simStartSimulation()][] API. 
- `simRosLoadScene.srv`: a ros service to load a scene in coppeliasim. Request part of the service contains the scene file,  The response is an `int` indicates if the scene is successfuly loaded or not. For more information check the [simLoadScene()][] API.
- `SimInfo.msg`: A ros message contains information about the simulation state inside coppeliasim e.g. simulatorState, simulationTime, and timeStep. 
- ....

For more information about all the services/messages and how it can be used to communicate with Coppeliasim, you can check the [coppeliasim_APIs][] page.



### Example 
Refer to [coppeliasim_ros_services][] or [ur5_coppeliasim_roscontrol][] packages on how you can use the messages and services generated by the coppeliasim_msgs_srvs package to control the simulation inside coppeliasim using ros.

[coppeliasim_APIs]: https://www.coppeliarobotics.com/helpFiles/en/apiFunctionListCategory.htm
[coppeliasim_ros_services]: https://github.com/mahmoud-a-ali/coppeliasim_ros_services
[Coppeliasim_msgs_srvs]: https://github.com/mahmoud-a-ali/Coppeliasim_msgs_srvs
[vrep_plugin]: https://github.com/jocacace/vrep_plugin
[coppeliasim]: https://www.coppeliarobotics.com/
[ur5_coppeliasim_roscontrol]: https://github.com/tud-cor/ur5_coppeliasim_roscontrol
[simStartSimulation()]: https://www.coppeliarobotics.com/helpFiles/en/regularApi/simStartSimulation.htm
 [simLoadScene()]: https://www.coppeliarobotics.com/helpFiles/en/regularApi/simLoadScene.htm

[vrep_common]: https://github.com/jocacace/vrep_common