#include "simLib.h"
#include "simExtRosControl_server.h"
#include "simExtRosControl_plugin.h"

#include "ros/ros.h"
#include <iostream>

#define PLUGIN_VERSION 1

std::string simExtRosControl_pluginName = "simExtRosControl";
LIBRARY simLib; // the coppeliasim library that we will dynamically load and bind

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the plugin start routine (called just once, just after the plugin was loaded):
SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{
	// Dynamically load and bind coppeliasim functions:
	char curDirAndFile[1024];
	getcwd(curDirAndFile, sizeof(curDirAndFile));
	std::string currentDirAndPath(curDirAndFile);

	// Append the coppeliasim library's name:
	std::string temp(currentDirAndPath);
	temp+="/libcoppeliaSim.so";

	// Load the coppeliasim library:
	simLib=loadSimLibrary(temp.c_str());
	if (simLib==NULL)
	{
	    std::cout << "Error, could not find or correctly load the coppeliasim library. Cannot start '" << simExtRosControl_pluginName << "' plugin.\n";
		return(0); // Means error, coppeliasim will unload this plugin
	}
	if (getSimProcAddresses(simLib)==0)
	{
	    std::cout << "Error, could not find all required functions in the coppeliasim library. Cannot start '" << simExtRosControl_pluginName << "' plugin.\n";
		unloadSimLibrary(simLib);
		return(0); // Means error, coppeliasim will unload this plugin
	}

	// Check the version of coppeliasim:
	int sim_ver;
	simGetIntegerParameter(sim_intparam_program_version,&sim_ver);
	if (sim_ver<30102) // if coppeliasim version is smaller than 3.01.02
	{
	    std::cout << "Sorry, your coppeliasim copy is somewhat old. Cannot start '" << simExtRosControl_pluginName << "' plugin.\n";
		unloadSimLibrary(simLib);
		return(0); // Means error, coppeliasim will unload this plugin
	}

	// Initialize the ROS part:
	if(!ROS_server::initialize()) 
	{
	    std::cout << "ROS master is not running. Cannot start '" << simExtRosControl_pluginName << "' plugin.\n";
		return (0); //If the master is not running then the plugin is not loaded.
	}

	return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the plugin end routine (called just once, when coppeliasim is ending, i.e. releasing this plugin):
SIM_DLLEXPORT void simEnd()
{
	ROS_server::shutDown();	// shutdown the ROS_server
	unloadSimLibrary(simLib); // release the library
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the plugin messaging routine (i.e. coppeliasim calls this function very often, with various messages):
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ 
	// This is called quite often. Just watch out for messages/events you want to handle
	// Keep following 4 lines at the beginning and unchanged:
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
	void* retVal=NULL;

	// Here we can intercept many messages from coppeliasim (actually callbacks). 
	// Only the most important messages are listed here:
	if (message==sim_message_eventcallback_instancepass)
	{ 
		// This message is sent each time the scene was rendered (well, shortly after) (very often)
		// When a simulation is not running, but you still need to execute some commands, then put some code here
		ROS_server::instancePass();
	}

	if (message==sim_message_eventcallback_mainscriptabouttobecalled)
	{ 
		// Main script is about to be run (only called while a simulation is running (and not paused!))
		// This is a good location to execute simulation commands
		ROS_server::mainScriptAboutToBeCalled();
	}

	if (message==sim_message_eventcallback_simulationabouttostart)
	{ 
	    // Simulation is about to start
		ROS_server::simulationAboutToStart();
	}

	if (message==sim_message_eventcallback_simulationended)
	{ 
		// Simulation just ended
		ROS_server::simulationEnded();
	}

	// Keep following unchanged:
	simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
	return(retVal);
}

