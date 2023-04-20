/*
 *  coppeliasim_run.cpp
 *
 *  Starts the coppeliasim Simulator using a system() call, so coppeliasim can be 
 *  started from a roslaunch file.
 *
 *  
 */

/*
 * Copyright (c) 2012, David Butterworth, KAIST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <iostream> // stringstream

/*

coppeliasim command line arguments: 
-h: runs CoppeliaSim in headless mode (i.e. without any GUI)

-sXXX: automatically start the simulation. 
    with XXX represents an optional simulation time in milliseconds after which simulation should stop again.

-q: automatically quits CoppeliaSim after the first simulation run ended.

-gREMOTEAPISERVERSERVICE_PORT_DEBUG_PREENABLESYNC: the argument can be used to request 
    a continuous legacy remote API server service to be started at CoppeliaSim start-up. 
    For that to happen, replace in above string following:
PORT is the port number
DEBUG is the debug mode (set to TRUE or FALSE)
PREENABLESYNC allows to preenable the synchronous mode (set to TRUE or FALSE)

-XXX.ttt: loads a CoppeliaSim scene.
-XXX.ttm: loads a CoppeliaSim model.
-XXX.brs: loads an XReality scene.
-XXX.brm: loads an XReality model.
*/

#define COPPELIASIM_EXECUTABLE "cd /root/CoppeliaSim && ./coppeliaSim.sh"
//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
    // node name: coppeliasim
    ros::init(argc, argv, "coppeliasim");
    ROS_INFO("Starting coppeliasim simulator... ");
    
    std::stringstream ss;
    switch  (argc){
        case 2:
            ss << COPPELIASIM_EXECUTABLE << " " << argv[1];
            break;

        case 3:
            ss << COPPELIASIM_EXECUTABLE << " " << argv[1] << " " << argv[2];
            break;

        case 4:
            ss << COPPELIASIM_EXECUTABLE << " " << argv[1] << " " << argv[2] << " " << argv[3];
            break;

        default:
            ss << COPPELIASIM_EXECUTABLE ;
    }

    if (system( ss.str().c_str() )) {}


    return 0;
}


