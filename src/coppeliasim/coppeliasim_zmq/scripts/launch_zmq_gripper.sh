#!/bin/bash

export PYTHONPATH="${PYTHONPATH}:/zmqRemoteApi/clients/python"
source /catkin_ws/devel/setup.bash
python3 /catkin_ws/src/coppeliasim/coppeliasim_zmq/scripts/attach_object.py