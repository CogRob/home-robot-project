#!/bin/bash
rm -rf download/
mkdir download
cp ../../../downloads/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu18_04.tar.xz download/
cp ../../../downloads/castxml.tar.xz download/
cp ../../../downloads/blender-3.4.0-linux-x64.tar.xz download/

docker build -t coppeliasim:melodic -f Dockerfile .
rm -rf download/
