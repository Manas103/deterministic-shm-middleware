#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/ros2"
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
