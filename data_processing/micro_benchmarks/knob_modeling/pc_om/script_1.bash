#!/bin/bash

./pre_mission.bash | roslaunch package_delivery package_delivery.launch budgetting_mode:="performance_modeling" use_pyrun:=false knob_performance_modeling:=true voxel_type_to_publish:="free" capture_size:=1




