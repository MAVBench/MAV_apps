#!/bin/bash
./pre_mission.bash | roslaunch package_delivery package_delivery.launch voxel_type_to_publish:="free"  budgetting_mode:="performance_modeling" use_pyrun:=false  use_pyrun:=false knob_performance_modeling:=true capture_size:=1 max_time_budget:=20 clct_data_mode:="all"

