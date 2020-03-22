#!/bin/bash
cur_dir=$PWD 
MAVApps_folder="../../../../../MAV_apps"
cd ${MAVApps_folder}"/run_time/"
python supervisor.py&
cd $cur_dir
sleep 10
#rosparam set budgetting_mode "performance_modeling"
#rosparam set use_pyrun false
#rosparam set knob_performance_modeling true
echo f 0 0 3 10
sleep 15
rosparam set knob_performance_modeling_for_pc_om true
echo f 0 2 0 800
sleep 800
echo kill > ${MAVApps_folder}"/data/package_delivery/supervisor_mailbox.txt"
#${MAVApss_folder}isc/generic/kill_all_ros.bash

