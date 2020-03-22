#!/bin/bash
cur_dir=$PWD 
MAVApps_folder="../../../../../MAV_apps"
cd ${MAVApps_folder}"/run_time/"
python supervisor.py&
cd $cur_dir
sleep 10
rosparam set budgetting_mode "performance_modeling"
rosparam set use_pyrun false
echo f 0 0 3 20 
sleep 20
#echo f 0 3 0 4
#sleep 30
echo f 0 0 0 10
sleep 1
echo c
echo 30 480 30
rosparam set knob_performance_modeling_for_piecewise_planner true
sleep 3000
echo kill > ${MAVApps_folder}"/data/package_delivery/supervisor_mailbox.txt"
#${MAVApss_folder}isc/generic/kill_all_ros.bash

