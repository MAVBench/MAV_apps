#!/bin/bash
cur_dir=$PWD 
MAVApps_folder="../../../../../MAV_apps"
cd ${MAVApps_folder}"/run_time/"
python supervisor.py&
cd $cur_dir
sleep 10
rosparam set knob_performance_modeling true
echo f 0 0 3 5
sleep 10
echo f 0 3 0 300
sleep 300
echo f 0 0 0 10
rosparam set knob_performance_modeling_for_om_to_pl true
rosparam set knob_performance_modeling_for_om_to_pl_no_interference true
sleep 500
echo kill > ${MAVApps_folder}"/data/package_delivery/supervisor_mailbox.txt"
#${MAVApss_folder}isc/generic/kill_all_ros.bash

