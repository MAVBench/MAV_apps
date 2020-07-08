#!/bin/bash
cur_dir=$PWD 
MAVApps_folder="../../../../../MAV_apps"
cd ${MAVApps_folder}"/run_time/"
python supervisor.py&
cd $cur_dir
sleep 10
echo f 0 0 2 15
sleep 20

echo f 2 0 0 25
sleep 30
echo f 0 2 0 200
sleep 200


echo f 2 0 0 25
sleep 30
echo f 0 -2 0 200
sleep 200
#
echo f 2 0 0 25
sleep 30
echo f 0 2 0 200
sleep 180
#
echo f 2 0 0 25
sleep 30
echo f 0 -2 0 200
sleep 200
##



echo f 0 3 0 4
sleep 30
echo f 0 0 0 10
sleep 1
echo c
echo 30 480 30
rosparam set knob_performance_modeling_for_piecewise_planner true
sleep 7000
echo kill > ${MAVApps_folder}"/data/package_delivery/supervisor_mailbox.txt"
#${MAVApss_folder}isc/generic/kill_all_ros.bash

