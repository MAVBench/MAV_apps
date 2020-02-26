#!/bin/bash
cur_dir=$PWD 
MAVApps_folder="../../../MAV_apps"
cd ${MAVApps_folder}"/run_time/"
python supervisor.py&
cd $cur_dir
#sleep 10
rosparam set knob_performance_modeling true


echo f 0 0 1 3
sleep 3
counter=1
while [ $counter -le 10 ]
do
    echo f 0 0 1 2
    sleep 20 
    ((counter++))
done

counter=1
while [ $counter -le 10 ]
do
    echo f 0 1 0 2
    sleep 20 
    ((counter++))
done

echo kill > ${MAVApps_folder}"/data/package_delivery/supervisor_mailbox.txt"

