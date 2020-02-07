#!/bin/bash
sleep 10
echo f 0 0 3 5
sleep 10
echo f 0 2 0 100
sleep 100
rosparam set knob_performance_modeling_for_om_to_pl true
rosparam set knob_performance_modeling_for_om_to_pl_no_interference true
sleep 100
./../../../../misc/generic/kill_all_ros.bash

