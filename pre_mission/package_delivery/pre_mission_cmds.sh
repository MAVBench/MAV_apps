#!/bin/bash
# sleep 10
# echo f 0 0 1 5
# sleep 5
# echo c
# sleep 3
# echo 30 120 6

#while true; do
#  str=$(rosnode list)
#  # Use the below when you want the output to contain some string
#  if [[ $str =~ future_collision ]]; then
#    break
#  fi
#  sleep 0.5
#done
#sleep 10

#python ~/catkin_ws/src/mav-bench/run_time/bind_nodes.py >/dev/null

## Factory case study
# echo s 2
# echo p
# echo s 3
# echo f 0 0 1 2
# echo s 3
# echo c 9.5 160 2

## Default
# echo f 0 0 1 2
# echo s 6
# echo f 0 0 1 2
# echo s 3
# 
# echo f -1 0 0 3
# echo s 3
# echo f 0 -1 0 3
# echo s 3
# echo fz 1 0 4 3
# echo fz 0 1 4 3
# 
# echo c 0 50 4

echo f 0 0 1 4
echo s 65
echo p

