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

echo f 0 0 1 4
echo s 4
echo c 0 280 5

