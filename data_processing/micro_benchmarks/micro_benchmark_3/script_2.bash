#!/bin/bash
sleep 60
rostopic  pub /occupancy_map_node/save_map std_msgs/Bool "true"

