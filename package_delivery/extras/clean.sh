#!/bin/bash
rosnode kill -a
killall package_delivery
killall occupancy_node_separated 
killall rosout
killall rosmaster

