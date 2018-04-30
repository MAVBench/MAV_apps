#!/bin/bash
sleep 10
echo Press enter to rise >&2
read
echo f 0 0 1 2
echo Press enter to initialize SLAM >&2
read

echo f 0 0 1 2
sleep 3
echo f -1 0 0 3
sleep 3
echo f 0 1 0 3
sleep 3
echo f 1 0 0 3
sleep 3
echo f 0 -1 0 3
sleep 3

echo Press enter to fly to destination >&2
read
sleep 5
echo c 0 50 4
cat

