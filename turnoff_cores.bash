END=24
for ((i=1;i<=END;i++)); do
	echo 0 | sudo tee /sys/devices/system/cpu/cpu$i/online
done


