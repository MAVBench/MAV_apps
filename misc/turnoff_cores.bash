END=27
for ((i=5;i<=END;i++)); do
	echo 0 | sudo tee /sys/devices/system/cpu/cpu$i/online
done


