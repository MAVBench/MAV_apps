END=27
num_of_processors=20
i_id=$(($num_of_processors - 1))
for (( i=$num_of_processors; i<=$END; i++ )); do
	echo 0 | sudo tee /sys/devices/system/cpu/cpu$i/online
done

for ((i=1; i<=$i_id; i++)); do
	echo 1 | sudo tee /sys/devices/system/cpu/cpu$i/online
done




