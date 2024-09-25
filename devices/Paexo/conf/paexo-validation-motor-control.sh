#!/bin/bash
# set n to 1
min_motor_pos=30
max_motor_pos=50
step_size=2

## Paexo initialization
echo "start" | yarp rpc /wearable/paexo/rpc:i
sleep 0.5
echo "en_bc_data" | yarp rpc /wearable/paexo/rpc:i
sleep 0.5
echo "init:r" | yarp rpc /wearable/paexo/rpc:i

# Set max position as initial motor position
current_pos=${max_motor_pos}

# continue until $n equals 5
while [ ${min_motor_pos} -le ${current_pos} ]
do
	read -s -n1 key
	#sleep 0.1
  echo "move:r:${current_pos}" | yarp rpc /wearable/paexo/rpc:i
  current_pos=$((current_pos - step_size))
done
