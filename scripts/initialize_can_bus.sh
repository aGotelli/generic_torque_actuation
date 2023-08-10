#! /bin/bash
CAN_NUM=7
BITRATE=1000000
QUEUE_LENGTH=1000

for (( i=0; i<=$CAN_NUM; i++ ))
do
    sudo ip link set down can$i  # Set can bus down
    sudo ip link set can$i type can bitrate $BITRATE  # Set the bitrate
    sudo ip link set up can$i  # Set can bus back up
    sudo ifconfig can$i txqueuelen $QUEUE_LENGTH  # Set the queue length
done

ip link show
netstat -i
