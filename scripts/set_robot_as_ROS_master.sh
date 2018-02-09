#!/bin/bash

# Get my IP
MY_IP=$(hostname -I | grep 192.168.137 | cut -f1 -d' ')
exit_status=$?
if [ $exit_status -eq 1 ]; then
    echo "ERROR: you do not have an IP on the robot network (192.168.137.x). Please set one. Aborting..."
    return 1
fi

# Check for online robots or default to localhost
echo -n "Searching for available robots"
TX1=192.168.137.212
TX2=192.168.137.213

# Ping robots
ping -c1 -W1 -q $TX1 > /dev/null 2>&1
TX1_UP=$?
echo -n "." # progress indication
ping -c1 -W1 -q $TX2 > /dev/null 2>&1
TX2_UP=$?
echo -n "." # progress indication

# Descision tree: based on ping, use IP of TX1 or TX2 or localhost or let the user pick between TX1 and TX2 if they're both available
if [ $TX1_UP -eq 0 ] && [ $TX2_UP -eq 0 ]; then
  echo "Detected both TX1 and TX2 are ONLINE."
  echo "Please pick (1) or (2)."
  read -p "Use TX1 or TX2?: " USER_PICK
  if [ $USER_PICK -eq 1 ]; then
    MASTER_PICK=$TX1
  elif [ $USER_PICK -eq 2 ]; then
    MASTER_PICK=$TX2
  fi
elif [ $TX1_UP -eq 0 ]; then
  MASTER_PICK=$TX1
elif [ $TX2_UP -eq 0 ]; then
  MASTER_PICK=$TX2
else
  MASTER_PICK=$MY_IP
fi

echo "Setting your ROS_IP to $MY_IP and ROS_MASTER to $MASTER_PICK!"
export ROS_IP=$MY_IP
export ROS_MASTER_URI=http://$MASTER_PICK:11311/
