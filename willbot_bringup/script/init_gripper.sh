#!/bin/sh
sleep 1
echo "Initializing gripper..."
rosservice call --wait -v /gripper/init