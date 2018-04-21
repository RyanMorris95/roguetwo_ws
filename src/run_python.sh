#!/usr/bin/env bash
python roguetwo_hardware/scripts/joy_to_ackermann.py &
echo "started: joy_to_ackermann.py"
python roguetwo_hardware/scripts/motor_servo_comm.py &
echo "started: motor_servo_comm.py"
#python roguetwo_hardware/scripts/gps_comm.py &
#echo "started: gps_comm.py"
#python roguetwo_perception/scripts/gps_imu_to_se2.py &
#echo "started: gps_imu_to_se2.py"
python roguetwo_hardware/scripts/encoder.py &
echo "started: encoder.py"
#python roguetwo_hardware/scripts/lidar2.py lidar1 -s /dev/ttyUSB0 -p /lidar_front_left &
#echo "started: lidar1 -> front_left"
#python roguetwo_hardware/scripts/lidar2.py lidar2 -s /dev/ttyUSB1 -p /lidar_left&
#echo "started: lidar2 -> left"
#python roguetwo_hardware/scripts/lidar2.py lidar3 -s /dev/ttyUSB2 -p /lidar_right &
#echo "started: lidar3 -> right"
#python roguetwo_hardware/scripts/lidar2.py lidar4 -s /dev/ttyAMA0 -p /lidar_front_right &
#echo "started: lidar4 -> front_right"
