#!/usr/bin/env bash
python roguetwo_hardware/scripts/joy_to_ackermann.py &
echo "started: joy_to_ackermann.py"
python roguetwo_hardware/scripts/motor_servo_comm.py &
echo "started: motor_servo_comm.py"
python roguetwo_hardware/scripts/gps_comm.py &
echo "started: gps_comm.py"
python roguetwo_perception/scripts/gps_imu_to_se2.py &
echo "started: gps_imu_to_se2.py"
