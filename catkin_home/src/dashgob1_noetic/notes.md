# Start Lidar
roslaunch ydlidar ydlidar1_up.launch

# Start Serial STM32
roslaunch dashgo_driver driver_imu.launch

# Check Linear Movement
rosrun dashgo_tools check_linear_imu.py

# Check Angular Movement
rosrun dashgo_tools check_angular_imu.py
