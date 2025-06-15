#!/bin/bash

# Charger les environnements ROS2 et du workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Lancer les noeuds ROS2

# 1. Lancer teleop_node avec les paramètres depuis le fichier YAML
ros2 run my_car_controller serial_bridge &

# 2. Lancer joy_node
ros2 run serbot_controller serbot_controller_2 &


ros2 launch lslidar_driver lslidar_launch.py &

# 3. Lancer serial_bridge_2 (au premier plan)
ros2 launch serbot_description display.launch.py
