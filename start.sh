#!/bin/bash

# Charger les environnements ROS2 et du workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Lancer les noeuds ROS2

# 1. Lancer teleop_node avec les paramètres depuis le fichier YAML
ros2 run teleop_twist_joy teleop_node \
--ros-args \
--params-file ~/ros2_ws/src/joy_config_pkg/config/teleop.yaml &

# 2. Lancer joy_node
ros2 run joy joy_node &

# 3. Lancer serial_bridge_2 (au premier plan)
ros2 run my_car_controller serial_bridge
