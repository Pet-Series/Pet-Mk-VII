########################################################################################
# (c) https://github.com/Pet-Series
#     https://github.com/Pet-Series/Pet-Mk-VII (seven)
#
# Maintainer: stefan.kull@gmail.com
# The MIT License (MIT)
#
# ROS2 lanchfile
# Launches ALL nodes for Pet-Mk.VII (aka "The Ackermann steering vehicle")
# This is the main launch-file when bring-up the robot.
# This is the entry point for Docker <ros-entrypoint.sh>
# 
# 1) Remember to add '<exec_depend>....</exec_depend>' in package.xml
# 2) Remember to add '(os.path.join('share', package_name), glob('launch/*.launch.py'))' in setup.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Launch node controllng the light-beacon
    light_beacon_node = Node( 
        package="pet_ros2_lightbeacon_pkg",
        executable="pet_light_beacon_node"
    )

    # Launch node controllng the LCD-display
    lcd_driver_node = Node( 
        package="pet_ros2_lcd_pkg",
        executable="pet_lcd_driver_node"
    )
  
    # Expose nodes
    ld.add_action(light_beacon_node)
    ld.add_action(lcd_driver_node)
    
    return ld