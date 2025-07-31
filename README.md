
# 🤖 Farm-Land AI Robot  
This project uses ROS 2, SLAM, soil sensors, and computer vision to recommend seeds and fertilizers and to automate watering.

## Launch Files
Run the SLAM, sensors, recommender, and pump control using the launch files in ros2_launch/.

## Nodes
- nav/: SLAM and navigation
- sensor/: Soil sensors and vision-based classification
- recommender/: Seed & fertilizer recommendation logic
- pump_ctrl/: Controls NodeMCU via MQTT

## Setup
- ROS 2 Humble or later
- Python 3.10
- Arduino IDE for NodeMCU
