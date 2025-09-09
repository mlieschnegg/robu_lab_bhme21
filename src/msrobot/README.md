# CandyBot – ROS2 Package

Dieses Package enthält den Code und die URDF-Dateien für den CandyBot  
(Knickarm Master/Slave-Roboter zum Greifen von Zuckerl).

## Features
- URDF-Modell des CandyBot  
- Publisher/Subscriber für Master-Slave-Bewegung  
- Launch-Files zum Starten von Leader & Follower  

## Starten
```bash
colcon build
source install/setup.bash
ros2 launch candybot master_slave.launch.py
```
