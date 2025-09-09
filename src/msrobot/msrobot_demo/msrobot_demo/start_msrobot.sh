#!/bin/bash
#Dieses Skript wird durch die msrobot.desktop Datei aufgerufen. Die msrobot.desktop Datei muss sich im Autostart-Ordner des Benutzers befinden ~/.config/autostart/
#Beide Dateien start_msrobot.sh und msrobot.desktop müssen ausführbar sein! -> chmod +x
#killall -9 /usr/bin/python3
/usr/bin/python3 /home/robu/work/robotic/robu_ws/src/msrobot/msrobot/teleoperate_gui.py -gui > /home/robu/work/robotic/robu_ws/src/msrobot/msrobot/teleoperate_gui.log 2>&1
