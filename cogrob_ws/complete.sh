#!/bin/bash
xdotool key ctrl+shift+t
source devel/setup.bash
roslaunch rasa_ros dialogue.xml
python3 main.py
