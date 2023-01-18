#! bin/bash
gnome-terminal --tab -e "bash -c 'cd cogrob_ws/src/tablet; python3 main.py; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'cd cogrob_ws; source devel/setup.bash; roslaunch pepper_nodes pepper_nodes.launch; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'cd cogrob_ws; source devel/setup.bash; roslaunch rasa_ros dialogue.launch; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'cd cogrob_ws; source devel/setup.bash; roslaunch rasa_ros id_server.launch; exec bash'"
gnome-terminal --tab -e "bash -c 'cd cogrob_ws; source devel/setup.bash; roslaunch ros_audio_pkg speech_recognition.launch; exec bash'"
#PROVVISORIO AL POSTO DEL SECONDO COMMENTATO
gnome-terminal --tab -e "bash -c 'cd cogrob_ws; source devel/setup.bash; roslaunch pepper_nodes face_cv.launch; exec bash'"
#gnome-terminal --tab -e "bash -c 'cd cogrob_ws; source devel/setup.bash; roslaunch pepper_nodes audio.launch; exec bash'"





