# ROS_todoList_chatbot

## Application Demo
[![IMAGE_ALT](https://img.youtube.com/vi/aW27XjsQDg4/0.jpg)](https://www.youtube.com/watch?v=aW27XjsQDg4)

## Authors
Course: Cognitive Robotics 2022/2023
 
Lecturers: 
* Saggese Alessia    asaggese@unisa.it
* Roberto Antonio	 aroberto@unisa.it
 
Group:
* Mandragora Geraldino 0622701875    g.mandragora@studenti.unisa.it
* Sullutrone Giovanni  0622701751    g.sullutrone2@studenti.unisa.it
* Tirino Francesca     0622701745    f.tirino@studenti.unisa.it
* Farina Luigi         0622701754    l.farina16@studenti.unisa.it

## Install
Install rasa and ros as done during the lectures 

In case of errors:

* Unable to find either executable 'empy' or Python module 'em'...  try installing the package 'python3-empy'

  * ```pip install empy ```

* ImportError: "from catkin_pkg.package import parse_package" failed: No module named 'catkin_pkg'

  * ```pip install catkin-pkg``` 
  

* ModuleNotFoundError: No module named 'yaml'

  * ```pip install pyyaml``` 

* Could not load model due to Please confirm that it_core_news_md is an available spaCy model. You need to download one upfront. For example:
```python -m spacy download en_core_web_md``` 
  * ```python -m spacy download it_core_news_md``` 
* ModuleNotFoundError: No module named 'rospkg'
  * ```pip install rospkg``` 

## How to use
Follow these steps:
* cd into the folder ROS_todoList_chatbot/cogrob_ws
* ```chmod u+x src/rasa_ros/scripts/*``` 
* ```catkin build``` 
* cd into the parent directory, so ROS_todoList_chatbot/
* update the pepper ip in the file cogrob_ws/src/pepper_nodes/launch/pepper_nodes.launch
* update the http tablet url in cogrob_ws/src/pepper_nodes/src/tablet_node.py
* if you want to use the pepper camera you'll to uncomment the code in cogrob_ws/src/pepper_nodes/src/image_input_node.py 
* ```sh terminals.sh``` 

the "terminals.sh" command will start 5 scripts in separates tabs of the same gnome-terminal's window

* first one: starts the webpage 
* second one: starts the pepper nodes
* third one: starts the dialogue servicies
* fourth one: starts the identification server
* fifth one: starts the speech recognition service

## Dependencies

to run 
```console
sh terminals.sh 
```
you'll need to install gnome-terminal.<br />
on Ubuntu gnome terminal is installed by default, otherwise you'll need to install it with the following command:<br />

```console
sudo apt-get install gnome-terminal
```

## License
[GNU](https://choosealicense.com/licenses/gpl-3.0/)
