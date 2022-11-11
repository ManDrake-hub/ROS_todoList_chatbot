# ROS_todoList_chatbot

## Authors
Course: Cognitive Robotics 2022/2023
 
Lecturers: 
* Saggese Alessia    asaggese@unisa.it
* Roberto Antonio	 aroberto@unisa.it
 
Group:
* Farina Luigi         0622701754    l.farina16@studenti.unisa.it
* Mandragora Geraldino 0622701875    g.mandragora@studenti.unisa.it
* Sullutrone Giovanni  0622701751    g.sullutrone2@studenti.unisa.it
* Tirino Francesca     0622701745    f.tirino@studenti.unisa.it

## Install
Install rasa and ros as done during the lectures 

In case of errors:

* Unable to find either executable 'empy' or Python module 'em'...  try installing the package 'python3-empy'

  - pip install empy

* ImportError: "from catkin_pkg.package import parse_package" failed: No module named 'catkin_pkg'

  * pip install catkin-pkg

* ModuleNotFoundError: No module named 'yaml'

  * pip install pyyaml

* Could not load model due to Please confirm that it_core_news_md is an available spaCy model. You need to download one upfront. For example:
python -m spacy download en_core_web_md

  * python -m spacy download it_core_news_md

* ModuleNotFoundError: No module named 'rospkg'
  * pip install rospkg

## How to use
Follow these steps:
* Move into the folder ROS_todoList_chatbot/cogrob_ws
* chmod u+x src/rasa_ros/scripts/*
* catkin build
* source devel/setup.bash
* roslaunch rasa_ros dialogue.xml

## License
[GNU](https://choosealicense.com/licenses/gpl-3.0/)
