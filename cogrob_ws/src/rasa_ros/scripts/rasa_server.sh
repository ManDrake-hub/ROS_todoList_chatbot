#!/bin/bash

BOT_DIR="/home/colander/projects/ROS_todoList_chatbot/chatbot/"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
