#!/bin/bash

BOT_DIR="/home/francesca/Scrivania/ROS_todoList_chatbot/chatbot"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
