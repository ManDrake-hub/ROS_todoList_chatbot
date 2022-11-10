#!/bin/bash

BOT_DIR="../../../../rasa_bot"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
