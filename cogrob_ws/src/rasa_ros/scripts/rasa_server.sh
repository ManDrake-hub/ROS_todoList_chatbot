#!/bin/bash

parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"
cd ../../../../chatbot

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
