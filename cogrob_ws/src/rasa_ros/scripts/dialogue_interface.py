#!/usr/bin/env python3

import rospy
import warnings
warnings.filterwarnings("ignore")
from rasa_ros.srv import Dialogue, ID
from pepper_nodes.srv import Text2Speech
from rasa_ros.msg import SAT
from std_msgs.msg import String, Int16MultiArray
import message_filters
import os

"""
Idea: Andiamo ad inserire il face recog nell'id server:
    1) Passiamo in aggiunta del soliti valori dell'audio, anche l'immagine della face
    2) Nell'id server utilizziamo le stesse tecniche per il riconoscimento della voce, ma sulla faccia
"""

class TerminalInterface:
    intent_goodbye = ["arrivederci", "addio", "ci sentiamo", "a risentirci", 
                      "ci vediamo", "buona giornata", "ci sentiamo in giro"]

    def __init__(self, id_service, dialogue_service) -> None:
        self.id_service = id_service
        self.dialogue_service = dialogue_service

        self.text2speech = rospy.Publisher("bot_answer", String, queue_size=10)

        self.waiting_name = False
        self.name = String(data="")
        self.save_to_file()

    def request_recognition(self, audio) -> None:
        """Sends an empty string to the id_service to request a recognition"""
        return self.id_service(audio, String(data="")).answer

    def add_relationship(self, audio) -> None:
        return self.id_service(audio, self.name)

    def say(self, phrase: String) -> None:
        self.text2speech.publish(phrase)

    def save_to_file(self)-> None:
        if self.name.data != "":
            with open("./name.txt", "w") as f:
                f.write(self.name.data)
        else:
            with open("./name.txt", "w") as f:
                f.write("default")

    def update_rasa_todo_list(self) -> None:
        phrase = "io sono "+ self.name.data
        bot_answer = self.dialogue_service(phrase)
        self.say(bot_answer.answer)
        self.save_to_file()
        print("bot answer: %s"%bot_answer.answer)

    #def write_to_rasa(self, phrase: String) -> String:
    #    return self.dialogue_service(phrase)

    def write_to_rasa_and_answer_aloud(self, phrase: String) -> None:
        bot_answer = self.dialogue_service(phrase.data)
        self.say(bot_answer.answer)
        print("bot answer: %s"%bot_answer.answer)
        

    def is_intent_goodbye(self, phrase: String) -> String:
        return phrase.data.lower() in TerminalInterface.intent_goodbye

    def get_name_from_phrase(self, phrase: String) -> String:
        return String(data=phrase.data.split(" ")[-1])

    def callback(self, data):
        phrase: String = data.text
        audio = data.audio

        if self.name.data != "":
            # If we have already loaded a name
            try:
                # Add new relationship
                self.add_relationship(audio)

                # Send to rasa and answer aloud
                self.write_to_rasa_and_answer_aloud(phrase)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            finally:
                # If intent was goodbye, clear the loaded name for the next person
                if self.is_intent_goodbye(phrase):
                    self.name.data = ""
                return

        if not self.waiting_name:
            # If we are not waiting for the response of the user and if we haven't already loaded a name
            try:
                # Ask the id_service for recognition
                id_answer: String = self.request_recognition(audio)

                if id_answer.data == "":
                    # If the id_service coudln't recognize the person
                    print("Couldn't recognize the person")
                    
                    # Ask the person for their name
                    self.text2speech.publish(String(data="Come ti chiami?"))
                    print("Come ti chiami?")
                    # Next time, wait for name
                    self.waiting_name = True
                else:
                    # If the id_service recognized the person
                    self.name = id_answer
                    self.write_to_rasa_and_answer_aloud(phrase)
                    self.update_rasa_todo_list()
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        else:
            # If was waiting for name
            self.waiting_name = False
            # Extract the name from the phrase
            real_name = self.get_name_from_phrase(phrase)
            # Save said name
            self.name = real_name
            # Add the new relationship to the dataset
            self.add_relationship(audio)

            # Update the rasa's todo list
            self.update_rasa_todo_list()

if __name__ == '__main__':
    rospy.init_node('writing')
    rospy.wait_for_service('dialogue_server')
    rospy.wait_for_service('id_server')
    print("IN:")

    try:
        dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
        id_service = rospy.ServiceProxy('id_server', ID)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    terminal = TerminalInterface(id_service, dialogue_service)

    rospy.Subscriber("voice_txt_data", SAT, terminal.callback)
    rospy.spin()