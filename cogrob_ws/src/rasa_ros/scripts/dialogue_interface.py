#!/usr/bin/env python3
import pathlib
import rospy
import warnings
warnings.filterwarnings("ignore")
from rasa_ros.srv import Dialogue, ID
from pepper_nodes.srv import Text2Speech
from rasa_ros.msg import SAT
from std_msgs.msg import String, Int16MultiArray
import message_filters
import os
from typing import Any

"""
Idea: Andiamo ad inserire il face recog nell'id server:
    1) Passiamo in aggiunta del soliti valori dell'audio, anche l'immagine della face
    2) Nell'id server utilizziamo le stesse tecniche per il riconoscimento della voce, ma sulla faccia
"""

class InteractionManager:
    intent_goodbye = ["arrivederci", "addio", "ci sentiamo", "a risentirci", 
                      "ci vediamo", "buona giornata", "ci sentiamo in giro"]

    def __init__(self, input_topic="voice_txt_data", output_topic="bot_answer") -> None:
        """
        Initialize an interface between pepper and rasa
        """
        self.dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
        rospy.wait_for_service('dialogue_server')
        self.id_service = rospy.ServiceProxy('id_server', ID)
        rospy.wait_for_service('id_server')

        rospy.Subscriber(input_topic, SAT, self.callback)
        self.text2speech = rospy.Publisher(output_topic, String, queue_size=10)

        self.waiting_name = False
        self.name = String(data="")
        self.save_to_file()

    def request_recognition(self, audio: Any) -> None:
        """Sends an empty string to the id_service to request a recognition"""
        return self.id_service(audio, String(data="")).answer

    def add_relationship(self, audio: Any) -> None:
        """
        Send the audio to the id_service with the loaded name to add the new
        relationship to the dataset.
        """
        return self.id_service(audio, self.name)

    def say(self, phrase: String) -> None:
        """Use text 2 speech service to say a phrase aloud"""
        print("bot answer: %s"%phrase.data)
        self.text2speech.publish(phrase)

    def save_to_file(self)-> None:
        """Write the loaded name to a file name.txt"""
        folder = str(pathlib.Path(__file__).parent.resolve())
        if self.name.data != "":
            with open(folder+"/name.txt", "w") as f:
                f.write(self.name.data)
        else:
            with open(folder+"/name.txt", "w") as f:
                f.write("")

    def update_rasa_todo_list(self) -> None:
        """Write to rasa to change selected todo list"""
        phrase = "io sono "+ self.name.data
        bot_answer = self.dialogue_service(phrase)
        self.say(bot_answer.answer)
        self.save_to_file()

    def write_to_rasa(self, phrase: String) -> String:
        """Write to rasa"""
        return self.dialogue_service(phrase).answer

    def write_to_rasa_and_answer_aloud(self, phrase: String) -> None:
        """Write to rasa and say aloud its answer"""
        bot_answer = self.write_to_rasa(phrase.data)
        self.say(bot_answer)

    def is_intent_goodbye(self, phrase: String) -> String:
        """Check if the phrase has the intent to close the conversation"""
        return phrase.data.lower() in InteractionManager.intent_goodbye

    def get_name_from_phrase(self, phrase: String) -> String:
        """Extract the name from the provided phrase"""
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
                    self.save_to_file()
                return

        if not self.waiting_name:
            # If we are not waiting for the response of the user and if we haven't already loaded a name
            try:
                # Ask the id_service for recognition
                id_answer: String = self.request_recognition(audio)

                if id_answer.data == "":
                    # If the id_service coudln't recognize the person                    
                    # Ask the person for their name
                    print("Come ti chiami di sicurezza")
                    self.say(String(data="Come ti chiami?"))
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

    terminal = InteractionManager(input_topic="voice_txt_data", output_topic="bot_answer")

    print("IN:")
    rospy.spin()