#!/usr/bin/env python3
import rospy
import requests
from intercom.msg import action_selector_cmd
from std_msgs.msg import String, Bool
from action_selectors.msg import RawInput
import os
import json
import sys

from time import sleep

'''
Voz: repetir de vuelta un comando de voz con el formato "Bring [OBJETO] from [LUGAR]."
Punto Inicial: enmedio de la arena.
Calificacion Base: 5 si entiende el objeto, 5 si entiende el lugar.
Reto Adicional: entender ambos el objeto y el lugar por medio de utilizar algun esquema de Interaccion Humano-Robot donde el robot
tiene la iniciativa de la conversacion; 2.5 puntos adicionales por la correcta comprension de ambos.

El robot comienza en el punto inicial, donde recibe el comando por voz en el formato "Bring [OBJETO] from [LUGAR]."
Se puede utilizar otro esquema, pero no se daran puntos adicionales.
El robot navega al lugar pedido.
El robot agarra el objeto pedido.

'''

class Parser(object):
    JSON_FILENAME = 'possible_actions.json'
    REST_ENDPOINT = "http://localhost:5005/webhooks/rest/webhook"
    PARSE_ENDPOINT = "http://localhost:5005/model/parse"
    DEBUG = True

    def __init__(self):
        self.say_publisher = rospy.Publisher('robot_text', String, queue_size=10)
        self.possible_actions = self.loadActions()
        self.actions_publisher = rospy.Publisher('action_requested', action_selector_cmd, queue_size=10)
        self.someone_to_talk_suscriber = rospy.Subscriber('someoneToTalkStatus', Bool, self.someone_to_talk_callback)
        self.input_suscriber = rospy.Subscriber("RawInput", RawInput, self.callback)

    def loadActions(self):
        '''
        Returns a dictionary with the possible actions.
        {
            cmd_id:{ cmd_category, cmd_priority, require_args }
        }
        '''
        directory = os.path.dirname(os.path.realpath(__file__))
        absolute_path = os.path.join(directory, self.JSON_FILENAME)
        dictionary_possible_actions = dict()
        with open(absolute_path, 'r') as actions_file:
            dictionary_possible_actions = json.load(actions_file)
        return dictionary_possible_actions

    def debug(self, text):
        if(self.DEBUG):
            rospy.loginfo(text)

    def say(self, text):
        response = String(text)
        self.say_publisher.publish(response)

    def callRASA(self, command):
        intent = ""
        args = []

        self.say("You just said:" + command)
        data = { 
            "sender": "HOME", 
            "message": command 
        }

        rest_response = requests.post(self.REST_ENDPOINT, json = data)
        parse_response = requests.post(self.PARSE_ENDPOINT, json = { "text": command })

        if(rest_response.status_code == 200 and parse_response.status_code == 200):
            if(len(rest_response.json()) > 0 and len(parse_response.json()) > 0):
                for responseData in rest_response.json():
                    self.debug("BOT SAYS: " + responseData["text"])
                    self.say(responseData["text"])
                  
                nlu_info = parse_response.json()
                if(nlu_info["intent"]["confidence"] >= 0.60):
                    self.debug("Intent: " + str(nlu_info["intent"]))
                    self.debug("Entities: " + str(nlu_info["entities"]))
                    intent = nlu_info["intent"]["name"]
                    args = nlu_info["entities"]
        else:
            self.say("Cant connect to server")
            self.debug("Cant connect to server")

        return intent, args

    def publish_bring_something(self, intent, args):
        target_location = ""
        target_object = ""

        for entity in args:
            if(entity.get("entity") == 'object'):
                target_object = entity.get("value")
            elif(entity.get("entity") == 'place'):
                target_location = entity.get("value")

        actions_needed = [["go_to", target_location], ["approach", target_object], [
            "center", target_object], ["pick_up", target_object], ["go_to", "original_location"]]

        for action in actions_needed:
            arg = ""
            action_request = action_selector_cmd()
            action_request.intent = action[0]

            if(len(action) > 1):
                arg = action[1]

            action_request.args = arg
            action_request.action_client_binded = self.possible_actions[action[0]]['action_client_binded']

            self.actions_publisher.publish(action_request)

    def callback(self, msg):
        inputText = msg.inputText
        self.debug("I heard: " + inputText)
        
        # Rasa Parser
        try:
            intent, args = self.callRASA(inputText)

            if intent in self.possible_actions:
                if(intent == "bring_something"):
                    self.publish_bring_something(intent, args)
                else:
                    action_request = action_selector_cmd()
                    action_request.intent = intent
                    action_request.args = ''.join(args)
                    action_request.action_client_binded = self.possible_actions[intent]['action_client_binded']
                    self.actions_publisher.publish(action_request)
        except:
            self.say("Cant connect to Server")
            self.debug("Failed response")
            print("Unexpected error:" + str(sys.exc_info()[0]))

    def someone_to_talk_callback(self, msg):
        pass

def main():
    rospy.init_node('parser', anonymous=True)
    parser = Parser()
    parser.debug('Parser Initialized.')
    rospy.spin()


if __name__ == '__main__':
    main()
#  echo "options snd_hda_intel index=1" >  /etc/modprobe.d/default.conf