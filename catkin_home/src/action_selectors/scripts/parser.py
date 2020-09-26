#!/usr/bin/env python
import rospy
import requests
from intercom.msg import action_selector_cmd
from std_msgs.msg import String
from vizbox.msg import Story
import os
import csv
from time import sleep

'''
Voz: repetir de vuelta un comando de voz con el formato "Bring [OBJETO] from [LUGAR]."
Punto Inicial: enmedio de la arena.
Calificacion Base: 5 si entiende el objeto, 5 si entiende el lugar.
Reto Adicional: entender ambos el objeto y el lugar por medio de utilizar algun esquema de Interaccion Humano-Robot donde el robot
tiene la iniciativa de la conversacion; 2.5 puntos adicionales por la correcta comprension de ambos.

El robot comienza en el punto inicial, donde recibe el comando por voz en el formato "Bring [OBJETO] frome [LUGAR]."
Se puede utilizar otro esquema, pero no se daran puntos adicionales.
El robot navega al lugar pedido.
El robot agarra el objeto pedido.


'''


class Parser(object):
    def __init__(self, pub, pub_resp, pub_story, filename):
        self.debug_option = True
        # Im sorry but i gues this is b etter than creating the publisher in each callback.
        self.pub = pub
        self.pub_resp = pub_resp
        self.pub_story = pub_story
        self.possible_actions = self.loadActions(filename)

    def loadActions(self, filename):
        '''
        Returns a dictionary with the possible actions defined in the csv file
        {
            cmd_id:{cmd_category,cmd_priority,require_args}
        }
        '''
        directory = os.path.dirname(os.path.realpath(__file__))
        absolute_path_to_csv = os.path.join(directory, filename)
        print("Opening possible actions: "+absolute_path_to_csv)
        dictionary_possible_actions = dict()
        with open(absolute_path_to_csv, 'r') as action_file:
            dict_reader = csv.DictReader(action_file)
            for row in dict_reader:
                cmd_id = row.get('cmd_id')
                del row['cmd_id']
                dictionary_possible_actions[cmd_id] = dict(row)
        print("Closed possible actions")
        return dictionary_possible_actions

    def debug(self, text):
        if(self.debug_option):
            print(str(text))

    def say(self, text):
        # Call servicios Speakers
        print(text)
        sleep(1)

    def callRASA(self, text):
        global response
        intent = ""
        args = [""]
        # make request to rasa server
        command = text
        # Publish to Vizbox
        # instantiate response msg object
        response_to_publish = String()
        # evaluate
        self.say("You just said:" + command)
        data = {"sender": "home",
                "message": command}
        responseHTTP = requests.post(
            "http://localhost:5005/webhooks/rest/webhook", json=data)
        nlu_response = requests.post(
            "http://localhost:5005/model/parse", json={"text": data["message"]})
        if(responseHTTP.status_code == 200 and nlu_response.status_code == 200):
            if(len(responseHTTP.json()) > 0):
                for responseData in responseHTTP.json():
                    self.debug("BOT SAYS: " + responseData["text"])
                    response_to_publish.data = responseData["text"]
                    self.pub_resp.publish(response_to_publish)
                    nlu_info = nlu_response.json()
                    if(nlu_info["intent"]["confidence"] >= 0.60):
                        self.debug(nlu_info["intent"])
                        intent = nlu_info["intent"]["name"]
                        args = nlu_info["entities"]
                        self.debug("Entities: " + str(nlu_info["entities"]))
        else:
            response_to_publish.data = "Cant connect to RASA server"
            self.pub_resp.publish(response_to_publish)
            self.debug("Failed response")
        return intent, args

    def publish_bring_something(self, intent, args):
        target_location = ""
        target_object = ""
        story = Story()
        story.title = "Bring Something"
        story.storyline = []
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
            action_request.action_client_binded = self.possible_actions[action[0]
                                                                        ]['action_client_binded']
            rospy.loginfo(action_request)
            story.storyline.append(
                action_request.intent + " : " + action_request.args)
            self.pub.publish(action_request)
            print(self.possible_actions[intent])
        self.pub_story.publish(story)

    def callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
        # Here the parsing is done
        try:
            intent, args = self.callRASA(msg.data)
            pass
        except:
            response_to_publish = String()
            response_to_publish.data = "Cant connect to RASA server"
            self.pub_resp.publish(response_to_publish)
            self.debug("Failed response")
            pass

        if(self.possible_actions.get(intent)):
            if(intent == "bring_something"):
                self.publish_bring_something(intent, args)
            else:
                action_request = action_selector_cmd()
                action_request.intent = intent
                action_request.args = ''.join(args)
                action_request.action_client_binded = self.possible_actions[
                    intent]['action_client_binded']
                rospy.loginfo(action_request)
                self.pub.publish(action_request)


def main():
    rospy.init_node('parser', anonymous=True)
    FILENAME_OF_CSV = 'possible_actions.csv'

    pub_resp = rospy.Publisher('robot_text', String, queue_size=10)
    pub = rospy.Publisher('action_requested',
                          action_selector_cmd, queue_size=10)
    pub_story = rospy.Publisher("story", Story, queue_size=10)
    parser = Parser(pub, pub_resp, pub_story, "possible_actions.csv")
    rospy.Subscriber("operator_text", String, parser.callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
