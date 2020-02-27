#!/usr/bin/env python
import rospy
import requests
from action_selectors.msg import RawInput
from intercom.msg import action_selector_cmd
from intercom.msg import response
import os
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
debug_option = True
#Im sorry but i gues this is better than creating the publisher in each callback.
pub = None
pub_resp = None
def debug(text):
    if(debug_option):
        print(str(text))

def say(text):
    #Call servicios Speakers
    print(text)
    sleep(1)


def callRASA(text, pub_resp):
    global response
    intent = ""
    args = [""]
    #make request to rasa server
    command = text
    # instantiate response msg object
    response_to_publish = response()
    # evaluate 
    say("You just said:" + command)
    data = {"sender": "home",
            "message": command}
    responseHTTP = requests.post("http://localhost:5005/webhooks/rest/webhook", json=data)
    nlu_response = requests.post("http://localhost:5005/model/parse", json={"text": data["message"]})
    if(responseHTTP.status_code == 200 and nlu_response.status_code ==200):   
        if(len(responseHTTP.json())>0):
            for responseData in responseHTTP.json():
                debug("BOT SAYS: "  + responseData["text"])
                response_to_publish.text = responseData["text"]
                pub_resp.publish(response_to_publish)
                nlu_info = nlu_response.json()
                debug(nlu_info["intent"])
                intent = nlu_info["intent"]
                args = nlu_info["entities"]
                debug("Entities: " + str(nlu_info["entities"]))
    else:
        response_to_publish.text = "Cant connect to RASA server"
        pub_resp.publish(response_to_publish)
        debug("Failed response")
    return intent, args


def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.inputText)
    global pub
    global pub_resp
    #open list of actions
    #call parser
    #send message to engine 
    
   
    #Here the parsing is done
    intent, args = callRASA(msg.inputText, pub_resp)




    action_request = action_selector_cmd()
    action_request.intent = intent
    action_request.args = args
    rospy.loginfo(action_request)
    pub.publish(action_request)
    

def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    global pub
    global pub_resp
    rospy.init_node('parser', anonymous=True)

    rospy.Subscriber("RawInput", RawInput, callback)
    pub_resp = rospy.Publisher('BotResponse', response, queue_size=10)
    pub = rospy.Publisher('action_requested', action_selector_cmd, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()