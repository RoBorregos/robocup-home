import requests
import pyttsx3
from gtts import gTTS
from mpyg321.mpyg321 import MPyg321Player
import os
from time import sleep

engine = pyttsx3.init()
player = MPyg321Player()
connected = False
response = requests.get("https://google.com")
debug_option = True

def debug(text):
    if(debug_option):
        print(str(text))

if(response.status_code ==200):
    connected = True
    debug("Connection established")


def disconnectedVoice(text):
    engine.say(text)
    engine.runAndWait()  

def connectedVoice(text): 
    tts = gTTS(text=text, lang='en')
    tts.save("play.mp3")
    player.play_song('./play.mp3')
    debug("Saying...")
    size_string = len(text.split())
    debug('Time to sleep ' + str(0.3 *size_string))
    sleep(0.3 *size_string )
    debug("stopped")

def say(text):
    if(connected):
        connectedVoice(text)
    else:
        disconnectedVoice(text)
    sleep(1)

while True:
    command = input("----commando---:")
    # make request
    # evaluate 
    say("You just said:" + command)
    data = {"sender": "home",
            "message": command}
    response = requests.post("http://localhost:5005/webhooks/rest/webhook", json=data)
    nlu_response = requests.post("http://localhost:5005/model/parse", json={"text": data["message"]})
    if(response.status_code == 200 and nlu_response.status_code ==200):   
        if(len(response.json())>0):
            for responseData in response.json():
                debug("BOT SAYS: "  + responseData["text"])
                nlu_info = nlu_response.json()
                debug(nlu_info["intent"])
                debug("Entities: " + str(nlu_info["entities"]))
                say(responseData['text'])
                
    else:
        debug("Failed response")
