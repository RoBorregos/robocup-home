import requests
import pyttsx3
from gtts import gTTS
import os

engine = pyttsx3.init()
connected = False
response = requests.get("https://google.com")
if(response.status_code ==200):
    connected = True
    print("Connection established")


def disconnectedVoice(text):
    engine.say(text)
    engine.runAndWait()  

def connectedVoice(text): 
    tts = gTTS(text=text, lang='en')
    tts.save("play.mp3")
    os.system("mpg321 play.mp3")


while True:
    command = input("----commando---:")
    # make request
    # evaluate 
    data = {"sender": "home",
            "message": command}
    response = requests.post("http://localhost:5005/webhooks/rest/webhook", json=data)
    nlu_response = requests.post("http://localhost:5005/model/parse", json={"text": data["message"]})
    if(response.status_code == 200 and nlu_response.status_code ==200):   
        print(response.json())
        if(len(response.json())>0):
            for responseData in response.json():
                print(responseData["text"])
                nlu_info = nlu_response.json()
                print(nlu_info["intent"])
                print("Entities: " + str(nlu_info["entities"]))
                if(connected):
                    connectedVoice(responseData["text"])
                else:
                    disconnectedVoice(responseData["text"])
    else:
        print("Failed response")
