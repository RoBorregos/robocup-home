import requests
import pyttsx3

engine = pyttsx3.init()
while True:
    command = input("----commando---:")
    # make request
    # evaluate 
    data = {"sender": "home",
            "message": command}
    response = requests.post("http://localhost:5005/webhooks/rest/webhook", json=data)
    nlu_response = requests.post("http://localhost:5005/model/parse", json={"text": data["message"]})
    if(response.status_code == 200 and nlu_response.status_code ==200):   
        responseData = response.json()[0]
        print(responseData["text"])
        nlu_info = nlu_response.json()
        print(nlu_info["intent"])
        print("Entities: " + str(nlu_info["entities"]))
        #engine.say(responseData["text"])
        #engine.runAndWait()
