from typing import Any, Text, Dict, List
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
import json




class ActionBring(Action):

     def name(self) -> Text:
         return "action_bring_something"


     def run(self, dispatcher: CollectingDispatcher,
             tracker: Tracker,
             domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
         object_in_place =tracker.get_slot("object")
         place=tracker.get_slot("place")
         dic={"Intent":'Bring something',
         'Object': object_in_place ,
         'Place': place
         }
         with open("jsons/results.json", "w") as write_file:
            json.dump(dic, write_file)

         dispatcher.utter_message("Json bring something created")

         return []