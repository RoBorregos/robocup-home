from typing import Any, Text, Dict, List
from rasa_sdk import Action, Tracker
from rasa_sdk.events import SlotSet
from rasa_sdk.executor import CollectingDispatcher
import json
import os.path
from os import path
from rasa_sdk.events import UserUtteranceReverted
from rasa_sdk.executor import CollectingDispatcher


#bring something
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

         dispatcher.utter_message()

         return []

class ActionStore(Action):

     def name(self) -> Text:
         return "action_store_object"

     def run(self, dispatcher: CollectingDispatcher,
             tracker: Tracker,
             domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
         object_in_place =tracker.get_slot("object")
         place=tracker.get_slot("place")
         dic={"Intent":'Store object',
         'Object': object_in_place ,
         'Place': place
         }
         with open("jsons/results.json", "w") as write_file:
            json.dump(dic, write_file)

         dispatcher.utter_message()

         return []

#action tidy up
class ActionClean(Action):

     def name(self) -> Text:
         return "action_clean_up"

     def run(self, dispatcher: CollectingDispatcher,
             tracker: Tracker,
             domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
    
         place=tracker.get_slot("place")
         dic={"Intent":'Clean',
         'Place': place
         }
         with open("jsons/results.json", "w") as write_file:
            json.dump(dic, write_file)

         dispatcher.utter_message()

         return []



#search for people
class ActionSearch_for_people(Action):

    def name(self) -> Text:
        return "action_search_for_people"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        name =tracker.get_slot("names")
        dic={"Intent":'Search for people',
        'name': name
         }
        with open("jsons/results.json", "w") as write_file:
            json.dump(dic, write_file)

        dispatcher.utter_message() 
        
        #espera a que exista el archivo results, este solo contiene un dic con el key person location y el valor de la location
        # una vez que lo recive lo manda a rasa 
        while True:
            if path.isfile("jsons/found.json"):
                with open("jsons/found.json","r") as f:
                    data = json.load(f)
                break
            
        status=data['person_location']

        return [SlotSet("place", status)]
#farewell go along
class ActionFarewell_yes(Action):

     def name(self) -> Text:
         return "action_farewell_yes"


     def run(self, dispatcher: CollectingDispatcher,
             tracker: Tracker,
             domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
         objeto =tracker.get_slot("object")
         color=tracker.get_slot("color")
         dic={"Intent":'Farewell',
         'Object': objeto ,
         'Color': color,
         'Go_along': "yes"
         }
         with open("jsons/results.json", "w") as write_file:
            json.dump(dic, write_file)

         dispatcher.utter_message()

         return []

#farewell not along
class ActionFarewell_no(Action):

     def name(self) -> Text:
         return "action_farewell_no"


     def run(self, dispatcher: CollectingDispatcher,
             tracker: Tracker,
             domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
         objeto =tracker.get_slot("object")
         color=tracker.get_slot("color")
         dic={"Intent":'Farewell',
         'Object': objeto ,
         'Color': color,
         'Go_along': "no"
         }
         with open("jsons/results.json", "w") as write_file:
            json.dump(dic, write_file)

         dispatcher.utter_message()

         return []


