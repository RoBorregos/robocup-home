## saludo
* greet
    - utter_greet
## interactive_story_1
* greet
    - utter_greet
* bring_something{"object": "banana"}
    - utter_understood
    - utter_command


## interactive_story_1
* bring_something{"object": "bottle", "place": "kitchen"}
    - utter_understood
    - utter_command
* thank
    - utter_you're_welcome

## interactive_story_1
* bring_something{"object": "pizza", "place": "kitchen"}
    - slot{"object": "pizza"}
    - slot{"place": "kitchen"}
    - utter_understood
    - utter_command
## interactive_story_1
* bring_something{"object": "soda", "place": "kitchen"}
    - slot{"object": "soda"}
    - slot{"place": "kitchen"}
    - utter_understood
    - utter_command
* bye
    - utter_bye

## interactive bye
* bye
    - utter_bye