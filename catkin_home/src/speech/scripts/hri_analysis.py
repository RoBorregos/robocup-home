#!/usr/bin/env python3
import rospy
import openai
from openai.embeddings_utils import get_embedding, cosine_similarity
import pandas as pd
from std_msgs.msg import String
from speech.msg import command, list_of_commands
from dotenv import load_dotenv
import os
load_dotenv()

openai.api_key = os.getenv("OPENAI_API_KEY")

N_OPTIONS = 1
MINIMAL_SIMILARITY = 0.84 # Minimum similarity percentage in which 2 words are considered the same
user_input = ""
user_input_prev = "" # Para comparar la entrada actual y la anterior
activation_call = ""

ACTIVATION_WORDS = ["teus", "robot"]

RAW_TEXT_INPUT_TOPIC = "RawInput"
COMMAND_TOPIC = "/speech/processed_commands"

# load embedding dataframes
actions_df = pd.read_pickle("./src/speech/dataFrames/embeddings_actions.pkl")

locations_df = pd.read_pickle("./src/speech/dataFrames/embeddings_locations.pkl")
location_categories_df = pd.read_pickle("./src/speech/dataFrames/embeddings_categories_locations.pkl")
table_df = pd.read_pickle("./src/speech/dataFrames/embeddings_table.pkl")
seating_df = pd.read_pickle("./src/speech/dataFrames/embeddings_seating.pkl")
utensil_df =  pd.read_pickle("./src/speech/dataFrames/embeddings_utensil.pkl")
shelve_df = pd.read_pickle("./src/speech/dataFrames/embeddings_shelf.pkl")
appliance_df = pd.read_pickle("./src/speech/dataFrames/embeddings_appliance.pkl")

items_df = pd.read_pickle("./src/speech/dataFrames/embeddings_items.pkl")
item_categories_df = pd.read_pickle("./src/speech/dataFrames/embeddings_categories_items.pkl")
cleaning_stuff_df = pd.read_pickle("./src/speech/dataFrames/embeddings_cleaning_stuff.pkl")
drink_df = pd.read_pickle("./src/speech/dataFrames/embeddings_drink.pkl")
food_df = pd.read_pickle("./src/speech/dataFrames/embeddings_food.pkl")
snack_df = pd.read_pickle("./src/speech/dataFrames/embeddings_snack.pkl")

names_df = pd.read_pickle("./src/speech/dataFrames/embeddings_names.pkl") 

def input_fineTuned(petition):
    # System es el contexto del sistema. Son las instrucciones basicas para funcionar
    context = {"role": "system",
                "content": "Command System parser. Filter the command from the input and Parse different commands in the text, separate them in comma separated commands that contain a main verb (go, grab, find, introduce, put) followed by the object or place or person"}
    messages = [context]

    # User es el usuario que hace preguntas
    messages.append({"role": "user", "content": petition})

    response = openai.ChatCompletion.create(model="ft:gpt-3.5-turbo-0613:personal:teus-bot:8GuXDqp1", messages=messages)
    response_content = response.choices[0].message.content

    # Assistant es la IA que da la respuesta
    messages.append({"role": "assistant", "content": response_content})

    print("Simplified sentence: ", response_content,"\n")

    return response_content

def create_embedding(item):
    response = openai.Embedding.create(input=item, model="text-embedding-ada-002")

    response_embedding = response['data'][0]['embedding']

    return response_embedding

def choose_specific_df(category):
    specific_df = None

    if category == "table":
        specific_df = table_df

    elif category == "seating":
        specific_df = seating_df

    elif category == "utensil":
        specific_df = utensil_df

    elif category == "shelf":
        specific_df = shelve_df

    elif category == "appliance":
        specific_df = appliance_df

    elif category == "cleaning_stuff":
        specific_df = cleaning_stuff_df
    
    elif category == "drink":
        specific_df = drink_df

    elif category == "food":
        specific_df = food_df

    elif category == "snack":
        specific_df = snack_df

    return specific_df

def get_product_similarities(type, embeddings_input, dataFrame, dataFrameCategories=None):
    BEST_SIMILARITY = 0
    INPUT_SIMILARITY = list() # Secondary options will be stored in case the first is not found

    dataFrame['similarity'] = dataFrame.text_embedding.apply(lambda x: cosine_similarity(x, embeddings_input))
    dataFrame_sorted = dataFrame.sort_values('similarity', ascending=False) # maximum similarity will be shown in the first row
    
    if dataFrameCategories is not None:
        dataFrameCategories['similarity'] = dataFrameCategories.text_embedding.apply(lambda x: cosine_similarity(x, embeddings_input))
        dataFrameCategories_sorted = dataFrameCategories.sort_values('similarity', ascending=False) # maximum similarity will be shown in the first row

        # Compare which similarity is greater
        # Option 1: general category
        # Option 2: specitic location/item
        if dataFrameCategories_sorted.iloc[0]['similarity'] > dataFrame_sorted.iloc[0]['similarity']: # Option 1
            BEST_SIMILARITY = dataFrameCategories_sorted['similarity'].iloc[0]
            
            if BEST_SIMILARITY >= MINIMAL_SIMILARITY:
                specific_df = choose_specific_df(str(dataFrameCategories_sorted['category'].iloc[0]))
                # A list of all options in that category is returned -> VISION DEPARTMENT WILL CHOOSE    
                INPUT_SIMILARITY = df2list(type=type, dataFrame=specific_df)

        else:   # Option 2
            # An ordered list of greater to lesser resemblance to the input is returned (tam max N_OPTIONS)
            BEST_SIMILARITY = dataFrame_sorted['similarity'].iloc[0]

            if BEST_SIMILARITY >= MINIMAL_SIMILARITY:
                INPUT_SIMILARITY = df2list(type, dataFrame_sorted, N_OPTIONS)

    else: # An ordered list of greater to lesser resemblance to the input is returned (tam max N_OPTIONS)
        # Same as option 2
        BEST_SIMILARITY = dataFrame_sorted['similarity'].iloc[0]

        if BEST_SIMILARITY >= MINIMAL_SIMILARITY:
            INPUT_SIMILARITY = df2list(type=type, dataFrame=dataFrame_sorted, longitud=N_OPTIONS)

    return INPUT_SIMILARITY
        
def df2list(type, dataFrame, longitud=None):
    lista = list()

    if longitud != None:
        for row in range(longitud):
            lista.append(dataFrame[type].iloc[row])
    else:
        for row in range(len(dataFrame)):
            lista.append(dataFrame[type].iloc[row])

    return lista

def count_words(input):
    words = input.split()
    word_count = len(words)

    return word_count

def process_user_input(pub):
    global user_input

    list_action_taken = list()
    list_complement_taken = list()

    action_taken = ""

    #Publicar la lista como mensaje options_list
    # mensaje = options_list()

    petition = input_fineTuned(user_input) # It uses our fine tuned model of ChatGPT

    # split the user_input into smaller sections
    items = petition.split(", ")
    comands = []
    for item in items: # for each section
        if count_words(item) == 1: # Introduce

            action_embedding = create_embedding(item)

            list_action_taken = get_product_similarities('verb', action_embedding, actions_df)

            action_taken = list_action_taken[0]

            print("It's an introduction action\n")
            # call the introduction function with no complements
            com = command()
            com.action = "introduce"
            com.complements = [""]
            comands.append(com)

        elif count_words(item) == 2: # Other actions
            action, complement = item.split() 

            action_embedding = create_embedding(action)
            complement_embedding = create_embedding(complement)

            list_action_taken = get_product_similarities('verb', action_embedding, actions_df)

            # Based on the main action found, we will look in an specific dataFrame
            if list_action_taken:
                action_taken = list_action_taken[0] # The first option will be the most similar

                if action_taken == "go" or action_taken == "put": 

                    list_complement_taken = get_product_similarities(type='location', embeddings_input=complement_embedding, dataFrame=locations_df, dataFrameCategories=location_categories_df)
                    
                elif action_taken == "grab":

                    list_complement_taken = get_product_similarities(type='item', embeddings_input=complement_embedding, dataFrame=items_df, dataFrameCategories=item_categories_df)

                elif action_taken == "find":

                    list_complement_taken = get_product_similarities('name', complement_embedding, names_df)  


            # If no similarity is found, it will mean that this statement cannot be done
            if not action_taken or not list_complement_taken:
                print ("This action is not posible")
                break
            else: # call the specific funtion
                
                if(action_taken == "go"): # movement actions
                    print("It's a movement action")
                    print("La lista de opciones a las que ir es: ", list_complement_taken, "\n")
                    # call the movement function with the complement (complement will be a place)
                    com = command()
                    com.action = "go"
                    com.complements = list_complement_taken
                    comands.append(com)

                elif(action_taken == "find") : 
                    print("It's a movement action")
                    print("La lista de opciones que buscar es: ", list_complement_taken, "\n")
                    # call the movement function with the complement (complement will be a person)
                    com = command()
                    com.action = "find"
                    com.complements = list_complement_taken
                    comands.append(com)
                    
                elif(action_taken == "grab"): # manipulation actions
                    print("It's a manipulation action")
                    print("La lista de opciones que agarrar es: ", list_complement_taken, "\n")
                    # call the manipulation function with the complement (complement will be a thing)
                    com = command()
                    com.action = "grab"
                    com.complements = list_complement_taken
                    comands.append(com)

                elif(action_taken == "put"):
                    print("It's a manipulation action (and movement action??)")
                    print("La lista de opciones a las que ir y dejar algo es: ", list_complement_taken, "\n")
                    # call the manipulation function with the complement (complement will be a thing)
                    com = command()
                    com.action = "put"
                    com.complements = list_complement_taken
                    comands.append(com)

        else:
            print ("This action is not posible. Problem with the number of words")
            break
    pub.publish(comands)

# cada vez que se recibe un mensaje en el topic tp1, se ejecuta esta función
def callback(data):
    global activation_call

    activation_call = str(data.data)

    print("recibido: ", activation_call)

def hri_analysis():
    global activation_call, user_input, user_input_prev

    rospy.init_node('hri_analysis', anonymous=True)
    rospy.Subscriber(RAW_TEXT_INPUT_TOPIC, String, callback)
    publisher_commands = rospy.Publisher(COMMAND_TOPIC, list_of_commands, queue_size=10)

    rate = rospy.Rate(0.2)

    while not rospy.is_shutdown():
        if activation_call in ACTIVATION_WORDS:
            # Request user input
            user_input = input("What do you want me to do?: ")
            activation_call = ""
            
            if user_input != "" and user_input != user_input_prev:
                user_input_prev = user_input
                process_user_input(publisher_commands)

        rate.sleep()

    rospy.spin()
    
if __name__ == '__main__':
    hri_analysis()


