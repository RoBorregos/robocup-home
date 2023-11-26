import openai
from openai.embeddings_utils import get_embedding, cosine_similarity
import pandas as pd
from dotenv import load_dotenv
import os
load_dotenv()

openai.api_key = os.getenv("OPENAI_API_KEY")

def create_actions():
    actions = [{
        "verb": "go"
    }, {
        "verb": "find"
    }, {
        "verb": "grab"
    }, {
        "verb": "put"
    }, {
        "verb": "introduce"
    }]
   
  
    actions_df = pd.DataFrame(actions)
    actions_df['text_embedding'] = actions_df.verb.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    #print(actions_df)

    # To save the Data Frame:
    actions_df.to_pickle("embeddings_actions.pkl")

def create_locations():
    locations = [{
        "location": "living_room_side_table"
    }, {
        "location": "living_room_table"
    }, {
        "location": "living_room"
    }, {
        "location": "kitchen"
    }, {
        "location": "kitchen_table"
    }, {
        "location": "kitchen_side_table"
    }, {
        "location": "outside"
    }, {
        "location": "hallway"
    }, {
        "location": "user"
    }]
   
    locations_df = pd.DataFrame(locations)
    locations_df['text_embedding'] = locations_df.location.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    #print(locations_df)

    # To save the Data Frame:
    locations_df.to_pickle("embeddings_locations.pkl")

def create_categories_locations():
    categories_locations = [{
        "category": "table"
    },  {
        "category": "shelf"
    }, {
        "category": "appliance"
    }]
   
    categories_locations_df = pd.DataFrame(categories_locations)
    categories_locations_df['text_embedding'] = categories_locations_df.category.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(categories_locations_df)

    # To save the Data Frame:
    categories_locations_df.to_pickle("embeddings_categories_locations.pkl")

def create_seatings():
    seatings = [{
        "location": "living_room_side_table"
    }, {
        "location": "living_room_table"
    }, {
        "location": "kitchen_table"
    }, {
        "location": "kitchen_side_table"
    }]
   
    seatings_df = pd.DataFrame(seatings)
    seatings_df['text_embedding'] = seatings_df.location.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(seatings_df)

    # To save the Data Frame:
    seatings_df.to_pickle("embeddings_seating.pkl")

def create_tables():
    tables = [{
        "location": "kitchen_table"
    }, {
        "location": "kitchen_side_table"
    }, {
        "location": "living_room_table"
    }, {
        "location": "living_room_side_table"
    }]
   
    tables_df = pd.DataFrame(tables)
    tables_df['text_embedding'] = tables_df.location.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(tables_df)

    # To save the Data Frame:
    tables_df.to_pickle("embeddings_table.pkl")

def create_utensils():
    utensils = [{
        "location": "kitchen"
    }]
   
    utensils_df = pd.DataFrame(utensils)
    utensils_df['text_embedding'] = utensils_df.location.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(utensils_df)

    # To save the Data Frame:
    utensils_df.to_pickle("embeddings_utensil.pkl")

def create_shelves():
    shelves = [{
        "location": "kitchen_table"
    }, {
        "location": "kitchen_side_table"
    }, {
        "location": "living_room_table"
    }, {
        "location": "living_room_side_table"
    }]
   
    shelves_df = pd.DataFrame(shelves)
    shelves_df['text_embedding'] = shelves_df.location.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(shelves_df)

    # To save the Data Frame:
    shelves_df.to_pickle("embeddings_shelf.pkl")

def create_appliance():
    appliance = [{
        "location": "kitchen"
    }]
   
    appliance_df = pd.DataFrame(appliance)
    appliance_df['text_embedding'] = appliance_df.location.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(appliance_df)

    # To save the Data Frame:
    appliance_df.to_pickle("embeddings_appliance.pkl")

def create_items():
    items = [{
        "item": "apple"
    }, {
        "item": "pringles"
    }, {
        "item": "cookies"
    }, {
        "item": "Coke"
    }, {
        "item": "Cereal"
    }, {
        "item": "Soap"
    }
    ]
   
    items_df = pd.DataFrame(items)
    items_df['text_embedding'] = items_df.item.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(items_df)

    # To save the Data Frame:
    items_df.to_pickle("embeddings_items.pkl")

def create_categories_items():
    categories_items = [{
        "category": "cleaning_stuff"
    }, {
        "category": "drink"
    }, {
        "category": "food"
    }]
   
    categories_items_df = pd.DataFrame(categories_items)
    categories_items_df['text_embedding'] = categories_items_df.category.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(categories_items_df)

    # To save the Data Frame:
    categories_items_df.to_pickle("embeddings_categories_items.pkl")

def create_cleaning_stuff():
    cleaning_stuff = [{
        "item": "soap",
    }]
   
    cleaning_stuff_df = pd.DataFrame(cleaning_stuff)
    cleaning_stuff_df['text_embedding'] = cleaning_stuff_df.item.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(cleaning_stuff_df)

    # To save the Data Frame:
    cleaning_stuff_df.to_pickle("embeddings_cleaning_stuff.pkl")

def create_drinks():
    drinks = [{ 
        "item": "coke"
    }]
   
    drinks_df = pd.DataFrame(drinks)
    drinks_df['text_embedding'] = drinks_df.item.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(drinks_df)

    # To save the Data Frame:
    drinks_df.to_pickle("embeddings_drink.pkl")

def create_foods():
    foods = [{
        "item": "cereal"
    }, {
        "item": "cookies"
    }, {
        "item": "apple"
    }]
   
    foods_df = pd.DataFrame(foods)
    foods_df['text_embedding'] = foods_df.item.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(foods_df)

    # To save the Data Frame:
    foods_df.to_pickle("embeddings_food.pkl")

def create_snacks():
    snacks = [{
        "item": "cereal"
    }, {
        "item": "cookies"
    }, {
        "item": "pringles"
    }]
   
    snacks_df = pd.DataFrame(snacks)
    snacks_df['text_embedding'] = snacks_df.item.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(snacks_df)

    # To save the Data Frame:
    snacks_df.to_pickle("embeddings_snack.pkl")

def create_names():
    names = [{
        "name": "person"
    }, {
        "name": "Marina"
    }, {
        "name": "Ivan"
    }, {
        "name": "Alejandra"
    }, {
        "name": "Kevin"
    }, {
        "name": "Paquito"
    }]
   
    names_df = pd.DataFrame(names)
    names_df['text_embedding'] = names_df.name.apply(lambda x: get_embedding(x, engine='text-embedding-ada-002'))

    # To print the Data Frame
    # print(names_df)

    # To save the Data Frame:
    names_df.to_pickle("embeddings_names.pkl")

if __name__ == "__main__":
    create_actions()

    create_locations()
    create_categories_locations()
    create_seatings()
    create_tables()
    create_utensils()
    create_shelves()
    create_appliance()

    create_items()
    create_categories_items()
    create_cleaning_stuff()
    create_drinks()
    create_foods()
    create_snacks()

    create_names()

    print("Dataframes created succesfully")
