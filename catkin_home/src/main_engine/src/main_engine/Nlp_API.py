#!/usr/bin/env python3

import os
import openai


# you must provide your own OpenAI API key in command line witht the format OPENAI_API_KEY= 
openai.api_key = os.getenv("OPENAI_API_KEY")

response = openai.Completion.create(
  model="text-davinci-003",
  prompt="Hello",
  temperature=0.7,
  max_tokens=256,
  top_p=1,
  frequency_penalty=0,
  presence_penalty=0
)

print(response) 