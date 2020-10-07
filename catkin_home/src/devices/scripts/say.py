#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import requests
import pyttsx3
from gtts import gTTS
from mpyg321.mpyg321 import MPyg321Player
import os
from time import sleep

def debug(text):
    if(debug_option):
        print(str(text))

engine = pyttsx3.init()
player = MPyg321Player()
connected = False
responseH = requests.get("https://google.com")
debug_option = True

if(responseH ==200):
    connected = True
    debug("Connection established")


def callback(data):
    print("I will say:")
    print(data.text)
    say(data.text)

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
    try:
        connectedVoice(text)
    except:
        disconnectedVoice(text)
    sleep(1)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('say', anonymous=True)

    rospy.Subscriber("robot_text", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()