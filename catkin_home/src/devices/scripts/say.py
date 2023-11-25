#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import requests
import pyttsx3
from gtts import gTTS
from playsound import playsound
import os
from time import sleep
import socket

class Say(object):
    DEBUG = True

    def __init__(self):
        self.engine = pyttsx3.init()
        self.engine.setProperty('voice', 'com.apple.speech.synthesis.voice.samantha')
        self.connected = self.is_connected()
        self.text_suscriber = rospy.Subscriber("robot_text", String, self.callback)
        self.hear_publisher = rospy.Publisher("saying", Bool, queue_size=20)
    
    @staticmethod
    def is_connected():
        '''
        Try to connect the fastest possible to a stablished server to see if
        there is internet connection. It connects to one of the Google's 
        dns servers (port 53) (https://developers.google.com/speed/public-dns/docs/using),
        this to avoid timeouts in DNS servers via a hostname. 
        https://stackoverflow.com/a/33117579
        '''
        try:
            # Connect to the host -- tells us if the host is actually reachable.
            socket.setdefaulttimeout(0.80)
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(("8.8.8.8", 53))
            sock.shutdown(socket.SHUT_RDWR)
            sock.close()
            return True
        except socket.error:
            pass
        return False

    def debug(self, text):
        if(self.DEBUG):
            rospy.loginfo(text)

    def callback(self, msg):
        self.debug("I will say: " + msg.data)
        self.trySay(msg.data)

    def disconnectedVoice(self, text):
        self.engine.say(text)
        self.engine.runAndWait()  

    def connectedVoice(self, text): 
        tts = gTTS(text=text, lang='en')
        tts.save("play.mp3")
        self.debug("Saying...")
        playsound('./play.mp3',block=True)
        size_string = len(text.split())
        self.debug("Stopped")

    def trySay(self, text):
        self.hear_publisher.publish(Bool(True))
        rospy.logwarn("Published: False ")
        self.connectedVoice(text)
        try:
            pass
        except  Exception as e:
            print(e)
            self.disconnectedVoice(text)
        sleep(1)
        self.hear_publisher.publish(Bool(False))
        rospy.logwarn("Published: True ")


def main():
    rospy.init_node('say', anonymous=True)
    say = Say()
    say.debug('Say Module Initialized.')
    rospy.spin()

if __name__ == '__main__':
    main()