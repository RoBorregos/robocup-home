#!/usr/bin/env python3

'''
This script creates the node `Whisper` that taking voice audio from topic
`UsefulAudioWhisper`, does speech-to-text and publishes the resulting text.

'''

# TODO: 
# Reduce audio messages in pipeline, while maintaining decoupling 
# Check which whisper model performs better

import rospy
import whisper
import wave
import tempfile
import torch
import pyaudio
import os

from audio_common_msgs.msg import AudioData
from action_selectors.msg import RawInput

DEBUG = True

class Timer():
    def __init__(self):
        self.timer = rospy.Time.now()

    def startTime(self):
        self.timer = rospy.Time.now()
    
    def endTimer(self, message):
        

class Whisper():
    def __init__(self):
        # Set the parameters for the temporary WAV file
        # Make sure that the parameters match UsefulAudio's settings
        self.sample_rate = 16000 
        self.n_channels = 1
        self.sample_width = 2  # in bytes
        self.load_model()

    def load_model(self):
        # Note: when using a model for the first time, the program will access the internet to download the model.
        # choices=["tiny.en", "base.en", "small.en", "medium.en", "large.en"]
        self.audio_model = whisper.load_model("tiny.en")
    
    def interpret(self, data):
        temp_file = self.generate_temp_wav(data)
        print(temp_file)
        # play_wav_file(temp_file) # Debug if file created sounds good 

        result = self.audio_model.transcribe(temp_file, fp16=torch.cuda.is_available())
        
        os.remove(temp_file)
        os.fil
        return result["text"]


    def generate_temp_wav(self, data):
        # data = bytes(data)
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
            with wave.open(temp_file, 'w') as wav_file:
                wav_file.setnchannels(self.n_channels)
                wav_file.setsampwidth(self.sample_width)
                wav_file.setframerate(self.sample_rate)
                # print(data)
                wav_file.writeframes(data)
                # wav_file.writeframes(data)
                # for i in data:
                #     wav_file.writeframes(i)

            # Get the temporary file name
            return temp_file.name
    
def on_audio_callback(data):
    rospy.loginfo("Whisper computing...")

    text = whisperModel.interpret(data.data)

    if text is None or len(text) == 0 or text.isspace():
       rospy.loginfo("Audio is empty")
       return

    rospy.loginfo("Voice audio said (whisper): \"{0}\".".format(text))

    msg = RawInput()
    msg.inputText = text
    publisher.publish(msg)
    rospy.loginfo("Published whisper result.")

def play_wav_file(file_path):
    chunk = 480
    wf = wave.open(file_path, 'rb')
    p = pyaudio.PyAudio()

    stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True)

    data = wf.readframes(chunk)

    while data:
        stream.write(data)
        data = wf.readframes(chunk)

    stream.stop_stream()
    stream.close()
    p.terminate()

def main():
    global DEBUG
    DEBUG = rospy.get_param('~debug', False)
    
    FORCE_ENGINE=rospy.get_param('~FORCE_ENGINE', 'online')
    
    rospy.init_node('Whisper')
    rospy.loginfo("*Starting Whisper Node*")

    global publisher
    publisher = rospy.Publisher('RawInput', RawInput, queue_size=10)
    
    global whisperModel
    whisperModel = Whisper()

    rospy.Subscriber("UsefulAudioAzure", AudioData, on_audio_callback, queue_size=10)        
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo("*Ready to callback whisper.*")
    rospy.spin()

    rospy.loginfo("*Node finished*")

if __name__ == '__main__':
    main()
