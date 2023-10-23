#!/usr/bin/env python3

'''
This script creates the node `Whisper` that taking voice audio from topic
`UsefulAudioWhisper`, does speech-to-text and publishes the resulting text.

'''

# TODO: 
# Test tempfile module
# Check which whisper model performs better
# Reduce audio messages in pipeline, while maintaining decoupling 

import rospy
import whisper
import wave
import tempfile
import torch
import pyaudio

from audio_common_msgs.msg import AudioData
from action_selectors.msg import RawInput

DEBUG = True

class Whisper():
    def __init__(self):
        # Set the parameters for the temporary WAV file
        # Make sure that the parameters match UsefulAudio's settings
        self.sample_rate = 48000
        self.n_channels = 1
        self.sample_width = 2  # in bytes
        self.load_model()

    def load_model(self):
        # Note: when using a model for the first time, the program will access the internet to download the model.
        # choices=["tiny.en", "base.en", "small.en", "medium.en", "large.en"]
        self.audio_model = whisper.load_model("tiny.en")
    
    def interpret(self, data):
        temp_file = self.generate_temp_wav(data)

        # return self.audio_model.transcribe(temp_file, fp16=torch.cuda.is_available())


    def generate_temp_wav(self, data):
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
            with wave.open(temp_file, 'w') as wav_file:
                wav_file.setnchannels(self.n_channels)
                wav_file.setsampwidth(self.sample_width)
                wav_file.setframerate(self.sample_rate)
                wav_file.setnframes(len(data))

                for i in range(data):
                    wav_file.writeframes(data[i])

            # Get the temporary file name
            return temp_file.name
    
def on_audio_callback(data):
    rospy.loginfo("Whisper computing...")

    text = whisperModel.interpret(data.data)

    if len(text) == 0 or text.isspace():
       rospy.loginfo("Audio is empty")
       return

    rospy.loginfo("Voice audio said: \"{0}\".".format(text))

    msg = RawInput()
    msg.inputText = text
    publisher.publish(msg)
    rospy.loginfo("Published whisper result.")

def play_wav_file(file_path):
    chunk = 1024  # Adjust the chunk size as needed
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
    rospy.init_node('Whisper')
    rospy.loginfo("*Starting Node*")

    rospy.loginfo("Starting Whisper")

    global publisher
    publisher = rospy.Publisher('RawInput', RawInput, queue_size=10)
    
    global whisperModel
    whisperModel = Whisper()

    rospy.Subscriber("UsefulAudioWhisper", AudioData, on_audio_callback, queue_size=10)        
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo("*Ready to callback.*")
    rospy.spin()

    rospy.loginfo("*Node finished*")

if __name__ == '__main__':
    main()
