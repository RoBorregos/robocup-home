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
        if DEBUG:
            self.timer = rospy.Time.now()
    
    def endTimer(self, message):
        if DEBUG:
            time_delta = rospy.Time.now() - self.timer
            time_delta_second = time_delta.to_sec()
            rospy.logdebug(f"{message}: {time_delta_second} seconds")

class Whisper():
    def __init__(self):
        # Discard parameters
        self.min_time = 2 # seconds
        self.max_time = 30 # seconds

        # Set the parameters for the temporary WAV file
        # Ensure parameters match UsefulAudio's settings (or Hear's settings, if it modifies the input)
        self.sample_rate = 16000 
        self.n_channels = 1
        self.sample_width = 2  # in bytes
        self.load_model()

    # Select model to load. This only needs to be done once.
    def load_model(self):
        # Note: when using a model for the first time, the program will access the internet to download the model.
        # choices=["tiny.en", "base.en", "small.en", "medium.en", "large.en"]
        # model = "tiny.en"
        model = "base.en"
        timer = Timer()
        self.audio_model = whisper.load_model(model)
        timer.endTimer(f"Finished loading whisper model [{model}]")
    
    # Audio interpretation
    def interpret(self, data):
        timer = Timer()
        temp_file = WavUtils.generate_temp_wav(self.n_channels, self.sample_width, self.sample_rate, data)
        timer.endTimer("Finished generating temp wav file")
        empty = True

        if WavUtils.within_time_frame(temp_file, self.min_time, self.max_time):
            # WavUtils.play_wav_file(temp_file) # Debug if file created sounds good, check when varying parameters 
            timer.startTime()
            result = self.audio_model.transcribe(temp_file, fp16=torch.cuda.is_available())
            timer.endTimer("Finished transcribing wav file")
            empty = False
        else:
            rospy.loginfo("Discarded audio as it didn't match expected length")

        # Remove temporary file, after using
        WavUtils.discard_wav(temp_file)

        if not empty:
            return result["text"] 
    

class WavUtils:
    @staticmethod
    # Convert incoming data (AudioData) to wav file for interpretation with model
    def generate_temp_wav(n_channels, sample_width, sample_rate, data):
        # data = bytes(data)
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
            with wave.open(temp_file, 'w') as wav_file:
                wav_file.setnchannels(n_channels)
                wav_file.setsampwidth(sample_width)
                wav_file.setframerate(sample_rate)

                wav_file.writeframes(data)
            # Return the temporary file name
            return temp_file.name

    @staticmethod
    def discard_wav(file_path):
        os.remove(file_path)

    @staticmethod
    # Return if the audio is over the minumum threshold 
    def within_time_frame(file_path, min_time, max_time):
        time = WavUtils.get_file_time(file_path)
        if max_time > time and time > min_time:
            return True
        return False 

    @staticmethod
    def get_file_time(file_path):
        with wave.open(file_path, 'r') as f:
            frames = f.getnframes()
            rate = f.getframerate()
            duration = frames/float(rate * f.getnchannels())
            return duration

    @staticmethod
    def play_wav_file(file_path):
        chunk = 480 # Chunk of same size as AudioCapturer 
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


def on_audio_callback(data):
    rospy.loginfo("Whisper computing...")
    
    text = whisperModel.interpret(data.data)

    if text is None or len(text) == 0 or text.isspace():
       rospy.loginfo("Audio is empty")
       return

    # Remove white spaces of resulting text
    text = text.strip()

    rospy.loginfo("Voice audio said (whisper): \"{0}\".".format(text))

    msg = RawInput()
    msg.inputText = text
    publisher.publish(msg)
    rospy.loginfo("Published whisper result.")

def main():
    global DEBUG
    DEBUG = rospy.get_param('~debug', False)
    
    rospy.init_node('Whisper')
    rospy.loginfo("*Starting Whisper Node*")

    global publisher
    publisher = rospy.Publisher('RawInput', RawInput, queue_size=10)
    
    global whisperModel
    
    whisperModel = Whisper()

    rospy.Subscriber("UsefulAudioWhisper", AudioData, on_audio_callback, queue_size=10)        
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo("*Ready to callback whisper.*")
    rospy.spin()

    rospy.loginfo("*Node finished*")

if __name__ == '__main__':
    main()
