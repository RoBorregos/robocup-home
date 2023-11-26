#!/usr/bin/env python3
import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool
import webrtcvad
import pyaudio
import collections

import numpy as np
import os
import onnxruntime

# Constants
FORMAT = pyaudio.paInt16
CHANNELS = 1
CHUNK_SIZE = 480
RATE = 16000
CHUNK_DURATION = CHUNK_SIZE / RATE
TIME_FOR_CHANGE = 0.25
COUNT_FOR_CHANGE = TIME_FOR_CHANGE / CHUNK_DURATION
MIN_AUDIO_LENGTH = 0.50
MIN_CHUNKS_AUDIO_LENGTH = MIN_AUDIO_LENGTH / CHUNK_DURATION

PADDING_DURATION = 0.50
NUM_PADDING_CHUNKS = int(PADDING_DURATION / CHUNK_DURATION)

# Modififed at main()
DEBUG = False
USE_SILERO_VAD = True
current_dir = os.path.dirname(os.path.abspath(__file__))

class UsefulAudio(object):
    triggered = False
    chunk_count = 0
    voiced_frames = []
    ring_buffer = collections.deque(maxlen = NUM_PADDING_CHUNKS)

    audioCollection = [] # Temporary storage for audio data, to be used in the VAD
    chunkCollection = None # Temporary storage for chunk data, to be sent to the ASR model

    def __init__(self, threshold: float = 0.1):
        if not USE_SILERO_VAD:
            self.vad = webrtcvad.Vad()
            self.vad.set_mode(3)
        else:
            model_path = os.path.join(current_dir, "../assets", "silero_vad.onnx")
            options = onnxruntime.SessionOptions()
            options.log_severity_level = 4

            self.inference_session = onnxruntime.InferenceSession(
                model_path, sess_options=options
            )
            self.SAMPLING_RATE = 16000
            self.threshold = threshold
            self.h = np.zeros((2, 1, 64), dtype=np.float32)
            self.c = np.zeros((2, 1, 64), dtype=np.float32)

        self.subscriber = rospy.Subscriber("rawAudioChunk", AudioData, self.callback)
        self.publisher = rospy.Publisher("UsefulAudio", AudioData, queue_size=20)
        rospy.Subscriber("inputAudioActive", Bool, self.callbackActive)
        self.inputAudioActive = True

    def debug(self, text):
        if(DEBUG):
            rospy.loginfo(text)

    def buildAudio(self, data):
        if self.voiced_frames == None:
            self.voiced_frames = data
        else:
            self.voiced_frames += data
        self.chunk_count += 1

    def discardAudio(self):
        self.ring_buffer.clear()
        self.voiced_frames = None
        self.chunk_count = 0

    def publishAudio(self):
        if self.chunk_count > MIN_CHUNKS_AUDIO_LENGTH:
            self.publisher.publish(AudioData(self.voiced_frames))
        self.discardAudio()

    def callbackActive(self, msg):
        self.inputAudioActive = msg.data
    
    def int2float(self, sound):
        abs_max = np.abs(sound).max()
        sound = sound.astype('float32')
        if abs_max > 0:
            sound *= 1/32768
        sound = sound.squeeze()  # depends on the use case
        return sound

    def vad_collector(self, chunk):
        is_speech = False

        if USE_SILERO_VAD:
            chunk_vad = np.frombuffer(chunk, dtype=np.int16)
            audio_float32 = self.int2float(chunk_vad)

            if self.chunkCollection == None:
                self.chunkCollection = chunk
            else:
                self.chunkCollection += chunk

            if (len(self.audioCollection) < 3):
                self.audioCollection.append(audio_float32)
                # print("Collecting audio...")
                return
            # rospy.loginfo("Audio collected, running VAD...")
            audio = np.concatenate(self.audioCollection)
            chunk = self.chunkCollection
            self.audioCollection = []
            self.chunkCollection = None
            is_speech = self.is_speech_silero_vad(audio)
        
        else:    
            is_speech = self.vad.is_speech(chunk, RATE)

        if is_speech:
            self.debug("Speech detected")
        else:
            self.debug("Speech not detected")

        if not self.triggered:
            self.ring_buffer.append((chunk, is_speech))
            num_voiced = len([f for f, speech in self.ring_buffer if speech])
            
            # If we're NOTTRIGGERED and more than 90% of the frames in
            # the ring buffer are voiced frames, then enter the
            # TRIGGERED state.
            if num_voiced > 0.75 * self.ring_buffer.maxlen:
                self.triggered = True
                print("Start talking...")
                # We want to publish all the audio we see from now until
                # we are NOTTRIGGERED, but we have to start with the
                # audio that's already in the ring buffer.
                for f, _ in self.ring_buffer:
                    self.buildAudio(f)
                self.ring_buffer.clear()
        else:
            # We're in the TRIGGERED state, so collect the audio data
            # and add it to the ring buffer.
            self.buildAudio(chunk)
            self.ring_buffer.append((chunk, is_speech))
            num_unvoiced = len([f for f, speech in self.ring_buffer if not speech])
            # If more than 90% of the frames in the ring buffer are
            # unvoiced, then enter NOTTRIGGERED and publish whatever
            # audio we've collected.
            if num_unvoiced > 0.75 * self.ring_buffer.maxlen:
                print("Stop talking...")
                self.triggered = False
                self.publishAudio()

    def is_speech_silero_vad(self, audio_data: np.ndarray) -> bool:
        input_data = {
            "input": audio_data.reshape(1, -1),
            "sr": np.array([self.SAMPLING_RATE], dtype=np.int64),
            "h": self.h,
            "c": self.c,
        }
        out, h, c = self.inference_session.run(None, input_data)
        self.h, self.c = h, c
        return out > self.threshold

    def callback(self, data):
        if self.inputAudioActive == False:
            self.discardAudio()
            return
        # rospy.loginfo("UsefulAudio: Received audio data")
        self.vad_collector(data.data)


def main():
    rospy.init_node('UsefulAudio', anonymous=True)

    global USE_SILERO_VAD
    USE_SILERO_VAD = rospy.get_param('~USE_SILERO_VAD', True)

    global DEBUG
    DEBUG = rospy.get_param('~debug', False)
    
    usefulAudio = UsefulAudio()
    usefulAudio.debug('UsefulAudio Initialized.')
    rospy.spin()

if __name__ == '__main__':
    main()