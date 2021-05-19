#!/usr/bin/env python3
import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool
import webrtcvad
import pyaudio
import collections

class UsefulAudio(object):
    DEBUG = True
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    CHUNK_SIZE = 480
    RATE = 48000
    CHUNK_DURATION = CHUNK_SIZE / RATE
    TIME_FOR_CHANGE = 0.25
    COUNT_FOR_CHANGE = TIME_FOR_CHANGE / CHUNK_DURATION
    MIN_AUDIO_LENGTH = 0.50
    MIN_CHUNKS_AUDIO_LENGTH = MIN_AUDIO_LENGTH / CHUNK_DURATION
    
    PADDING_DURATION = 0.600
    NUM_PADDING_CHUNKS = int(PADDING_DURATION / CHUNK_DURATION)
    
    triggered = False
    chunk_count = 0
    voiced_frames = []
    ring_buffer = collections.deque(maxlen = NUM_PADDING_CHUNKS)

    def __init__(self):
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(3)
        self.subscriber = rospy.Subscriber("rawAudioChunk", AudioData, self.callback)
        self.publisher = rospy.Publisher("UsefulAudio", AudioData, queue_size=20)
        rospy.Subscriber("inputAudioActive", Bool, self.callbackActive)
        self.inputAudioActive = True

    def debug(self, text):
        if(self.DEBUG):
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
        if self.chunk_count > self.MIN_CHUNKS_AUDIO_LENGTH:
            self.publisher.publish(AudioData(self.voiced_frames))
        self.discardAudio()

    def callbackActive(self, msg):
        self.inputAudioActive = msg.data

    def vad_collector(self, chunk):
        is_speech = self.vad.is_speech(chunk, self.RATE)

        if not self.triggered:
            self.ring_buffer.append((chunk, is_speech))
            num_voiced = len([f for f, speech in self.ring_buffer if speech])
            
            # If we're NOTTRIGGERED and more than 90% of the frames in
            # the ring buffer are voiced frames, then enter the
            # TRIGGERED state.
            if num_voiced > 0.9 * self.ring_buffer.maxlen:
                self.triggered = True
                print("Start talking...")
                # We want to publish all the audio we see from now until
                # we are NOTTRIGGERED, but we have to start with the
                # audio that's already in the ring buffer.
                for f, _ in self.ring_buffer:
                    self.buildAudio(chunk)
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
            if num_unvoiced > 0.9 * self.ring_buffer.maxlen:
                print("Stop talking...")
                self.triggered = False
                self.publishAudio()

    def callback(self, data):
        if self.inputAudioActive == False:
            self.discardAudio()
            return
        
        self.vad_collector(data.data)


def main():
    rospy.init_node('UsefulAudio', anonymous=True)
    usefulAudio = UsefulAudio()
    usefulAudio.debug('UsefulAudio Initialized.')
    rospy.spin()

if __name__ == '__main__':
    main()