#!/usr/bin/env python3
import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool
import webrtcvad
import pyaudio

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

    isTalking = False
    currentValue = None
    falseCount = 0

    currentAudio = None
    chunkCount = 0

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
        if self.currentAudio == None:
            self.currentAudio = data
        else:
            self.currentAudio += data
        self.chunkCount += 1

    def discardAudio(self):
        self.currentAudio = None
        self.chunkCount = 0

    def publishAudio(self):
        if self.chunkCount > self.MIN_CHUNKS_AUDIO_LENGTH:
            self.publisher.publish(AudioData(self.currentAudio))
        self.discardAudio()

    def callbackActive(self, msg):
        self.inputAudioActive = msg.data

    def callback(self, data):
        if self.inputAudioActive == False:
            self.discardAudio()
            return
        
        currentValue = self.vad.is_speech(data.data, self.RATE)
        if currentValue == True:
            if self.isTalking == False:
                self.debug("Start Talking...")
            self.isTalking = True
            self.falseCount = 0
        elif self.isTalking == True and self.falseCount + 1 >= self.COUNT_FOR_CHANGE:
            self.publishAudio()
            self.isTalking = False
            self.falseCount = 0
            self.debug("Stop Talking...")
        else:
            self.falseCount += 1
        
        if self.isTalking == True:
            self.buildAudio(data.data)


def main():
    rospy.init_node('UsefulAudio', anonymous=True)
    usefulAudio = UsefulAudio()
    usefulAudio.debug('UsefulAudio Initialized.')
    rospy.spin()

if __name__ == '__main__':
    main()