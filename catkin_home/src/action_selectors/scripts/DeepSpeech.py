#!/usr/bin/env python
'''
This script creates the node `DeepSpeech` that taking voice audio from topic
`UsefulAudioDeepSpeech`, does speech-to-text and publishes the resulting text.

'''

import rospy
import scipy

from audio_common_msgs.msg import AudioData
from action_selectors.msg import RawInput
from SpeechApiUtils import SpeechApiUtils
from DeepSpeech.deploy.ros_server import ASRServer

# Num of process for searching in the LM scorer.
NUM_PROCESSES_BEAM_SEARCH = 2

publisher = None
asr_server = None


def on_audio_callback(data):
    rospy.loginfo("DeepSpeech computing...")
    unicode_text = asr_server.bytes_speech_to_text(data.data)
    text = unicode_text.encode('ascii', 'ignore')

    if len(text) == 0 or text.isspace():
       rospy.loginfo("Audio is empty")
       return

    rospy.loginfo("Voice audio said: \"{0}\".".format(text))

    msg = RawInput()
    # TODO(Josecisneros001): See posibility of gender and age detection.
    msg.isWoman = False
    msg.inputText = text
    publisher.publish(msg)
    rospy.loginfo("Published the msg.")


def main():
    rospy.init_node('DeepSpeech')
    rospy.loginfo("*Starting Node*")

    rospy.loginfo("Starting DeepSpeech (ASRServer)")
    global asr_server
    asr_server = ASRServer(
        num_processes_beam_search=NUM_PROCESSES_BEAM_SEARCH,
    )
    rospy.loginfo("ASRServer ready")

    global publisher
    # For resulting text when using DeepSpeech offline.
    publisher = rospy.Publisher('RawInput', RawInput, queue_size=10)
    
    rospy.Subscriber("UsefulAudioDeepSpeech", AudioData, on_audio_callback, queue_size=10)        
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo("*Ready to callback.*")
    rospy.spin()

    rospy.loginfo("*Node finished*")

if __name__ == '__main__':
    main()
