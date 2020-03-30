#!/usr/bin/env python
'''
This script creates the node `hear` that taking voice audio from topic
`UsefulAudio`, does speech-to-text and publishes the resulting text.

For this, it checks everytime if there is internet connection to use
offline stt or online Azure service.
Note that azure (online) node can takes almost 10sec to say no internet.
Also, because DeepSpeech (offline) uses a lot of RAM and cpu, then 
disabling it can be desired.

TODO: The connection checking doesn't mean a good one for the online service.
Then, Azure's code takes like 10sec to say error and this node never 
knows when that happens. Maybe use a ROS service to get feedback.
'''
import rospy
import scipy

from audio_common_msgs.msg import AudioData
from action_selectors.msg import RawInput
# TODO: Analyze why inside the DeepSpeech dir, everything can be imported
# directly without the package. Maybe is because the _init_paths.py and/or
# the `absolute_import`.
from DeepSpeech.deploy.ros_server import ASRServer
from SpeechApiUtils import SpeechApiUtils


# TODO: Get this values from flags.
# 'online' doesn't even loads DeepSpeech to memory, it
# always sends the request to Azure node without
# even checking for internet.
# 'offline' always uses DeepSpeech even with internet.
FORCE_USE_STT = ["none", "online", "offline"][0]

# Num of process for searching in the LM scorer.
NUM_PROCESSES_BEAM_SEARCH = 2


publisher = None
publisher_16k = None
asr_server = None


def callback_deepspeech(data):
    rospy.loginfo("DeepSpeech computing...")
    unicode_text = asr_server.bytes_speech_to_text(data.data)
    text = unicode_text.encode('ascii', 'ignore')

    if len(text) == 0 or text.isspace():
       rospy.loginfo("Audio is empty")
       return

    rospy.loginfo("Voice audio said: \"{0}\".".format(text))

    msg = RawInput()
    # TODO: Check this isWoman field.
    msg.isWoman = False
    msg.inputText = text
    publisher.publish(msg)
    rospy.loginfo("Published the msg.")

def callback_azure(data):
    #Change Sample Rate
    resample=SpeechApiUtils.resample_ratecv(data.data,48000,16000)
    #getAllSamples
    allsamples=SpeechApiUtils.get_all_samples(resample[0])
    #Publish
    publisher_16k.publish(allsamples)
    
    rospy.loginfo("Sent to azure node.")

def both_callback(data):
    rospy.loginfo("*Received a voice audio, computing...*")
    if (FORCE_USE_STT == "online" or 
        (FORCE_USE_STT == "none" and SpeechApiUtils.is_connected())):
        rospy.loginfo("Using Azure online")
        callback_azure(data)
    else:
        rospy.loginfo("Using DeepSpeech offline")
        callback_deepspeech(data)

def main():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hear', anonymous=True)
    rospy.loginfo("*Starting Node*")
    rospy.loginfo("Remember have running Azure node if using online.")

    if FORCE_USE_STT != "online":
        rospy.loginfo("Starting DeepSpeech (ASRServer)")
        global asr_server
        asr_server = ASRServer(
            num_processes_beam_search=NUM_PROCESSES_BEAM_SEARCH,
        )
        rospy.loginfo("ASRServer ready")

    global publisher, publisher_16k
    # For resulting text when using DeepSpeech offline.
    publisher = rospy.Publisher('RawInput', RawInput, queue_size=10)
    # For publishing when online to Azure node to it compute it and publish.
    publisher_16k = rospy.Publisher('UsefulAudio16kHZ', AudioData, queue_size=5)

    rospy.Subscriber("UsefulAudio", AudioData, both_callback, queue_size=10)        
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo("*Ready to callback.*")
    rospy.spin()

    rospy.loginfo("*Node finished*")

if __name__ == '__main__':
    main()
