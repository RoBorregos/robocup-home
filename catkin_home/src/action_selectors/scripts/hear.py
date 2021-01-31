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
from SpeechApiUtils import SpeechApiUtils

# Force the use of an specific engine.
# 'online' always sends the request to Azure node 
# without even checking for internet.
# 'offline' always uses DeepSpeech even with internet.
FORCE_ENGINE = None

publisher_azure = None
publisher_deepspeech = None

def callback_azure(data):
    # Change Sample Rate.
    resample=SpeechApiUtils.resample_ratecv(data.data,48000,16000)
    # getAllSamples.
    allsamples=SpeechApiUtils.get_all_samples(resample[0])
    # Publish.
    publisher_azure.publish(allsamples)
    rospy.loginfo("Sent to Azure node.")

def callback_deepspeech(data):
    # Publish.
    publisher_deepspeech.publish(data)
    rospy.loginfo("Sent to Deepspeech node.")

def both_callback(data):
    rospy.loginfo("*Received a voice audio, computing...*")
    if (FORCE_ENGINE == "online" or 
        (FORCE_ENGINE == "none" and SpeechApiUtils.is_connected())):
        callback_azure(data)
    else:
        callback_deepspeech(data)

def main():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hear', anonymous=True)
    rospy.loginfo("*Starting Hear Node*")

    global FORCE_ENGINE
    FORCE_ENGINE=rospy.get_param('~FORCE_ENGINE')

    global publisher_azure, publisher_deepspeech
    # For publishing when online to Azure node to it compute it and publish.
    publisher_azure = rospy.Publisher('UsefulAudioAzure', AudioData, queue_size=5)
    # For publishing when online to Azure node to it compute it and publish.
    publisher_deepspeech = rospy.Publisher('UsefulAudioDeepSpeech', AudioData, queue_size=5)

    rospy.Subscriber("UsefulAudio", AudioData, both_callback, queue_size=10)        
    
    # spin() simply keeps python from exiting until this node is stopped.
    rospy.loginfo("*Ready to callback.*")
    rospy.spin()

    rospy.loginfo("*Node finished*")

if __name__ == '__main__':
    main()
