#!/usr/bin/env python
import rospy
import scipy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
# TODO: Analyze why inside the DeepSpeech dir, everything can be imported
# directly without the package. Maybe is because the _init_paths.py and/or
# the `absolute_import`.
from DeepSpeech.deploy.ros_server import ASRServer
from SpeechApiUtils import SpeechApiUtils

publisher = None
publisher_16k = None
asr_server = None

def callback_deepspeech(data):
    rospy.loginfo("Received a voice audio, computing...")
    
    unicode_text = asr_server.bytes_speech_to_text(data.data)
    text = unicode_text.encode('ascii', 'ignore')
    rospy.loginfo("Voice audio said: \"{0}\".".format(text))

    msg = String()

    msg.data = text
    publisher.publish(msg)
    rospy.loginfo("Published the msg.")



def callback_azure(data):
    rospy.loginfo("Received a voice audio, computing...")
    
    #Change Sample Rate
    resample=SpeechApiUtils.resample_ratecv(data.data,48000,16000)
    #getAllSamples
    allsamples=SpeechApiUtils.get_all_samples(resample[0])
    #Publish
    publisher_16k.publish(allsamples)
    
    rospy.loginfo("Published the msg.")


def main():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hear', anonymous=True)
    if not (SpeechApiUtils.is_connected()):
        rospy.loginfo("Deepspeech ONLINE")
        rospy.loginfo("*Node initiated, starting ASRServer (DS2)*")
        global asr_server
        asr_server = ASRServer()
        rospy.loginfo("*ASRServer ready.*")

        global publisher
        publisher = rospy.Publisher('RawInput', RawInput, queue_size=10)
        rospy.Subscriber("UsefulAudio", AudioData, callback_deepspeech, queue_size=10)
    
    else:
        rospy.loginfo("Azure ONLINE")
        global publisher_16k
        publisher_16k = rospy.Publisher('UsefulAudio16kHZ', AudioData, queue_size=5)
        rospy.Subscriber("UsefulAudio", AudioData, callback_azure, queue_size=10)

        
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo("*Ready to callback.*")
    rospy.spin()

if __name__ == '__main__':
    main()
