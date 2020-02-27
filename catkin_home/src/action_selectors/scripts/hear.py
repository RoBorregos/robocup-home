#!/usr/bin/env python
import rospy
from audio_common_msgs.msg import AudioData
from action_selectors.msg import RawInput
# TODO: Analyze why inside the DeepSpeech dir, everything can be imported
# directly without the package. Maybe is because the _init_paths.py and/or
# the `absolute_import`.
from DeepSpeech.deploy.ros_server import ASRServer


publisher = None
asr_server = None


def callback(data):
    rospy.loginfo("Received a voice audio, computing...")
    
    unicode_text = asr_server.bytes_speech_to_text(data.data)
    text = unicode_text.encode('ascii', 'ignore')
    rospy.loginfo("Voice audio said: \"{0}\".".format(text))

    msg = RawInput()
    # TODO: Check this isWoman field.
    msg.isWoman = False
    msg.inputText = text
    publisher.publish(msg)
    rospy.loginfo("Published the msg.")


def main():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hear', anonymous=True)
    rospy.loginfo("*Node initiated, starting ASRServer (DS2)*")

    global asr_server
    asr_server = ASRServer()
    rospy.loginfo("*ASRServer ready.*")

    global publisher
    publisher = rospy.Publisher('RawInput', RawInput, queue_size=10)
    rospy.Subscriber("UsefulAudio", AudioData, callback, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo("*Ready to callback.*")
    rospy.spin()

if __name__ == '__main__':
    main()
