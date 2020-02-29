#!/usr/bin/env python
import rospy
import scipy
import socket
import struct
import audioop
from audio_common_msgs.msg import AudioData
from action_selectors.msg import RawInput
# TODO: Analyze why inside the DeepSpeech dir, everything can be imported
# directly without the package. Maybe is because the _init_paths.py and/or
# the `absolute_import`.
from DeepSpeech.deploy.ros_server import ASRServer


publisher = None
publisher_16k = None
asr_server = None

def is_connected():
    try:
        # connect to the host -- tells us if the host is actually
        # reachable
        sock=socket.create_connection(("www.google.com", 80),1)
        sock.shutdown(socket.SHUT_RDWR)
        sock.close()
        return True
    except:
        pass
    return False

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

def resample_ratecv(data,samplerate=48000, resample_rate=16000):
    """Resamples the given PCM stream to resample_rate.
    """
    return audioop.ratecv(b"".join(data), 2, 1, samplerate, resample_rate, None)
def get_samples_2B(data):
    """GetSamples 2Bytes
    """
    last=-1;
    my_list_16b=[]
    for i in data:
        if last==-1:
            last=i
        else:
            my_list_16b.append(struct.pack("B",int(last))+struct.pack("B",int(i)))
            last=-1

    return my_list_16b

def get_all_samples(data):
    allsamples=[]
    index=0
    while True:
        try:
            index=index+1
            sample=audioop.getsample(data,2,index)
            sample_8a = sample & 0xff
            sample_8b = (sample >> 8) & 0xff
            allsamples.append(int(str(sample_8a)))
            allsamples.append(int(str(sample_8b)))
        except:
            break;

    return allsamples;

def callback_azure(data):
    rospy.loginfo("Received a voice audio, computing...")
    
    #Parse to int
    dataOrd=[(ord(i)) for i in data.data]
    
    #GetSamples 2Bytes
    Samples2B=get_samples_2B(dataOrd)
    #Change Sample Rate
    resample=resample_ratecv(Samples2B,48000,16000)
    #getAllSamples
    allsamples=get_all_samples(resample[0])
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
    if not (is_connected()):
        rospy.loginfo("Deepspeech ONLINE")
        rospy.loginfo("*Node initiated, starting ASRServer (DS2)*")
        global asr_server
        asr_server = ASRServer()
        rospy.loginfo("*ASRServer ready.*")

        global publisher
        publisher = rospy.Publisher('RawInput', RawInput, queue_size=10)
        rospy.Subscriber("UsefulAudio", AudioData, callback, queue_size=10)
    
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
