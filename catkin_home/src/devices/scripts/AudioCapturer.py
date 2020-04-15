#!/usr/bin/env python
import rospy
from audio_common_msgs.msg import AudioData
import pyaudio


# Format for the recorded audio by PyAudio. It is exactly as RNNoise
# (devices/InputAudio) needs it.
CHUNK_SIZE = 480
# Signed 2 bytes.
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 48000


def main():
    rospy.init_node('AudioCapturer', anonymous=True)
    publisher = rospy.Publisher("rawAudioChunk", AudioData, queue_size=20)

    p = pyaudio.PyAudio()
    stream = p.open(input_device_index=None, # Default device
                    format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK_SIZE)
    print("*Recording*")

    # Loop while node is not closed and audio is working. Note that
    # here is "not needed" something like `spin()` or `loop_sleep()`
    # because (presumably) PyAudio manages the blocked times waiting for 
    # IO and puts the process/thread to sleep, also there are no callbacks
    # by ROS.
    while stream.is_active() and not rospy.is_shutdown():
        try:
            # TODO: Check for byte order.
            in_data = stream.read(CHUNK_SIZE) # exception_on_overflow=True(?)

            # TODO: Look for the lightest way to do this.
            # msg = tuple(bytearray(in_data))
            # or
            # https://stackoverflow.com/questions/13401600/unpack-binary-data-with-python
            msg = [ord(b) for b in in_data]

            publisher.publish(AudioData(data=msg))
        except IOError as e:
            print("I/O error({0}): {1}".format(e.errno, e.strerror))
            # break
 
    if not stream.is_active():
        print("Stream was not active.")

    stream.stop_stream()
    stream.close()
    p.terminate()

if __name__ == '__main__':
    main()

