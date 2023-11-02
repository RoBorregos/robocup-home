# TODO
# #!/usr/bin/env python3
# import rospy
# from audio_common_msgs.msg import AudioData
# from std_msgs.msg import Bool
# import webrtcvad
# import pyaudio
# import collections
# from SpeechApiUtils import SpeechApiUtils


# # Format for the recorded audio by PyAudio. It is exactly as RNNoise
# # (devices/InputAudio) needs it.
# CHUNK_SIZE = 480
# # Signed 2 bytes.
# FORMAT = pyaudio.paInt16
# CHANNELS = 1
# RATE = 48000

# # Hear variables
# # Force the use of an specific engine.
# # 'online' always sends the request to Azure node 
# # without even checking for internet.
# # 'offline' always uses Whisper even with internet (uses DeepSpeech if whisper flag is off).
# FORCE_ENGINE = None

# USE_WHISPER_OFFLINE = True

# publisher_azure = None
# publisher_deepspeech = None
# publisher_whisper = None


# class Object(object):
#     pass


# def callback_azure(data):
#     # Change Sample Rate.
#     resample=SpeechApiUtils.resample_ratecv(data.data, 48000, 16000)
#     # getAllSamples.
#     allsamples=SpeechApiUtils.get_all_samples(resample[0])
#     # Publish.
#     publisher_azure.publish(allsamples)
#     rospy.loginfo("Sent to Azure node.")

# def callback_deepspeech(data):
#     # Publish.
#     publisher_deepspeech.publish(data)
#     rospy.loginfo("Sent to Deepspeech node.")


# def callback_whisper(data):
#     # Whisper resamples audio to 16 kHz
#     resample=SpeechApiUtils.resample_ratecv(data.data, 48000, 16000)
#     # getAllSamples.
#     allsamples=SpeechApiUtils.get_all_samples(resample[0])
#     # Publish.
#     publisher_whisper.publish(allsamples)
#     rospy.loginfo("Sent to Whisper node.")



# def both_callback(data):
#     rospy.loginfo("*Received a voice audio, computing...*")
#     if (FORCE_ENGINE == "online" or 
#         (FORCE_ENGINE == "none" and SpeechApiUtils.is_connected())):
#         callback_azure(data)
#     else:
#         if USE_WHISPER_OFFLINE:
#             callback_whisper(data)
#         else:
#             callback_deepspeech(data)


# class UsefulAudio(object):
#     DEBUG = True
#     FORMAT = pyaudio.paInt16
#     CHANNELS = 1
#     CHUNK_SIZE = 480
#     RATE = 48000
#     CHUNK_DURATION = CHUNK_SIZE / RATE
#     TIME_FOR_CHANGE = 0.25
#     COUNT_FOR_CHANGE = TIME_FOR_CHANGE / CHUNK_DURATION
#     MIN_AUDIO_LENGTH = 0.50
#     MIN_CHUNKS_AUDIO_LENGTH = MIN_AUDIO_LENGTH / CHUNK_DURATION
    
#     PADDING_DURATION = 0.50
#     NUM_PADDING_CHUNKS = int(PADDING_DURATION / CHUNK_DURATION)
    
#     triggered = False
#     chunk_count = 0
#     voiced_frames = []
#     ring_buffer = collections.deque(maxlen = NUM_PADDING_CHUNKS)

#     def __init__(self):
#         self.vad = webrtcvad.Vad()
#         self.vad.set_mode(3)
#         rospy.Subscriber("inputAudioActive", Bool, self.callbackActive)
#         self.inputAudioActive = True

#     def debug(self, text):
#         if(self.DEBUG):
#             rospy.loginfo(text)

#     def buildAudio(self, data):
#         if self.voiced_frames == None:
#             self.voiced_frames = data
#         else:
#             self.voiced_frames += data
#         self.chunk_count += 1

#     def discardAudio(self):
#         self.ring_buffer.clear()
#         self.voiced_frames = None
#         self.chunk_count = 0

#     def publishAudio(self):
#         if self.chunk_count > self.MIN_CHUNKS_AUDIO_LENGTH:
#             # Replace with call
#             self.publisher.publish(AudioData(self.voiced_frames))

#         self.discardAudio()

#     def callbackActive(self, msg):
#         self.inputAudioActive = msg.data

#     def vad_collector(self, chunk):
#         is_speech = self.vad.is_speech(chunk, self.RATE)

#         if not self.triggered:
#             self.ring_buffer.append((chunk, is_speech))
#             num_voiced = len([f for f, speech in self.ring_buffer if speech])
            
#             # If we're NOTTRIGGERED and more than 90% of the frames in
#             # the ring buffer are voiced frames, then enter the
#             # TRIGGERED state.
#             if num_voiced > 0.75 * self.ring_buffer.maxlen:
#                 self.triggered = True
#                 print("Start talking...")
#                 # We want to publish all the audio we see from now until
#                 # we are NOTTRIGGERED, but we have to start with the
#                 # audio that's already in the ring buffer.
#                 for f, _ in self.ring_buffer:
#                     self.buildAudio(f)
#                 self.ring_buffer.clear()
#         else:
#             # We're in the TRIGGERED state, so collect the audio data
#             # and add it to the ring buffer.
#             self.buildAudio(chunk)
#             self.ring_buffer.append((chunk, is_speech))
#             num_unvoiced = len([f for f, speech in self.ring_buffer if not speech])
#             # If more than 90% of the frames in the ring buffer are
#             # unvoiced, then enter NOTTRIGGERED and publish whatever
#             # audio we've collected.
#             if num_unvoiced > 0.75 * self.ring_buffer.maxlen:
#                 print("Stop talking...")
#                 self.triggered = False
#                 self.publishAudio()

#     def callback(self, data):
#         if self.inputAudioActive == False:
#             self.discardAudio()
#             return
        
#         self.vad_collector(data.data)


# def main():

#     global usefulAudio_node
#     usefulAudio_node = UsefulAudio()

#     p = pyaudio.PyAudio()

#     global FORCE_ENGINE
#     FORCE_ENGINE=rospy.get_param('~FORCE_ENGINE', 'online')

#     global USE_WHISPER_OFFLINE
#     USE_WHISPER_OFFLINE=rospy.get_param('~USE_WHISPER_OFFLINE', True)

#     global publisher_azure, publisher_deepspeech, publisher_whisper
#     # For publishing when online to Azure node to it compute it and publish.
#     publisher_azure = rospy.Publisher('UsefulAudioAzure', AudioData, queue_size=5)
#     # For publishing when online to Azure node to it compute it and publish.
#     publisher_deepspeech = rospy.Publisher('UsefulAudioDeepSpeech', AudioData, queue_size=5)

#     # For publishing when online to Whispe node to compute it and publish.
#     publisher_whisper = rospy.Publisher('UsefulAudioWhisper', AudioData, queue_size=5)

#     rospy.init_node('SpeechNode', anonymous=True)
#     usefulAudio_node.debug("Speech node initialized")

#     stream = p.open(input_device_index=None, # Default device
#                     format=FORMAT,
#                     channels=CHANNELS,
#                     rate=RATE,
#                     input=True,
#                     frames_per_buffer=CHUNK_SIZE)
    
#     print("*Recording*")

#     # Loop while node is not closed and audio is working. Note that
#     # here is "not needed" something like `spin()` or `loop_sleep()`
#     # because (presumably) PyAudio manages the blocked times waiting for 
#     # IO and puts the process/thread to sleep, also there are no callbacks
#     # by ROS.
#     while stream.is_active() and not rospy.is_shutdown():
#         try:
#             in_data = stream.read(CHUNK_SIZE, exception_on_overflow = False)
#             data = Object()
#             data.data = in_data
#             usefulAudio_node.callback(data)
#         except IOError as e:
#             print("I/O error({0}): {1}".format(e.errno, e.strerror))
#             # break
 
#     if not stream.is_active():
#         print("Stream was not active.")

#     stream.stop_stream()
#     stream.close()
#     p.terminate()


# if __name__ == '__main__':
#     main()