
import socket
import struct
import audioop

class SpeechApiUtils(object):
    @staticmethod
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
    
    @staticmethod
    def resample_ratecv(data,samplerate=48000, resample_rate=16000):
        #Resamples the given PCM stream to resample_rate.
        return audioop.ratecv(str(bytearray(data)), 2, 1, samplerate, resample_rate, None)

    @staticmethod
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
    