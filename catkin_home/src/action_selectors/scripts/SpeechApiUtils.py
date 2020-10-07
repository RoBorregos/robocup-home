
import socket
import struct
import audioop

class SpeechApiUtils(object):
    @staticmethod
    def is_connected():
        '''
        Try to connect the fastest possible to a stablished server to see if
        there is internet connection. It connects to one of the Google's 
        dns servers (port 53) (https://developers.google.com/speed/public-dns/docs/using),
        this to avoid timeouts in DNS servers via a hostname. 
        https://stackoverflow.com/a/33117579

        TODO: Maybe try to do this to also ensure a enough good internet.
        '''
        try:
            # connect to the host -- tells us if the host is actually
            # reachable
            socket.setdefaulttimeout(0.80)
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(("8.8.8.8", 53))
            sock.shutdown(socket.SHUT_RDWR)
            sock.close()
            return True
        except socket.error:
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
    