import socket
import io
from pydub import AudioSegment
import struct

# Set the IP address and port to listen on
listen_ip = "0.0.0.0"  # Listen on all available network interfaces
port = 5005

# Create a raw socket to receive RTP packets
sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_UDP)
sock.bind((listen_ip, port))

# Create an empty audio segment to store audio data
audio_data = AudioSegment.empty()

try:
    while True:
        # Receive RTP packets
        packet, _ = sock.recvfrom(65535)
        
        if not packet:
            # If the packet is empty, the connection is closed
            break

        # Extract the audio payload (16-bit little-endian PCM)
        audio_payload = packet[12:]  # Skip the RTP header (12 bytes)

        # Convert the binary audio payload to a list of 16-bit integers
        audio_samples = struct.unpack(f'{len(audio_payload) // 2}h', audio_payload)

        # Create an AudioSegment from the samples
        audio_data += AudioSegment(audio_samples, sample_width=2, frame_rate=16000, channels=1)

except KeyboardInterrupt:
    print("Received KeyboardInterrupt. Stopping...")

finally:
    # Export the audio data to an MP3 file
    audio_data.export("audio.mp3", format="mp3")

#