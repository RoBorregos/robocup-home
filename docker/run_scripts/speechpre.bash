# Ref: https://github.com/mviereck/x11docker/wiki/Container-sound:-ALSA-or-Pulseaudio

# Create pulseaudio socket.
pactl load-module module-native-protocol-unix socket=/tmp/pulseaudio.socket

# Create pulseaudio clients config.
echo 'default-server = unix:/tmp/pulseaudio.socket
# Prevent a server running in the container
autospawn = yes
daemon-binary = /bin/true
# Prevent the use of shared memory
enable-shm = false' > /tmp/pulseaudio.client.conf

# Test Microphone inside container: https://linuxconfig.org/how-to-test-microphone-on-ubuntu-20-04-focal-fossa
# arecord --format=S16_LE --duration=5 --rate=48000 --file-type=raw out.raw
# aplay --format=S16_LE --rate=48000 out.raw
