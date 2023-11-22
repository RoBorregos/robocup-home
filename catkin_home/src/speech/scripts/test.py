#!/usr/bin/env python3
import rospy
from hri_pkg.msg import human_sentence
from std_msgs.msg import String
from speech.msg import 

def voz_humana():

    rospy.init_node('voz_humana', anonymous=True)
    pub = rospy.Publisher('/tp1', human_sentence, queue_size=10)
    rate = rospy.Rate(0.2) # 10hz

    while not rospy.is_shutdown():
        mensaje = human_sentence()
        mensaje.sentence = "Bring me some Cleaning stuff from a seating."

        envio_str = "Envio: %s" % (mensaje.sentence)
        rospy.loginfo(envio_str)

        pub.publish(mensaje)
        rate.sleep()
        

if __name__ == '__main__':
    voz_humana()
