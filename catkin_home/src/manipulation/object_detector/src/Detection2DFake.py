#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from object_detector.msg import objectDetection, objectDetectionArray
from geometry_msgs.msg import Point

activeFlag = False
def activeFlagSubscriber(msg):
    global activeFlag
    activeFlag = msg.data

def main():
    global activeFlag
    rospy.Subscriber('detectionsActive', Bool, activeFlagSubscriber)
    pub = rospy.Publisher('detections', objectDetectionArray, queue_size=5)
    rospy.init_node('Vision2D', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    res = []
    point3D_ = Point()
    point3D_.x = 1.4683277606964111
    point3D_.y = -0.5442641973495483
    point3D_.z = 0.6405620574951172          
    res.append(objectDetection(
        label = 1,
        labelText = "dog_doll",
        score = 1.0,
        ymin =  0.0,
        xmin =  0.0,
        ymax =  620,
        xmax =  620,
        point3D = point3D_
    ))
    while not rospy.is_shutdown():
        if activeFlag:
            pub.publish(objectDetectionArray(detections=res))
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
