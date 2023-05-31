#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from object_detector.msg import objectDetection, objectDetectionArray
from geometry_msgs.msg import Point

dicPoints = {
    'Coca-Cola': Point(x=0.45, y=-0.73, z=0.88),
    'Zucaritas': Point(x=0.45, y=-0.73, z=0.88),
    'Harpic':    Point(x=0.45, y=-0.73, z=0.88),
    'Principe':  Point(x=0.42, y=-0.73, z=0.88)
}

activeFlag = True
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
    for element in dicPoints:
        res.append(objectDetection(
            label = 1,
            labelText = element,
            score = 1.0,
            ymin =  0.0,
            xmin =  0.0,
            ymax =  620,
            xmax =  620,
            point3D = dicPoints[element]
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
