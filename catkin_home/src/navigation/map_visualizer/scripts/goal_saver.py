#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    coordinateX = data.pose.pose.position.x
    coordinateY = data.pose.pose.position.y
    quaternionX = data.pose.pose.orientation.x
    quaternionY = data.pose.pose.orientation.y
    quaternionZ = data.pose.pose.orientation.z
    quaternionW = data.pose.pose.orientation.w
    f= open("mock_path.csv","a+")
    f.write("%f,%f,%f,%f,%f,%f\r\n" % (coordinateX, coordinateY, quaternionX, quaternionY, quaternionZ, quaternionW))
    f.close()
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('goal_listener', anonymous=True)

    rospy.Subscriber("/move_base_simple/goal", PoseWithCovarianceStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()