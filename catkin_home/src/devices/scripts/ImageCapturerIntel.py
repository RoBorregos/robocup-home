#!/usr/bin/env python3
import glob
import cv2
import os
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import CompressedImage

# Show Images.
VERBOSE = False

def main():
    rospy.init_node("ImageCapturerIntel")
    # FPS.
    RATE = int(rospy.get_param('~RATE', 15))
    # Default webcam.
    CAMERA = int(rospy.get_param('~CAMERAID', 0))

    rospy.loginfo("INTEL" + str(CAMERA))
    image_publisher = rospy.Publisher("camaras/" + str(CAMERA) + "/" , CompressedImage, queue_size = RATE)
    rospy.loginfo("*Node " + "ImageCapturerIntel-" + str(CAMERA) + " started*")

    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Start streaming
    pipeline.start(config)

    while(True):
    # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        if VERBOSE:
            cv2.imshow('imgIntel', color_image) # display the captured image
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', color_image)[1]).tostring()    
        image_publisher.publish(msg)


    pipeline.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass