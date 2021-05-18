'''
	This file takes pictures with the Intel RS camera and saves them to 
	the Directory: object_detection_images_train
'''
import glob
import cv2
import os
import numpy as np
import pyrealsense2 as rs

base_directory = 'object_detection'
path_to_file = os.path.abspath(__file__) # get absolute path to current working file
index_of_base_directory = path_to_file.find(base_directory)

WORKING_DIR = path_to_file[0:index_of_base_directory + len(base_directory)]
IMG_DIR = os.path.join(WORKING_DIR, 'images', 'train')

# Get amount of files in train directory to set the image IDs
totalFiles = len(glob.glob1(IMG_DIR, "*.png"))
print('Amount of images so far: ' + str(totalFiles))

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
	depth_frame = frames.get_depth_frame()
	color_frame = frames.get_color_frame()
	if not depth_frame or not color_frame:
			continue

	# Convert images to numpy arrays
	color_image = np.asanyarray(color_frame.get_data())

	cv2.imshow('img1', color_image) # display the captured image

	if cv2.waitKey(1) & 0xFF == ord('y'): #save on pressing 'y'
		print('Writing image: ' + str(totalFiles))
		cv2.imwrite(IMG_DIR + '/c' + str(totalFiles) + '.png',color_image)
		totalFiles += 1

	elif cv2.waitKey(1) & 0xFF == ord('q'): # end process by pressing 'q'
		break

pipeline.stop()