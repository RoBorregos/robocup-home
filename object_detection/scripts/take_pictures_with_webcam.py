'''
	This file hekos ti take pictures from your webcam with OpenCV
'''
import glob
import cv2
import os

base_directory = 'object_detection'
path_to_file = os.path.abspath(__file__) # get absolute path to current working file
index_of_base_directory = path_to_file.find(base_directory)

WORKING_DIR = path_to_file[0:index_of_base_directory + len(base_directory)]
IMG_DIR = os.path.join(WORKING_DIR, 'images', 'train')

# Get amount of files in train directory to set the image IDs
totalFiles = len(glob.glob1(IMG_DIR,"*.png"))
print('Amount of images so far: ' + str(totalFiles))

cap = cv2.VideoCapture(0) # video capture source camera (Here webcam of laptop) 
while(True):
	ret,frame = cap.read() # return a single frame in variable `frame`
	cv2.imshow('img1',frame) # display the captured image

	if cv2.waitKey(1) & 0xFF == ord('y'): #save on pressing 'y'
		print('Writing image: ' + str(totalFiles))
		cv2.imwrite('images/c' + str(totalFiles) + '.png',frame)
		totalFiles += 1

	elif cv2.waitKey(1) & 0xFF == ord('q'): # end process on pressing 'q'
		cv2.destroyAllWindows()
		break

cap.release()