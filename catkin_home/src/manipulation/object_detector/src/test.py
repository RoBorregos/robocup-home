import rospy
import cv2
import torch
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError   
from object_detector.msg import objectDetection, objectDetectionArray

MODEL_PATH = '../models/yolo9ClassHome.pt'
IMAGE_TOPIC = '/zed2/zed_node/rgb/image_rect_color'

class ObjectDetector():
    image = None
    def __init__(self):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=MODEL_PATH, force_reload=True)
        rospy.init_node('Vision2D', anonymous=True)
        rospy.Subscriber(IMAGE_TOPIC, Image, self.imageCallback)
        rospy.Publisher('detections', objectDetectionArray, queue_size=5)

    def detect(self, image):
        results = self.model(image)
        res = []
        for *xyxy, conf, cls in results.pandas().xyxy[0].itertuples(index=False):
            res.append([objectDetection(
                label = cls,
                labelText = cls,
                score = conf,
                ymin =  xyxy[1],
                xmin =  xyxy[0],
                ymax =  xyxy[3],
                xmax =  xyxy[2],
                point3D = None
            ), {
                'xmin': xyxy[0],
                'ymin': xyxy[1],
                'xmax': xyxy[2],
                'ymax': xyxy[3],
                'label': cls,
                'score': conf
            }])
        return res
    
    def imageCallback(self, msg):
        self.image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def run(self):
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            if self.image is not None:
                cv2.imshow('image', self.image)
                detections = self.detect(self.image)
                print(detections)
                for _, det in detections:
                    print('deteccion: ')
                    print(det)
                    print(det['xmin'], det['ymin'], det['xmax'], det['ymax'])
                    cv2.rectangle(img = self.image, pt1=(int(det['xmin']), int(det['ymin'])), pt2=(int(det['xmax']), int(det['ymax'])), color=(0, 255, 0),thickness= 2)
                    cv2.putText(self.image, str(det['label']), (int(det['xmin']), int(det['ymin']) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
                    #cv2.putText(self.image, str(det.labelText), (det.xmin, det.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            rate.sleep()

if __name__ == '__main__':
    try:
        ObjectDetector().run()
    except rospy.ROSInterruptException:
        pass




    
    
    