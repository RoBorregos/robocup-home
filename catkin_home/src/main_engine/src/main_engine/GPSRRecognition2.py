#!/usr/bin/env python3
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from time import sleep
#from deepface import DeepFace
from typing import TypedDict, List
from util_types.detection import DetectionResult, Facial_area
from util_types.recognition import FaceRecognitionRow
from humanAnalyzer.msg import pose_positions
from humanAnalyzer.msg import face, face_array
from humanAnalyzer.srv import imagetoAnalyze, imagetoAnalyzeResponse
from std_msgs.msg import Bool, Int32
from deepface import DeepFace 
import time
import tensorflow as tf
import json

from urllib.parse import urlparse
from azure.cognitiveservices.vision.face import FaceClient
from msrest.authentication import CognitiveServicesCredentials
from dotenv import load_dotenv

from supabase import create_client

# disable gpu
# os.environ["CUDA_VISIBLE_DEVICES"]="-1"

# Toggle to run Deepface on GPU or CPU
useGPU = False
load_dotenv()
# Toggle to delete Azure Face List
deleteAzureListonStart = False
deleteAzureListonStop = False

class AzureFaceList:
    def __init__(self, face_list_id, azure_key, azure_endpoint):
        self.face_list_id = face_list_id
        self.face_client = FaceClient(azure_endpoint, CognitiveServicesCredentials(azure_key))
        try:
            self.face_client.face_list.get(face_list_id)
            print("Face list already exists")
        except:
            print("Face list does not exist, creating")
            self.face_client.face_list.create(face_list_id, name = face_list_id)

    def addFace(self, single_face_image_url):
        response = self.face_client.face_list.add_face_from_url(self.face_list_id, single_face_image_url, user_data=None, target_face=None, detection_model='detection_01',
                    custom_headers=None, raw=False)
        print(response.persisted_face_id)
        return response.persisted_face_id

    def deleteFace(self, persisted_face_id):
        self.face_client.face_list.delete_face(self.face_list_id, persisted_face_id, custom_headers=None, raw=False)
    
    def getFaceList(self):
        print(self.face_client.face_list.get(self.face_list_id))
        return self.face_client.face_list.get(self.face_list_id)
    
    def verifyFace(self, face_id):
        candidates = self.face_client.face.find_similar(face_id, self.face_list_id, max_num_of_candidates_returned=1)
        for candidate in candidates:
            print(candidate)
            # if candidate.confidence > 0.5:
            #     return candidate
            # else:
            #     return None
            if candidate.confidence > 0.5:
                return candidate
            else:
                return None
            
    def verifyFaceFromUrl(self, single_face_image_url):
        detected_faces = self.face_client.face.detect_with_url(single_face_image_url, recognition_model='recognition_01', detectionModel='detection_02')
        if not detected_faces:
            raise Exception('No face detected from image {}'.format(single_face_image_url))
        face_id = detected_faces[0].face_id
        candidates = self.face_client.face.find_similar(face_id, self.face_list_id, max_num_of_candidates_returned=1)
        for candidate in candidates:
            print(candidate)
            if candidate.confidence > 0.5:
                return candidate
            else:
                return None
    
    # def detectFace(self, single_image_name):
    #     detected_faces = self.face_client.face.detect_with_stream(image=io.BytesIO(single_image_name), detectionModel='detection_02')
    #     if not detected_faces:
    #         raise Exception('No face detected from image {}'.format(single_image_name))
    #     return detected_faces
    
    def deleteFaceList(self):
        self.face_client.face_list.delete(self.face_list_id, custom_headers=None, raw=False)

class faceRecognition:
    def __init__(self):
        self.IDDetected = 0

        # Initializing Azure Face List
        
        AZURE_KEY=os.getenv("AZURE_KEY1")
        AZURE_ENDPOINT=os.getenv("AZURE_ENDPOINT")
        self.face_list_id ="home_list"
        self.azureFaceList = AzureFaceList(self.face_list_id, AZURE_KEY, AZURE_ENDPOINT)
        
        # Creating supabase client
        url = 'https://nvuidsrualaiwfoyvfum.supabase.co'
        key = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Im52dWlkc3J1YWxhaXdmb3l2ZnVtIiwicm9sZSI6ImFub24iLCJpYXQiOjE2ODE1ODkyOTYsImV4cCI6MTk5NzE2NTI5Nn0.R1T5n-hDz2KtjsrK2vwrRLzGB1ooszHWlXXyCA71jsE'
        self.supabase = create_client(url, key)
        self.supabase.storage.empty_bucket('bucketTest')
        self.bucketSize = 1

        self.prev_detections = {}
        self.faces_tracked = []
        self.margin = 10
        self.prev_frame_time = 0
        self.azureMaxCalls = 100000000000000000
        self.azureLimit = self.azureMaxCalls # 20 calls per minute
        self.minFaceArea = 6000
        self.minuteCounter = time.time()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontsize = 0.5
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('humanAnalyzer')
        self.faces_path = f"{pkg_path}/src/scripts/faces"
        self.img_counter = len(os.listdir(self.faces_path))
        self.json_path = f"{pkg_path}/src/scripts/identities.json"
        self.representations_path = f"{pkg_path}/src/scripts/faces/representations_vgg_face.pkl"

        #self.img_couter = len(os.listdir("faces"))
        self.received_image = None
        self.bridge = CvBridge()

        rospy.init_node('FaceRecognition')

        #camSubscriber = '/zed2_down/zed_down_node/rgb/image_rect_color'
        camSubscriber = 'image'
        self.imageSub = rospy.Subscriber(
            camSubscriber, Image, self.process_image, queue_size=10)
        self.faceAnalysisStateSub = rospy.Subscriber(
            'faceAnalysisState', Bool, self.faceAnalysisListener, queue_size=1
        )
        self.faceAnalysisState = True # busy by default to avoid calling unactive service
        self.facePub = rospy.Publisher(
            'faces', face_array, queue_size=10
        )

        self.personPub = rospy.Publisher(
            'idPerson', Int32, queue_size=10
        )

        #Checking if Azure facelist is empty
        facelistsize = len(self.azureFaceList.face_client.face_list.get(self.face_list_id).persisted_faces)
        if facelistsize > 0:
            self.faceListEmpty = False
        else:
            self.faceListEmpty = True

    def faceAnalysisListener(self, msg):
        self.faceAnalysisState = msg.data
    
    def process_image(self, msg):
        try:
            self.received_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            #self.received_image = cv2.cvtColor(self.received_image, cv2.COLOR_BGR2RGB)
        except Exception as e:
            print("Image not received")
            print(e)

    def show_image(img):
        cv2.imshow('image', img)
        cv2.waitKey(1)
    
    def detect_faces(self, frame, detector_backend = 'ssd'):
        # detect faces
        imgFrame = np.asarray(frame)
        margin = self.margin
        try:
            faces_results: List[DetectionResult] = DeepFace.extract_faces(imgFrame, detector_backend = detector_backend)
        except:
            print("No faces detected")
            return None, None, None, None
        
        # dict_keys(['face', 'facial_area', 'confidence'])
        faces = []
        bboxes = []
        xy_list = []
        confidences = []
        for face in faces_results:
            #detecting faces
            face_area = face["facial_area"]
            face_img = imgFrame[max(0, face_area["y"]-margin):min(imgFrame.shape[0], face_area["y"]+face_area["h"]+margin), max(0, face_area["x"]-margin):min(imgFrame.shape[1], face_area["x"]+face_area["w"]+margin)]
            faces.append(face_img)
            confidences.append(face["confidence"])
            xy_list.append((face_area["x"], face_area["y"]))
            bbox = (max(0, face_area["x"]-margin), max(0, face_area["y"]-margin), face_area["w"]+margin, face_area["h"]+margin)
            bboxes.append(bbox)
            cv2.rectangle(frame, (max(0, face_area["x"]-margin), max(0, face_area["y"]-margin)), (min(imgFrame.shape[1], face_area["x"]+face_area["w"]+margin), min(imgFrame.shape[0], face_area["y"]+face_area["h"]+margin)), (0, 255, 0), 2)
        return faces, bboxes, xy_list , confidences
    
    def draw_prev_detections(self, frame):
        prev_detections = self.prev_detections
        for prev_detection in prev_detections:
            #drawing previous detections
            prev_detection_bbox = prev_detections[prev_detection]
            p1 = (int(prev_detection_bbox[0]), int(prev_detection_bbox[1]))
            p2 = (int(prev_detection_bbox[0] + prev_detection_bbox[2]), int(prev_detection_bbox[1] + prev_detection_bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
    
    def show_fps(self, frame, ):
        font = cv2.FONT_HERSHEY_SIMPLEX
        new_frame_time = time.time()
        # Calculating the fps
        fps = 1/(new_frame_time-self.prev_frame_time)
        self.prev_frame_time = new_frame_time
        fps = int(fps)
        fps = str(fps) + ' fps'
        # putting the FPS count on the frame
        cv2.putText(frame, fps, (7, 20), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    def check_prev_detections(self, frame, face):
        face_tracked = False
        prev_detections = self.prev_detections
        for prev_detection in prev_detections:
            #print(f"checking prev detection: {prev_detection}")
            face_id = prev_detection
            prev_detection_bbox = prev_detections[prev_detection]
            margin_w = prev_detection_bbox[2] * 0.3
            margin_h = prev_detection_bbox[3] * 0.3
            tracker_x = prev_detection_bbox[0]
            tracker_y = prev_detection_bbox[1]
            tracker_w = prev_detection_bbox[2]
            tracker_h = prev_detection_bbox[3]
            face_x = face[2][0]
            face_y = face[2][1]
            face_w = face[2][2]
            face_h = face[2][3]
            if ((tracker_x - margin_w < face_x < tracker_x + margin_w) and 
                (tracker_y - margin_h < face_y < tracker_y + margin_h)):
                face_tracked = True
                cv2.putText(frame, face_id, (face[0][0], face[0][1]), self.font, self.fontsize, (100, 255, 0), 1, cv2.LINE_AA)
                # updating tracker
                prev_detections[prev_detection] = face[2]
                #print(f"appending {face_id} to faces tracked")
                self.faces_tracked.append(face_id)
                face_tracked = True
                # return face_id to know which face 
                return face_tracked, face_id
        return face_tracked, None

    def checkAvailableCalls(self):

        print(f"comparing {time.time() - self.minuteCounter}")
        if (time.time() - self.minuteCounter) >= 60:
            print("RESTARTED AZURE LIMIT")
            self.azureLimit = self.azureMaxCalls
            self.minuteCounter += 60
        
    def findFace(self, face, frame):
        # face zip is face[0] = xy_lists, face[1] = faces, face[2] = bboxes, face[3] = confidences
        # Uploading to supabase

        

        if face[3] < 0.95:
            print("Not enough confidence")
            return
        if self.azureLimit <=0:
            print("Out of calls available")
            return
        self.azureLimit = self.azureLimit - 1
        print(f"Calls available : {self.azureLimit}")
        self.bucketSize +=1
        print(f"Size of bucket: {self.bucketSize}")
        
        
        time1 = time.time()
        face_jpg = cv2.imencode(".jpg", face[1])[1].tobytes()
        destination = f"image_{self.bucketSize+4}.jpg"
        print(destination)
        try:
            res = self.supabase.storage.from_('bucketTest').upload(destination, face_jpg)
        except:
            res = self.supabase.storage.from_('bucketTest').upload(destination, face_jpg)
        time2 = time.time()
        print("Time to upload: ", time2-time1)
        print(res)

        supabase_url = f"https://nvuidsrualaiwfoyvfum.supabase.co/storage/v1/object/public/bucketTest/{destination}"

        # Verifying if face is in azure face list
        print("Verifying face on azure...")
        time.sleep(0.2) #Wait for the face to appear on url
        verify_result = None
        
        if not self.faceListEmpty:
            try:
                verify_result = self.azureFaceList.verifyFaceFromUrl(supabase_url)
            except:
                print("Could not recognize face")
                return
        else:
            print("Face list is empty")
        if verify_result is not None:
            face_id = verify_result.persisted_face_id
            print(f"Verify succesful, found face_id: {face_id}")
            if (face_id == '82f8c353-c942-4979-a1e2-7817966bc94d'):
                #Michael - Ivan
                self.IDDetected = 1
            elif (face_id == '4c33f70d-ed75-418e-8aca-282474c86c0c'):
                #Morgan - Emiliano
                self.IDDetected = 2
            # Adding Tracker
            if face_id not in self.prev_detections:
                #Creating tracker for face with identity face_id
                print("adding tracker")
                self.prev_detections[face_id] = face[2]
                self.faces_tracked.append(face_id)
                print(self.prev_detections)
                print(f"init tracker with id {face_id}")
            
            # If attributes are generated on external service
            data = {}
            try:
                f = open(self.json_path)
                data = json.load(f)
            except:
                print("Creating json file")
            if face_id not in data:
                data[face_id] = {}
                with open(self.json_path, 'w') as outfile:
                    json.dump(data, outfile)

            cv2.putText(frame, face_id, (face[0][0], face[0][1]), self.font, self.fontsize, (100, 255, 0), 1, cv2.LINE_AA)
            print("text added to frame")
        else:
            print("Face not found on azure, will not add")
            

            
            '''
            find_result:List[FaceRecognitionRow] = DeepFace.find(face[1], db_path = self.faces_path, enforce_detection=False)[0]
            if(len(find_result) > 0):
                print("Face found")
                #print(find_result)
                print(face[0])
                face_id_path = find_result['identity'][0]
                path_to_face, face_id = os.path.split(face_id_path)
                print(f"face id is {face_id}")

                if face_id not in self.prev_detections:
                    #Creating tracker for face with identity face_id
                    print("adding tracker")
                    self.prev_detections[face_id] = face[2]
                    self.faces_tracked.append(face_id)
                    print(self.prev_detections)
                    print(f"init tracker with id {face_id}")
                
                # If attributes are generated on external service
                data = {}
                try:
                    f = open(self.json_path)
                    data = json.load(f)
                except:
                    print("Creating json file")
                if face_id not in data:
                    data[face_id] = {}
                    with open(self.json_path, 'w') as outfile:
                        json.dump(data, outfile)

                cv2.putText(frame, face_id, (face[0][0], face[0][1]), self.font, self.fontsize, (100, 255, 0), 1, cv2.LINE_AA)
                print("text added to frame")
            
            else:
                print("face not found")
                # save face to database
                cv2.imwrite(f"{self.faces_path}/face_{self.img_counter}.jpg", face[1])
                self.img_counter += 1
                # erase .pkl file
                os.remove(self.representations_path)
                #no find result (get data from face)
                '''
    
    def checkAttributes(self, face_id, bbox):
        #load json
        data = {}
        try:
            f = open(self.json_path)
            data = json.load(f)
        except:
            print("No json file")
            return
        
        if face_id in data:
            #print("Face in json")
            if "age" not in data[face_id]:
                if not self.faceAnalysisState:
                    try:
                        callSrv = rospy.ServiceProxy("faceAnalysisSrv", imagetoAnalyze)
                        print("calling service")
                        facetoAnalyze = face()
                        facetoAnalyze.identity = face_id
                        facetoAnalyze.x = bbox[0]
                        facetoAnalyze.y = bbox[1]
                        facetoAnalyze.w = bbox[2]
                        facetoAnalyze.h = bbox[3]
                        callSrv(facetoAnalyze)
                        print("call ended")
                    except rospy.ServiceException as e:
                        print("Service call failed: %s"%e)
                else:
                    print("Face analysis service is busy")
        else:
            print("face not in json")
            return

    def publishFaces(self):
        faces_publish = []
        if self.prev_detections:
            for i, prev_detection in enumerate(self.prev_detections):
                face_id = prev_detection
                bbox = self.prev_detections[prev_detection]
                face_publish = face()
                face_publish.identity = face_id
                face_publish.x = bbox[0]
                face_publish.y = bbox[1]
                face_publish.w = bbox[2]
                face_publish.h = bbox[3]
                faces_publish.append(face_publish)
            #print(f"Published {faces_publish}")
            self.facePub.publish(faces_publish)

    def publishID(self):
        self.personPub.publish(self.IDDetected)
    
    def run(self):
        while not rospy.is_shutdown():
            #self.checkAvailableCalls()
            self.publishID()
            if self.received_image is not None:
                print("----------------------------------------------------")
                frame = self.received_image
                # face detection, detector_backend can be changed
                faces, bboxes, xy_lists, confidences = self.detect_faces(frame, detector_backend = 'ssd')
                # Optional, to draw previous detection bounding box
                self.draw_prev_detections(frame)
                self.faces_tracked = []

                if faces == None:
                    #to avoid error
                    faces = []
                if(len(faces) > 0):
                    for face in zip(xy_lists, faces, bboxes, confidences):
                        # Check if face is being tracked, if not, run recognition
                        #Calculate face area
                        faceArea = face[2][2] * face[2][3]
                        print(f"Face area is {faceArea}")
                        if faceArea > self.minFaceArea:
                            
                            face_tracked, face_tracked_id = self.check_prev_detections(frame, face)
                            if not face_tracked:
                                self.findFace(face, frame)
                            else:
                                self.checkAttributes(face_tracked_id, face[2])
                        else:
                            print("Face too far")
                else:
                    self.IDDetected=0
                    # if no faces, empty faces tracked list

                    self.faces_tracked = []
                
                # Delete trackers that are not in frame
                detections_to_delete = []
                detections_to_preserve = []
                for prev_detection in self.prev_detections:
                    if prev_detection not in self.faces_tracked:
                        #deleting tracker
                        #print(f"Deleting tracker with id {prev_detection}")
                        detections_to_delete.append(prev_detection)
                        #break
                    else:
                        detections_to_preserve.append(prev_detection)
                
                #self.prev_detections = detections_to_preserve

                for detection_to_delete in detections_to_delete:
                    del self.prev_detections[detection_to_delete]
                        
                self.show_fps(frame)
                self.publishFaces()
                cv2.imshow('image', frame)
                cv2.waitKey(1)

            else:
                print("Waiting for image")
                sleep(1)
                continue
        print("Shutting down")
        if deleteAzureListonStop:
            self.azureFaceList.deleteFaceList()

if useGPU:
    faceRecognition().run()
else:
    with tf.device('/cpu:0'):
        faceRecognition().run()