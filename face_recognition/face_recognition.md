# Face Recognition Research

For the purpose of documentation, I will be writing the most important information that I could obtain from different resources that give insights of state of the art algorithms for face recognition applications.

## How facial recognition works

[Link to the article](https://electronics.howstuffworks.com/gadgets/high-tech-gadgets/facial-recognition.htm)

### Important points about a face

- The more social an animal is, the more plain face they have
- Each face has landmarks, which are distinguishable features that differ from one to other
- Nodal points == landmarks; there could be up to 80 of these on a face (e.g. distance between the eyes, width of nose, etc.)

### Steps when running a face detection algorithm

- Detection
- Alignment
- Measurement
- Representation
- Matching
- Verification/Identification

## Face recognition with stendex

[Link to the video](https://www.youtube.com/watch?v=88HdqNDQsEk&list=PLdrGOnC-LQ1SwdwdUXW2MbF-alendP6P4&index=1)

For this video, OpenCV and Numpy libraies were used. It uses the concept of Haar Cascades, which are XML files
with the most important features of a specific objetc, e.g. an eye or a face.

## Face recognition with CodingEntrepreneurs

[Link to GitHub repo](https://github.com/codingforentrepreneurs/OpenCV-Python-Series)

[Link to personal fork](https://github.com/sebasrivera96/OpenCV-Python-Series)

[Link to the video](https://www.youtube.com/watch?v=PmZ29Vta7Vc&list=PLEsfXFp6DpzRyxnU-vfs3vk-61Wpt7bOS&index=6&t=0s)

Contains useful information about the detection of a face and the training to idenity a specific person. It uses a simple Machine Learning (ML) technique called **Haar Cascades** which transforms the features into information that could be stored on a XML file.

### Update 26.10.2018

For this code, images were taken to train the algorithm and the recognition is poorly working. For better results, a greater amount of images should be taken. Even while testing with images that were used for the training, the face is not always correctly identified. Up to this moment, no real time test has been done with a webcam; this is one of the next steps. For testing pourposes, images from the repo are being used to test the algorithm.

### Next Steps

1. Read about how to improve a [Haar Cascade](https://docs.opencv.org/3.3.0/d7/d8b/tutorial_py_face_detection.html)
2. Real time test with a webcam and on a RaspberryPi

## Face recognition with ROS

This is a good option when planning to build a robot or a complex system that has different component interacting in real time, e.g. sensors, actuators, microcontrollers, etc.

## Face recognition with Python's Face recognition library

[Link to GitHub repo](https://github.com/ageitgey/face_recognition)

[Link to video](https://www.youtube.com/watch?v=IWoigw6-crg&list=PLdrGOnC-LQ1SwdwdUXW2MbF-alendP6P4&index=4&t=110s)

[Link to PyPI](https://pypi.org/project/face_recognition/)

[Link to official documentation](https://media.readthedocs.org/pdf/face-recognition/latest/face-recognition.pdf)

[API Docs](https://face-recognition.readthedocs.io/en/latest/face_recognition.html)

- Based on [__dlib__](http://dlib.net/) which is a state-of-the-art model built using deep learning.

- This library also provides a CLI tool. For more information on usage, check the __official documentation__.

## Face recognition with ROS People Object Detection Tensorflow

ROS Toolbox for Object Detection & tracking. This seems to be an excelent option for face recognition, the issue is that it has multiple dependencies.

[Link to Github repo](https://github.com/cagbal/ros_people_object_detection_tensorflow)

## tensorflow_ros package

The easy way to get C++ Tensorflow API in ROS

[Link to official announcement](https://discourse.ros.org/t/announcing-the-tensorflow-ros-package-the-easy-way-to-get-to-c-tensorflow-api-in-ros/5112)

[Link to documentation](http://wiki.ros.org/tensorflow_ros_cpp)

## Face recognition with Google API
