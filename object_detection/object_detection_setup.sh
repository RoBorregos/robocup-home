#!/bin/bash

cat << EOF	
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

Important Disclaimer

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

It's very important that you run this bash script under the directory
Robocup-Home/object_detection to expect a right functionality of 
the dependencies and files that are going to be installed. 

EOF

read -p "Are you sure you want to continue? [y/n] " -n 1 -r
if [[ $REPLY =~ ^[Yy]$ ]]
then
	echo $'\nStarting to install packages'
	conda create -n object_detection_env pip python=3.8 # create the working virtual environment with conda
	conda activate object_detection_env # activate the virtual env
	cd ./models/requirements
	mkdir TensorFlow && cd TensorFlow/ # create directory contaning object detection api
	git clone https://github.com/tensorflow/models.git
	cd models/research # models/requirements/Tensorflow/models/research
	protoc object_detection/protos/*.proto --python_out=. # generate dependency from object detection API
	cd ../../../ # models/requirements
	mkdir coco && cd coco/
	git clone https://github.com/cocodataset/cocoapi.git
	cd cocoapi/PythonAPI # models/requirements/coco/cocoapi/PythonAPI/
	make
	cd ../../../ # models/requirements
	cp -r pycocotools ./TensorFlow/models/research/
	pip install -r ../../requirements.txt 
	python -m pip install .
	cd ./TensorFlow/models/research/
	python object_detection/builders/model_builder_tf2_test.py # test for correct installation
	echo $'\nInstallation completed'

else
	echo $'\nOperation cancelled successfully'
fi