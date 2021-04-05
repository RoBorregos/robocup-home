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
	pip install -r ./requirements.txt 
	cd ./models/requirements
	mkdir TensorFlow && cd TensorFlow/ # create directory contaning object detection api
	git clone https://github.com/tensorflow/models.git
	cd models/research
	protoc object_detection/protos/*.proto --python_out=. # generate dependency from object detection API
	cd ../../../ # models/
	mkdir coco && cd coco/
	git clone https://github.com/cocodataset/cocoapi.git
	cd cocoapi/PythonAPI
	make
	cp -r pycocotools ../../../TensorFlow/models/research/
	cd ../../../TensorFlow/models/research/
	cp object_detection/packages/tf2/setup.py .
	python -m pip install .
	python object_detection/builders/model_builder_tf2_test.py

else
	echo $'\nOperation cancelled successfully'
fi