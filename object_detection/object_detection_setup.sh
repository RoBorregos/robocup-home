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
	source ~/anaconda3/etc/profile.d/conda.sh # allow to activate the conda env from shell
	conda activate object_detection_env # activate the virtual env
	cd ./models/dependencies
	mkdir TensorFlow && cd TensorFlow/ # create directory contaning object detection api
	git clone https://github.com/tensorflow/models.git
	cd models/research # models/dependencies/Tensorflow/models/research
	protoc object_detection/protos/*.proto --python_out=. # generate dependency from object detection API
	cd ../../../ # models/dependencies
	mkdir coco && cd coco/ # models/dependencies/coco
	git clone https://github.com/cocodataset/cocoapi.git # models/dependencies/coco/cocoapi
	pip install -r ../../../requirements.txt
	cd ./cocoapi/PythonAPI # models/dependencies/coco/cocoapi/PythonAPI/
	make
	cd ../../../ # models/dependencies
	cp -r coco/cocoapi/PythonAPI/pycocotools ./TensorFlow/models/research/ 
	mv ./setup.py ./TensorFlow/models/research
	python -m pip install ./TensorFlow/models/research
	echo $'\nInstallation completed'

else
	echo $'\nOperation cancelled successfully'
fi