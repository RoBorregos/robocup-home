# ----------------------------------------------------------------------
#  Development Noetic
# ----------------------------------------------------------------------

#: Builds a Docker image with the corresponding Dockerfile file
noetic.build:
	@docker build -t ros:home -f docker/noetic/Dockerfile .
noetic.tiago.build:
	@docker build -t ros:hometgo -f docker/noetic/Dockerfile.tiago .
noetic.jetson.build:
	@docker build -t ros:home -f docker/noetic/Dockerfile.jetson .
noetic.speech.build:
	@docker build -t ros:home -f docker/noetic/Dockerfile.speech .
noetic.speech.prod.build:
	@docker login ghcr.io
	@docker build -t ros:homesp -f docker/noetic/Dockerfile.speech.prod .
noetic.navigation.build:
	@docker build -t ros:home -f docker/noetic/Dockerfile.navigation .
noetic.objectDetection.build:
	@docker build -t ros:home -f docker/noetic/Dockerfile.objectDetection .
noetic.objectDetection.prod.build:
	@docker login ghcr.io
	@docker build -t ros:homeod -f docker/noetic/Dockerfile.objectDetection.prod .

#: Create Generic Docker container
noetic.create: 
	@./docker/run_scripts/run.bash
noetic.create.intel: 
	@./docker/run_scripts/runIntelGpu.bash
noetic.create.nvidia: 
	@./docker/run_scripts/runNvidiaGpu.bash

#: Create Navigation Docker container
noetic.navigation.create: 
	@./docker/run_scripts/run.bash IS_NAVIGATION
noetic.navigation.create.intel: 
	@./docker/run_scripts/runIntelGpu.bash IS_NAVIGATION
noetic.navigation.create.nvidia: 
	@./docker/run_scripts/runNvidiaGpu.bash IS_NAVIGATION

#: Create Speech Docker container
noetic.speech.create: 
	@./docker/run_scripts/run.bash IS_SPEECH
noetic.speech.create.intel: 
	@./docker/run_scripts/runIntelGpu.bash IS_SPEECH
noetic.speech.create.nvidia: 
	@./docker/run_scripts/runNvidiaGpu.bash IS_SPEECH
	
#: Create Speech Docker container
noetic.speech.prod.create: 
	@./docker/run_scripts/runspeech.bash IS_SPEECH
noetic.speech.prod.create.intel: 
	@./docker/run_scripts/runIntelGpu.bash IS_SPEECH
noetic.speech.prod.create.nvidia: 
	@./docker/run_scripts/runNvidiaGpu.bash IS_SPEECH

#: Create ObjectDetection Docker container
noetic.objectDetection.create: 
	@./docker/run_scripts/runobj.bash IS_OBJECT_DETECTION
noetic.objectDetection.create.intel: 
	@./docker/run_scripts/runIntelGpu.bash IS_OBJECT_DETECTION
noetic.objectDetection.create.nvidia: 
	@./docker/run_scripts/runNvidiaGpu.bash IS_OBJECT_DETECTION

#: Create ObjectDetection Prod Docker container
noetic.objectDetection.prod.create: 
	@./docker/run_scripts/runobjdtc.bash IS_OBJECT_DETECTION_PROD
noetic.objectDetection.prod.create.intel: 
	@./docker/run_scripts/runIntelGpu.bash IS_OBJECT_DETECTION_PROD
noetic.objectDetection.prod.create.nvidia: 
	@./docker/run_scripts/runNvidiaGpuOd.bash IS_OBJECT_DETECTION_PROD

#: Start the container in background
noetic.up:
	@xhost +
	@docker start ros-home

noetic.speech.up:
	@xhost +
	@docker start ros-homesp

noetic.objectDetection.up:
	@xhost +
	@docker start ros-homeod

noetic.tiago.up:
	@xhost +
	@docker start ros-hometgo

#: Stop the container
noetic.down:
	@docker stop ros-home

noetic.speech.down:
	@docker stop ros-homesp

noetic.objectDetection.down:
	@docker stop ros-homeod

#: Restarts the container
noetic.restart:
	@docker restart ros-home

#: Shows the logs of the ros-home service container
noetic.logs:
	@docker logs --tail 50 ros-home

#: Fires up a bash session inside the ros-home service container
noetic.shell:
	@docker exec -it ros-home bash

noetic.speech.shell:
	@docker exec -it ros-homesp bash

noetic.tiago.shell:
	@docker exec -it ros-hometgo bash
#: Remove ros-home container. 
noetic.remove: noetic.down
	@docker container rm ros-home

noetic.speech.remove: noetic.speech.down
	@docker container rm ros-homesp

noetic.objectDetection.remove: noetic.objectDetection.down
	@docker container rm ros-homeod

# ----------------------------------------------------------------------
#  General Docker
# ----------------------------------------------------------------------

#: Show a list of containers.
list:
	@docker container ls -a

#: Show a list of containers.
listUp:
	@docker ps
