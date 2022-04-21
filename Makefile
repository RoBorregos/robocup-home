# ----------------------------------------------------------------------
#  Development Noetic
# ----------------------------------------------------------------------

#: Builds a Docker image with the corresponding Dockerfile file
noetic.build:
	@docker build -t ros:home -f docker/noetic/Dockerfile .
noetic.speech.build:
	@docker build -t ros:home -f docker/noetic/Dockerfile.speech .
noetic.navigation.build:
	@docker build -t ros:home -f docker/noetic/Dockerfile.navigation .
noetic.objectDetection.build:
	@docker build -t ros:home -f docker/noetic/Dockerfile.objectDetection .
noetic.objectDetection.prod.build:
	@docker login ghcr.io
	@docker build -t ros:home -f docker/noetic/Dockerfile.objectDetection.prod .

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

#: Create ObjectDetection Docker container
noetic.objectDetection.create: 
	@./docker/run_scripts/run.bash IS_OBJECT_DETECTION
noetic.objectDetection.create.intel: 
	@./docker/run_scripts/runIntelGpu.bash IS_OBJECT_DETECTION
noetic.objectDetection.create.nvidia: 
	@./docker/run_scripts/runNvidiaGpu.bash IS_OBJECT_DETECTION

#: Create ObjectDetection Prod Docker container
noetic.objectDetection.prod.create: 
	@./docker/run_scripts/run.bash IS_OBJECT_DETECTION_PROD
noetic.objectDetection.prod.create.intel: 
	@./docker/run_scripts/runIntelGpu.bash IS_OBJECT_DETECTION_PROD
noetic.objectDetection.prod.create.nvidia: 
	@./docker/run_scripts/runNvidiaGpu.bash IS_OBJECT_DETECTION_PROD

#: Start the container in background
noetic.up:
	@xhost +
	@docker start ros-home

#: Stop the container
noetic.down:
	@docker stop ros-home

#: Restarts the container
noetic.restart:
	@docker restart ros-home

#: Shows the logs of the ros-home service container
noetic.logs:
	@docker logs --tail 50 ros-home

#: Fires up a bash session inside the ros-home service container
noetic.shell:
	@docker exec -it ros-home bash

#: Remove ros-home container. 
noetic.remove: noetic.down
	@docker container rm ros-home

# ----------------------------------------------------------------------
#  General Docker
# ----------------------------------------------------------------------

#: Show a list of containers.
list:
	@docker container ls -a

#: Show a list of containers.
listUp:
	@docker ps
