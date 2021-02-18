# ----------------------------------------------------------------------
#  Development Noetic
# ----------------------------------------------------------------------

#: Builds a Docker image from the current Dockerfile file
noetic.build:
	@docker build -t ros:noetic -f Dockerfile.noetic .


#: Create Docker container
noetic.create: 
	@docker run \
		-it -d \
		-v ${shell pwd}/catkin_home:/catkin_home \
		--network host \
		--name ros-noetic \
		ros:noetic

#: Start the container in background
noetic.up:
	@docker start ros-noetic

#: Stop the container
noetic.down:
	@docker stop ros-noetic

#: Restarts the container
noetic.restart:
	@docker restart ros-noetic

#: Shows the logs of the ros-noetic service container
noetic.logs:
	@docker logs --tail 50 ros-noetic

#: Fires up a bash session inside the ros-noetic service container
noetic.shell:
	@docker exec -it ros-noetic bash

#: Remove ros-noetic container. 
noetic.remove: noetic.down
	@docker container rm ros-noetic

# ----------------------------------------------------------------------
#  Development Melodic
# ----------------------------------------------------------------------

#: Builds a Docker image from the current Dockerfile file
melodic.build:
	@docker build -t ros:melodic -f docker/melodic/Dockerfile .
melodic.speech.build:
	@docker build -t ros:melodic-speech -f docker/melodic/Dockerfile.speech .
melodic.navigation.build:
	@docker build -t ros:melodic-navigation -f docker/melodic/Dockerfile.navigation .

#: Create Docker container
melodic.create: 
	@docker run \
		-it -d \
		-v ${shell pwd}/catkin_home/src:/catkin_home/src \
		-v ${shell pwd}/catkin_home/typings:/catkin_home/typings \
		--network host \
		--name ros-melodic \
		ros:melodic
melodic.speech.create:
	@docker run \
		-it -d \
		--gpus all \
		--device /dev/snd:/dev/snd \
		-v ${shell pwd}/catkin_home/src:/catkin_home/src \
		-v ${shell pwd}/catkin_home/typings:/catkin_home/typings \
		--network host \
		--name ros-melodic-speech \
		ros:melodic-speech
melodic.navigation.create:
	@./docker/melodic/runNavigation.bash

#: Start the container in background
melodic.up:
	@docker start ros-melodic
melodic.speech.up:
	@docker start ros-melodic-speech
melodic.navigation.up:
	@docker start ros-melodic-navigation

#: Stop the container
melodic.down:
	@docker stop ros-melodic
melodic.speech.down:
	@docker stop ros-melodic-speech
melodic.navigation.down:
	@docker stop ros-melodic-navigation

#: Restarts the container
melodic.restart:
	@docker restart ros-melodic
melodic.speech.restart:
	@docker restart ros-melodic-speech
melodic.navigation.restart:
	@docker restart ros-melodic-navigation

#: Shows the logs of the ros-melodic service container
melodic.logs:
	@docker logs --tail 50 ros-melodic
melodic.speech.logs:
	@docker logs --tail 50 ros-melodic-speech
melodic.navigation.logs:
	@docker logs --tail 50 ros-melodic-navigation

#: Fires up a bash session inside the ros-melodic service container
melodic.shell:
	@docker exec -it ros-melodic bash
melodic.speech.shell:
	@docker exec -it ros-melodic-speech bash
melodic.navigation.shell:
	@docker exec -it ros-melodic-navigation bash

#: Remove ros-melodic container. 
melodic.remove: melodic.down
	@docker container rm ros-melodic
melodic.speech.remove: melodic.speech.down
	@docker container rm ros-melodic-speech
melodic.navigation.remove: melodic.navigation.down
	@docker container rm ros-melodic-navigation

# ----------------------------------------------------------------------
#  General Docker
# ----------------------------------------------------------------------

#: Show a list of containers.
list:
	@docker container ls -a

#: Show a list of containers.
listUp:
	@docker ps
