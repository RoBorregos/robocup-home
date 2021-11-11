# ----------------------------------------------------------------------
#  Development Noetic
# ----------------------------------------------------------------------

#: Builds a Docker image from the current Dockerfile file
noetic.build:
	@docker build -t ros:noetic -f docker/noetic/Dockerfile .
noetic.speech.build:
	@docker build -t ros:noetic-speech -f docker/noetic/Dockerfile.speech .
noetic.navigation.build:
	@docker build -t ros:noetic-navigation -f docker/noetic/Dockerfile.navigation .

#: Create Docker container
noetic.create: 
	@docker run \
		-it -d \
		-v ${shell pwd}/catkin_home/src:/catkin_home/src \
		-v ${shell pwd}/catkin_home/typings:/catkin_home/typings \
		--network host \
		--name ros-noetic \
		ros:noetic
noetic.speech.create:
	@./docker/noetic/runSpeech.bash
noetic.navigation.create:
	@./docker/noetic/runNavigation.bash

#: Start the container in background
noetic.up:
	@docker start ros-noetic
noetic.speech.up:
	@docker start ros-noetic-speech
noetic.navigation.up:
	@docker start ros-noetic-navigation

#: Stop the container
noetic.down:
	@docker stop ros-noetic
noetic.speech.down:
	@docker stop ros-noetic-speech
noetic.navigation.down:
	@docker stop ros-noetic-navigation

#: Restarts the container
noetic.restart:
	@docker restart ros-noetic
noetic.speech.restart:
	@docker restart ros-noetic-speech
noetic.navigation.restart:
	@docker restart ros-noetic-navigation

#: Shows the logs of the ros-noetic service container
noetic.logs:
	@docker logs --tail 50 ros-noetic
noetic.speech.logs:
	@docker logs --tail 50 ros-noetic-speech
noetic.navigation.logs:
	@docker logs --tail 50 ros-noetic-navigation

#: Fires up a bash session inside the ros-noetic service container
noetic.shell:
	@docker exec -it ros-noetic bash
noetic.speech.shell:
	@docker exec -it ros-noetic-speech bash
noetic.navigation.shell:
	@docker exec -it ros-noetic-navigation bash

#: Remove ros-noetic container. 
noetic.remove: noetic.down
	@docker container rm ros-noetic
noetic.speech.remove: noetic.speech.down
	@docker container rm ros-noetic-speech
noetic.navigation.remove: noetic.navigation.down
	@docker container rm ros-noetic-navigation

# ----------------------------------------------------------------------
#  General Docker
# ----------------------------------------------------------------------

#: Show a list of containers.
list:
	@docker container ls -a

#: Show a list of containers.
listUp:
	@docker ps
