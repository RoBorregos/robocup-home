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
noetic.remove:
	@docker container rm ros-noetic

# ----------------------------------------------------------------------
#  Development Melodic
# ----------------------------------------------------------------------

#: Builds a Docker image from the current Dockerfile file
melodic.build:
	@docker build -t ros:melodic -f Dockerfile.melodic .

#: Create Docker container
melodic.create: 
	@docker run \
		-it -d \
		-v ${shell pwd}/catkin_home:/catkin_home \
		--network host \
		--name ros-melodic \
		ros:melodic

#: Start the container in background
melodic.up:
	@docker start ros-melodic

#: Stop the container
melodic.down:
	@docker stop ros-melodic

#: Restarts the container
melodic.restart:
	@docker restart ros-melodic

#: Shows the logs of the ros-melodic service container
melodic.logs:
	@docker logs --tail 50 ros-melodic

#: Fires up a bash session inside the ros-melodic service container
melodic.shell:
	@docker exec -it ros-melodic bash

#: Remove ros-melodic container. 
melodic.remove:
	@docker container rm ros-melodic

# ----------------------------------------------------------------------
#  General Docker
# ----------------------------------------------------------------------

#: Show a list of containers.
list:
	@docker container ls -a

#: Show a list of containers.
listUp:
	@docker ps

