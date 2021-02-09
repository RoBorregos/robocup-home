# ----------------------------------------------------------------------
#  Development
# ----------------------------------------------------------------------

#: Builds a Docker image from the current Dockerfile file
build:
	@docker build -t ros:noetic .


#: Create Docker container
create: 
	@docker run \
		-it -d \
		-v ${shell pwd}/catkin_home:/catkin_home \
		--network host \
		--name ros-noetic \
		ros:noetic

#: Start the container in background
up:
	@docker start ros-noetic

#: Stop the container
down:
	@docker stop ros-noetic

#: Restarts the container
restart:
	@docker restart ros-noetic

#: Shows the logs of the ros-noetic service container
logs:
	@docker logs --tail 50 ros-noetic

#: Fires up a bash session inside the ros-noetic service container
shell:
	@docker exec -it ros-noetic bash

#: Remove ros-noetic container. 
remove:
	@docker container rm ros-noetic

#: Show a list of containers.
list:
	@docker container ls -a

#: Show a list of containers.
listUp:
	@docker ps

