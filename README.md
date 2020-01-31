# RoBorregos @Home

This repository contains the development of RoBorregos robotic's solution for RoboCup's @HOME competition, showing its logical process adapted to the mechanical design of the robot.

## Areas of development


### Mechanical structure

>#### Luis Fernando Garza Vera
>Design and manufacture of the robotic arm system.

>#### Carlos Amín Méndez Cáceres
>Solidwork design of the base and manufacturing of structures.

>#### Aldo Jesus Samaniego Silva
>CAD and mechanical structure manufacturing.


### Speech recognition
>#### Diego Alejandro Cardozo Campos
>Creation flow for processing speech in real-time. A python node that captures the audio and publishes it to a topic. A C++ node takes the chunks of audio, checks for a voice, removes the noise and publishes it to a topic. Finally, another Python node that takes the voice audio, converts to text and publishes it.

>#### José Alfonso Cisneros Morales
>Created a program to efficiently obtain audio samples from diverse gender/age pronunciations for the dataset, focusing on the phrases and vocabulary of the competition, as well as training the Tensorflow Speech Model.

>[Audio dataset obtainer](bit.ly/home-dataset)

### Natural language processing
>#### Paul Enrique Vazquez Badillo
>Building of command parsing unit using the RASA stack for python 3.5 as an external API outside the ROS project. The action-selectors package calls the API with the text to process. The response generated contains the user_intent, the entities identified in the text, and the appropriate response from the bot to the user.

### Computer vision

>#### Salvador Alexis Virgen Flores
>Detection of objects using Tensorflow.

>#### Iqui Balam Heredia Marin
>Detection of objects position in 3D that will the targets of the Robot Arm.

### Navigation

>#### Clara Gutiérrez Jaime
>Research to develop the base controller for our robot, which involves converting distances from the robot's current position and orientation (odometry) into velocity commands. These make the robot move to the desired location. 

>#### Ricardo Abraham Chapa Romero
>Research in lidar usage, RPLidar package management, and communication with the Navigation Stack. Readings from the lidar are passed to the ROS NAV architecture by topic communication. This helps in the creation of the map, SLAM, and pathfinding.

>#### Aurora Tijerina Berzosa
>Communication with base controller and odometry topic messaging with the Navigation Stack.
Robot's movements are converted into bi-dimensional velocities with a specific direction and then sent into the Odom topic for navigation.


### Electronics

>#### Paul Enrique Vazquez Badillo
>Base Motor Control Board Design and Layout.
This board will be used to control the four main motors for the robot movement. It has two Monster Shields, an Atmega 2560 and several I/O power components.

