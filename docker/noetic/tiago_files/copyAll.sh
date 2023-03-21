# Modify tiago files
docker cp default_controllers.launch 372d434031a1:/tiago_public_ws/src/tiago_simulation/tiago_controller_configuration_gazebo/launch/default_controllers.launch
docker cp tiago.srdf.xacro 372d434031a1:/tiago_public_ws/src/tiago_moveit_config/config/srdf/tiago.srdf.xacro
docker cp upload.launch 372d434031a1:/tiago_public_ws/src/tiago_robot/tiago_description/robots/upload.launch
docker cp tiago.urdf.xacro 372d434031a1:/tiago_public_ws/src/tiago_robot/tiago_description/robots/tiago.urdf.xacro
# Related to base controller
docker cp controllers_pal-gripper.yaml 372d434031a1:/tiago_public_ws/src/tiago_moveit_config/config/controllers/controllers_pal-gripper.yaml
docker cp controllers_pal-gripper_schunk-ft.yaml 372d434031a1:/tiago_public_ws/src/tiago_moveit_config/config/controllers/controllers_pal-gripper_schunk-ft.yaml
docker cp tiago_pal-gripper_schunk-ft.srdf 372d434031a1:/tiago_public_ws/src/tiago_moveit_config/config/srdf/tiago_pal-gripper_schunk-ft.srdf
docker cp joint_limits.yaml 372d434031a1:/tiago_public_ws/src/tiago_moveit_config/config/joint_limits.yaml
docker cp kinematics_trac_ik.yaml 372d434031a1:/tiago_public_ws/src/tiago_moveit_config/config/kinematics_trac_ik.yaml
docker cp kinematics_kdl.yaml 372d434031a1:/tiago_public_ws/src/tiago_moveit_config/config/kinematics_kdl.yaml
docker cp ompl_planning.yaml 372d434031a1:/tiago_public_ws/src/tiago_moveit_config/config/ompl_planning.yaml
docker cp stomp_planning.yaml 372d434031a1:/tiago_public_ws/src/tiago_moveit_config/config/stomp_planning.yaml
