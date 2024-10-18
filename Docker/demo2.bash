xhost local:root
XAUTH=/tmp/.docker.xauth
docker run --rm -it \
    --name=grasp_control_container\
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/home/$USER/catkin_ws/src/robotiq_gripper:/catkin_ws/src/robotiq_gripper" \
    --volume="/home/$USER/catkin_ws/src/ur10e_gripper:/catkin_ws/src/ur10e_gripper" \
    --volume="/home/$USER/catkin_ws/src/ur10e_gripper_moveit:/catkin_ws/src/ur10e_gripper_moveit" \
    --volume="/home/$USER/catkin_ws/src/grasp_eyeinhand:/catkin_ws/src/grasp_eyeinhand" \
    --volume="/home/$USER/catkin_ws/src/eyeinhand_pkg/detect_pose_pkg:/catkin_ws/src/detect_pose_pkg" \
    --volume="/home/$USER/catkin_ws/src/eyeinhand_pkg/calc_pose_pkg:/catkin_ws/src/calc_pose_pkg" \
    --volume="/home/$USER/catkin_ws/src/eyeinhand_pkg/grasp_object_pkg:/catkin_ws/src/grasp_object_pkg" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --net=host \
    --privileged \
    ros-noetic-lab-automation-demo \
    bash
echo "Done."
