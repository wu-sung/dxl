# dxl

ros2 test package for dynamixel on Jetson nano

dependency : ros2 foxy, dynamixel C++ SDK, cmake 3.16

Publisher node captures an image from csi camera via gstreamer and publishes a compressed image topic with jpg format using a ros2 interface sensor_msgs/msg/CompressedImage.

Subscriber node subscribes the compressed image topic and sends it to PC via gstreamer.

Open linux terminal on Jetson nano

$ cd ~/ros2_ws/src

$ git clone https://github.com/2sungryul/camera.git

Make sure you need to change the ip address in sub.cpp to that of your own PC.

$ cd ~/ros2_ws

$ colcon build --symlink-install --packages-select camera

$ source install/local_setup.bash

$ ros2 run camera pub

Open new linux terminal on Jetson nano

$ ros2 run camera sub
