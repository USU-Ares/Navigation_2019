# Navigation_2019

Autonomous navigation package for a rover using ROS.

## Prepare a bag file 

First, have to filter the .bag file sense the .bag file has several messages in it.

run `rosbag filter [bag file to filter] [output_file.bag] "topic == '[topic you want to look at]'"`
 - Example: pulls out the color image from the bag file

    run `rosbag filter [bag file to filter] [output_file.bag] "topic == '/device_0/sensor_1/Color_0/image/data'"`

 - Have to be in same directory as the bag file to execute

To check what topics are in the .bag file, type:

  `rosbag info [bag file you want to look at]`


## Play a bag file
run `roslaunch vision ball_detection.launch`
run `rosbag play [name of bag file]` 
 - Whatever "output_file.bag" is from `rosbag filter` is the name that goes in for "name of bag file"

## Running RealSense ROS Node
download at https://github.com/intel-ros/realsense/#installation-instructions

Run the .launch file for the camera:

 `roslaunch realsense2_camera rs_rgbd.launch`
 - The camera has to be plugged into a USB 3.0 port for it to work.
 - If having problems running the rs_rgbd.launch file, open the file and turn the "enable_fisheye", "enable_infra1", "enable_infra2", "enable_gyro", and "enable_accel" to "false". It helps saves on processing
 - Don't drop frames to less than 30 fps, it wouldn't work.

If it can't detect the camera, this link provides method to overcome it: `https://github.com/IntelRealSense/librealsense/issues/2363`

In a sepearate terminal run the live_ball_detector.py code. It should then show the video from the camera

   `$ python live_ball_detector.py`

 - Have to be in the same directory as live_ball_detector to make the above command to work.
