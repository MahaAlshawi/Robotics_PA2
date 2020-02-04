# State Estimation

This project is about estimating the robot's state using kalman filter. Two kalman filters have been implemented using a single node to estimate the distance that has been traveled by the robot. The first filter depends on /cmd_vel topic to calculate the robot's state, While the second filter depends on subscribing to the /pose topic to compute the robot’s state. matplotlib library has also been used in this node to plot the estimated states and the error in these estimations in different scenarios.

## Getting Started

You will need the following to get you a copy of the project up and running on your local machine for development and testing purposes:

* Add the state_estimation package to src folder in your catkin workspace.
* Then Catkin_make while you are in catkin_ws folder in terminal. 
* Source devel/setup.bash while you are in catkin_ws.

### Installing

You only need to install python and ROS on ubuntu.

### Running the tests

you will need to do the following to test the project.

* Run: roscore.
* play the bag file attached with the project files as follows: rosbag play FILE_NAME.
* In another terminal, run: rosrun state_estimation state_estimation.py.
* In a new terminal, run: rosrun image_transport republish compressed in:=/camera/depth/image_raw/compressed raw out:=/camera/depth/image_raw/ while playing the bag file to decompress the /camera/depth/image_raw/compressed topic.
* In case you need to get the LaserScan readings from the camera depth instead of the LIDAR, you have to run: rosrun depthimage_to_laserscan depthimage_to_laserscan
* Finally, run: rqt_tf_tree rqt_tf_tree to get the transformation between odom_kf frame and base_footprint frame.

### Authors

* **Maha Hasan** - *PhD fellow at Dartmouth* -

