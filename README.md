# ros_sandbox
This repository can be used to generate custom test data in the form of "ROS bags".
ROS bags record a live replay of ROS topic data that is transmitted by a ROS stack, and
accumulates the data passed on any number of topics and saves it in a database. 
You can then replay the data to reproduce the data recorded by the ROS bag. More documentation can be found 
[here](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html).

This is extremely useful for testing specific nodes in our ROS stack -- we can create a ROS bag of the direct inputs to the node we want to test
which creates a very controlled testing environment.

## `traffic_light_spawner` example
To test the `traffic_light_state_determination` node, artificial test data (in the form of a ROS bag)
was created that consisted of an image topic and an array of bounding boxes (a.k.a Region Of Interest) topic --
the inputs to the `traffic_light_state_determination` node. This was done using the `traffic_light_spawner` node.
This node is largely the same as the basic tutorial publisher node [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

The image topic and bounding boxes topic each get their data from a files in the `data` directory of the `traffic_light_spawner` node.
The image is just a `.jpg` file, and the bounding boxes are stored in YOLOv5 label format in a `.txt` file.
Three ROS parameters are used to tell `traffic_light_spawner` where the files are.

https://github.com/WisconsinAutonomous/ros_sandbox/blob/b653f9629aecd7c3d5d3cfac72e6be9dce4472ae/workspace/src/traffic_light_spawner/traffic_light_spawner/traffic_light_spawner_node.py#L25-L27

`package_path` is the absolute path to the package directory.<br>
`image_path` is the path to the image file within the package directory.<br>
`roi_path` is the path to the label file within the package directory.

Inside the `timer_callback` (which goes off every 0.5 seconds), the node loads in the data from the image and label file if it hasn't already. <br>
For the image:
https://github.com/WisconsinAutonomous/ros_sandbox/blob/b653f9629aecd7c3d5d3cfac72e6be9dce4472ae/workspace/src/traffic_light_spawner/traffic_light_spawner/traffic_light_spawner_node.py#L32-L36

For the bounding boxes:
https://github.com/WisconsinAutonomous/ros_sandbox/blob/b653f9629aecd7c3d5d3cfac72e6be9dce4472ae/workspace/src/traffic_light_spawner/traffic_light_spawner/traffic_light_spawner_node.py#L43-L48


