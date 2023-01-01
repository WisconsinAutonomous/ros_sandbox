# ros_sandbox
This repository can be used to generate custom test data in the form of "ROS bags".
ROS bags record a live replay of ROS topic data that is transmitted by a ROS stack, and
accumulates the data passed on any number of topics and saves it in a database. 
You can then replay the data to reproduce the data recorded by the ROS bag. More documentation can be found 
[here](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html).

This is extremely useful for testing specific nodes in our ROS stack -- we can create a ROS bag of the direct inputs to the node we want to test
which creates a very controlled testing environment. <b>Add your own nodes to produce artificial test data!</b>

## `image_roi_publisher` example
To test the `traffic_light_state_determination` node, artificial test data (in the form of a ROS bag)
was created that consisted of an image topic and an array of bounding boxes (a.k.a Region Of Interest) topic --
the inputs to the `traffic_light_state_determination` node. This was done using the `image_roi_publisher` node.
This node is largely the same as the basic tutorial publisher node [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

### Node walkthrough
The image topic and bounding boxes topic each get their data from a files in the `data/traffic_lights` directory of the `image_roi_publisher` node.
The image is just a `.jpg` file, and the bounding boxes are stored in YOLOv5 label format in a `.txt` file.
Three ROS parameters are used to tell `image_roi_publisher` where the files are.

https://github.com/WisconsinAutonomous/ros_sandbox/blob/12051e5bf902230643709188d40874b85229e2ce/workspace/src/image_roi_publisher/image_roi_publisher/image_roi_publisher_node.py#L25-L27

`package_path` is the absolute path to the package directory.<br>
`image_path` is the path to the image file within the package directory.<br>
`roi_path` is the path to the label file within the package directory.

Inside the `timer_callback` (which goes off every 0.5 seconds), the node loads in the data from the image and label file if it hasn't already.

For the image:
https://github.com/WisconsinAutonomous/ros_sandbox/blob/12051e5bf902230643709188d40874b85229e2ce/workspace/src/image_roi_publisher/image_roi_publisher/image_roi_publisher_node.py#L32-L36

For the bounding boxes:
https://github.com/WisconsinAutonomous/ros_sandbox/blob/12051e5bf902230643709188d40874b85229e2ce/workspace/src/image_roi_publisher/image_roi_publisher/image_roi_publisher_node.py#L43-L48

The image and bounding boxes now have to be packaged into a ROS message before they can be published to a ROS topic. For the image topic, we use the `sensor_msgs` `Image`, as this is what is used in the `WAutoDrive` ROS stack. For the bounding boxes topic, we use a custom ROS message called `ROIArray`. we copied over the `wauto_perception_msgs` from `WAutoDrive`, which has all of the custom ROS messages used by the perception stack.

For the image message:
https://github.com/WisconsinAutonomous/ros_sandbox/blob/12051e5bf902230643709188d40874b85229e2ce/workspace/src/image_roi_publisher/image_roi_publisher/image_roi_publisher_node.py#L39-L41

For the bounding boxes message:
https://github.com/WisconsinAutonomous/ros_sandbox/blob/12051e5bf902230643709188d40874b85229e2ce/workspace/src/image_roi_publisher/image_roi_publisher/image_roi_publisher_node.py#L50-L62

Finally, we can publish these messages.
https://github.com/WisconsinAutonomous/ros_sandbox/blob/12051e5bf902230643709188d40874b85229e2ce/workspace/src/image_roi_publisher/image_roi_publisher/image_roi_publisher_node.py#L69-L70

### Launch file
A simple launch file was used to run the node executable. It can be found in `workspace/launch/`, and ran with the command (from the `workspace` directory)
```
ros2 launch launch/image_roi_publisher_launch.py
```
The most notable aspect of the launch file is the parameters section, where we tell the node where to look for the data files that we are going to publish.
https://github.com/WisconsinAutonomous/ros_sandbox/blob/12051e5bf902230643709188d40874b85229e2ce/workspace/launch/image_roi_publisher_launch.py#L14-L18

Everything else is pretty standard, inspired the basic launch file tutorial [here](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html).

### Recording and using ROS bags
Once the node has been launched (using the launch file), it will start publishing the information to ROS topics. We can record this data using ROS bags. In a separate terminal window inside the Docker container (i.e. use tmux to create a new tab and then use `atk dev -a` to attach to the running container), run the following command:
```
ros2 topic list
```
You should see a list of all the topics that are being published. These will include `image_roi_publisher/image_roi_publisher/output/image` and `image_roi_publisher/image_roi_publisher/output/rois`, which are the two topics that we are publishing from the `image_roi_publisher` node. To record a ROS bag of these topics, use the following command:
```
ros2 bag record --all
```
This will record all topics until you Ctrl+C to stop it. It will save the data, along with a .yml, in a folder that corresponds to the current date and time. Congrats, you just recorded a ROS bag!

Now, we can transfer this folder over to the `WAutoDrive` ROS stack and play the ROS bag to replay the data that was captured:
```
ros2 bag play <folder_name> -l
```
The `-l` option tells it to loop indefinitely. When we do this, we should be able to list the ROS topics and see the topics that we captured in `ros_sandbox`. This data can then be used as input to nodes we want to test by mapping the topics to the input topics of the node in the launch file. For example, in `traffic_light_state_determination`:
https://github.com/WisconsinAutonomous/WAutoDrive/blob/f7e803d08a13c75095ae7cb9cf59cfc619950276/workspace/src/common/launch/wauto_perception_launch/launch/traffic_light_state_determination.launch.py#L28-L30

