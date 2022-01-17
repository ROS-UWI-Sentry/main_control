# main_control
This directory contains files that are examples and templates for using ROS nodes as state machines.
It also holds the files used for main control of the sentry robot.
Also inculed is the counter ROS node script.
There are flow charts and diagrams that aid with the explination of each script that is used in the final robot. No diagrams for testing scripts.

Since this is the main control for the entire robot, I will document things that aren't included in this repository.
There are three parts to this system. The ROS software and human detection software that runs on the Jetson AGX, the browser remote software that is hosted by the AGX. 

A naming convention will be used to differentiate them. 
Packages made for ROS will have a descriptive name.
The yolov5 software will be named yolov5.
The remote files (html+javascript) will be called remote_server. 

See Google Drive Additional Resources [Flashing the Jetson...](https://docs.google.com/document/d/1WZLdgXxbXff8g58E_jaLMqHgyO9Tv8HMU45z1B0EHVc/edit) for more details on yolov5, ROSBridge and issue.



### Required packages inside /home/user_name/catkin_ws/src folder:
- main_control
- human_detection
- light_control
- remote_communication

### Required packages inside /home/user_name/ folder:
- yolov5
- remote_server

### Required ROS packages:
- ROSBridge
- roslibpy (for yolov5 python script to communicate with ROS)
- roslibjs (for remote's javascript to communicate with ROS)

### Required Python packages:
[Jetson.GPIO](https://github.com/NVIDIA/jetson-gpio#installation)
and Others listed for the yolov5 installation

### Other info:
[Tutorial for communicating with ROS from a web UI](https://medium.com/husarion-blog/bootstrap-4-ros-creating-a-web-ui-for-your-robot-9a77a8e373f9)

### Script names of Nodes that need to be running for operation. These should be started with the launch file so no need to launch them individually.

- **main_control/sanitization_navigation_fsm.py**
- **main_control/counter.py**

- **remote_communication/remote_reports.py**
- **remote_communication/heartbeat_jetson_remote.py**

- **light_control/light_control_listener.py**

### Script names of launch files that will be called while running.

- **human_detection/launch/launch_detector.launch**
- **human_detection/launch/end_detector.launch**

- **main_control/launch/launch_webcam_counter.launch**

### Other nodes that need to run (will be called my main_control)

**ROSBridge:**
To allow the remote to communicate using ROS protocols.
There was a bug where after refreshing the page the publisher would not work again, this is a common complaint for ROSBridge and a quick solution was setting a large timeout time in the launch file. 
[Issue and quick solution:...](https://github.com/RobotWebTools/rosbridge_suite/issues/298)

**YOLOv5:**
There are instructions for YOLOV5 install on the Google Drive. Because the Jetson AGX is an embedded system the installation is not as straightforward.
The detect.py file must be the same as the Sentry repository.
Can clone that repo then follow the install instructions to have the same version.

### NOTE:
There are several scripts where files are read and written to. In some cases this file is found by using a path. This path has the name of the user account of the Ubuntu system so it must be changed so the script can find the file. To find all files that need this go to the ROS-UWI-Sentry github organisation and search for /home/uwi/catkin to find the files. Files inside of testing and unused folders can be ignored.
For example: the saving and loading of reports a path to the text file is needed.
This path changes depending on the user name. Look under remote_reports.py.
A future upgrade could be to save this file in a local that doesn't depend on user name. 


### A rough outline of the operation of the system:

1. sanitization_navigation_fsm.py (fsm) accepts instructions from the remote's sanitization page. These instructions include starting, pausing and stopping the sanitization process. Based on the user input, from the remote, and the results of the yolov5 human detetor, messages for light control are published.
2. counter.py keeps a track of how long the lights are on. fsm tells it when to start, pause and stop. It indicates to fsm when it has reached its tally.
3.  remote_reports accepts and sends data to the remote's reportspage. Data consists of when sanitization occured and completed.
4. heatbeat_jetson_remote.py consistently asks the remote for a response to check if there is a connection. if there is no response within 30 seconds this node publishes the shutdown signal.
5. light_control_listener listens for messages about light control from fsm. Based on the message it outputs a high or a low to a pin on the AGX. 
6. launch/end_detector are called by fsm to start and cleanly stop the yolov5 human detetor.
7. launch_webcam_counter is run by the state mahcine when starting up yolov5 to find out how many cameras are attatched so the sript won't crash if none are detected.
