# main_control
This directory contains files that are examples and templates for using ROS nodes as state machines.
It also holds the files used for main control of the sentry robot.
Also inculed is the counter ROS node script.
There are flow charts and diagrams that aid with the explination of each script that is used in the final robot. No diagrams for testing scripts.

Since this is the main control, I will document things that aren't included in this repository.
There are three parts to this system. The ROS software and human detection software that runs on the Jetson AGX, the browser remote software that is hosted by the AGX. 

A naming convention will be used to differentiate them. 
Packages made for ROS will have a descriptive name.
The yolov5 software will be named yolov5.
The remote files (html+javascript) will be called remote_server. 

### Script names of Nodes that need to be running for operation. These should be started with the launch file.

**main_control/sanitization_navigation_fsm.py**
**main_control/counter.py**

**remote_communication/remote_reports.py**
**remote_communication/heartbeat_jetson_remote.py**

**light_control/light_control_listener.py**

### Script names of launch files that will be called while running.

**human_detection/launch_detector.launch**
**human_detection/end_detector.launch**


There are instructions for YOLOV5 install on the Google Drive. Because the Jetson AGX is an embedded system the installation is not as straightforward.
The detect.py file must be the same as the Sentry repository.
Can clone that repo then follow the install instructions to have the same version.

There are several scripts where files are read and written to. In some cases this file is found by using a path. This path has the name of the user account of the Ubuntu system so it must be changed so the script can find the file. To find all files that need this go to the ROS-UWI-Sentry github organisation and search for /home/uwi/catkin to find the files. Files inside of testing and unused folders can be ignored.
For example: the saving and loading of reports a path to the text file is needed.
This path changes depending on the user name. Look under remote_reports.py.
A future upgrade could be to save this file in a local that doesn't depend on user name. 


### A rough outline of the operation of the system:

1. sanitization_navigation_fsm.py (fsm) accepts instructions from the remote's sanitization page. These instructions include starting, pausing and stopping the sanitization process.
2. counter.py keeps a track of how long the lights are on. fsm tells it when to start, pause and stop. It indicates to fsm when it has reached its tally.
3.  remote_reports accepts and sends data to the remote's reportspage. Data of when sanitization occured and completed.
4. heatbeat_jetson_remote.py consistently asks the remote for a response to check if there is a connection. if there is no response within 30 seconds this node publishes the shutdown signal.
5. light_control_listener listens for messages from fsm. Based on the message it outputs a high or a low to a pin on the agx. 
6. launch/end_detector are called by fsm to start and cleanly stop the yolov5 human detetor.
