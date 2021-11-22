# main_control
This directory contains files that are examples and templates for using ROS nodes as state machines.
It also holds the files used for main control of the sentry robot.
Also inculed is the heartbeat ROS node script.
There are flow charts and diagrams that aid with the explination of each script that is used in the final robot. No diagrams for testing scripts.

###### Nodes that need to be running for operation. These should be started with the launch file.

**main_control/sanitization_navigation_fsm.py**
**main_control/counter.py**

**browser_communication/browser_reports.py**
**browser_communication/heartbeat_jetson_remote.py**

**light_control/light_control_listener.py**


There are instructions for YOLOV5 install on the Google Drive. Because the Jetson AGX is an embedded system the installation is not as straightforward.
The detect.py file must be the same as the Sentry repository.
Can clone that repo then follow the install instructions to have the same version.

For the saving and loading of reports a path to the text file is needed.
This path changes depending on the user name. Look under browser_reports.py.
A future upgrade is to save this file in a local that doesn't depend on user name. 


#### A rough outline of the operation of the system:

1. sanitization_navigation_fsm.py accepts instructions from the remote. 
