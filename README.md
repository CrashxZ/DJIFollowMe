# DJIFollowMe
Repository for a bunch of automation scripts using a monocular camera, a DJI N3/A3 Flight Controller.
To use the scripts, set up the repository as a ROS Workspace
### 1. April tag Landing.
#### Start the camera : 
```
roslaunch usb_cam usb_cam-test.launch
```
#### Start the Apriltag Detection in continious mode : 
```
cd $HOME/Apriltag_detection
source devel/setup.bash
roslaunch apriltag_ros continuous_detection.launch 
```
#### Start the Assisted landing : 
```
cd $HOME/followtag
source devel/setup.bash
rosrun startn pland.py
```

### 2. Track and Follow the April tag.
#### Start the camera : 
```
roslaunch usb_cam usb_cam-test.launch
```
#### Start the Apriltag Detection in continious mode : 
```
cd $HOME/Apriltag_detection
source devel/setup.bash
roslaunch apriltag_ros continuous_detection.launch 
```
#### Start the Assisted landing : 
```
cd $HOME/followtag
source devel/setup.bash
rosrun startn follow.py
```

or
run :
```
./april.sh
```

### 3. Land using IR Guidance.
#### Start Detection : 
```
python irRecog.py
```
#### Start the Assisted landing : 
```
cd $HOME/followtag
source devel/setup.bash
rosrun startn infraredLanding.py
```
or
run :
```
./irLand.sh
```




