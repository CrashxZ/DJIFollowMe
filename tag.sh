
#!/bin/bash
#sudo usermod -a -G dialout $USER
# shellcheck disable=SC2164
cd $HOME/Apriltag_detection
source devel/setup.bash
roslaunch apriltag_ros continuous_detection.launch 
$SHELL
