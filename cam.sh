
#!/bin/bash
#sudo usermod -a -G dialout $USER
echo "Starting Camera!"
# shellcheck disable=SC2164
roslaunch usb_cam usb_cam-test.launch
$SHELL
