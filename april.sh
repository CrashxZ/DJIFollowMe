
#!/bin/bash
sudo usermod -a -G dialout $USER
echo "Starting Camera!"
# shellcheck disable=SC2164
export DISPLAY=:0
gnome-terminal -- bash -c "sleep 1;roslaunch usb_cam usb_cam-test.launch;exec bash"
echo "Starting Detection!"
gnome-terminal -- bash -c "sleep 5;./tag.sh;exec bash"

cd $HOME/dji_sdk
source devel/setup.bash
echo "Activation!"
rosservice call /dji_sdk/activation {}
echo "Setting LocalPosition!"
rosservice call /dji_sdk/set_local_pos_ref {}
rosservice call /dji_sdk/drone_task_control "task: 4"
echo "Giving control to SDK (Autonomous Control)!"
rosservice call /dji_sdk/sdk_control_authority "control_enable: 1" 
sleep 8
cd $HOME
./follow.sh



$SHELL
