
#!/bin/bash
sudo usermod -a -G dialout $USER
echo "Starting IR Detection!"
# shellcheck disable=SC2164
export DISPLAY=:0
gnome-terminal -- bash -c "sleep 1;python irRecog.py;exec bash"

cd $HOME/dji_sdk
source devel/setup.bash
echo "Activation!"
rosservice call /dji_sdk/activation {}
echo "Setting LocalPosition!"
#rosservice call /dji_sdk/set_local_pos_ref {}

sleep 5
cd $HOME/followtag
source devel/setup.bash
rosrun startn test.py
$SHELL



$SHELL
