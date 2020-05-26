 
#!/bin/bash

echo "Setting Up!"

cd $HOME/dji_sdk
source devel/setup.bash
echo "Activation!"
rosservice call /dji_sdk/activation {}
echo "Setting LocalPosition!"
rosservice call /dji_sdk/set_local_pos_ref {}
rosservice call /dji_sdk/drone_task_control "task: 4"
echo "Giving control to SDK (Autonomous Control)!"
rosservice call /dji_sdk/sdk_control_authority "control_enable: 1" 


$SHELL
