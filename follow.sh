
#!/bin/bash

echo "Starting Detection!"

echo "Starting Reach Target!"

# shellcheck disable=SC2164
cd $HOME/followtag
source devel/setup.bash
rosrun startn follow.py
$SHELL
