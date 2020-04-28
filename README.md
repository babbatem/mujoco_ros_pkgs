# mujoco_ros

Wrapper, tools for using ROS with the MuJoCo simulator.

Forked for our use. Changelog below.

Paths were hardcoded in the original code.
Now using following ENV variables. Modify as needed if your paths differ.   
```
export MUJOCO_VERSION="mjpro150"
export MUJOCO_PRO_PATH="$HOME/.mujoco/${MUJOCO_VERSION}/"
export MUJOCO_KEY_PATH="$HOME/.mujoco/mjkey.txt"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/${MUJOCO_VERSION}/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia-387
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
```
