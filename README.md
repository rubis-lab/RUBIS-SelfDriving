# RUBIS-SelfDriving

## Environment

- Ubuntu 18.04
- ROS Melodic

## How to build Autoware

* ROS Melodic Install
```
sudo apt update
sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin
sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
pip3 install -U setuptools
```

* Autoware Build
```
# Move the rubis_ws to safe location
cd {$WORKSPACE_DIR}
mv rubis_ws ..

rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# If you have CUDA
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# If you don't have CUDA
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Restore the rubis_ws location
mv ../rubis_ws .

```

Since Autoware recommend to use directory name 'autoware.ai', you should make soft link with autoware.ai to this repository
```
ln -s ${WORKSPACE_DIR}/RUBIS-SelfDriving ~/autoware.ai
```

And it is recommned to add below sourcing command in your `~/.bashrc` file.
```
source ~/autoware.ai/install/setup.bash
source ~/autoware.ai/rubis_ws/devel/setup.bash
```

## How to build package in rubis_ws

* Initialize ROS workspace
```
cd ${WORKSPACE_DIR}/rubis_ws/src
catkin_init_workspace
```

* Build rubis_ws packages
```
cd rubis_ws
catkin_make
```

## How to launch LGSVL scrips
* Setup environments
```
cd ${WORKSPACE_DIR}/autoware_files/lgsvl/scripts
pip3 install --user .
```

* Launch LGSVL scripts
```
sudo chomod 755 {TARGET_SCRIPTS}
./{TARGET_SCRIPTS}
```
