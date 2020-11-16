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
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# If you have CUDA
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# If you don't have CUDA
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## How to build fake object generator (in test_package)

```
cd test_package
catkin_make
```
