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

* Eigen build
```
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz
mkdir eigen
tar --strip-components=1 -xzvf eigen-3.3.7.tar.gz -C eigen
cd eigen
mkdir build
cd build
cmake ..
make
sudo make install
```

Older versions may already be installed. If `/usr/lib/cmake/eigen3/Eigen3Config.cmake` is older than 3.3.7 version, copy files in `/usr/local/share/eigen3/cmake` to `/usr/lib/cmake/eigen3`.


* Autoware Build
```
# Move the rubis_ws and autoware_files to safe location
cd {$WORKSPACE_DIR}
mv rubis_ws autoware_files ..

rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# If you have CUDA
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# If you don't have CUDA
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Restore the rubis_ws and autoware_files location
mv ../rubis_ws ../autoware_files .

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

## Configurable Parameter

### Dynamic Parameter (Can configure on runtime)

* `/op_trajectory_evaluator/weightPriority` : weight for priority cost
* `/op_trajectory_evaluator/weightTransition` : weight for transition cost
* `/op_trajectory_evaluator/weightLong` : weight for longitudinal cost
* `/op_trajectory_evaluator/weightLat` : weight for lateral cost
* `/op_trajectory_evaluator/LateralSkipDistance` : skip an object whose horizontal distance is this much

### Static Parameter (Should configure in launch file)

* `/op_common_params/stopLineMargin` : Stop ahead of this value at the stop line
* `/op_common_params/stopLineDetectionDistance` : detect stop lines whose distance are closer than thie value.


