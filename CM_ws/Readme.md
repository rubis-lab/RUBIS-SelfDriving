# CarMaker

## Dir Structure
```
CM_ws/
+-- Readme.md
+-- keyboard_example.py (keyboard module)
+-- build_cmrosif.sh (build script)
+-- CMStart.sh (Carmaker run script)
+-- Data/
|   +-- Road/ (maps)
|   +-- TestRun/ (testrun config files)
|   +-- ...
+-- ros/ros1_ws/src/
|   +-- hellocm_cmnode/
|   |   +--CMakeLists.txt
|   |   +--src/
|   |   |   +-- CMNode_ROS1_HelloCM.cpp (ROS1 CMnode)
|   +-- hellocm_msgs/
|   |   +--CMakeLists.txt
|   |   +--msg/
|   |   |   +--Ext2CM_Test.msg (ros to Carmaker msgs, ctrl_cmd)
|   |   |   +--cmd.msg ( ctrl_cmd/cmd
|   |   |   +-- ...
+-- ...
```

### shell setup(Optional)
source ~/autoware.ai/rubis_ws/devel/setup.bash
source ~/autoware.ai/CM_ws/ros/ros1_ws/devel/setup.bash
source ~/autoware.ai/rubis_ws/devel/setup.bash
source ~/autoware.ai/install/setup.bash
alias gb='gedit ~/.bashrc'
alias sb='source ~/.bashrc'
alias scm='source ~/autoware.ai/CM_ws/ros/ros1_ws/devel/setup.bash'
alias sr='source ~/autoware.ai/rubis_ws/devel/setup.bash'
alias bcm='cd ~/autoware.ai/CM_ws && ./build_cmrosif.sh'
alias key='python3 ~/autoware.ai/CM_ws/keyboard_example.py'
alias cm='cd ~/autoware.ai/CM_ws && ./CMStart.sh'

```sh
$ echo "source ~/autoware.ai/rubis_ws/devel/setup.bash
source ~/autoware.ai/CM_ws/ros/ros1_ws/devel/setup.bash
source ~/autoware.ai/rubis_ws/devel/setup.bash
source ~/autoware.ai/install/setup.bash
alias gb='gedit ~/.bashrc'
alias sb='source ~/.bashrc'
alias scm='source ~/autoware.ai/CM_ws/ros/ros1_ws/devel/setup.bash'
alias sr='source ~/autoware.ai/rubis_ws/devel/setup.bash'
alias bcm='cd ~/autoware.ai/CM_ws && ./build_cmrosif.sh'
alias key='python3 ~/autoware.ai/CM_ws/keyboard_example.py'
alias cm='cd ~/autoware.ai/CM_ws && ./CMStart.sh'" >> ~/.bashrc
```

## Build(cmros)
```sh
~/autoware.ai/CM_ws $ bcm
~/autoware.ai/CM_ws $ ./build_cmrosif.sh
```


## Run
```sh
~/autoware.ai/CM_ws $ cm
~/autoware.ai/CM_ws $ ./CMStart.sh
```

## QuickStart

1. Extras > CMRisIF > Launch & Start Application
(Sometimes Error occurs, Try again)

2. File > Open > {testrun file} > OK
3. File > IPGMovie
4. Click Start


## Keyboard control
```sh
~/autoware.ai/CM_ws $ key
~/autoware.ai/CM_ws $ python3 keyboard_example.py
```
To exit Keyboard control
Ctrl+C & backspace

if you do not have some python package, you may need to install it
```sh
$ pip3 install pynput
$ pip3 install rospkg
```

## Turn off route restrictions
/autoware.ai/CM_ws/Data/TestRun
open testrun file with text editor

>...
>Road.VhclRoute = NotSpecified      (modify)
>Road.RouteId =                     (modify)
>Road.LinkObjId = 46308             (insert)    //start position
>...

## Trouble Shootings
run ./build_cmrosif.sh
Building ros/ros1_ws
    - scm (source ~/autoware.ai/CM_ws/ros/ros1_ws/devel/setup.bash)
Building src
    - add object files to /autoware.ai/CM_ws/lib
    - linux64, win64

## run Carmaker from scratch
    - pubdocs.md

## Change from original CM_Project
ros/ros1_ws/build.sh

>cmd="source /opt/ros/ros1/setup.bash"; echo $cmd; $cmd
>cmd="source /opt/ros/melodic/setup.bash"; echo $cmd; $cmd

ros/ros1_ws/src/hellocm_cmnode/CMakeLists.txt
>set(IPGHOME /opt/ipg)