# !/bin/bash

# Processes called inside CM GUI will inherit environment variables!
# - Ensure ros workspace is already built!
source ros_setup.bash

CM-9.0.2 . -apphost localhost -ext GUI/CMExt-CMRosIF.mod
