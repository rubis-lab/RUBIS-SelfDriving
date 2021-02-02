#
# Message from CarMaker ROS Node to external ROS Node
#

# General
Header  header                                 # General ROS Header (optional)
time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)
float64 synthdelay                             # Synthetic delay to check synchronization (optional)

float64 id                                     # Signal ID
float64 state                                  # Signal State
float64 remain_time                            # Remain Time of Signal
float64 remain_distance                        # Distance between Traffic Signal and Vehicle Position 