#!/bin/bash

SESSION="server"                                                                         # So we can reference $SESSION later
SERVERNAME="/commandServer"                                                              # Name of the yarp server for controlling the robot

# Options
CONFIG="~/workspace/icub-bimanual/config/ergocub.ini"
PORT="/ergocubSim"
URDF="~/workspace/robotology-superbuild/src/ergocub-software/urdf/ergoCub/robots/ergoCubGazeboV1/model.urdf"
WORLD="~/workspace/icub-bimanual/gazebo/worlds/ergocub_grasp_demo.sdf"

#CONFIG="~/workspace/icub-bimanual/config/icub2.ini"                                                             
#PORT="/icubSim"                                                                          # Port name
#URDF="~/workspace/robotology-superbuild/src/icub-models/iCub/robots/iCubGazeboV2_7/model.urdf"
#WORLD="~/workspace/icub-bimanual/gazebo/worlds/icub2_grasp_demo.sdf"                     # Location of the Gazebo world


# Create first window & panel
tmux new-session   -d -s $SESSION                                                        # Start new session with given name
tmux rename-window -t  0 'Server'                                                        # Give a name to this window

# Divide up the screen (default pane 0)
#######################################
#                  #                  #
#                  #         1        #
#                  #                  #
#        0         ####################
#                  #                  #
#                  #         2        #
#                  #                  #
#######################################

# Split Pane 1 to the right, run YARP
tmux split-window -h
tmux send-keys    -t $SESSION "yarpserver --write" Enter

# Split Pane 2, launch Gazebo

if pidof -x "gzserver" >/dev/null; then
    killall -9 gzserver
fi

tmux split-window -v
tmux send-keys    -t $SESSION "gazebo $WORLD" Enter

# Select Pane 0, launch the yarp server
tmux select-pane -t 0
tmux send-keys   -t $SESSION "sleep 4" Enter                                                        # Wait for Gazebo to launch
tmux send-keys   -t $SESSION "~/workspace/icub-bimanual/build/bin/command_server $SERVERNAME $PORT $URDF $CONFIG" Enter

tmux attach-session -t $SESSION:0                                                        # REQUIRED or the above won't execute

