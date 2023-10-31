#!/bin/bash
SESSION="server"                                                                         # So we can reference $SESSION later
SERVERNAME="/controlServer"                                                              # Name of the yarp server for controlling the robot
CURRENT_DIR=$(pwd)

# Options
CONFIG="$CURRENT_DIR/config/ergocub_gazebo_sim.ini"
PORT="/ergocubSim"
URDF="$CURRENT_DIR/../robotology-superbuild/build/install/share/ergoCub/robots/ergoCubGazeboV1/model.urdf"
WORLD="$CURRENT_DIR/gazebo/worlds/ergocub_nav_test.sdf"

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
tmux send-keys -t "export YARP_CLOCK=/clock && YARP_CLOCK=/clock yarprobotinterface --from ecub_yarprobotinterface.ini" Enter
tmux send-keys    -t $SESSION "gazebo $WORLD -s libgazebo_yarp_clock.so -s libgazebo_ros_init.so " Enter

# Select Pane 0, launch the yarp server
tmux select-pane -t 0
tmux send-keys   -t $SESSION "sleep 120" Enter                                             # Wait for Gazebo to launch
tmux send-keys   -t $SESSION "$CURRENT_DIR/build/command_server $SERVERNAME $PORT $URDF $CONFIG" Enter

tmux attach-session -t $SESSION:0                                                        # REQUIRED or the above won't execute
