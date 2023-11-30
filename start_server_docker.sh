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
tmux rename-window -t  1 'Yarpserver'                                                        # Give a name to this window

# Split Pane 2, launch Gazebo

if pidof -x "gzserver" >/dev/null; then
    killall -9 gzserver
fi

# Launching gazebo with custom nav world
tmux split-window -v
tmux rename-window -t  2 'Gazebo'                                                        # Give a name to this window
tmux select-pane -t 2
tmux send-keys    -t $SESSION "export YARP_CLOCK=/clock && gazebo -s libgazebo_yarp_clock.so -s libgazebo_ros_init.so $WORLD " Enter 
              
#Launching Yarprobotinterface              
tmux split-window -v
tmux rename-window -t  3 'Ecubrobotinterface'                                                       
tmux select-pane -t 3
tmux send-keys -t $SESSION "export YARP_CLOCK=/clock && YARP_CLOCK=/clock yarprobotinterface --from ecub_yarprobotinterface.ini" Enter
tmux attach-session -t $SESSION:3                                                        

# Select Pane 0, launch the yarp server
tmux select-pane -t 0
tmux send-keys   -t $SESSION "sleep 12" Enter                                             # Wait for Gazebo to launch
tmux send-keys   -t $SESSION "$CURRENT_DIR/build/command_server $SERVERNAME $PORT $URDF $CONFIG" Enter

tmux attach-session -t $SESSION:0                                                        # REQUIRED or the above won't execute
