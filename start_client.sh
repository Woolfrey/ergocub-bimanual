#!/bin/bash

SESSION="client"                                                                         # So we can reference $SESSION later
SERVERNAME="/controlServer"                                                              # Name of the yarp server for controlling the robot
CURRENT_DIR=$(pwd)

# Options
CONFIG="$CURRENT_DIR/config/ergocub.ini"

# Window layout:

#######################################
#                 #                   #
#                 #                   #
#                 #                   #
#        0        #         1         #
#                 #                   #
#                 #                   #
#                 #                   #
#######################################

# Create first window & panel
tmux new-session   -d -s $SESSION
tmux rename-window -t  0 'Client'

# Split Pane 1 to the right and launch the client
tmux split-window -h
tmux send-keys -t $SESSION "$CURRENT_DIR/build/command_prompt $SERVERNAME $CONFIG" Enter

# Switch back to Pane 0 and run yarp rpc
tmux select-pane -t 0
tmux send-keys -t $SESSION "sleep 1" Enter                                               # Wait a few seconds before continuing
tmux send-keys -t $SESSION "yarp rpc /commandPrompt" Enter

tmux attach-session -t $SESSION:0                                                        # REQUIRED or the above won't execute

