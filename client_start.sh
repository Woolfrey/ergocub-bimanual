#!/bin/bash

SESSION="client"                                                                         # So we can reference $SESSION later
SERVERNAME="/commandServer"                                                              # Name of the yarp server for controlling the robot

# Options
CONFIG="~/workspace/icub-bimanual/config/ergocub.ini"
#CONFIG="~/workspace/icub-bimanual/config/icub2.ini"                                                             

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
tmux send-keys -t $SESSION "~/workspace/icub-bimanual/build/bin/command_prompt $SERVERNAME $CONFIG" Enter

# Switch back to Pane 0 and run yarp rpc
tmux select-pane -t 0
tmux send-keys -t $SESSION "sleep 1" Enter                                               # Wait a few seconds before continuing
tmux send-keys -t $SESSION "yarp rpc /commandPrompt" Enter

tmux attach-session -t $SESSION:0                                                        # REQUIRED or the above won't execute

