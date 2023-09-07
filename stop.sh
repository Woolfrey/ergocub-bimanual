#!/bin/bash

killall -9 gzserver

tmux kill-session -t 'client'
tmux kill-session -t 'server'



