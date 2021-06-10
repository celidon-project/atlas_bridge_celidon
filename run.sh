#!/bin/sh

SESSION_NAME="atlas_bridge"

SETUP_BASH_PATH='../../devel/setup.bash'

tmux has-session -t ${SESSION_NAME}

if [ $? != 0 ]
then
  # Create the session
  tmux new-session -d -s ${SESSION_NAME}
  tmux new-window -t ${SESSION_NAME}:0
  sleep 0.1

  tmux split-window -v -p 80  

  tmux select-pane -t 1
  sleep 0.1
  tmux send-keys 'source ' $SETUP_BASH_PATH 'C-m' 
  sleep 0.1   
  tmux send-keys 'rosrun atlas_celidon_bridge atlas_celidon_parameter_bridge.py' 
  sleep 0.1

  tmux select-pane -t 2
  sleep 0.1
  tmux send-keys 'source ' $SETUP_BASH_PATH 'C-m' 
  sleep 0.1   
  tmux send-keys 'rosrun atlas_celidon_bridge atlas_celidon_bridge.py'
  sleep 0.1

fi
tmux attach -t ${SESSION_NAME}
