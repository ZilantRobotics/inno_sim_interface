#!/bin/bash

FIRMWARE_PATH="~/Work/vtol/Firmware"
INNOSIM_PATH="~/Work/InnoSim/Linux"
QGC_PATH="~/Work"

START_LAT="55.7544426"
START_LON="48.742684"
START_ALT="-6.5"


tmux start-server

sleep 1

tmux new -s innosim -d
tmux rename-window -t innosim innosim


tmux split-window -v -t innosim

tmux select-pane -t innosim:0.0
tmux split-window -h -t innosim
tmux split-window -h -t innosim

tmux select-pane -t innosim:0.3
tmux split-window -h -t innosim
tmux split-window -h -t innosim


tmux select-pane -t innosim:0.2
tmux send-keys "cd $FIRMWARE_PATH
export PX4_HOME_LAT=$START_LAT
export PX4_HOME_LON=$START_LON
export PX4_HOME_ALT=$START_ALT
make px4_sitl gazebo_standard_vtol" C-m

tmux select-pane -t innosim:0.3
tmux send-keys "roscd inno_sim_interface/cfg
$INNOSIM_PATH/InnoSimulator.x86_64 --config config.yaml" C-m

tmux select-pane -t innosim:0.4
tmux send-keys "$QGC_PATH/QGroundControl.AppImage" C-m

sleep 1

tmux select-pane -t innosim:0.0
tmux send-keys 'roslaunch inno_sim_interface innosim_relay.launch' C-m

sleep 1

tmux select-pane -t innosim:0.1
tmux send-keys "roslaunch rosbridge_server rosbridge_websocket.launch" C-m


tmux select-pane -t innosim:0.5

tmux attach -t innosim