#!/bin/bash

SESSION="jetauto"

echo "Encerrando todos os processos do ROS 2 e Gazebo..."

# 1. Mata a sessão do tmux
tmux kill-session -t $SESSION 2>/dev/null

# 2. Mata processos específicos do ROS e do simulador
pkill -9 -f rviz2
pkill -9 -f gazebo
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f robot_state_publisher
pkill -9 -f async_slam_toolbox_node
pkill -9 -f seguidor_inteligente

# 3. Limpa o servidor do ROS (opcional, mas ajuda se houver travamentos)
ros2 daemon stop

#mata o gazebo
killall -9 gzserver gzclient

