#!/bin/bash
set -e

# Ativar o ambiente ROS 2
source /opt/ros/humble/setup.bash

# Adicionar o caminho do CUDA
export PATH=/usr/local/cuda-10.2/bin${PATH:+:${PATH}}

# Executar o comando passado para o contêiner
exec "$@"
