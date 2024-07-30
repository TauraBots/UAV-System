# Taura Air System

## Descrição
Este projeto é um sistema avançado para gerenciamento de drones, desenvolvido utilizando o ROS 2 e MAVROS. O sistema permite controle detalhado de drones via comandos MAVLink, fornecendo funcionalidades de telemetria, planejamento de voo e muito mais.

## Pré-Requisitos
Antes de instalar e usar este projeto, você deve ter o ROS 2 instalado em seu sistema. Este projeto foi desenvolvido e testado usando ROS 2 Humble Hawksbill. 

### Dependências
- ROS 2 Humble Hawksbill
- MAVROS

## Instalação

### 1. Clone o Repositório
Primeiro, clone este repositório em seu workspace ROS 2:
```bash
git clone https://github.com/TauraBots/taura-air-system.git
```
```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
```

## Uso 

```bash
ros2 launch base_controller start_controller.launch.py
```