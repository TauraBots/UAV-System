# Simulação e Controle de Drone com PX4, MAVROS, e ROS 2

Este projeto utiliza PX4, MAVROS, e ROS 2 para simular e controlar um drone em um ambiente virtual com a possibilidade de navegação manual via teclado.

## Pré-requisitos

Certifique-se de ter instalados os seguintes componentes antes de iniciar:

- PX4 Autopilot
- MAVROS
- [**ROS 2**](https://docs.ros.org/en/humble/Installation.html) (necessário para executar os comandos)
- [Gazebo](https://docs.px4.io/main/en/sim_gazebo_gz/) (Simulador)
- [Micro XRCE-DDS](https://micro-xrce-dds.docs.eprosima.com/en/latest/)
- [QGroundControl](https://qgroundcontrol.com/) (para monitoramento e controle do drone)

## Passo a Passo


## Dependencias
```bash
sudo apt install -y python3-vcstool python3-rosinstall-generator python3-osrf-pycommon python3-ament-package
```
Dentro da workspace, execute:

```bash
rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos
```

```bash
rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos
```
```bash
vcs import src < /tmp/mavlink.repos
```
```bash
vcs import src < /tmp/mavros.repos
```

### 1. Executar o **Micro XRCE-DDS Agent  (Para o Simulador)**

```bash
MicroXRCEAgent udp4 -p 8888
```
Este comando inicia o agente Micro XRCE-DDS na porta 8888 utilizando o protocolo UDP4. O agente é necessário para a comunicação entre o cliente XRCE-DDS (executado no drone) e os outros sistemas como MAVROS e PX4. Ele estabelece uma ponte de comunicação eficiente para troca de dados entre o ROS 2 e os sistemas embarcados.


### 2. Compilar e iniciar a simulação PX4 no Gazebo

```bash
make px4_sitl gz_x500
```
Esse comando compila e inicia a simulação do PX4 (firmware de piloto automático) no simulador Gazebo, utilizando o modelo de drone x500. A simulação SITL ("Software In The Loop") permite testar o software do drone sem a necessidade de hardware físico. O modelo gz_x500 é usado para representar o drone no ambiente simulado.

### 3. Iniciar o MAVROS com ROS 2

```bash
ros2 launch mavros px4.launch
```
Este comando lança o MAVROS, que atua como uma ponte entre o PX4 e o ROS 2. O arquivo px4.launch configura todos os nós e parâmetros necessários para MAVROS se comunicar com o PX4, permitindo a troca de comandos de controle e informações de telemetria do drone.


### 4. Iniciar o Nó de Interface

```bash
ros2 run interface_package interface_node
```
Este comando executa o nó interface_node, que faz parte do pacote interface_package. Ele serve como interface entre o sistema de controle ROS 2 e outros pacotes ou sistemas, gerenciando a interação com o drone ou os módulos de controle de navegação.

### 5. Controlar o Drone Usando o Teclado (Opcional)

```bash
ros2 run teleop_package teleop_node
```
O teleop_node permite o controle manual do drone via teclado. Ele captura as entradas do teclado e converte-as em comandos de controle que são enviados ao drone por meio do MAVROS. Esse co

## Navegação Manual

- Para navegar manualmente pelo drone na simulação, use o comando do teleop_node mencionado acima.
- A simulação pode ser controlada via comandos de teclado, permitindo controlar a movimentação do drone durante os testes.


# Conclusão

Seguindo esses passos, você terá o sistema de simulação do drone em pleno funcionamento, pronto para ser controlado manualmente ou automaticamente por meio de comandos do ROS 2.


Este `README.md` contém uma explicação clara e bem formatada dos comandos e seus propósitos para a simulação e controle do drone.
