# Simulação e Controle de Drone com PX4, MAVROS, e ROS 2

Este projeto utiliza PX4, MAVROS, e ROS 2 para simular e controlar um drone em um ambiente virtual com a possibilidade de navegação manual via teclado.

## Pré-requisitos

Certifique-se de ter instalados os seguintes componentes antes de iniciar:

- PX4 Autopilot
- MAVROS
- [**ROS 2**](https://docs.ros.org/en/humble/Installation.html) 
- [Micro XRCE-DDS](https://micro-xrce-dds.docs.eprosima.com/en/latest/)
- [QGroundControl](https://qgroundcontrol.com/) 


#### Dependencias
* Vamos criar a workspace de trabalho do nosso projeto!
  ```bash
  mkdir uav_project && cd uav_project
  ```
* Vamos instalar o ambiente de simulação
  Baixe o código fonte do PX4 :
  ```bash
  git clone git@github.com:TauraBots/PX4-Autopilot.git --recursive
  ```
  Execute o ubuntu.sh sem argumentos (em um shell bash) para instalar tudo:
  ```bash
  bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
  ```
  - Aceite quaisquer prompts conforme o script avança.
  - Você pode usar as opções --no-nuttxe --no-sim-toolspara omitir o NuttX e/ou as ferramentas de simulação.
  
* Reinicie o computador após a conclusão.
  - Podemos testar o simulador entrando da pasta
    ```bash
    cd PX4-Autopilot
    ```
    > [!NOTE]
    > É Necessario que o ROS2 e o gazebo estejam instalado no SO.
    
    E podemos fazer um teste simples com o seguinte comando:
    ```bash
    make px4_sitl gz_x500
    ```
    Para podemos continua, temos que retornar para a workspace de origem
    ```bash
    cd ..
    ```
* Vamos instalar o nosso sistema! Clone o este Repositorio da seguinte forma:
  ```bash
  git clone git@github.com:TauraBots/UAV-System.git
  ```
  
#### Next Steps







# Conclusão

Seguindo esses passos, você terá o sistema de simulação do drone em pleno funcionamento, pronto para ser controlado manualmente ou automaticamente por meio de comandos do ROS 2.


Este `README.md` contém uma explicação clara e bem formatada dos comandos e seus propósitos para a simulação e controle do drone.
