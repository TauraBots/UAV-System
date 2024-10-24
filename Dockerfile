# Usando a imagem base especificada
FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

# Atualiza e instala pacotes necessários
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    python-pip \
    python-vcstool \
    python-rosinstall-generator \
    python-osrf-pycommon \
    python-ament-package \
    geographiclib-tools \
    libasio-dev \
    && rm -rf /var/lib/apt/lists/*

# Copia o script de instalação do GeographicLib para o container
COPY install_geographiclib_datasets.sh /root/install_geographiclib_datasets.sh

# Torna o script executável
RUN chmod +x /root/install_geographiclib_datasets.sh

# Executa o script
RUN /root/install_geographiclib_datasets.sh

# Configura o ambiente do ROS 2
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Define o diretório de trabalho para onde o Dockerfile está
WORKDIR /root/UAV-System

# Gerar o arquivo .repos para MAVLink
RUN rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos

# Gerar o arquivo .repos para MAVROS
RUN rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos

# Importa os repositórios para a workspace
RUN vcs import src < /tmp/mavlink.repos
RUN vcs import src < /tmp/mavros.repos

# Instala as dependências
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Compila o projeto ROS 2
RUN colcon build

# Seta o entrypoint para inicializar o ambiente do ROS
ENTRYPOINT ["bash", "-c", "source /opt/ros/humble/setup.bash && bash"]
