# Usando a imagem oficial do ROS 2 Humble
FROM ros:humble

# Configura o fuso horário e a região
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Sao_Paulo

# Define o diretório de trabalho
WORKDIR /app

# Instala pacotes essenciais e cria links simbólicos
RUN apt-get update && apt-get install -y \
    wget \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    nano \
    usbutils \
    curl \
    libusb-1.0-0-dev \
    unzip \
    python3-dev \
    zstd \
    gcc-9 \
    g++-9 \
    apt-utils \
    && ln -sf /usr/bin/python3 /usr/bin/python \
    && ln -sf /usr/bin/pip3 /usr/bin/pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


# Definindo GCC e G++ como padrões
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-9

# Cria o arquivo nv_tegra_release para emulação
RUN echo "# R32 (release), REVISION: 7.1, GCID: 28947456, BOARD: t210ref, EABI: aarch64, DATE: Fri Dec 11 03:22:57 UTC 2020" > /etc/nv_tegra_release

# Cria a variável de ambiente CUDA_VERSION para emulação
ENV CUDA_VERSION=10.2
RUN ln -s /usr/local/cuda-10.2 /usr/local/cuda

# Instala as dependências do requirements.txt
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Adiciona o ambiente ROS ao bash para o usuário
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "export PATH=/usr/local/cuda-10.2/bin\${PATH:+:\${PATH}}" >> /root/.bashrc

# Instala o ZED SDK
RUN wget -O ZED_SDK_Linux.run "https://download.stereolabs.com/zedsdk/4.2/l4t32.7/jetsons" && \
    chmod +x ZED_SDK_Linux.run && \
    ./ZED_SDK_Linux.run silent skip_tools skip_python || true && \
    rm ZED_SDK_Linux.run

# Configurar variáveis de ambiente do ZED SDK
ENV PATH="/usr/local/zed/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/zed/lib:/usr/local/cuda-10.2/lib64:${LD_LIBRARY_PATH}"

# Copia o script de entrypoint
COPY entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Comando padrão ao iniciar o contêiner
ENTRYPOINT ["/ros_entrypoint.sh"]
