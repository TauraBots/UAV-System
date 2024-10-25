# Nome da imagem Docker
IMAGE_NAME = uav-system

# Nome do container
CONTAINER_NAME = uav-container

# Diretório de trabalho no container
WORKDIR = /root/UAV-System

# Comando para construir a imagem Docker
build:
	@echo "Building Docker image..."
	docker build -t $(IMAGE_NAME) .

# Comando para rodar o container com privilégios e acesso às portas USB
run:
	@echo "Running Docker container with USB access..."
	docker run -it --privileged --runtime nvidia -v /dev/bus/usb:/dev/bus/usb --network=host --name $(CONTAINER_NAME) $(IMAGE_NAME)

# Comando para entrar no container se ele estiver rodando
shell:
	@echo "Entering Docker container..."
	docker exec -it $(CONTAINER_NAME) /bin/bash

# Comando para parar o container
stop:
	@echo "Stopping Docker container..."
	docker stop $(CONTAINER_NAME)

# Comando para remover o container
remove:
	@echo "Removing Docker container..."
	docker rm $(CONTAINER_NAME)

# Comando para limpar tudo (container e imagem)
clean:
	@echo "Cleaning Docker container and image..."
	docker rm -f $(CONTAINER_NAME) || true
	docker rmi -f $(IMAGE_NAME) || true

sh:
	@echo "Script Running"
	./docker.sh l4t-r35.4.1 zedsdk4.1.2
