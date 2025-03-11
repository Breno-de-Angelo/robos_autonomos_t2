# Trabalho 2 - Robôs Autônomos

Para executar a simulação foi criado um Dockerfile que utiliza o ROS2 Humble. Primeiramente instale o Docker.

Em seguida:
```bash
docker build . -t robos_autonomos_t2
xhost + local:docker
export DISPLAY=:1
docker run --name robos_autonomos_t2 -it --net=host --device /dev/dri/ -e DISPLAY=$DISPLAY -v $HOME/.Xauthority:/root/.Xauthority:ro -v .:/root/robos_autonomos_t2 robos_autonomos_t2
```

Com o terminal dentro do container faça
```bash
source install/setup.bash
ros2 launch clearpath_gz simulation.launch.py
```

