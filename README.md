# Trabalho 2 - Robôs Autônomos

Para executar a simulação foi criado um Dockerfile que utiliza o ROS2 Humble. 

Primeiramente instale o Docker e Docker Compose

Em seguida:

```bash
xhost + local:docker
export DISPLAY=:1
docker compose build
docker compose up -d
```

Com o terminal dentro do container faça

```bash
cd ~/robos_autonomos_t2
colcon build --symlink-install - or use alias "cbs"
source install/setup.bash - or use alias "srcinst"
ros2 launch person_follower demo.launch.py 
```

Para movimentar o robô no gazebo, altere "/cmd_vel" para "/a200_0000/cmd_vel"

## TODO

- Fazer execução rápida no Gazebo
- Rodar algumas vezes (10) e testar quantas vezes dá certo
- Se incomodar, trocar o robô
