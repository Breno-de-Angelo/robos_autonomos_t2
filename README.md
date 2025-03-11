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
cd ~/robos_autonomos_t2
wget -O pose_landmarker.task -q https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/1/pose_landmarker_heavy.task
source install/setup.bash
ros2 launch person_follower demo.launch.py
```

O comando wget serve para baixar o modelo de reconhecimento de pessoas e só precisa ser executado uma vez.

Para movimentar o robô no gazebo, altere "/cmd_vel" para "/a200_0000/cmd_vel"
