# 允许本地的 root 用户访问 X Server（Docker 容器默认以 root 运行）
xhost +local:root
IMAGE_NAME="pointcloud_607:v4"
HOST_WORK_DIR="/home/ztl/Documents/Projects"
DOCKER_WORK_DIR="/root/Projects"
HOST_DATA_DIR="/home/ztl/Documents/Data"
DOCKER_DATA_DIR="/root/Data"

sudo docker run -it --rm \
    --network host \
    --ipc=host \
    --device=/dev/dri:/dev/dri \
    --device=/dev/mali0:/dev/mali0 \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="http_proxy=http://192.168.10.1:10808" \
    --env="https_proxy=http://192.168.10.1:10808" \
    --privileged \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOST_WORK_DIR:$DOCKER_WORK_DIR" \
    --volume="$HOST_DATA_DIR:$DOCKER_DATA_DIR" \
    $IMAGE_NAME \
    bash

