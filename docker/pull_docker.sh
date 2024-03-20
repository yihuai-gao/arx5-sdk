#!/bin/zsh
SCRIPT_DIR=$(realpath $(dirname $0))
WORKSPACE_DIR=$(realpath $SCRIPT_DIR/..)
REMOTE_USER=arx

docker image pull yihuai1013/arx5:latest
docker run \
  -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
  -v $WORKSPACE_DIR:/home/$REMOTE_USER/arx5-python-sdk \
  -v ~/.zsh_history:/home/$REMOTE_USER/.zsh_history \
  --workdir /home/$REMOTE_USER/arx5-python-sdk \
  --net=host \
  -it \
  arx5:latest