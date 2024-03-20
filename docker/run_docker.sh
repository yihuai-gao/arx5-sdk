#!/bin/zsh
SCRIPT_DIR=$(realpath $(dirname $0))
WORKSPACE_DIR=$(realpath $SCRIPT_DIR/..)
REMOTE_USER=arx

SYSTEM=$(uname -s)

if [ "$SYSTEM" = "Linux" ]; then
  USER_UID=$(id -u)
  USER_GID=$(id -g)
elif [ "$SYSTEM" = "Darwin" ]; then
  USER_UID=1000
  USER_GID=1000
else
  echo "Unsupported system: $SYSTEM"
  exit 1
fi

docker build \
  --build-arg USER_UID=$USER_UID \
  --build-arg USER_GID=$USER_GID \
  -t arx5-python-sdk:latest $SCRIPT_DIR

# Check devcontainer.json for detailed explanation of the following arguments
docker run \
  -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
  -v $WORKSPACE_DIR:/home/$REMOTE_USER/arx5-python-sdk \
  -v ~/.zsh_history:/home/$REMOTE_USER/.zsh_history \
  --workdir /home/$REMOTE_USER/arx5-python-sdk \
  --net=host \
  -it \
  arx5-python-sdk:latest