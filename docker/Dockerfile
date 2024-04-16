FROM ubuntu:22.04

RUN apt-get update &&\
    apt-get install git zsh curl git vim net-tools wget htop \
    x11-apps neofetch build-essential libboost-all-dev libeigen3-dev \
    cmake sudo -y

RUN apt-get install ca-certificates --reinstall -y

ARG USERNAME=arx
ARG USER_UID
ARG USER_GID

RUN groupadd -g ${USER_GID} ${USERNAME} && \
    useradd ${USERNAME}  -u ${USER_UID} -g ${USER_GID} -m -p "$(openssl passwd -1 ${USERNAME})"
RUN usermod -aG sudo ${USERNAME}
# Give ownership of the user's home directory to the new user
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

USER ${USERNAME}
ARG HOME=/home/${USERNAME}
WORKDIR ${HOME}

RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

RUN git clone https://github.com/marlonrichert/zsh-autocomplete.git \
    ${ZSH_CUSTOM:-/home/${USERNAME}/.oh-my-zsh/custom}/plugins/zsh-autocomplete && \
    git clone https://github.com/zsh-users/zsh-autosuggestions \
    ${ZSH_CUSTOM:-/home/${USERNAME}/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git \
    ${ZSH_CUSTOM:-/home/${USERNAME}/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    git clone --depth=1 https://github.com/romkatv/powerlevel10k.git \
    ${ZSH_CUSTOM:-/home/${USERNAME}/.oh-my-zsh/custom}/themes/powerlevel10k

RUN mv ${HOME}/.zshrc ${HOME}/.zshrc.bak && \
    echo 'export ZSH="$HOME/.oh-my-zsh"\nplugins=(\n  git\n  zsh-autosuggestions\n  zsh-autocomplete\n  zsh-syntax-highlighting\n  themes\n)\n' >> ${HOME}/.zshrc && \
    echo 'ZSH_THEME="eastwood"\nsource $ZSH/oh-my-zsh.sh' >> ${HOME}/.zshrc && \
    echo 'bindkey -M menuselect "\\r" .accept-line' >> ${HOME}/.zshrc && \
    echo 'bindkey -M menuselect -s "^R" "^_^_^R" "^S" "^_^_^S"' >> ${HOME}/.zshrc && \
    echo 'bindkey -M menuselect "\\e[D" .backward-char "\\eOD" .backward-char "\\e[C" .forward-char "\\eOC" .forward-char' >> ${HOME}/.zshrc && \
    echo 'bindkey '^H' backward-kill-word' >> ${HOME}/.zshrc


RUN ARCH=$(uname -m) && SYS=$(uname -s) && \
    wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-${SYS}-${ARCH}.sh && \
    bash Miniforge3-${SYS}-${ARCH}.sh -b -p ${HOME}/miniforge3 && \
    rm Miniforge3-${SYS}-${ARCH}.sh && \
    ${HOME}/miniforge3/bin/conda init zsh && \
    ${HOME}/miniforge3/bin/conda init bash && \
    ${HOME}/miniforge3/bin/mamba init zsh && \
    ${HOME}/miniforge3/bin/mamba init bash


# pre-install vscode server and helpful plugins
RUN git clone https://gist.github.com/0a16b6645ab7921b0910603dfb85e4fb.git /home/${USERNAME}/vscode-install && \
    chmod +x /home/${USERNAME}/vscode-install/download-vs-code-server.sh && \
    /home/${USERNAME}/vscode-install/download-vs-code-server.sh linux
ENV PATH=/home/${USERNAME}/.vscode-server/bin/default_version/bin:$PATH
RUN code-server --install-extension ms-python.python && \
    code-server --install-extension mhutchie.git-graph && \
    code-server --install-extension eamodio.gitlens && \
    code-server --install-extension github.copilot && \
    code-server --install-extension kevinrose.vsc-python-indent && \
    code-server --install-extension streetsidesoftware.code-spell-checker && \
    code-server --install-extension ms-python.black-formatter && \
    code-server --install-extension ms-vscode.cpptools && \
    code-server --install-extension ms-vscode.cmake-tools


# Enable using `code` command in terminal to open file or attach new window to a folder
RUN echo "export PATH=/home/${USERNAME}/.vscode-server/bin/default_version/bin/remote-cli:\$PATH" >> /home/${USERNAME}/.zshrc

USER root

RUN apt-get update && \
    apt-get install libtinyxml2-dev libcppunit-dev liborocos-kdl-dev liburdfdom-dev iproute2 -y
RUN ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen

RUN git clone https://github.com/ros/kdl_parser.git && \
    cd kdl_parser && \
    git checkout 74d4ee3bc6938de8ae40a700997baef06114ea1b && \
    cd kdl_parser && \
    sed -i 's/kdl_parser\/visibility_control.hpp/visibility_control.hpp/g' include/kdl_parser/kdl_parser.hpp && \
    mkdir build && cd build && \
    cmake .. && make -j$(($(nproc)-1)) && make install

ARG DEBIAN_FRONTEND=noninteractive

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y ros-humble-desktop ros-dev-tools \
    ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl \
    ros-humble-geometry-msgs ros-humble-ament-lint-auto ros-humble-rosidl-default-generators \
    ros-humble-rviz2



USER ${USERNAME}

RUN ${HOME}/miniforge3/bin/pip install pybind11 numpy pyzmq atomics 


# Install unitree ros
RUN git clone https://github.com/unitreerobotics/unitree_ros2.git && \
    cd unitree_ros2/cyclonedds_ws/src && \
    git clone https://github.com/ros2/rmw_cyclonedds -b humble && \
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x && \
    cd .. && \
    colcon build --packages-select cyclonedds

RUN cd unitree_ros2/cyclonedds_ws && \
    /home/${USERNAME}/miniforge3/bin/pip install empy==3.3.2 catkin_pkg pyparsing lark && \
    /bin/zsh -c "cd ~/unitree_ros2/cyclonedds_ws && source /opt/ros/humble/setup.zsh && colcon build"


RUN echo "source /opt/ros/humble/install/setup.zsh" >> /home/${USERNAME}/.zshrc && \
    echo "source /home/\${USERNAME}/unitree_ros2/cyclonedds_ws/install/setup.zsh" >> /home/${USERNAME}/.zshrc && \
    echo "export CYCLONEDDS_URI=/home/\${USERNAME}/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml" >> /home/${USERNAME}/.zshrc && \
    sed -i "s/enp2s0/eth0/g" /home/${USERNAME}/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml




# RUN ${HOME}/miniforge3/bin/pip install \
#     https://github.com/cheng-chi/spnav/archive/c1c938ebe3cc542db4685e0d13850ff1abfdb943.tar.gz
SHELL ["/bin/zsh", "-ec"]
CMD ["zsh"]