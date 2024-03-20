#! /bin/bash
SCRIPT_DIR=$(realpath $(dirname $0))
WORKSPACE_DIR=$(realpath $SCRIPT_DIR/..)
cd $WORKSPACE_DIR

sudo apt-get update
sudo apt-get install libtinyxml2-dev libcppunit-dev liborocos-kdl-dev liburdfdom-dev iproute2 -y
sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen

sudo apt-get install libspnav-dev spacenavd -y
pip install https://github.com/cheng-chi/spnav/archive/c1c938ebe3cc542db4685e0d13850ff1abfdb943.tar.gz
pip install pybind11 numpy pyzmq atomics 


git clone https://github.com/ros/kdl_parser.git && \
cd kdl_parser && \
git checkout 74d4ee3bc6938de8ae40a700997baef06114ea1b && \
cd kdl_parser && \
sed -i 's/kdl_parser\/visibility_control.hpp/visibility_control.hpp/g' include/kdl_parser/kdl_parser.hpp && \
mkdir build && cd build && \
cmake .. && make -j$(($(nproc)-1)) && sudo make install
cd $WORKSPACE_DIR && rm -rf kdl_parser


mkdir build && cd build && \
cmake .. -DPYBIND_PATH=$(pip show pybind11 | grep Location | cut -d' ' -f2) && make -j$(($(nproc)-1))