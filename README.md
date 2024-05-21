# C++ && Python SDK for ARX5 robot arm
## Features
- Run without ROS
- No `sudo` requirement (all dependencies are managed under conda environment, thanks to [Cheng Chi](https://cheng-chi.github.io/))
- Simple python interface with type hint
- Joint controller runs at 500Hz in the background (motor communication delay ~0.4ms)
- Control multiple arms in the same process

## Install
```bash
mamba env create -f conda_environment.yaml # for python3.9, or py310_environment.yaml for python3.10
conda activate arx # arx-py310
mkdir build && cd build
cmake ..
make -j
```
## Usage
```bash
cd python
python examples/test_joint_control.py
python examples/test_bimanual.py # For two arms using can0 and can1, each arm will act the same as test_joint_control.py
```
To use python sdk from other directories, please make sure `./python` is in `$PYTHONPATH` and `./lib/x86_64` or `./lib/aarch64` (depend on your computer system) is in `$LD_LIBRARY_PATH`.
After compiling the `arx5_interface` pybind dynamic library, you can run it under other python environments (need to be the same python version as the one you built).