# SDK for ARX5 robot arm

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
python examples/joint_control_test.py
```

To use python sdk from other directories, please make sure `./python` is in `$PYTHONPATH` and `./lib/x86_64` or `./lib/aarch64` (depend on your computer system) is in `$LD_LIBRARY_PATH`.

After compiling the `arx5_interface` pybind dynamic library, you can run it under other python environments (need to be the same python version as the one you built). 