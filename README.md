# C++ && Python SDK for ARX5 robot arm
## Features
- Run without ROS
- No `sudo` requirement (all dependencies are managed under conda environment, thanks to [Cheng Chi](https://cheng-chi.github.io/))
- Simple python interface with type hint
- Joint controller runs at 500Hz in the background (motor communication delay ~0.4ms)
- Control multiple arms in the same process

## Build & Install
```bash
mamba env create -f conda_environments/py310_environment 
# Currently available python versions: 3.8, 3.9, 3.10, 3.11 
conda activate arx-py310
mkdir build && cd build
cmake ..
make -j
# At this point, you should be able to run test scripts below.
# To install the C++ package your system, run:
make install
```

## CAN setup

``` sh
sudo apt install can-utils
sudo apt install net-tools
```

There are 2 popular firmware types of usb-can adapter, `SLCAN` and `candleLight`. After plugging the adapter, you can find out the correct firmware type by the following rules:
- Run `ls /dev/ttyACM*`. If there is a **new** `/dev/ttyACM*` device (where * is a number), this adapter is using `SLCAN` firmware
- Run `ip a`. If there is a **new** `can*` interface (where * is a number), this adapter is using `candleLight` firmware.

### For adapters using SLCAN framework
Get serial number by:
``` sh
udevadm info -a -n /dev/ttyACM* | grep serial
# Replace the * by the actual number if there are multiple ttyACM devices connected to your computer.
```
You will get something like:
```
ATTRS{serial}=="209738924D4D"
ATTRS{serial}=="0000:00:14.0"
```
Then edit CAN rules file:
``` sh
sudo vim /etc/udev/rules.d/arx_can.rules
```
Copy and paste the following, and replace the serial number with yours. If you are registering multiple adapters, you can use other `SYMLINK` names (e.g. `canable1`) and make sure the following commands are updated accordingly.
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="117e", ATTRS{serial}=="209738924D4D", SYMLINK+="canable0"
```

Finally, activate CAN connection by:
``` sh
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo slcand -o -f -s8 /dev/canable0 can0 && sudo ifconfig can0 up
```

## Test scripts
```bash
cd python
python examples/test_joint_control.py
python examples/test_bimanual.py # For two arms using can0 and can1, each arm will act the same as test_joint_control.py
```
To use python sdk from other directories, please make sure `./python` is in `$PYTHONPATH` and `./lib/x86_64` or `./lib/aarch64` (depend on your computer system) is in `$LD_LIBRARY_PATH`.
After compiling the `arx5_interface` pybind dynamic library, you can run it under other python environments (need to be the same python version as the one you built).