# MTI Instruments ProTrack Scanner Robot Raconteur Driver

MTI_RR_bridge is a Robot Raconteur driver wrapping the MTI Instruments ProTrack SDK. It provides Robot Raconteur
clients access to MTI 2D/3D sensors. It has been tested using MTI's 2D Laser Profiler (type: 8000-1066-002).

## Installation

The Releases section on GitHub contains Windows x64 binaries in a zip file. Unzip the file, change to the `bin`
directory, and run `mti2D_RR`.

## Usage

Execute by running `mti2D_RR.exe` in the `bin` directory. By default, the driver will connect to the scanner using 
IP address `192.168.100.1` and
port `32001`. These are the factory defaults for the scanner.

The driver supports the following command line options:

* `--scanner-ip-address=` - IP address of the scanner. Default is `192.168.100.1`
* `--scanner-port=` - Port of the scanner. Default is `32001`

The standard Robot Raconteur node options can also be passed to the driver. See 
https://github.com/robotraconteur/robotraconteur/wiki/Command-Line-Options for more information.

## Example

An example capturing scans and plotting the scatter plot and intensity:

```python
# Read a frame from the scanner and plot

from RobotRaconteur.Client import *
import matplotlib.pyplot as plt
import time

c = RRN.ConnectService("rr+tcp://localhost:60830/?service=MTI2D")

c.setExposureTime("25")
time.sleep(0.5)

frame = c.Capture()

plt.figure()
plt.plot(frame.X_data, frame.Z_data, "x")
plt.title("XY Scatter Plot")
plt.figure()
plt.plot(frame.X_data, frame.I_data, "x")
plt.title("XI Scatter Plot")

plt.show()

```

## Building

The driver is written in C++ and uses CMake as its build system. It has been tested on Windows 10 using Visual Studio
2019. It should be possible to build on other platforms supported by Robot Raconteur, but this has not been tested.
`vcpkg` is used to manage dependencies.

Building will also require the `EthernetScanner` directory from the MTI Instruments ProTrack SDK.

```
git clone https://github.com/robotraconteur-contrib/MTI_RR_Interface.git
cd MTI_RR_Interface
git clone --depth 1 https://github.com/robotraconteur/vcpkg-robotraconteur.git
git clone --depth 1 https://github.com/microsoft/vcpkg.git
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_BUILD_TYPE=Release -DVCPKG_TARGET_TRIPLET=x64-windows-release -DX_VCPKG_APPLOCAL_DEPS_INSTALL=ON -DETHERNET_SCANNER_DIR=../EthernetScanner
cmake --build build --config Release
```

To create the binary zip file:

```
cd build
cpack --config CPackConfig.cmake
```



