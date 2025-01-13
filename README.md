# INAOE-Tesis-Quadrotor
Actualización de trayectoria en tiempo real de cuadricóptero para la evasión óptima de obstáculos no detectados
Developer: Arturo Aceves.


## Getting started
### Hardware
For the image processing and communications side, we used the following components:

* Intel Realsense Depth Camera D455
* Jetson Nano (with a 64 GB memory card)
* Raspberry Pi 4 (with a 32 GB memory card)

For the UAV, we used as a base the F450 drone, which has the following components:

* Pixhawk 1
* 4 Brushless motor x2212-10 KV:1250
* 4 ESC 20 Amp
* GPS module M8N
* RC FS-iAGB receiver
* RC FS-i6X 10-channel transmitter
* 5v Buzzer
* Safety switch
* LiPo Battery 3 cells 5000 mA 35C
* 4 Drone propeller 9045
* F450 drone frame

### Software
* Halcon Student Edition 23.11
* Intel Realsense SDK
* Python 3.10

It is important to mention that the main OS used was Ubuntu, as most of the simulation software is available (or more stable in some cases) on that system.


### Initial Setup

#### Jetson Nano
To start, we needed to install the basic OS for this board. The tutorial can be seen on the official [Jetson Nano installation guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit). This OS was used because it is the one recommended by the brand and it already has all the drivers needed for using the GPU integrated on the board.

It is mandatory to run an update on your first boot using the following commands: 

	sudo apt-get update
	sudo apt-get upgrade

Now, we can go ahead with the installation of all the other dependencies:

1. Realsense Python library and SDK
2. Halcon Runtime for embedded devices
3. Halcon python library
4. Serial communication

##### Realsense Python library and SDK

Unfortunately for us, there is no official ARM or AARCH-based official distribution from Intel, so we need to build it. Fortunately for us, the user ysozkaya already built a program that can clean our Jetson and set everything to build the library, [you can see it here](https://github.com/ysozkaya/RealSense-Jetson)

As it is a fresh build, it can take a while to finish. It is important to make sure to know what the code does, so we can be sure what it is modifying or changing on your device.

As we followed the normal installation, the library `pyrealsense2` can now be used on all the environments of the device and the viewer present on the SDK.

To try the SDK, you can run on the terminal the following command:

	realsense-viewer
	
To try the python library, you can use the [Streaming example](https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/python-tutorial-1-depth.py) from the Intel Github.

##### Halcon Runtime for embedded devices

##### Halcon python library

##### Serial communication

#### Raspberry Pi 4
As made with the Jetson Nano board, we need to do the OS installation for this board. We are using the basic Raspbian installation present on the official Raspberry Pi Imager. The installation steps are available on the [Raspberry documentation](https://www.raspberrypi.com/documentation/computers/getting-started.html). 

It is mandatory to run an update on your first boot using the following commands: 

	sudo apt-get update
	sudo apt-get upgrade


#### Pixhawk 1



