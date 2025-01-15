# INAOE-Tesis-Quadrotor
Actualización de trayectoria en tiempo real de cuadricóptero para la evasión óptima de obstáculos no detectados
Developer: Arturo Aceves.


## Getting started
### Hardware
For the image processing and communications side, we used the following components:

* Intel Realsense Depth Camera D455
* Jetson Nano (with a 64 GB SD memory card)
* Raspberry Pi 4 (with a 32 GB memory card)

For the UAV, we used as a base the F450 drone, which has the following components:

* Pixhawk 1 (with a 4 GB SD memory card)
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

Finally, all the development and testing was made on an Asus FX505DT

### Software
* Halcon Student Edition 23.11
* Halcon Runtime Student Edition 23.11
* Intel Realsense SDK
* ROS 2 Humble
* Gazebo Harmonic
* Python 3.10
* ArduCopter

### Python libraries
* HALCON/Python
* OpenCV2
* Numpy
* Dronekit
* Dronekit-sitl
* Pymavlink

It is important to mention that the main OS used was Ubuntu, as most of the simulation software is available (or more stable in some cases) on that system.


### Initial Setup

#### Asus FX505DT

This device needs some pre-parameters to start any development that doesn't need a whole explanation, which are listed as follow:

1. Ubuntu 22.04.5 LTS
2. Python 3.10 or a more recent version

There are some other programs or applications needed for the development that we are going to explain a little bit:

1. ROS 2 Humble.
2. Gazebo Harmonic.
3. Intel Realsense SDK.
4. Halcon Student Edition
4. Python libraries.
  
##### ROS 2 Humble
This version is used as it is the best one for our Ubuntu 22.04 version.
The steps for installing it on Ubuntu can be seen on the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) from ROS org.
This program is used for the communication between programs, as the Gazebo and the multiple codes need them to simulate the whole system.

##### Gazebo Harmonic

Again, this version is used as it is the best one for our Ubuntu and ROS2 version.
The installation is pretty straigthforward, you just need to follow the [official Gazebo Harmonic installation guide](https://gazebosim.org/docs/harmonic/install_ubuntu/)

##### Intel Realsense SDK

This SDK is mostly used for testing the behavior and calibration for the Realsense D455 camera.
The installation process can also be completed by following the official [IntelRealsense installation guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

##### Halcon Student Edition

The version used on the project is the Student Edition 23.11, which can be installed by following the steps present on the [Halcon on Campus site](https://license.mvtec.com/campus/index.php). It is important to mention that you would need a specific license for this installation and is different from the one used on the Jetson Nano.

##### Python libraries

The process for these is also straigthforward if you are already familiar with installing any library on your environment. 
For the development, the virtual environment was created and we executed the following commands:

	pip install numpy
	pip install pyrealsense2
	pip install opencv-python
	pip install mvtec-halcon==23110
	pip install pymavlink
	pip install dronekit
	pip install dronekit-sitl

There are 2 important steps needed for the mvtec-halcon and the dronekit-sitl libraries, as they need additional configuration to work with our python version.

First, for the mvtec-halcon we need to set up the halcon root path and the library path on the .bashrc file. To do so, w need to execute the following commands on the console:

	cd
	sudo nano .baschrc
	
And write at the end of the file:

	export HALCONROOT=<The root path of the Halcon installation>
	export LD_LIBRARY_PATH=$HALCONROOT/lib/x64-linux
	
This is needed because the python library doesn't get automatically the location of the program and it needs to be set manually

Save the file and restart the computer.
Also, you need to do a change on the dronekit-sitl installation, as the python 3.10 package came with an error that doesnt let you execute any command on the SITL model of the quadcopter. For this, we need to go to the `/usr/local/lib/python3.10/distpackages/pymavlink` path and edit the `mavutil.py .py` code, commenting all the `self.mav.commando_long_send()` variable (it would be from line 686 to 697 aprox) under the `def set_mode_apm` definition.
This is needed because the variable is not compatible with the version of Mavproxy and uses different variables to comunicate the messages between the SITL mode and Mavlink, and the version 2.4.8 is not available on python 3.10.


#### Jetson Nano
To start, we needed to install the basic OS for this board. The tutorial can be seen on the official [Jetson Nano installation guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit). This OS was used because it is the one recommended by the brand and it already has all the drivers needed for using the GPU integrated on the board.

It is mandatory to run an update on your first boot using the following commands: 

	sudo apt-get update
	sudo apt-get upgrade
	
It is important also to install python 3.8 at least, as the board comes out of the box with python 3.6.

Now, we can go ahead with the installation of all the other dependencies:

1. Realsense Python library and SDK
2. Halcon Runtime for embedded devices
3. Halcon python library
4. Serial communication

##### Realsense Python library and SDK

Unfortunately for us, there is no official ARM or AARCH-based official distribution from Intel, so we need to build it. Fortunately for us, the user `ysozkaya` already built a program that can clean our Jetson and set everything to build the library, [you can see it here](https://github.com/ysozkaya/RealSense-Jetson)

As it is a fresh build, it can take a while to finish. It is important to make sure to know what the code does, so we can be sure what it is modifying or changing on your device.

As we followed the normal installation, the library `pyrealsense2` can now be used on all the environments of the device and the viewer present on the SDK.

To try the SDK, you can run on the terminal the following command:

	realsense-viewer
	
To try the python library, you can use the [Streaming example](https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/python-tutorial-1-depth.py) from the Intel Github.

It is important to mention that the code from `ysozkaya` also install python3 and the dependencies needed for the realsense library, so this is the first step needed for the process.

##### Halcon Runtime for embedded devices

For this

##### Halcon python library


And write at the end of the file:

	export HALCONROOT=<The root path of the Halcon installation>
	export HALCONARCH=aarch64-linux
	export LD_LIBRARY_PATH=$HALCONROOT/lib/$HALCONARCH
	

##### Serial communication

By default, the UART port is used automatically to start a console once the board is energized. We need to disable this service to automatically start the communication with other devices. To do so, we need to run the following commands on the console:

	systemctl stop nvgetty
	systemctl disable nvgetty
	udevadm trigger

Once this is finished, you need to reboot the Jetson Nano.

#### Raspberry Pi 4
As made with the Jetson Nano board, we need to do the OS installation for this board. We are using the basic Raspbian installation present on the official Raspberry Pi Imager. The installation steps are available on the [Raspberry documentation](https://www.raspberrypi.com/documentation/computers/getting-started.html). 

It is mandatory to run an update on your first boot using the following commands: 

	sudo apt-get update
	sudo apt-get upgrade
	
Now we can proceed with the with the installation/configuration of the other dependencies:

1. Serial communication
2. Drone control libraries 

##### Serial communication

To start any communication between the Raspberry, the Pixhawk and the Jetson Nano, it is important to enable the serial communication on the boards. For the Raspberry, we need to run the command:

	sudo raspi-config
	
and then go to `Interface Options` and then `Serial Port`. Here, we enable only the serial port hardware. Normally, it first ask us if we like a login shell to be accessible over serial, but thats not required.

It is important to mention that out of the box, the raspberry has the first serial port connected to the bluetooth controller and it doesnt have all the UART ports enabled, so it is needed to configure the ports in the correct way to avoid any problem. For the first implementation, we disabled the bluetooth device to connect the Pixhawk to the UART0 and the Jetson Nano to the UART4. To do so, we need to edit the `config.txt` file, which can be made runing on the terminal:

	sudo nano /boot/config.txt
	
and adding the follow lines at the bottom of the text and saving it:

	enable_uart=1
	dtoverlay=disable-bt
	dtoverlay=uart5

Once this is done, you need to reboot the Raspberry and the communication can be made without any problem.

##### Drone control libraries

Controlling the Pixhawk using the Raspberry as a main computer can be achieved by installing some dependencies and libraries from python.

You only need to run each of the following commands on the console:

	sudo apt-get install python-pip
	sudo apt-get install python-dev
	sudo apt-get install screen python-wxgtk4.0 python-lxml
	
	sudo pip install pyserial
	sudo pip install dronekit
	sudo pip install MAVProxy
	
#### Pixhawk 1

The Pixhawk used for this building already had the OS installed, but a fresh installation could be empty of any OS. To install it, it is recommended to use Mission Planer on Windows to do the initial configuration, as we are using Ardupilot as firmware. Check the official [Ardupilot/Mission Planer installation guide](https://ardupilot.org/copter/docs/configuring-hardware.html) for more information.
