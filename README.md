# INAOE-Tesis-Quadrotor
Actualización de trayectoria en tiempo real de cuadricóptero para la evasión óptima de obstáculos no detectados
Developer: Arturo Aceves.


## Getting started
### Hardware
For the image processing and communications side, we had used the following components:

* Intel Realsense Depth Camera D455
* Jetson Nano 
* Raspberry Pi 4

For the UAV, we used as a base the F450 drone, which has the following components:

* Pixhawk 1
* 4 Brushless motor x2212-10 KV:1250
* 4 ESC 20 Amp
* GPS module M8N
* RC FS-iAGB receiver
* RC FS-i6X 10 channel transmitter
* Buzer
* Safety switch
* LiPo Baterry 3 cells 5000 mA 35C
* 4 Drone propeller 9045
* F450 drone frame

### Software
* Halcon Student Edition -VERSION-
* Intel Realsense SDK
* Python 3.10

It is important to mention that the main OS used was Ubuntu, as most of the simulation software is available (or more stable in some cases) on that system.


### Initial Setup

#### Jetson Nano
https://github.com/ysozkaya/RealSense-Jetson


