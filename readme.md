# Laser scanner [![Documentation Status](https://readthedocs.org/projects/luna/badge/?version=latest)](https://hexastorm.readthedocs.io/en/latest/?badge=latest)
[Laser scanning](https://en.wikipedia.org/wiki/Laser_scanning) is the controlled deflection of laser beams, visible or invisible.
In a high-speed polygon scanner system, the laser is deflected by a rotating prism or mirror. 
The position of the laser is determined via a photodiode.  
Hexastorm is a full toolkit for working with polygon lasers scanners using FPGA technology; and provides hardware, gateware, and software 
to enable laser scanning applications.  
<img src="https://cdn.hackaday.io/images/490011635348687883.jpg" align="center" height="300"/>  
Code is tested on the system shown above, branded as [Hexastorm](https://www.hexastorm.com). 
The FreeCAD design is shared [here](https://github.com/hstarmans/hexastorm_design) 
and PCB designs are found [here](https://github.com/hstarmans/firestarter).
A blog can be found on [Hackaday](https://hackaday.io/project/21933-open-hardware-fast-high-resolution-laser).
The code took most inspiration from [LDGraphy](https://github.com/hzeller/ldgraphy).  
Making a PCB with scanner is shown in the video;  
[![video in action not showing](https://img.youtube.com/vi/dR09Tev0cPk/0.jpg)](http://www.youtube.com/watch?v=dR09Tev0cPk "Making PCB with Laser Direct Imaging").

The alignment procedure is shown in the following video.

[![Alignment procedure image not showing](http://img.youtube.com/vi/Ri6DAneEzw4/0.jpg)](http://www.youtube.com/watch?v=Ri6DAneEzw4 "Alignment procedure")

## Requirements
Code is tested on a Raspberry Pi 3B and 4B. The SD-card should be at least 8 GB, ideally 16 GB.  
Both 32 bit and 64 bit are tested, with camera.

# Overall note
Development is done on the main or master branch.
Look through to commit history to find the right working version which was used in the video.

# Install
Install poetry and python.
```console
sudo apt install python3 python3-pip
curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python3 -
```
Use poetry to install the dependencies in pyproject.toml.
Poetry creates a virtual environment for you. 
So only using ```poetry run python``` enables you to access the packages.
```console
poetry install
```
Depending on your operating system you also need to install certain extras.
Look into the pyproject.toml file for the possibilities.
Yosys must be compiled or installed via [oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build).
An alternative is to install python packages which use wasm but these are experimental.
```console
poetry install --extras "64bit"
```
If this doesn't work try 
You can test code as follows. 
```console
poetry run python3 -m hexastorm.movement
```
If you want to interact with the stepper motors and flash the fpga.
```console
./install.sh
```
## Micropython
An idea, I am working on is to call controller.py from Micropython.
Most dependencies can be moved to ESP32.

## Raspberry pi
The following lines need to be in the /boot/config.txt;
```
# I2C for laserdriver and camera
dtparam=i2c_arm=on
dtparam=i2c_vc=on
# SPI
dtoverlay=spi0-1cs,cs0_pin=18
dtoverlay=spi1-1cs,cs0_pin=7
# camera
dtoverlay=vc4-fkms-v3d
start_x=1
gpu_mem=300
```
There should not be dtparam=spi=on, somewhere. This would enable two chip selects for SPI0 and 
creates a conflict with the pin select of SPI1. 

## Arducam
Install my version of the Python libary available at [ArducamPython](https://github.com/hstarmans/Arducampython).
