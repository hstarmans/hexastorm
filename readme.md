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
A video of the scanhead exposing is seen below;  
[![video in action not showing](https://img.youtube.com/vi/KQgnZkochu4/0.jpg)](http://www.youtube.com/watch?v=KQgnZkochu4 "Moving laserhead").


The alignment procedure is shown in the following video.

[![Alignment procedure image not showing](http://img.youtube.com/vi/Ri6DAneEzw4/0.jpg)](http://www.youtube.com/watch?v=Ri6DAneEzw4 "Alignment procedure")

## Requirements
Code is tested on a Raspberry Pi 3B and 4B. The SD-card should be at least 8 GB, ideally 16 GB.
Both Raspbian and Ubuntu can be used. Raspbian is not yet available at 64 bit.
The ArduCam, a camera used for alignment, does not have a driver for 64 bit.

# Install
Install poetry and python
```console
sudo apt install python3 python3-pip
curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python3 -
```
Use poetry to install the dependencies in pyproject.toml
```console
poetry install
```
The above allows you to test the code. 
```console
poetry run python3 src/hexastorm/movement.py
```
If you want to interact with the stepper motors and flash the fpga.
```console
./install.sh
```

## Raspberry pi
The following lines need to be in the /boot/config.txt;
```
# I2C for laserdriver and camera
i2c_arm=on
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
