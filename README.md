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
Code can be deployed on microcontroller ESP32S3 and FPGA UP5K.  
This requires additional library [esp32_hexastorm](https://github.com/hstarmans/esp32_hexastorm).
Part of the code can be run on Micropython, rest is Python only. 

# Install
Install poetry and python.
```console
sudo apt install python3 python3-pip
curl -sSL https://install.python-poetry.org | python3 -
```
Use poetry to install the dependencies in pyproject.toml.
The virtual environment needs to be installed in project for VSCode to detect it.
```console
poetry config virtualenvs.in-project true
poetry install --without raspberry,64bit
```
Poetry creates a virtual environment for you. 
 ```poetry run python``` gives you access to the packages installed.
Depending on your operating system you also need to install certain extras.
Look into the pyproject.toml file for the possibilities.
Yosys must be compiled or installed via [oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build).
An alternative is to install python packages which use wasm but these are experimental.
```console
poetry install --without raspberry
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


