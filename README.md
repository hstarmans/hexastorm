# Laser scanner
[Laser scanning](https://en.wikipedia.org/wiki/Laser_scanning) is the controlled deflection of laser beams, visible or invisible.
In a high-speed polygon scanner system, the laser is deflected by a rotating prism or mirror. 
The position of the laser is determined via a photodiode.  
Hexastorm is a full toolkit for working with polygon lasers scanners using FPGA technology and provides hardware, gateware, and software 
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
Install the dependency manager uv.
```console
sudo apt install pipx
pipx ensurepath
pipx install uv
```
Use uv to install the dependencies in pyproject.toml.
```console
uv sync
```
Code can be executed as follows. 
```console
uv run python -m hexastorm.movement
uv run python -m hexastorm.platforms
uv sync --group plot
uv run python -m hexastorm.interpolator.interpolator
```


