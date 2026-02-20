# Hexastorm: Open Source Polygon Laser Scanner

**Hexastorm** is a comprehensive open-source toolkit for building high-speed, high-resolution polygon [laser scanners](https://en.wikipedia.org/wiki/Laser_scanning). Designed for applications like **PCB Laser Direct Imaging (LDI)** and **SLA 3D printing**, it leverages **FPGA technology** to achieve the precise timing required for polygon scanning.

Unlike traditional galvanometer scanners, Hexastorm uses a high-speed rotating prism or mirror to deflect the laser beam, with position feedback determined via a photodiode. The system is powered by a **Lattice iCE40UP5K FPGA** (utilizing **Amaranth HDL**) and an **ESP32-S3** (utilizing **Micropython**), providing a complete stack of hardware, gateware, and software to enable advanced laser scanning applications.

<img src="https://cdn.hackaday.io/images/490011635348687883.jpg" height="300" alt="Hexastorm System"/>

### Project Resources
* **Website:** [Hexastorm.com](https://www.hexastorm.com)
* **Blog & Updates:** [Hackaday.io Project Page](https://hackaday.io/project/21933-open-hardware-fast-high-resolution-laser)
* **Hardware Design:**
    * **Mechanical (FreeCAD):** [hstarmans/hexastorm_design](https://github.com/hstarmans/hexastorm_design)
    * **Electronics (PCB):** [hstarmans/firestarter](https://github.com/hstarmans/firestarter)
    * **Firmware:** [hstarmans/esp32_hexastorm](https://github.com/hstarmans/esp32_hexastorm)

*Acknowledgement: This code draws inspiration from [LDGraphy](https://github.com/hzeller/ldgraphy).*

### Demos

**Making a PCB with Laser Direct Imaging**   
[![Making PCB with Hexastorm](https://img.youtube.com/vi/dR09Tev0cPk/mqdefault.jpg)](http://www.youtube.com/watch?v=dR09Tev0cPk "Making PCB with Laser Direct Imaging")

# Install
First, install the [uv](https://github.com/astral-sh/uv) dependency manager:
```console
curl -LsSf https://astral-sh/uv/install.sh | sh
```
Sync the project dependencies from [pyproject.toml](./pyproject.toml).
```console
uv sync
```
Activate camera support via, see [developer.md](./developer.md).
```console 
uv sync --group camera
``` 
You can execute tests and run the main modules as follows:  
**Core tests**
```console
uv run pytest tests/test_core.py
```
**Pattern generation and interpolator**
```console
uv run python -m hexastorm.interpolator.patterns.machine
uv run python -m hexastorm.interpolator.interpolator
```


