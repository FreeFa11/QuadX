# QuadX
A small Drone designed to be simple and cheap and not require a physical controller. It can be controlled by any Smartphone or Computer through Wifi connection using a web browser. 

## Project Outline
- <B>Firmware:</B> Flight Controller and Communication codes.
- <B>Hardware:</B> Schematic and 3D Files.
- <B>Webpage:</B> Controller Webpage Files


## [Frame](./Hardware/Frame/)

<p align="center">
    <img src="./Hardware/Frame/RenderNOBG.png" alt="Image 1" width="70%"/>
</p>

*Work In Progress* :)

## [PCB](./Hardware/PCB/)

<p align="center">
    <img src="./Hardware/PCB/Renders/NOBG1.png" alt="Image 1" width="40%"/>
    <img src="./Hardware/PCB/Renders/NOBG2.png" alt="Image 1" width="40%"/>
</p>

### Features

- ESP32S3 Microncontroller
- Onboard BMI270 IMU
- Onboard AO3416 MOSFETS
- Onboard Camera Connector
- Onboard 1S Battery Charger

External GPS + Compass module can be connected using the JST-SH (1.00 mm) connector.

## [Firmware](./Firmware/)

*Work In Progress* :)
<!-- The [Firmware](./Firmware/) has been built using PlatformIO. -->

## License

All the files in [Hardware](./Hardware/) folder are lincensed under [CERN OHL-P v2](./Hardware/LICENSE).

All other files are lincensed under [BSD 3-Clause](./LICENSE).