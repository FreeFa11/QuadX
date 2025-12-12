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
- Onboard BMI160 IMU
- Onboard AO3416 MOSFETS
- Onboard Camera Connector
- Onboard 1S Battery Charger

External GPS + Compass module can be connected using the JST-SH (1.00 mm) connector.

## [Firmware](./Firmware/)

The Firmware has been developed using PlatformIO. This Firmware shares a lot of code with my [TwinBlade](https://github.com/FreeFa11/TwinBlade) project.

- **Build**: Open the [Firmware](./Firmware/) folder with PlatformIO installed and build:
  ```
  platformio run
  ```
  Build the filesystem that contains all the required files:
  ```
  platformio run --target buildfs
  ```

- **Flash**: Connect the target microcontroller to flash and run:
  ```
  platformio run --target uploadfs
  ```
  After flashing the filesystem, flash the firmware with:
  ```
  platformi run --target upload
  ```

- **Connect**: After the firmware has been successfully flashed, the microcontroller will act as a Wifi access point with the name `QuadX`. Connect your smartphone to it with password `letsrock`. Open your web browser and type the URL `quadx.local`. This will connect you to the webserver and load the webpage where you can control and modify the drone and its settings.

  *`quadx.local` only works in Wifi AP mode*

The Web Interface has mostly been completed in terms of functionality. Still needs some work in terms of UI. 

*FLIGHT SYSTEM is Yet to be developed*

## License

All the files in [Hardware](./Hardware/) folder are licensed under [CERN OHL-P v2](./Hardware/LICENSE).

All other files are lincensed under [BSD 3-Clause](./LICENSE).