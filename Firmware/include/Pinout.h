#pragma once


// Camera Connector
#define PCB_CAM_PWDN        46
#define PCB_CAM_RESET       -1

#define PCB_CAM_VSYNC       21
#define PCB_CAM_HREF        14
#define PCB_CAM_PCLK        9
#define PCB_CAM_XCLK        12

#define PCB_CAM_SIOD        45
#define PCB_CAM_SIOC        47

#define PCB_CAM_Y9          13      // Y9  D7
#define PCB_CAM_Y8          11      // Y8  D6
#define PCB_CAM_Y7          10      // Y7  D5
#define PCB_CAM_Y6          6       // Y6  D4
#define PCB_CAM_Y5          15      // Y5  D3
#define PCB_CAM_Y4          18      // Y4  D2
#define PCB_CAM_Y3          17      // Y3  D1
#define PCB_CAM_Y2          7       // Y2  D0


// Serial IO
#define PCB_SDA             1
#define PCB_SCL             2

#define PCB_TX              43
#define PCB_RX              44

#define PCB_SDI             39
#define PCB_SDO             40
#define PCB_CLK             41
#define PCB_CSE             42      // External Device
#define PCB_CSI             3       // Internal Device (BMI160)

#define PCB_M1              48
#define PCB_M2              8
#define PCB_M3              16
#define PCB_M4              38