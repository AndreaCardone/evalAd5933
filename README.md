# AD5933 EVALUATION BOARD SOFTWARE

The aim of this software is to provide an abstraction layer for the ad5933/ad5934 evaluation board

To build this program are needed:
- **cyusb-linux** with libcyusb built, cyusb-linux should be placed in the parent folder of this project. 
- **libusb-1.0**

1. Build libcyusb
2. Build libad5933 and main

## libAD5933

- **Ad5933**: AD5933 hardware interface object to setup device, perform impedance sweep and save data 

## STRUCTURE
```
MyProject/
|-- CMakeLists.txt
|-- src/
|   |-- main.cpp
|-- lib/
|   |-- libad5933.cpp
|-- include/
|   |-- libad5933.h
|   |-- ad5933types.h
```
