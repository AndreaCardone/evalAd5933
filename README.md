# AD5933 EVALUATION BOARD SOFTWARE

The aim of this software is to provide an abstraction layer for the ad5933/ad5934 evaluation board, i.e. abstracting cypress usb bridge and ad5933 i2c interface. 

To build this program are needed:
- **cyusb-linux** with libcyusb built, cyusb-linux should be placed in the parent folder of this project. 
- **libusb-1.0**

1. Build libcyusb
2. Build libad5933
3. Build example software

## libAD5933

- **Ad5933**: AD5933 hardware proxy object 
- **AcquisitionHelper**: helper object to capture and save data

## PROJECT STRUCTURE
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
