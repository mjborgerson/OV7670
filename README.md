# OV7670

## Hardware Limitations

This library works only with the Teensy 4.1 from PJRC.com.   This is the only processor with the pins necessary  to connect to the CMOS Sensor Interface (CSI) built into the IMXRT1062 MPU in the Teensy 4.x series of modules.

## Required Connections

Your OV7670 camera module will have to be wired as shown in the library documentation in order to work with this library.

## Implementation

This library instantiates a single instance of the clOV7670 object.  In this way it is like the Serial interface implemented in the TeensyDuino additions to the standard Arduino system.  This approach was selected because there is only one CSI interface and it simplifies the implementation of the interrupt handler.

