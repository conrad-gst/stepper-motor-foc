# Field Oriented Control of a Stepper Motor

This repository provides an implementation of a field oriented control (FOC) scheme for a hybrid stepper motor with an STM32 NUCLEO-L432KC microcontroller board. A schematic of the used driver board can be found in the folder "hardware".

Running the FOC scheme
1. set up the hardware (see folder "hardware")
2. import the CubeIDE project into CubeIDE
3. adjust the macros in the file CubeIDE project/Core/Src/main.c to match the used motor and encoder parameters
4. build the project and load it onto the NUCLEO board

Tests, derivations and further informations can be found [here](https://conrad-gst.github.io/articles/microcontrollerProjects/stepperFOC.html).
