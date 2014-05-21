# Introduction
_Arbitros_ (pronounced are-bee-trose) is a lightweight real time operating system developed initially—but not solely— for the Atmel XMEGA 8/16-bit series microcontroller. The name _arbitros_ is a fusion between the Latin arbitror (meaning to judge or decide) and OS (operating system). The software was developed over three years of meticulous work and is comprised of five thoroughly tested modular core components named **RTOS**, **HAL**, **UTILITIES**, **DRIVERS**, and **PACKAGES**. These components were designed to abstract common functionality, providing a user-space application with a modular, readable, layered architecture as seen in the figure below.

<img src=https://github.com/rmmurphy/arbitros/raw/master/ref/images/arbLayers.png width=450 height=400>

## Modular Core Components

### RTOS

The RTOS module performs kernel operations such as thread, semaphore, and file management;in addition to priority based task switching, driver management, time management, error handling, inter-thread communication (mailboxing), watchdog management, system debug management, and system terminal control.

### HAL

The hardware abstraction layer provides a common interface for an application when communicating with the following device specific peripherals: ADCs, clocks, DMAs, EEPROM, GPIOs, PMIC, SPIs, timers, TWIs, UARTs, and watchdog timers.

### UTILITIES

The UTILITIES module consists of software designed to help with common programming tasks such as buffer, linked-list, and state-machine management. Additionally, the module contains a mathematical library designed to perform floating and fixed-point operations such as multiply, divide, cos, sin, atan2, log10, alog10, and sqrt. In addition to these common math routines, the package contains software for performing floating point matrix operations such as inversion, transposition, addition, subtraction, multiplication, division, normalization, and eigenvalue decomposition.

### DRIVERS

A collection of Linux inspired device drivers that control a variety of external components such as Sparkfun’s 9 DOF sensor module, Newhaven’s serial character LCD module, a generic pan/tilt servo, micro SD cards, sonar modules, the RN-XV wifly module, and a terminal window controller.

### PACKAGES

External open source libraries such as libfat32, modified to operate within the _arbitros_ framework.

## Designed with Portability in Mind

Although _arbitros_ currently works with the Atmel atxmega128A1, the software was designed to be completely portable—simply modify the contents of the ‘HAL’ layer in order to port the core components to a multitude of processor architectures.

## Easy to use and rich in capability

_Abitros_ was created to fit a gap in the hobbyist market. Development platforms such as Arduino were originally designed for individuals with minimal or no programming experience. As a result, their monolithic software architecture is limiting—lacking many of the features of a higher level multi-threaded operating environment. Conversely, single board computers like Raspberry Pi’s were intended for experienced programmers providing a richer set of capability; however, setting up the build environment (installing the root file system and cross compilation tools) to run Linux is a daunting task—even for seasoned professionals. 
_Arbitros_ combines the best attributes of Arduino—simplicity—with that of Linux based systems—capability—in one easy to use package. 

<img src=https://github.com/rmmurphy/arbitros/raw/master/ref/images/complexity.png width=350 height=350>

## Standardized Software Interfaces
Unlike Arduino, _arbitros_ incorporates Linux inspired function calls such as open, close, read, write, and ioctl in order to abstract–from the user–the intricate details that define how a driver communicates with an off-chip device (such as a shield). As seen in the example below, this framework allows a user-space application to interact with a particular entity–for instance a WIFI module–without having to know the specific details about how the device is controlled (such as the particular UART and GPIO ports used). Furthermore, this driver "blue print" makes it easier for a hobbyist to upgrade external hardware capability without effecting existing _arbitros_ user-space software applications. For example, imagine that a hobbyist has developed an _arbitros_ application that controls an autonomous vehicle using inputs from Sparkfun’s 9DOF sensor module . At a future date the hobbyist decides to upgrade the IMU to a newer more capable model. With _arbitros_ this is easy, the hobbyist simply swaps the old device driver for the new one–without requiring changes to his/her user-space application software. 

<img src=https://github.com/rmmurphy/arbitros/raw/master/ref/images/arbdeviceTopLevel.png width=600 height=200>
