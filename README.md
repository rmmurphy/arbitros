<img src=https://github.com/rmmurphy/arbitros/raw/master/ref/images/arbitros.jpg> 
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

# Why was _arbitros_ Created?

## Easy to use and rich in capability

_Abitros_ was created to fit a gap in the hobbyist market. Development platforms such as Arduino were originally designed for individuals with minimal or no programming experience. As a result, their monolithic software architecture is limiting—lacking many of the features of a higher level multi-threaded operating environment. Conversely, single board computers like Raspberry Pi’s were intended for experienced programmers providing a richer set of capability; however, setting up the build environment (installing the root file system and cross compilation tools) to run Linux is a daunting task—even for seasoned professionals. 
_Arbitros_ combines the best attributes of Arduino—simplicity—with that of Linux based systems—capability—in one easy to use package. 

<img src=https://github.com/rmmurphy/arbitros/raw/master/ref/images/complexity.png width=350 height=350>

## Standardized Software Interfaces
Unlike Arduino, _arbitros_ incorporates Linux inspired function calls such as open, close, read, write, and ioctl in order to abstract–from the user–the intricate details that define how a driver communicates with an off-chip device (such as a shield). As seen in the example below, this framework allows a user-space application to interact with a particular entity–for instance a WIFI module–without having to know the specific details about how the device is controlled (such as the particular UART and GPIO ports used). Furthermore, this driver "blue print" makes it easier for a hobbyist to upgrade external hardware capability without effecting existing _arbitros_ user-space software applications. For example, imagine that a hobbyist has developed an _arbitros_ application that controls an autonomous vehicle using inputs from Sparkfun’s 9DOF sensor module . At a future date the hobbyist decides to upgrade the IMU to a newer more capable model. With _arbitros_ this is easy, the hobbyist simply swaps the old device driver for the new one–without requiring changes to his/her user-space application software. 

<img src=https://github.com/rmmurphy/arbitros/raw/master/ref/images/arbdeviceTopLevel.png width=600 height=200>

# What Runs _arbitros_?
## Sparkfun XMega100 Breakout, Atmel XMEGA-A1 Xplained, and Primus Development Board

In order to highlight the capability of _arbitros_ a development board named _Primus_ was created. As seen in the picture, _Primus_ evolved from Sparkfun’s XMega100 Breakout board (https://www.sparkfun.com/products/9546) and the architecture was designed using Sparkfun’s design rules and Eagle parts library. Although _Primus_ is a more robust solution (offering PDI debugging, USB console, and a fat32 filesystem) _arbitros_ can still operate on Sparkfun's XMega100 Breakout and Atmel's XMEGA-A1 Xplained (http://www.atmel.com/tools/XMEGA-A1XPLAINED.aspx).

<img src=https://github.com/rmmurphy/arbitros/raw/master/ref/images/primusTopLevel.png>

# What can _arbitros_ do?
## Console with Fat32 File System

As seen in the putty window, _abitros_ offers a rich set of Linux inspired console commands. The first directive ’ls’, outputs a list of all directories and files currently stored on the SD card. In this example, only the 'LOGS' directory with the file 'DMSG.TXT' is present. These particular files are created during system initialization and used by the kernel for logging debug ‘printf’ messages. Looking closely at the terminal output, one can see that directories are traversed in a familiar way via the ‘cd’ command. In order to promote readability, the root directory is highlighted green and all subsequent directories are highlighted red. The third directive ’help’ tells the system to display a list of the sytem commands recognized by the kernel. Finally, the instruction ’dev’, displays a list of device drivers registered with the kernel and their corresponding number of open handles. In this example only the  status (LED controller), console, and SD drivers have been registered.

<img src=https://github.com/rmmurphy/arbitros/raw/master/ref/images/console1.png>

Looking closely at the second figure below, the command ’top’ displays metrics describing the size of various memory sections (.data, .bss, and .heap) including the amount RAM left on the system. Additionally, ’top’ shows an estimate of processor loading, which is comprised of metrics averaged over a one and five minute interval. This feature allows an _abitros_ developer to tune the performance of a particular thread or collection of threads in order to minimize the burden on the system. The instruction ’sdl’ with argument 0,1, or 2 turns on one of the kernel’s three prioritized debug levels. As seen in the example, ’sdl 0’ enables the lowest priority debug level, displaying a message every time the ‘idle’ thread resets the system watchdog timer. Increasing the debug level, such as configuring ’sdl 1’ (or 2), will tell the kernel to print all messages with the same or higher priority. The ability to display debug messages based on priority eases software integration, allowing a developer to "de-clutter" the terminal output by informing the kernel to highlight the most important information. The final command ’head’, tells the kernel to print the contents of one of the files stored on the SD card. As seen in the example, the debug log was displayed ( ‘/logs/dmsg.txt’), showing the results from the previous ‘sdl 0’ debug session.

<img src=https://github.com/rmmurphy/arbitros/raw/master/ref/images/console2.png>

## Attitude Heading Reference System Sensor Fusion

As part of the core ‘DRIVERS’ package, _abitros_ offers a device driver designed to fuse the measurements from Sparkfun’s 9DOF sensor module using a 9-state fixed-point error-state Kalman filter. Unlike other hobbyist sensor fusion systems available on the market , _abitros‘_ Kalman filter was designed using fixed-point rather than floating point notation. This methodology decreases processor loading enabling the algorithm to update at rates in excess of 50msec—vastly improving  the tracking capability of fast maneuvering objects. Moreover, the Kalman filter not only corrects for attitude and heading error, but it corrects gyro  bias and axis misalignment as well. In addition to the nominal filtering operations, the device driver performs magnetometer and accelerometer calibration using a ellipsoid fitting routine. Again, unlike other open source AHRS systems that perform calibration external to the microcontroller (using Matlab); _abitros‘_ AHRS performs the calibration natively using matrix operations from its built in math library (utl_math.c) provided in the ‘UTILITIES’ package. 

<img src=https://github.com/rmmurphy/arbitros/raw/master/ref/images/ahrs.png>

## Interactive AHRS GUI (arhsGui.py)

In order to demonstrate the capability of _abitros’_ sensor fusion system, an interactive graphical user interface was created using a combination of Pyqt and Pyopengl. As seen in the figure, the GUI displays three main views. The first view (‘3D’), shows a 3D orientation of the sensor along with heading (i.e. compass) and roll/pitch instrumentation. The second view (‘XY’), displays real-time plots of the roll, pitch, and yaw. Colors are used to differentiate measurement types  with green representing raw meter readings, yellow representing Kalman filtered readings, and red highlighting the difference between filtered and non-filtered measurements. The final view ‘Calibration’, controls calibration of the magnetometer and accelerometer showing the pre and post calibration data by way of the green and red point cloud.

# Getting started

The examples directory (https://github.com/rmmurphy/arbitros/raw/master/boards/primus/examples) has three folders containing software–with accompanying Atmel Studio 6 projects (http://www.atmel.com/microsite/atmel_studio6) —that demonstrate the capability of the _abitros_ operating system and _Primus_ development board. The first folder, ‘ins’, contains the AHRS software and Python GUI discussed in detail in the section titled, #What can arbitros do?. The second folder, ‘mathEval’, provides software comparing the performance differences between _abitros’_ native math library (utl_math.c) and the standard ANSII mathematics library (math.h). The final folder, ‘primusEval’, provides examples on using various _abitros_ software components such as semaphores, mailbox modules, and peripherals.
