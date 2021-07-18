
![alt text](https://embeddedproto.com/wp-content/uploads/2020/03/Embedded-Proto-e1583834233386.png "Embedded Proto Logo")


Copyrights 2020 Embedded AMS B.V. Amsterdam, [www.EmbeddedAMS.nl](https://www.EmbeddedAMS.nl), [info@EmbeddedAMS.nl](mailto:info@EmbeddedAMS.nl)


Looking for a more elaborate description of this example? Please visit: https://embeddedproto.com/sensor-example-with-embedded-proto/


# Introduction

This repository hosts example code for Embedded Proto, the embedded implementation of Google Protocol Buffers. It is a simple example showing how a microcontroller and desktop pc can communicate over UART. Command messages are send from a desktop script over an UART comport to the MCU. 

![alt text](https://embeddedproto.com/wp-content/uploads/2020/05/PC_to_MCU_over_UART.png "PC to MCU over UART")

This example makes use of a FRDM-KE02Z board made by NXP. This board holds an ARM Cortex-M0+ processor. To build the source code and program the hardware MCUXpresso has been used. 

The desktop program is a simple terminal python script. This python script let you send commands from a PC to the MCU. 
You can send the following commands:
1. Pressing 'a' gets the accelerometer data.
2. Pressing 't' gets the thermistor data.
3. Pressing 'l' let you turn on/off the red, green and blue channel from the RGB led.


# Installation

1. Install MCUXpresso if you have not already.
2. Install the dependencies required by Embedded Proto. They are listed [here](https://github.com/Embedded-AMS/EmbeddedProto).
3. Checkout this example repository including the submodule of Embedded Proto: `git clone --recursive https://github.com/Embedded-AMS/EmbeddedProto_Example_FRDM_sensors.git`.
4. Setup the environment required for Embedded Proto and the desktop script by running the setup script: `./setup.sh`.

The setup script already does it for you but you can regenerate the source code using the `./generate_source_files.sh` script. This is required when you have changed the \*.proto file.


# Running the code

Connect the FRDM-KE02Z via the usb programmer and use MCUXpresso to build and program the microcontroller. Next find out which comport has been allocated for the FRDM-KE02Z. In the example code below it was ttyACM0. Next go to the desktop folder, activate the virtual environment and run the script. 

```bash
cd desktop
source venv/bin/activate
python3 main.py --com /dev/ttyACM0
```

Have fun!
