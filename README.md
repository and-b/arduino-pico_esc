# Controlling ESCs with Raspberry Pi Pico and Arduino IDE

This is the first part of an experiment for a flight controller based on the new Raspberry Pi Pico - RP2040 microcontroller.

This example shows how to control 4 ESCs using PWM, and how to read throttle from a FlySky receiver using IBUS protocol.

## What you need to run this example

- a Raspberry Pi Pico microcontroller
- Arduino IDE version 1.8.x
- Arduino MBED OS RP2040 Boards version 2.1.0 
- FlySky transmitter & receiver with IBUS capabilities (I used FS-I6 and FS-IA6B)
- 4 ESCs and brushless motors (I used a custom quadcopter)

## Connections

You need to connect Pico VSYS to receiver VCC (5V), Pico GND to receiver GND and ESCs GND.

Pin connections are:
- from IBUS signal to GPIO1 - pin2
- from GPIO10 - pin14 to ESC1
- from GPIO11 - pin15 to ESC2
- from GPIO12 - pin16 to ESC3
- from GPIO13 - pin17 to ESC4


## Decoding IBUS

The IBUS protocol has a buffer of 32 bytes, the sequence is:
- length: 0x20
- protocol: 0x40
- channel1 LSB+MSB
- ......
- channel14 LSB+MSB
- checksum LSB+MSB

The sum of all the 30 bytes + the checksum sent in bytes 31 & 32 must be 0xFFFF

## Setting PWM properties

In this example, I want a simple PWM signal (1000ms -> 2000ms) sent to the ESCs. Frequency is 490Hz, period is 2040 micros.

The steps we need to setup the PWM are:
- set the 4 pins to GPIO_FUNC_PWM
- set the clock divider to 125, so the PWM is counting 1 every 1 microsecond instead of every 8 nanoseconds
- set the counter (TOP register) to 2039, in order to have a signal with a period of 2040 microseconds

This way, the duty cycle of the PWM is controlled by the CC register, the value of the register is the amplitude in microseconds.

