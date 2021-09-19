# RC Input to I2C for Arduinos

This project aims to use Arduino variant, mostly Arduino Pro Mini and Arduino Pro Micro, to read RC(Radio Control) signals and transmit it over I2C protocol, which then can be used by other microcontrollers or single board computers like Raspberry Pi.

## Hardware Requirement
- Arduino Pro Mini(3.3V or 5V)
- An I2C master computer/Arduino
- Level shifter x 2, depending on the signal's source, the master device and the MCU.

## Hardware Setup
- Wire RC receiver's servo/steering and throttle/motor signal into a level shifter, so you get a proper voltage to the MCU otherwise you are risking frying the chip. Unless you are absolutely sure the RC receiver outputs 3.3V signals which is acceptable for most MCU nowadays.
- From the level shifter, wire the converted servo/steering and throttle/motor into MCU's pin 2 and pin 3 respectively
- From the MCU, wire SCL and SDA to I2C master. For Arduino Pro Mini the I2C pins are 19(SCL) and 18(SDA). The master can be other Arduino or computers. Mind the voltage difference you might need to wire through another level shifter.

## LED Behaviours
- When the microcontroller successfully reads the RC signals, the onboard LED blinks with frequencies corresponding to the signal's PWM width. For now, it's the steering signal affects the LED blinking.

## I2C protocol
Super simple protocol over I2C, 6 bytes in one transmission. Refer to the implementation of onRequestData().
```
-------- -------- -------- -------- -------- --------
|  01   |   16bit_servo   |   02   |  16bit_throttle |
-------- -------- -------- -------- -------- --------
```

## Requirements
- ServoInput library(https://github.com/dmadison/ServoInput)