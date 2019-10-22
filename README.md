# StmMultiBoot
Serial bootloader for STM32-based MultiModules.

* Enables flashing MultiModule firmware from OpenTX or erSkyTX
* Uses the official STM32 Arduino core ([Arduino_Core_STM32](https://github.com/stm32duino/Arduino_Core_STM32)) and HAL
* Supports modules based on STM32F103CB and STM32F303CC MCUs
  * Does *not* include USB support for STM32F103CB boards

## Compiling
The bootloader can be compiled in the Arduino IDE.  

* Arduino IDE 1.8.9 or newer is recommended
* Version 1.7.0 or later of the official STM32 core is required

### IDE Configuration
Select the board from the **Tools** -> **Board** menu:
* Select **Generic F1 series** -> **BluePill F103C8 (128K)** to build for the STM32F103
* Select **Generic F3 series** -> **RobotDyn BlackPill F303CC** to build for the STM32F303

To keep the bootloader size below 8KB, HAL serial support must be disabled:
* Select **Disabled (no serial support)** from the **Tools** -> **U(S)ART Support** menu

## Flashing the Bootloader
[ To Do ]


## Using the Bootloader
The bootloader is intended to be used to flash the MultiModule from a compatible radio running erSkyTX or OpenTX.

It can also be tested using AVRDUDE with a USB-to-serial adpater connected to either the serial header pins (RX=`PA10` and TX=`PA9`) or the 'JR' module bay pins (RX=`PA3` and TX=`PB10`) .

* Baud rate should be `57600`
* MCU type is `atmega128` when using the serial header pins

Example:
```
avrdude.exe -p atmega128 -P COM5 -b 57600 -c arduino -V -U flash:w:"C:\Temp\multi-stm-1.2.1.85.bin":a
```

## Credits
This version of the MultiModule bootloader is based on [StmMultiBoot](https://github.com/MikeBland/StmMultiBoot/) and [StmDualBoot](https://github.com/MikeBland/StmDualBoot/) by Mike Blandford, which are in turn based on the [OptiBoot bootloader](https://github.com/Optiboot/optiboot/) for AVR boards.

Akos Pasztor's [STM32 Bootloader](https://github.com/akospasztor/stm32-bootloader/) was also a helpful resource.
