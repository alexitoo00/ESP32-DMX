# ESP32 DMX512

Example code for receiving and transmitting DMX512 with ESP32 and RS485 transceiver IC like MAX485 or isolated ADM2486.

Can be used with Arduino or plain ESP-IDF.

Driver inputs DMX using UART2 on GPIO pin 27 (originally 16). Feel free to experiment with other UARTs and Pins (that's work).
Driver outputs DMX using UART2 on GPIO pin 14 (originally 17). Feel free to experiment with other UARTs and Pins (that's work).
Driver uses pin 4 to change direction of dmx dataflow (it can be desabled by #define, if the hardware
set directly this pin in RX or TX)

/!\ If you plan to use ESP32-WROOVER don't use GPIO 16 & 17 as they are used for internal PSRAM.

Even if it is an Arduino library, it is possible to use the two source files in IDF projects since it has no dependency on Arduino libraries.

Reviewed 
- to ensure a better stability (upadting the state machine of RX and tmp memory, adding a all 0 filter, with blackout detection)
- use of the UART_SCLK_REF_TICK for source clock of the UART
