/* 
 * This file is part of the ESP32-DMX distribution (https://github.com/luksal/ESP32-DMX).
 * Copyright (c) 2021 Lukas Salomon.
 * 
 * Reviewed by Yoann Darche 2022-2023 (https://github.com/yoann-darche/ESP32-DMX)
 * to ensure a better stability (upadting the state machine of RX and tmp memory)
 * used the UART_SCLK_REF_TICK for source clock of the UART
 * - Adding a filter when all data are 0 (possibility of UART sync error) with blackout detection
 * - Adding a define to disable the I/O Dir pin (in case the direction is defined directly with the hardware)
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

#ifndef DMX_h
#define DMX_h


enum DMXDirection { DMX_DIR_INPUT, DMX_DIR_OUTPUT };
enum DMXState { DMX_IDLE, DMX_BREAK, DMX_DATA,DMX_DONE, DMX_OUTPUT };

class DMX
{
    public:
        static void Initialize(DMXDirection direction, 
                               uint16_t StartAddr=1, uint16_t NbChannels=512);    // initialize library

        static void SetDmxStartAdress(uint16_t StartAddr);
        static void SetDmxNbChannels(uint16_t nb);

        static uint8_t Read(uint16_t channel);              // returns the dmx value for the givven address (values from 1 to 512)

        static void ReadAll(uint8_t * data, uint16_t start, size_t size);   // copies the defined channels from the read buffer

        static void Write(uint16_t channel, uint8_t value); // writes the dmx value to the buffer
        
        static void WriteAll(uint8_t * data, uint16_t start, size_t size);  // copies the defined channels into the write buffer

        static uint8_t IsHealthy();                            // returns true, when a valid DMX signal was received within the last 500ms
        
    private:
        DMX();                                              // hide constructor
        ~DMX();                                             // hide destructor

        static uint16_t _StartDMXAddr;                      // First adress liestend
        static uint16_t _NbChannels;                        // Number of channels listened from the start address

        static QueueHandle_t  dmx_rx_queue;                  // queue for uart rx events
        
        static SemaphoreHandle_t sync_dmx;                  // semaphore for syncronising access to dmx array

        static DMXState dmx_state;                           // status, in which recevied state we are

        static uint16_t current_rx_addr;                    // last received dmx channel

        static long last_dmx_packet;                        // timestamp for the last received packet

        static uint8_t * dmx_data;                          // stores the validated dmx data
        static uint8_t * tmp_dmx_data;                      // stores the received dmx data



        // Filter on Zéros DMX Frame
        static bool isAllZero;                              // indicate when a zéro frame is detected
        static uint8_t CptAllZeroFrame;                     // store the number of successive zéro frame detected, if more than 12 ==> Blackout


        static void uart_event_task(void *pvParameters);    // event task

        static void uart_send_task(void*pvParameters);      // transmit task

        static void createBuffer(bool TmpBufferCreate=false);    // Manage the allocation/reallocation of buffer
};

#endif