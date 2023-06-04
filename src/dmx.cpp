/* 
 * This file is part of the ESP32-DMX distribution (https://github.com/luksal/ESP32-DMX).
 * Copyright (c) 2021 Lukas Salomon.
 * 
 * Reviewed by Yoann Darche 2022-2023 (https://github.com/yoann-darche/ESP32-DMX)
 * to ensure a better stability (upadting the state machine of RX and tmp memory)
 * used the UART_SCLK_REF_TICK for source clock of the UART
 * - Adding a filter when all data are 0 (possibility of UART sync error), with blackout detection
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

#include <Arduino.h>
#include "driver/gpio.h"
#include "hal/uart_types.h"
#include <dmx.h>

/*
#define DMX_SERIAL_INPUT_PIN    GPIO_NUM_16 // pin for dmx rx
#define DMX_SERIAL_OUTPUT_PIN   GPIO_NUM_17 // pin for dmx tx
*/

#define DMX_SERIAL_INPUT_PIN    GPIO_NUM_27 // pin for dmx rx
#define DMX_SERIAL_OUTPUT_PIN   GPIO_NUM_14 // pin for dmx tx


#define DMXHW_DONT_USE_DIR      1           // Disable the Use of direction pin (in case of receive or send defined by hardware)

#define DMX_SERIAL_IO_PIN       GPIO_NUM_4  // pin for dmx rx/tx change



#define DMX_UART_NUM            UART_NUM_2  // dmx uart

#define HEALTHY_TIME            500         // timeout in ms 

#define BUF_SIZE                513         //  buffer size for rx events (il y a 513 trames en DMX512)

#define DMX_CORE                1           // select the core the rx/tx thread should run on

//#define DMX_IGNORE_THREADSAFETY 0         // set to 1 to disable all threadsafe mechanisms



#define NBZEROFRAME_TRIGGER_BLACKOUT    12  // floor for black out detection , nb successive zeros frame.

QueueHandle_t DMX::dmx_rx_queue;

SemaphoreHandle_t DMX::sync_dmx;

DMXState DMX::dmx_state = DMX_IDLE;


uint16_t DMX::_StartDMXAddr = 1;
uint16_t DMX::_NbChannels = 512;
uint8_t* DMX::dmx_data = nullptr;
uint8_t* DMX::tmp_dmx_data = nullptr;

uint16_t DMX::current_rx_addr = 0;

// Filter on Zéros DMX Frame
bool DMX::isAllZero = true;
uint8_t DMX::CptAllZeroFrame = 0;

long DMX::last_dmx_packet = 0;

DMX::DMX()
{
    dmx_data = nullptr;
    tmp_dmx_data = nullptr;
    isAllZero = true;
}

DMX::~DMX()
{
    if(dmx_data != nullptr) {
        free(dmx_data);
    }

    if(tmp_dmx_data != nullptr) {
        free(tmp_dmx_data);
    }
}


void DMX::Initialize(DMXDirection direction, uint16_t StartAddr, uint16_t NbChannels)
{
    // configure UART for DMX
    uart_config_t uart_config =
    {
        .baud_rate = 250000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_REF_TICK                // Better source clock for ESP32 in my case
    };

    uart_param_config(DMX_UART_NUM, &uart_config);

    // Set pins for UART
    if ( uart_set_pin(DMX_UART_NUM, DMX_SERIAL_OUTPUT_PIN, DMX_SERIAL_INPUT_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)!= ESP_OK ) {
        Serial.printf("DMX::Initialize : Error when assigning UART Pin (uart_set_pin). ESP_FAIL!\n");
    }
    // install queue
    if ( uart_driver_install(DMX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &dmx_rx_queue, 0) != ESP_OK ) {
        Serial.printf("DMX::Initialize : Error when installaing the UART Driver. ESP_FAIL!\n");
    }

    // Check if the queue has correctly created
    if(&dmx_rx_queue == nullptr) {
        Serial.printf("DMX::Initialize : Error when installaing the UART Driver. Queue pointure is NULL!\n");
    }

    // create mutex for syncronisation
    sync_dmx = xSemaphoreCreateMutex();

    // set gpio for direction

#ifndef DMXHW_DONT_USE_DIR
    gpio_pad_select_gpio(DMX_SERIAL_IO_PIN);
    gpio_set_direction(DMX_SERIAL_IO_PIN, GPIO_MODE_OUTPUT);
#endif

    // depending on parameter set gpio for direction change and start rx or tx thread
    if(direction == DMX_DIR_OUTPUT)
    {

        // Default value for back compability to the orrignal librairie code
        _StartDMXAddr = 1;
        _NbChannels = 512;
        createBuffer(false);

#ifndef DMXHW_DONT_USE_DIR
    gpio_set_level(DMX_SERIAL_IO_PIN, 1);
#endif
        
        dmx_state = DMX_OUTPUT;
        
        // create send task
        xTaskCreatePinnedToCore(DMX::uart_send_task, "uart_send_task", 1024, NULL, 1, NULL, DMX_CORE);
    }
    else
    {    
        _StartDMXAddr = StartAddr;
        _NbChannels = NbChannels;
        createBuffer(true);

#ifndef DMXHW_DONT_USE_DIR
    gpio_set_level(DMX_SERIAL_IO_PIN, 0);
#endif    

        dmx_state = DMX_IDLE;

        // create receive task
        xTaskCreatePinnedToCore(DMX::uart_event_task, "uart_event_task", 2048, NULL, 1, NULL, DMX_CORE);
    }
}


void DMX::createBuffer(bool TmpBufferCreate) {

    // Manage deletion of the Tmp Buffeur if not any more needed 
    if(not(TmpBufferCreate) && (tmp_dmx_data != nullptr)) {
        free(tmp_dmx_data);
        tmp_dmx_data = nullptr;
    }

    // Check if buffer is already existing
    if(dmx_data != nullptr) {
        dmx_data = (uint8_t *) realloc(dmx_data, sizeof(uint8_t)*(_NbChannels+1));
    } else {
         dmx_data = (uint8_t *) malloc(sizeof(uint8_t)*(_NbChannels+1));
    }

    memset(dmx_data,0,sizeof(uint8_t)*(_NbChannels+1));

    // Manage the temporrary buffer if needed
    if(TmpBufferCreate) {
        if(tmp_dmx_data != nullptr) {
            tmp_dmx_data = (uint8_t *) realloc(tmp_dmx_data, sizeof(uint8_t)*(_NbChannels+1));
        } else {
            tmp_dmx_data = (uint8_t *) malloc(sizeof(uint8_t)*(_NbChannels+1));
        }
    }

}


//*****************************************************************************
//** Update the listenning startAddress and NbChannel => with buffer realloc **
//*****************************************************************************
void DMX::SetDmxStartAdress(uint16_t StartAddr) {

    // if we are writting on the DMX Bus, all channels are needed
    if(dmx_state == DMX_OUTPUT) return;

    if(StartAddr == _StartDMXAddr) return;

    if((StartAddr == 0) || (StartAddr > 512)) return;

    if(StartAddr+_NbChannels > 513) return;

#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    _StartDMXAddr = StartAddr;
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif

}

void DMX::SetDmxNbChannels(uint16_t nb) {

    // if we are writting on the DMX Bus, all channels are needed
    if(dmx_state == DMX_OUTPUT) return;

    if(nb == _NbChannels) return;

    if((nb == 0) || (nb > 512)) return;

    if(_StartDMXAddr+nb > 513) return;

#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    _NbChannels = nb;
    createBuffer(true);
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
}

uint8_t DMX::Read(uint16_t channel)
{
    // restrict acces to dmx array to valid values
    if(channel < 1 || channel > _NbChannels)
    {
        return 0;
    }

    // take data threadsafe from array and return
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    uint8_t tmp_dmx = dmx_data[channel];
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
    return tmp_dmx;
}

void DMX::ReadAll(uint8_t * data, uint16_t start, size_t size)
{
    // restrict acces to dmx array to valid values
    if(start < 1 || start > _NbChannels || start + size > (_NbChannels+1))
    {
        return;
    }
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    memcpy(data, (uint8_t *)dmx_data + start, size);
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
}

void DMX::Write(uint16_t channel, uint8_t value)
{
    // restrict acces to dmx array to valid values
    if(channel < 1 || channel > 512)
    {
        return;
    }

#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    dmx_data[channel] = value;
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
}

void DMX::WriteAll(uint8_t * data, uint16_t start, size_t size)
{
    // restrict acces to dmx array to valid values
    if(start < 1 || start > 512 || start + size > 513)
    {
        return;
    }
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    memcpy((uint8_t *)dmx_data + start, data, size);
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
}

uint8_t DMX::IsHealthy()
{
    // get timestamp of last received packet
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    long dmx_timeout = last_dmx_packet;
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
    // Serial.printf("DMX_IsHealthy: %d\n",dmx_timeout);
    // check if elapsed time < defined timeout
    if(xTaskGetTickCount() - dmx_timeout < HEALTHY_TIME)
    {
        return 1;
    }
    return 0;
}

void DMX::uart_send_task(void*pvParameters)
{
    uint8_t start_code = 0x00;
    for(;;)
    {
        // wait till uart is ready
        uart_wait_tx_done(DMX_UART_NUM, 1000);
        // set line to inverse, creates break signal
        uart_set_line_inverse(DMX_UART_NUM, UART_SIGNAL_TXD_INV);
        // wait break time
        ets_delay_us(184);
        // disable break signal
        uart_set_line_inverse(DMX_UART_NUM,  0);
        // wait mark after break
        ets_delay_us(24);
        // write start code
        uart_write_bytes(DMX_UART_NUM, (const char*) &start_code, 1);
#ifndef DMX_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
        // transmit the dmx data
        uart_write_bytes(DMX_UART_NUM, (const char*) dmx_data+1, 512);
#ifndef DMX_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_dmx);
#endif
    }
}

void DMX::uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
    size_t data_size = 0;

    Serial.print("DMX::uart_event_task::Started\n");

    for(;;)
    {
        // wait for data in the dmx_queue
        if(xQueueReceive(dmx_rx_queue, (void * )&event, (portTickType)portMAX_DELAY))
        {

            //Serial.printf("DMX::uart_event_task::xQueueReceive (code %d)\n",event.type);

            bzero(dtmp, BUF_SIZE);
            switch(event.type)
            {
                case UART_DATA:
                    // read the received data
                    uart_read_bytes(DMX_UART_NUM, dtmp, event.size, portMAX_DELAY);

                    //Serial.print("DMX_UART_DATA\n");

                    // check if break detected
                    if(dmx_state == DMX_BREAK)
                    {
                        // if not 0, then RDM or custom protocol
                      if(dtmp[0] == 0)
                        {
                        dmx_state = DMX_DATA;
                        
                        // reset dmx adress to 0
                        current_rx_addr = 0;
#ifndef DMX_IGNORE_THREADSAFETY
                        xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
                        // store received timestamp
                        last_dmx_packet = xTaskGetTickCount();
#ifndef DMX_IGNORE_THREADSAFETY
                        xSemaphoreGive(sync_dmx);
#endif
                       }
                       else {
                        dmx_state = DMX_IDLE;
                       }
                    }
                    // check if in data receive mode
                    if(dmx_state == DMX_DATA)
                    {
#ifndef DMX_IGNORE_THREADSAFETY
                        xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif

                        // copy received bytes to dmx data array
                        for(int i = 0; i < event.size; i++)
                        {
                            if(current_rx_addr < 513)
                            {
                                isAllZero = isAllZero && (dtmp[i] == 0);
                                //tmp_dmx_data[current_rx_addr++] = dtmp[i];
                                if((current_rx_addr >= _StartDMXAddr) && 
                                   (current_rx_addr <  (_StartDMXAddr+_NbChannels)) ) {

                                    tmp_dmx_data[current_rx_addr-_StartDMXAddr+1] = dtmp[i];
                                }

                                current_rx_addr++;
                                //Serial.print(current_rx_addr);
                                //Serial.print(" = ");

                                if(current_rx_addr == 513)  {
                                    dmx_state = DMX_DONE; 
                                    //Serial.print("READ DMX_DONE\n");
                                }
                            } else {
                                dmx_state = DMX_DONE;    
                                //Serial.print("DMX_DONE\n");                            
                            }
                        }
#ifndef DMX_IGNORE_THREADSAFETY
                        xSemaphoreGive(sync_dmx);
#endif
                    }
                    break;
                case UART_BREAK:
                    // break detected
                    // clear queue und flush received bytes  
                    //Serial.print("DMX_RX::DMX_BREAK>"); 
                    if((dmx_state == DMX_DONE) || (dmx_state == DMX_DATA))  { 
                        uart_flush_input(DMX_UART_NUM);
                        xQueueReset(dmx_rx_queue);
                        //Serial.print("state = (DMX_DONE ou DMX_DATA) > Change a DMX_BREAK\n");
                        dmx_state = DMX_BREAK;
                        //Serial.print("Last Chan :");
                       // Serial.println(current_rx_addr-1);

                        if (!isAllZero) {

                        
#ifndef DMX_IGNORE_THREADSAFETY
                            xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif                        
                            memcpy((uint8_t *)dmx_data, tmp_dmx_data, _NbChannels+1);
#ifndef DMX_IGNORE_THREADSAFETY
                            xSemaphoreGive(sync_dmx);                 
#endif                        
                            isAllZero = true;    
                            CptAllZeroFrame = 0;   
                        }
                        else {
                            
                            CptAllZeroFrame++;

                            if(CptAllZeroFrame >= NBZEROFRAME_TRIGGER_BLACKOUT) {
#ifndef DMX_IGNORE_THREADSAFETY
                                xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif                                                        
                                memset((uint8_t *)dmx_data, 0, _NbChannels+1);
#ifndef DMX_IGNORE_THREADSAFETY
                                xSemaphoreGive(sync_dmx);                 
#endif  
                                CptAllZeroFrame = 0; 
                            } 
                            else {
                                if (CptAllZeroFrame == 1) Serial.print("DMX_RX::DMX_BREAK > state = (DMX_DONE/DATA) && Trame ALL Zero !\n"); 
                            }


                            isAllZero = true;
                        }
                       // Serial.print("UART_BREAK");
                    }  else if (dmx_state == DMX_IDLE) {
                        uart_flush_input(DMX_UART_NUM);
                        xQueueReset(dmx_rx_queue);                        
                        dmx_state = DMX_BREAK;    
                        Serial.print("DMX_RX::DMX_BREAK > state = (DMX_IDLE) > Change a DMX_BREAK\n");                    
                    } else {
                        Serial.print("DMX_RX::DMX_BREAK > state = (xxxx) > Change a DMX_IDLE\n");
                        uart_flush_input(DMX_UART_NUM);
                        xQueueReset(dmx_rx_queue);
                        dmx_state = DMX_IDLE;
                    }

                    break;
                case UART_FRAME_ERR:
                case UART_PARITY_ERR:
                case UART_BUFFER_FULL:
                case UART_FIFO_OVF:
                default:
                    // error recevied, going to idle mode
                    uart_flush_input(DMX_UART_NUM);
                    xQueueReset(dmx_rx_queue);
                    dmx_state = DMX_IDLE;
                    Serial.print("DMX::uart_event_task::UART_ERROR");
                    break;
            }
        }
    }
}
