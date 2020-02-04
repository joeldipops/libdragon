/**
 * @file siinternal.c
 * @brief Low-level controller communication
 * @ingroup controller
 */
#include <stdint.h>
#include "controller.h"

/** The following is based on libjoy64 https://github.com/fraser125/libjoy64/ and is licensed as follows. **/

/**
BSD 3-Clause License

Copyright (c) 2018, fraser
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define MAX_CONTROLLERS 4
#define RDRAM_DIRECT_MASK   0x1FFFFF

#define SI_GSTAT_INIT           0x00
#define SI_GSTAT_BACKGR         0x01
#define SI_GSTAT_ACC_CHANGED    0x08
#define SI_GSTAT_ACC_ON         0x80 // Rumble / Pulse

#define SI_STATUS_DMA_BUSY      (1 << 0)
#define SI_STATUS_IO_BUSY       (1 << 1)
#define SI_STATUS_DMA_ERROR     (1 << 3)
#define SI_STATUS_BUSY_ERROR_6  (1 << 6) // Not sure but they would flip high while waiting for PIF
#define SI_STATUS_BUSY_ERROR_8  (1 << 8) // Not sure but they would flip high while waiting for PIF
#define SI_STATUS_INTERRUPT     (1 << 12)

#define PIF_RAM_ADDR    0x1FC007C0

typedef struct si_console_t {
    uint16_t state;
    uint8_t eeprom;
    uint8_t portcount;
    _SI_condat port[MAX_CONTROLLERS];
} _si_console;

typedef struct si_request_t {
    union
    {
        uint8_t bytes[64];
        uint16_t shorts[32];
        uint32_t uints[16];
    } raw;
} _si_request_t __attribute__ ((aligned (16)));

static _si_request_t si_buffer_tx __attribute__((aligned (16)));
static _si_request_t si_buffer_rx __attribute__((aligned (16)));
static _si_console si_console;
static uint32_t si_buffer_tx_ram_addr;
static uint32_t si_buffer_rx_ram_addr;
uint32_t si_controller_background_index;

/**
 * @brief
 * @param[in] port_enable_count
 *          Number of controllers to enable.
 * @param[in] enable_background 
 *      Enable the easy use of si accessories
 *      Example:
 *          Identify Controller Accessories
 *          Enable/Disable Rumble
 *          Get Pulse Sensor Readings
 *          Read the Table of Contents for Memory PAK's
 *          Read the Header of Gameboy Carts
 *          Initialize the VRU/VRS, except Word List
 *          If you are using any of these features this will make your work easier.
 *          The processing time taken for these actions is very minimal
 */
void si_init(uint32_t port_enable_count, uint32_t enable_background)
{
    if (port_enable_count > MAX_CONTROLLERS)
        port_enable_count = 1;

    si_console.portcount = port_enable_count;
    
    if (enable_background)
        si_console.state = SI_GSTAT_BACKGR;
    else
        si_console.state = SI_GSTAT_INIT;
    
    si_buffer_tx_ram_addr = (((uint32_t)&si_buffer_tx) & RDRAM_DIRECT_MASK);
    si_buffer_rx_ram_addr = (((uint32_t)&si_buffer_rx) & RDRAM_DIRECT_MASK);
    si_controller_background_index = 0;
    si_init_tx_buffer();
    si_init_devices();
}

void si_init_tx_buffer()
{
    for(uint32_t idx = 0; idx < 16; idx++)
        si_buffer_tx.raw.uints[idx] = 0x0;
}

void si_init_devices()
{
    for(uint32_t idx = 0;idx < MAX_CONTROLLERS; idx++)
    {
        si_console.port[idx].device.raw = 0x0;
        si_console.port[idx].data.raw[0] = 0x0;
        si_console.port[idx].data.raw[1] = 0x0;
    }
}

void si_status_zero_wait()
{
    uint32_t status = 0;
    do
    {
        status = si_status();
    }
    while (status > 0);
}

void si_status_dma_io_wait()
{
    uint32_t status = 0;
    do
    {
        status = (si_status() & (SI_STATUS_IO_BUSY | SI_STATUS_DMA_BUSY));
    }
    while (status > 0);
}

void si_send_request(uint32_t buffer_addr)
{
    si_set_dram_addr(buffer_addr);
    si_dma_write(PIF_RAM_ADDR);
}

void si_recv_response(uint32_t buffer_addr)
{
    si_set_dram_addr(buffer_addr);
    si_dma_read(PIF_RAM_ADDR);
}
