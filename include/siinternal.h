/**
 * @file siinternal.h
 * @brief Low-level controller communication
 * @ingroup controller
 */
#ifndef __LIBDRAGON_SI_H
#define __LIBDRAGON_SI_H

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

void si_send_request(uint32_t buffer_addr);
void si_recv_response(uint32_t buffer_addr);

__attribute__((always_inline))
static inline uint32_t si_status(void) {
    return *(volatile const uint32_t *) 0xA4800018;
}

__attribute__((always_inline))
static inline void si_set_dram_addr(uint32_t buffer_addr) {
    *(volatile uint32_t *) 0xA4800000 = buffer_addr;
}

__attribute__((always_inline))
static inline void si_dma_read(uint32_t addr) {
    *(volatile uint32_t *) 0xA4800004 = addr;
}

__attribute__((always_inline))
static inline void si_dma_write(uint32_t addr) {
    *(volatile uint32_t *) 0xA4800010 = addr;
}
#endif