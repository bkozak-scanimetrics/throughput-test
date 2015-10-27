/*
 * Copyright (c) 2015, Scanimetrics - http://www.scanimetrics.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _TPT_H_
#define _TPT_H_

/******************************************************************************
*                                   INCLUDES                                  *
******************************************************************************/
#include "sys/etimer.h"
#include "sys/rtimer.h"
/******************************************************************************
*                                     TYPES                                   *
******************************************************************************/
enum test_type{TCP_TEST, UDP_TEST};
/******************************************************************************
*                                    DEFINES                                  *
******************************************************************************/
#define TPT_SERVER_IPADDR {0xaaaa,0,0,0,0,0,0,1}
#define TPT_SERVER_PORT   3000

//#define SEND_INTERVAL           (CLOCK_SECOND)
#define SEND_INTERVAL           (0)
#define PAYLOAD_LEN             64
#define TPT_PACKET_COUNT        1024

#define TPT_IN_BUFFER_SIZE  256
#define TPT_OUT_BUFFER_SIZE 256

#define TPT_TEST_TYPE     TCP_TEST
#define TPT_DUTY_CYCLE_ON 1

#define TPT_UDP_STOP_RETRIES 4

#define TPT_UDP_PING_PONG              1
#define TPT_PING_PONG_RESPONSE_TIMEOUT (CLOCK_SECOND*2)
#define PING_PONG_RESPONSE_LEN         4
/******************************************************************************
*                               ERROR CHECKING                                *
******************************************************************************/
#if (PING_PONG_RESPONSE_LEN < 4)
#error "PING_PONG_RESPONSE_LEN must be at least 4"
#endif
/******************************************************************************
*                                   FUNCTIONS                                 *
******************************************************************************/
#endif //_TPT_H_
