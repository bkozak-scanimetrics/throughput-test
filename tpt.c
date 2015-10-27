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
/******************************************************************************
* Implements a throughput test                                                *
*                                                                             *
* Binaries built from this project will send a predefined number of           *
* packets to a predefined address at a predefined packet send rate when the   *
* appropriate button is pressed                                               *
*                                                                             *
* Author - Billy Kozak, Scanimetrics                                          *
* Date - Dec. 9th, 2014                                                       *
******************************************************************************/

/******************************************************************************
*                                   INCLUDES                                  *
******************************************************************************/
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/resolv.h"
#include "tcp-socket.h"

#include "net/rime/rime.h"
#include "net/mac/mac.h"

#include "pt.h"

#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "dev/leds.h"

#if CONTIKI_TARGET_SRF06_CC26XX
#include "srf06/button-sensor.h"
#else
#include "dev/button-sensor.h"
#endif

#include "tpt.h"
#include "tpt_util.h"

#include <stdio.h>
#include <stdbool.h>
/******************************************************************************
*                                    DEFINES                                  *
******************************************************************************/
#define HAVE_UDP_SENT_EVENTS 0

#define UIP_IP_BUF ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#if (!HAVE_UDP_SENT_EVENTS)
#define tcpip_udp_sent_event 0
#define tcpip_udp_sent_status tcpip_udp_sent_status_tmp
#define uip_udp_packet_send2(a,b,c,d) uip_udp_packet_send(a,b,c)
#endif

#if (BOARD_SENSORTAG)
#define TPT_START_BUTTON button_sensor
#elif ( CONTIKI_TARGET_CC2538DK || CONTIKI_TARGET_SRF06_CC26XX)
#define TPT_START_BUTTON button_select_sensor
#else
#define TPT_START_BUTTON button_sensor
#endif

#define DEBUG 0

#if defined(DEBUG) && (DEBUG)
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif
/******************************************************************************
*                                    TYPES                                    *
******************************************************************************/
enum testMode{TEST_ON, TEST_OFF};
enum connectState{UNCONNECTED,CONNECT_FAIL,CONNECTED};
enum udp_sent_result{NOT_SENT, SENT_OK, SENT_MAYBE, SENT_BAD};

#if (!HAVE_UDP_SENT_EVENTS)
struct tcpip_udp_sent_status_tmp{int status; struct uip_udp_conn* conn;};
#endif
/******************************************************************************
*                                    GLOBALS                                  *
******************************************************************************/
#if HAVE_UDP_SENT_EVENTS
static struct tcpip_sent udp_sent;
#endif

static struct uip_conn* tcp_connection;

static struct uip_udp_conn* udp_connection;

static struct psock tpt_psock;
static struct pt    tpt_test_pt;

static uint8_t tpt_inBuf[TPT_IN_BUFFER_SIZE];
static uint8_t tpt_outBuf[TPT_OUT_BUFFER_SIZE];

static uint32_t tpt_send_count;

static struct etimer pkt_timer;
static struct etimer response_timer;

static char (*tpt_doTest)(struct pt *,process_event_t,process_data_t);
static void (*tpt_connect_server)(void);
static enum connectState (*tpt_connect_ev)(process_event_t);
/******************************************************************************
*                              FUNCTION PROTOTYPES                            *
******************************************************************************/
static void tpt_fillBuffer(uint32_t word);
static void load_tpt_server_addr(uip_ipaddr_t* addr);
static bool tpt_startTrigger(process_event_t ev,process_data_t data);
static void tpt_config_dutyCycle(enum testMode mode);
static void reset_throttle(void);
static bool throttle_expired(void);
static void setup_throttle(void);
static void tpt_fillBuffer(uint32_t word);

static bool tcp_connection_error(void);
static int  socket_task(struct psock *p);
static char doTest_tcp(struct pt *p,process_event_t ev,process_data_t dat);
static void connect_TCP(void);

static void connect_UDP(void);
static enum connectState udp_connect_ev(process_event_t ev);
static void udp_setup_response_timer(void);
static void udp_reset_response_timer(void);
static bool udp_response_expired(void);
static bool udp_have_response(uint32_t seq);
static enum udp_sent_result udp_acked_or_timeout(
		process_event_t ev,uint32_t seq
	);
static enum udp_sent_result udp_sent_event(
		process_event_t ev,process_data_t data
	);
static enum udp_sent_result udp_send_done(
		process_event_t ev,process_data_t data,uint32_t seq
	);

PROCESS(tpt_process, "tpt process");
AUTOSTART_PROCESSES(&tpt_process);
/******************************************************************************
*                             FUNCTION DEFINITIONS                            *
******************************************************************************/
static void tpt_fillBuffer(uint32_t word){
	int i;
	for(i = 0; i < PAYLOAD_LEN-(PAYLOAD_LEN%4); i+=4){
		tpt_outBuf[i+0] = (word>> 0)&0xFF;
		tpt_outBuf[i+1] = (word>> 8)&0xFF;
		tpt_outBuf[i+2] = (word>>16)&0xFF;
		tpt_outBuf[i+3] = (word>>24)&0xFF;
	}
}
/*****************************************************************************/
static void load_tpt_server_addr(uip_ipaddr_t* addr){
	uint16_t words[8] = TPT_SERVER_IPADDR;
	uip_ip6addr(
		addr,words[0],words[1],words[2],words[3],
		words[4],words[5],words[6],words[7]
	);
}
/*****************************************************************************/
static bool tpt_startTrigger(process_event_t ev,process_data_t data){
	return (ev == sensors_event) && (data == &(TPT_START_BUTTON));
}
/*****************************************************************************/
static void reset_throttle(void){
	if(SEND_INTERVAL && throttle_expired() ){
		etimer_reset(&pkt_timer);
	}
}
/*****************************************************************************/
static bool throttle_expired(void){
	if(!SEND_INTERVAL){
		return true;
	}

	return etimer_expired(&pkt_timer);
}
/*****************************************************************************/
static void setup_throttle(void){
	if(SEND_INTERVAL){
		etimer_set(&pkt_timer, SEND_INTERVAL);
	}
}
/*****************************************************************************/
static void tpt_config_dutyCycle(enum testMode mode){
	if(TPT_DUTY_CYCLE_ON){
		return;
	}

	if(mode == TEST_ON){
		NETSTACK_MAC.off(1);
	} else {
		NETSTACK_MAC.on();
	}
}
/*****************************************************************************/
static enum connectState tcp_connect_ev(process_event_t ev){
	if(ev != tcpip_event){
		return UNCONNECTED;
	}

	return (uip_connected())?(CONNECTED):(CONNECT_FAIL);
}
/*****************************************************************************/
static bool tcp_connection_error(void){
	return (uip_aborted() || uip_timedout() || uip_closed());
}
/*****************************************************************************/
static void connect_TCP(void){
	uip_ipaddr_t serverAddr;

	load_tpt_server_addr(&serverAddr);

	tcp_connection = tcp_connect(
			&serverAddr,UIP_HTONS(TPT_SERVER_PORT),NULL
		);
}
/*****************************************************************************/
static int socket_task(struct psock *p){
	PSOCK_BEGIN(p);

	while(tpt_send_count != TPT_PACKET_COUNT){
		if(tcp_connection_error()){
			PRINTF("connection error!\n");
			break;
		}
		tpt_send_count += 1;
		PSOCK_SEND(p,tpt_outBuf,PAYLOAD_LEN);
		PT_YIELD(&(p->pt));
	}

	PSOCK_CLOSE(p);
	PSOCK_WAIT_UNTIL(p,uip_closed());

	PSOCK_END(p);
}
/*****************************************************************************/
static char doTest_tcp(struct pt *p,process_event_t ev,process_data_t dat){

	PT_BEGIN(p);

	PSOCK_INIT(&tpt_psock, tpt_inBuf, sizeof(tpt_inBuf));
	setup_throttle();
	tpt_send_count = 0;

	do{
		if(ev == tcpip_event){
			char ret = socket_task(&tpt_psock);

			if(ret == PT_WAITING) {
				PT_YIELD(p);
			} else if(ret == PT_ENDED) {
				break;
			} else if(ret == PT_YIELDED) {
				PT_WAIT_UNTIL(p,throttle_expired());

				reset_throttle();

				if(ev != tcpip_event){
					tcpip_poll_tcp(tcp_connection);
				}
			}
		}
		else{
			//ignore this event!
			PT_YIELD(p);
		}

	}while(1);

	PT_END(p);
}
/*****************************************************************************/
static void udp_send_tpt_payload(uint32_t seq){
	tpt_fillBuffer(seq);
	reset_throttle();
	udp_reset_response_timer();

	uip_udp_packet_send2(udp_connection,tpt_outBuf,PAYLOAD_LEN,&udp_sent);
}
/*****************************************************************************/
static void udp_setup_response_timer(void){
	if(TPT_UDP_PING_PONG){
		etimer_set(&response_timer,TPT_PING_PONG_RESPONSE_TIMEOUT);
	}
}
/*****************************************************************************/
static void udp_reset_response_timer(void){
	if(!TPT_UDP_PING_PONG){
		return;
	}

	if(udp_response_expired()){
		etimer_reset(&response_timer);
	} else {
		etimer_restart(&response_timer);
	}
}
/*****************************************************************************/
static bool udp_response_expired(void){
	if(!TPT_UDP_PING_PONG){
		return true;
	}

	return etimer_expired(&response_timer);
}
/*****************************************************************************/
static enum udp_sent_result udp_send_done(
		process_event_t ev,process_data_t data,uint32_t seq
	){
	if(!TPT_UDP_PING_PONG){
		return udp_sent_event(ev,data);
	} else {
		return udp_acked_or_timeout(ev,seq);
	}
}
/*****************************************************************************/
static bool udp_have_response(uint32_t seq){
	uint32_t      resp_seq = 0;
	uint8_t*      appdata;
	uip_ipaddr_t* server;
	uip_ipaddr_t* source;

	if(!uip_newdata() || (uip_datalen() != PING_PONG_RESPONSE_LEN)){
		return false;
	}

	server = &udp_connection->ripaddr;
	source = &UIP_IP_BUF->srcipaddr;

	if( memcmp(server,source,sizeof(*source)) ){
		return false;
	}

	if(PING_PONG_RESPONSE_LEN < 4){
		//this should never be the case...
		PRINTF("bad response length setting\n");
		return false;
	}

	appdata = uip_appdata;

	resp_seq |= (appdata[0] << 0 );
	resp_seq |= (appdata[1] << 8 );
	resp_seq |= (appdata[2] << 16);
	resp_seq |= (appdata[3] << 24);

	return seq == resp_seq;
}
/*****************************************************************************/
static enum udp_sent_result udp_acked_or_timeout(
		process_event_t ev,uint32_t seq
	){

	if( (ev == tcpip_event) && udp_have_response(seq)){
		return SENT_OK;
	}

	if( udp_response_expired() ){
		return SENT_MAYBE;
	}

	return NOT_SENT;
}
/*****************************************************************************/
static enum udp_sent_result udp_sent_event(
		process_event_t ev,process_data_t data
	){
	struct tcpip_udp_sent_status* info = data;

	if(!HAVE_UDP_SENT_EVENTS){
		//todo take this away once we have tcpip_udp_sent_event merged
		return false;
	}

	if(ev != tcpip_udp_sent_event){
		return false;
	}

	if(info->status){
		PRINTF("status %d\n",info->status);
	}

	if(info->conn != udp_connection){
		PRINTF("bad data\n");
	}

	if(info->status == MAC_TX_OK){
		return SENT_MAYBE;
	}

	return SENT_BAD;
}
/*****************************************************************************/
static void connect_UDP(void){
	uip_ipaddr_t addr;

	load_tpt_server_addr(&addr);

	udp_connection = udp_new(&addr, UIP_HTONS(TPT_SERVER_PORT), NULL);
}
/*****************************************************************************/
static enum connectState udp_connect_ev(process_event_t ev){
	return (udp_connection)?(CONNECTED):(CONNECT_FAIL);
}
/*****************************************************************************/
static void udp_send_stop_sequence(void){
	uint8_t stop_buf[] = {0xFF,0xFF,0xFF,0xFF};

	uip_udp_packet_send(udp_connection,stop_buf,sizeof(stop_buf));
}
/*****************************************************************************/
static char doTest_udp(struct pt *p,process_event_t ev,process_data_t dat){

	PT_BEGIN(p);

	setup_throttle();
	udp_setup_response_timer();

	tpt_send_count = 0;

	if(TPT_PACKET_COUNT) {
		udp_send_tpt_payload(tpt_send_count);

		PT_WAIT_UNTIL(
			p,udp_send_done(ev,dat,tpt_send_count) != NOT_SENT
		);
		tpt_send_count += 1;
	}

	while(tpt_send_count != TPT_PACKET_COUNT){
		PT_WAIT_UNTIL(p,throttle_expired());

		tcpip_poll_udp(udp_connection);
		PT_WAIT_UNTIL(p,ev == tcpip_event);

		udp_send_tpt_payload(tpt_send_count);

		PT_WAIT_UNTIL(
			p,udp_send_done(ev,dat,tpt_send_count) != NOT_SENT
		);
		tpt_send_count += 1;
	}

	tpt_send_count = 0;

	while(tpt_send_count != TPT_UDP_STOP_RETRIES){
		enum udp_sent_result res;

		tcpip_poll_udp(udp_connection);
		PT_WAIT_UNTIL(p,ev == tcpip_event);

		udp_send_stop_sequence();

		while(1){
			res = udp_send_done(ev,dat,0xFFFFFFFF);
			if(res != NOT_SENT){
				break;
			} else {
				PT_YIELD(p);
			}
		}

		if(res == SENT_OK){
			break;
		}

		tpt_send_count += 1;
	}

	/* TODO: note sure if this is actually correct... */
	uip_close();

	PT_END(p);
}
/*****************************************************************************/
PROCESS_THREAD(tpt_process, ev, data){

	PROCESS_BEGIN();

	if(TPT_TEST_TYPE == TCP_TEST){
		tpt_connect_server = connect_TCP;
		tpt_connect_ev = tcp_connect_ev;
		tpt_doTest = doTest_tcp;
	}
	else if(TPT_TEST_TYPE == UDP_TEST){
		tpt_connect_server =  connect_UDP;
		tpt_connect_ev = udp_connect_ev;
		tpt_doTest = doTest_udp;
	}

	tpt_setGlobalAddresses();
	print_local_addresses();

	while(1){
		enum connectState state;

		PROCESS_YIELD();

		PROCESS_WAIT_UNTIL(tpt_startTrigger(ev,data));

		PRINTF("tpt start\n");
		tpt_config_dutyCycle(TEST_ON);

		tpt_connect_server();

		while(1){
			state = tpt_connect_ev(ev);
			if(state == UNCONNECTED){
				PROCESS_YIELD();
			} else {
				break;
			}
		}

		if(state ==  CONNECT_FAIL){
			PRINTF("tpt connect failed\n");
			continue;
		}

		PT_INIT(&tpt_test_pt);
		while(tpt_doTest(&tpt_test_pt,ev,data) != PT_ENDED){
				PROCESS_YIELD();
		}

		tpt_config_dutyCycle(TEST_OFF);
		PRINTF("tpt done\n");
	}

	PROCESS_END();
}

