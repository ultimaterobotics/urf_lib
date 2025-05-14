#include <stdint.h>
#include "urf_radio.h"
#include "urf_timer.h"
#include "nrf.h"

#define STAR_MAX_NODES 64
#define STAR_NODE_PAYLOAD_SIZE 8

int role_central = 0;
int cycle_step_mcs = 2500;
uint8_t cur_out_pack[256];
uint8_t in_pack[256];
uint8_t buf_pack[256]; //in_pack has the most recent radio packet, and it's copied into buf_pack if it matches protocol
int cur_pack_len = 0;
int buf_pack_len = 0;
int star_has_new_packet = 0;

uint32_t star_unit_id = 0x11223344;

//central unit variables
int cycle_phase = 0; //0 - accepting new nodes, +1 phase for each known node
int active_nodes = 0; //nodes that didn't respond for a while are removed from list
uint32_t node_list[STAR_MAX_NODES]; //IDs of active units
uint32_t node_active[STAR_MAX_NODES]; //last activity in milliseconds
uint8_t node_pack_id[STAR_MAX_NODES];
uint8_t node_payload[STAR_MAX_NODES*STAR_NODE_PAYLOAD_SIZE];
uint32_t last_phase_start = 0;
int active_timeout = 30000; //when consider unit not responsive and stop requesting

//node unit variables
uint32_t last_sync_time = 0;
uint32_t last_asked_time = 0;
uint32_t no_ask_timeout = 3000; //when to consider that base is unaware of the unit and send registration request
uint32_t no_sync_timeout = 10000; //when to consider that base doesn't use star protocol and just send data
uint32_t last_pack_sent = 0;
uint32_t nonstar_send_interval_mcs = 3000;
uint32_t last_star_sent_ms = 0;

void star_init(int channel, int speed, int phase_length_mcs, int is_central)
{
	rf_init(channel, speed, 3);
	cycle_step_mcs = phase_length_mcs;
	role_central = is_central;
	rf_listen();
	NRF_RNG->SHORTS = 0;
	NRF_RNG->CONFIG = 1;
	NRF_RNG->TASKS_START = 1;
}

void star_set_id(uint32_t id)
{
	star_unit_id = id;
}

void star_send_to_node(uint32_t node_id, uint8_t *payload_8b)
{
	int idx = -1;
	for(int x = 0; x < active_nodes; x++)
		if(node_list[x] == node_id)
		{
			idx = x;
			break;
		}
	if(idx < 0) return;
	for(int x = 0; x < STAR_NODE_PAYLOAD_SIZE; x++)
	{
		node_payload[STAR_NODE_PAYLOAD_SIZE*idx+x] = payload_8b[x];
	}
}

void star_queue_send(uint8_t *pack, int length)
{
	if(role_central)
	{
		for(int x = 0; x < length; x++)
			cur_out_pack[16+x] = pack[x];
		cur_pack_len = 16+length;
	}
	else
	{
		for(int x = 0; x < length; x++)
			cur_out_pack[x] = pack[x];
		cur_pack_len = length;
	}
}

int star_has_packet()
{
	return star_has_new_packet;
}

int star_get_packet(uint8_t *pack, int max_length) //returns actual length
{
	star_has_new_packet = 0;
	int res_length = buf_pack_len;
	if(res_length > max_length) res_length = max_length;
	for(int x = 0; x < res_length; x++)
		pack[x] = buf_pack[x];
	return buf_pack_len;
}

void add_node(uint32_t id)
{
//	uprintf("adding id %08X\n", id);
	if(active_nodes >= STAR_MAX_NODES) return;
	for(int x = 0; x < active_nodes; x++)
		if(node_list[x] == id) return;
	node_list[active_nodes] = id;
	node_active[active_nodes] = millis();
	active_nodes++;
//	uprintf("...added, nodes %d\n", active_nodes);
}

void update_node_list()
{
	uint32_t ms = millis();
	for(int x = 0; x < active_nodes; x++)
	{
		if(ms - node_active[x] > active_timeout)
		{
			for(int x2 = x+1; x2 < active_nodes; x2++)
			{
				node_list[x2-1] = node_list[x2];
				node_active[x2-1] = node_active[x2];
			}
			active_nodes--;
		}
	}
}

void loop_central()
{
	if(rf_has_new_packet())
	{
		int len = rf_get_packet(in_pack);
		int is_good = NRF_RADIO->CRCSTATUS;
		if(len < 6) is_good = 0; //unexpected packet, not what we are looking for
		
		if(is_good)
		{
			uint32_t rx_id = in_pack[2]<<24;
			rx_id |= in_pack[3]<<16;
			rx_id |= in_pack[4]<<8;
			rx_id |= in_pack[5];
			if(cycle_phase == 0)
			{
				if(rx_id != 0)
					add_node(rx_id);
				else
					;
			}
			else if(rx_id == node_list[cycle_phase-1])
			{
				node_active[cycle_phase-1] = millis();
				star_has_new_packet = 1;
				buf_pack_len = len;
				for(int x = 0; x < len; x++)
					buf_pack[x] = in_pack[x];
				node_pack_id[cycle_phase-1] = in_pack[0];
			}
		}
	}
	uint32_t mcs = micros();
	if(mcs - last_phase_start < cycle_step_mcs && mcs > last_phase_start) //second part for overflowing uint32
		return;

	last_phase_start = mcs;
	cycle_phase++;
	if(cycle_phase > active_nodes) cycle_phase = 0;

	if(cycle_phase == 0) //enumeration
	{
		update_node_list();
		cur_out_pack[0] = 0; //id, 0 suggests enum
		cur_out_pack[1] = 16; //length
		if(cur_pack_len > 0) cur_out_pack[1] += cur_pack_len;
		cur_out_pack[2] = 0; //4-bytes ID, zeros indicate enum
		cur_out_pack[3] = 0;
		cur_out_pack[4] = 0;
		cur_out_pack[5] = 0;
		cur_out_pack[6] = 1; //protocol version
		cur_out_pack[7] = active_nodes;
		cur_out_pack[8] = (cycle_step_mcs>>24)&0xFF;
		cur_out_pack[9] = (cycle_step_mcs>>16)&0xFF;
		cur_out_pack[10] = (cycle_step_mcs>>8)&0xFF;
		cur_out_pack[11] = cycle_step_mcs&0xFF;
		uint32_t ms = millis();
		cur_out_pack[12] = ms>>24;
		cur_out_pack[13] = ms>>16;
		cur_out_pack[14] = ms>>8;
		cur_out_pack[15] = ms;
	}
	else 
	{
		cur_out_pack[0] = 10 + cycle_phase; //id
		cur_out_pack[1] = 16; //length
		if(cur_pack_len > 0) cur_out_pack[1] += cur_pack_len;
		uint32_t tx_id = node_list[cycle_phase-1];
		cur_out_pack[2] = (tx_id>>24)&0xFF;
		cur_out_pack[3] = (tx_id>>16)&0xFF;
		cur_out_pack[4] = (tx_id>>8)&0xFF;
		cur_out_pack[5] = tx_id&0xFF;
		cur_out_pack[6] = 1; //protocol version
		cur_out_pack[7] = active_nodes;
//		cur_out_pack[8] = node_pack_id[cycle_phase-1];
		for(int x = 0; x < STAR_NODE_PAYLOAD_SIZE; x++)
			cur_out_pack[8+x] = node_payload[STAR_NODE_PAYLOAD_SIZE*(cycle_phase-1)+x];
		
//		static uint32_t rep_sent_ms = 0;
//		static int rep_cnt = 0;
//		rep_cnt++;
//		uint32_t ms = millis();
//		if(ms - rep_sent_ms > 100)
//		{
//			uprintf("SL req %08X %d\n", tx_id, rep_cnt);
//			rep_sent_ms = ms;
//		}
	}
	while(rf_is_busy()) ;
	rf_send_and_listen(cur_out_pack, cur_out_pack[1]);
	if(cycle_phase == 0)
		cur_pack_len = 0;
}

uint32_t star_base_sync_time_ms = 0;
uint32_t star_local_sync_time_ms = 0;

void loop_node()
{
	uint32_t ms = millis();
	if(rf_has_new_packet())
	{
		int len = rf_get_packet(in_pack);
		if(!NRF_RADIO->CRCSTATUS) return;
		if(len < 6) return; //unexpected packet, not what we are looking for
		uint32_t rx_id = in_pack[2]<<24;
		rx_id |= in_pack[3]<<16;
		rx_id |= in_pack[4]<<8;
		rx_id |= in_pack[5];
		uint8_t rx_pack_id = in_pack[8];
		int is_good = 1;
		if(rx_id != star_unit_id) is_good = 0;
		if(rx_id == 0)
		{
			star_base_sync_time_ms = in_pack[12]<<24;
			star_base_sync_time_ms |= in_pack[13]<<16;
			star_base_sync_time_ms |= in_pack[14]<<8;
			star_base_sync_time_ms |= in_pack[15];
			star_local_sync_time_ms = millis();
		}
		if(rx_id == 0 && ms - last_asked_time > no_ask_timeout)
		{
			last_sync_time = ms;
			uint32_t rnd = NRF_RNG->VALUE;
			NRF_RNG->EVENTS_VALRDY = 0;
			
			if((rnd%100) > 97) //random - so other units will have a chance to report as well
			{
				buf_pack[0] = 1; //id
				buf_pack[1] = 16; //length
				buf_pack[2] = (star_unit_id>>24)&0xFF;
				buf_pack[3] = (star_unit_id>>16)&0xFF;
				buf_pack[4] = (star_unit_id>>8)&0xFF;
				buf_pack[5] = star_unit_id&0xFF;
				buf_pack[6] = 1; //protocol version
				rf_send_and_listen(buf_pack, 16);
			}
		}
		if(!is_good)
		{
			if(rx_id == 0)
			{
				star_has_new_packet = 1;
				buf_pack_len = len;
				for(int p = 0; p < len; p++)
					buf_pack[p] = in_pack[p];
			}
			return;
		}
		star_has_new_packet = 1;
		buf_pack_len = len;
		for(int p = 0; p < len; p++)
			buf_pack[p] = in_pack[p];
		last_asked_time = ms;
		last_sync_time = ms;
		if(cur_pack_len > 0 && rx_pack_id != cur_out_pack[0] &&!rf_is_busy())
		{
			rf_send_and_listen(cur_out_pack, cur_pack_len);
			last_star_sent_ms = ms;
		}
	}
	if(0)if(ms - last_sync_time > no_sync_timeout && cur_pack_len > 0)
	{
		uint32_t mcs = micros();
		if(mcs - last_pack_sent > nonstar_send_interval_mcs)
		{
			rf_send_and_listen(cur_out_pack, cur_pack_len);
			last_pack_sent = mcs;
		}
	}
}

void star_loop_step()
{
	if(role_central) loop_central();
	else loop_node();
}

uint32_t star_get_synced_time()
{
	uint32_t ms = millis();
	int dt = ms - star_local_sync_time_ms;
	return star_base_sync_time_ms + dt;
}

uint32_t star_get_last_sent_time()
{
	return last_star_sent_ms;
}
