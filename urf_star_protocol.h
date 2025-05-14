#include <stdint.h>

void star_init(int channel, int speed, int phase_length_mcs, int is_central);
void star_set_id(uint32_t id);
//!
//! standard sending data from nodes to central
void star_queue_send(uint8_t *pack, int length);
//!
//! sending data from central to given node
void star_send_to_node(uint32_t node_id, uint8_t *payload_8b);
int star_has_packet();
int star_get_packet(uint8_t *pack, int max_length); //returns actual length
//void star_set_on_packet_callback(void *callback);
void star_loop_step(); //required to process protocol, chosen instead of internal timed event
	//so it won't interrupt any critical tasks more than minimally required 
	//(when radio gets packet, interrupt that switches the radio mode and copies received data
	//is called, although it's a really short one
	//star_loop_step takes somewhat longer, and you have to call it every millisecond or so, 
	//depending on time dedicated for each unit communication

uint32_t star_get_synced_time();
uint32_t star_get_last_sent_time();
