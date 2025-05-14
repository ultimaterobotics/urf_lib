#include <stdint.h>
#include "ble_const.h"

typedef struct sATT_link
{
	uint16_t mtu;
	uint16_t mtu_change_pending;
	uint8_t response_pending;
	uint16_t response_length;
	uint8_t response_buf[250];
}sATT_link;

typedef struct sCharacteristic
{
	int mem_idx;

	uint8_t properties;
	uint16_t handle;

	uint16_t value_handle;
	
	int descriptor_count;
	uint16_t descriptor_uuids[16];
	uint16_t descriptor_handles[16];
	uint16_t descriptor_values[16];
	
	uint16_t uuid_16;
	uint8_t uuid_128[16];
	
	int val_type;
	volatile int val_length;
	volatile uint8_t value[96];
	volatile int val_cached_length;
	volatile uint8_t value_write_cache[96];
	
	volatile uint8_t changed; //changed from the program, if indication/notification is required - it will be performed
	volatile uint8_t had_write; //way to notify the program that value was read - it may want to update it
	volatile uint8_t had_read; //way to notify the program that value was read - it may want to update it
}sCharacteristic;

typedef struct sService
{
	int mem_idx;
	
	uint16_t type; //primary, secondary
	uint16_t handle;
	uint16_t group_end;

	uint16_t uuid_16;
	uint8_t uuid_128[16];
	
	uint16_t char_idx[32]; //max 32 characteristics per service - just to save static memory
	int char_count;
}sService;

void ble_update_our_mtu(int size);
int ble_add_service(sService* srv);
int ble_add_characteristic(sCharacteristic* chr);
void ble_uuid_from_text(uint8_t* uuid, char *uuid_str);
void ble_clear_notifications(sATT_link *att_link);
void ble_ATT_error_respond(uint8_t failed_opcode, uint16_t failed_attribute, uint8_t error_code, sATT_link *att_link);
void ble_process_ATT(int length, volatile uint8_t *pdu, sATT_link *att_link);
int ble_check_ATT(sATT_link *att_link);
