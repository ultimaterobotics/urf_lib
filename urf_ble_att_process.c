#include "urf_ble_att_process.h"
#define MAX_SERVICE_COUNT 16
#define MAX_CHARACTERISTICS_COUNT 64

sService* all_services_idx[MAX_SERVICE_COUNT];
sCharacteristic *all_characteristics_idx[MAX_CHARACTERISTICS_COUNT];
int service_count = 0;
int characteristics_count = 0;

uint8_t ble_our_max_mtu = 37; //default, before LL length update

void ble_update_our_mtu(int size)
{
	if(size < 37) ble_our_max_mtu = 37;
	else if(size > 96) ble_our_max_mtu = 96;
	else ble_our_max_mtu = size;
}

int ble_add_service(sService* srv)
{
	if(service_count >= MAX_SERVICE_COUNT) return -1;
	srv->mem_idx = service_count;
	all_services_idx[service_count++] = srv;
	return service_count;
}

int ble_add_characteristic(sCharacteristic* chr)
{
	if(characteristics_count >= MAX_CHARACTERISTICS_COUNT) return -1;
	chr->mem_idx = characteristics_count;
	all_characteristics_idx[characteristics_count++] = chr;
	return characteristics_count;
}

void ble_uuid_from_text(uint8_t* uuid, char *uuid_str)
{
	//03B80E5A EDE84B33 A7516CE3 4EC4C700
//	MIDI_service.uuid_128[0] = 0x4EC4C700;
//	MIDI_service.uuid_128[1] = 0xA7516CE3;
//	MIDI_service.uuid_128[2] = 0xEDE84B33;
//	MIDI_service.uuid_128[3] = 0x03B80E5A;
//UUID: 03B80E5A-EDE8-4B33-A751-6CE34EC4C700

	int spos = 0;
	while(uuid_str[spos]) spos++;
	spos--;
	for(int x = 0; x < 16; x++)
	{
		int lL = -1, lU = -1;
		while(1)
		{
			lL = uuid_str[spos];
			spos--;
			if(lL >= '0' && lL <= '9') break;
			if(lL >= 'a' && lL <= 'f') break;
			if(lL >= 'A' && lL <= 'F') break;
			if(spos < 1) break;
		}
		while(1)
		{
			lU = uuid_str[spos];
			spos--;
			if(lU >= '0' && lU <= '9') break;
			if(lU >= 'a' && lU <= 'f') break;
			if(lU >= 'A' && lU <= 'F') break;
			if(spos < 0) break;
		}
		int vU = 0, vL = 0;
		if(lU >= '0' && lU <= '9') vU = lU-'0';
		if(lU >= 'a' && lU <= 'f') vU = 10 + lU-'a';
		if(lU >= 'A' && lU <= 'F') vU = 10 + lU-'A';
		if(lL >= '0' && lL <= '9') vL = lL-'0';
		if(lL >= 'a' && lL <= 'f') vL = 10 + lL-'a';
		if(lL >= 'A' && lL <= 'F') vL = 10 + lL-'A';
		uuid[x] = vU*16 + vL;
	}	
}

void ble_ATT_error_respond(uint8_t failed_opcode, uint16_t failed_attribute, uint8_t error_code, sATT_link *att_link)
{
	att_link->response_buf[0] = ATT_ERROR_RSP;
	att_link->response_buf[1] = failed_opcode;
	att_link->response_buf[2] = failed_attribute;
	att_link->response_buf[3] = failed_attribute>>8;
	att_link->response_buf[4] = error_code;
	
	att_link->response_pending = 1;
	att_link->response_length = 5;

#ifdef BLE_DEBUG_PRINTS
	uprintf("ATT ERR %02X %d %02X\n", failed_opcode, failed_attribute, error_code);
#endif	
}

int fill_characteristics_value_to_buf(sCharacteristic *chr, uint8_t *buf)
{
	int pos = 0;
	for(int x = 0; x < chr->val_length; x++)
		buf[pos++] = chr->value[x];
	return pos;
}


void ble_process_ATT(int length, volatile uint8_t *pdu, sATT_link *att_link)
{
	uint8_t opcode = pdu[0];
	int handled = 0;
	if(opcode == ATT_EXCHANGE_MTU_REQ)
	{
		handled = 1;		
		uint16_t mtu = pdu[1] | (pdu[2]<<8);
#ifdef BLE_DEBUG_PRINTS
		uprintf("MTU EXCH REQ %d\n", mtu);
#endif
		
		if(mtu >= ATT_MTU_DEFAULT)
		{
			att_link->response_buf[0] = ATT_EXCHANGE_MTU_RSP;
			att_link->response_buf[1] = ble_our_max_mtu;
			att_link->response_buf[2] = ble_our_max_mtu>>8;
			att_link->response_pending = 1;
			att_link->response_length = 3;
			att_link->mtu_change_pending = mtu;
			if(ble_our_max_mtu < mtu)
				att_link->mtu_change_pending = ble_our_max_mtu;
		}
		else
		{
			ble_ATT_error_respond(ATT_EXCHANGE_MTU_REQ, 0, ATT_ERROR_INVALID_PDU, att_link);
			goto BLE_ATT_PROCESSING_END;
		}
	}
	if(opcode == ATT_FIND_INFORMATION_REQ)
	{
		handled = 1;
		uint16_t start_handle = pdu[1] | (pdu[2]<<8);
		uint16_t end_handle = pdu[3] | (pdu[4]<<8);
		if(start_handle > end_handle || start_handle == 0)
		{
			ble_ATT_error_respond(ATT_FIND_INFORMATION_REQ, start_handle, ATT_ERROR_INVALID_HANDLE, att_link);
			goto BLE_ATT_PROCESSING_END;
		}
		att_link->response_buf[0] = ATT_FIND_INFORMATION_RSP;
		int pos = 1;
		for(int s = 0; s < service_count; s++)
		{
			if(all_services_idx[s]->handle >= start_handle && all_services_idx[s]->handle <= end_handle)
			{
				if(all_services_idx[s]->uuid_16 > 0)
				{
					if(pos + 6 > att_link->mtu) break;
					att_link->response_buf[pos++] = 1;
					att_link->response_buf[pos++] = all_services_idx[s]->handle;
					att_link->response_buf[pos++] = all_services_idx[s]->handle>>8;
					att_link->response_buf[pos++] = all_services_idx[s]->uuid_16;
					att_link->response_buf[pos++] = all_services_idx[s]->uuid_16>>8;
				}
				else
				{
					if(pos + 19 > att_link->mtu) break;
					att_link->response_buf[pos++] = 2;
					att_link->response_buf[pos++] = all_services_idx[s]->handle;
					att_link->response_buf[pos++] = all_services_idx[s]->handle>>8;
					for(int x = 0; x < 16; x++)
						att_link->response_buf[pos++] = all_services_idx[s]->uuid_128[x];
				}
			}
		}
		if(pos < 3)
		{
			ble_ATT_error_respond(ATT_FIND_INFORMATION_REQ, start_handle, ATT_ERROR_ATTRIBUTE_NOT_FOUND, att_link);
		}
		else
		{
			att_link->response_pending = 1;
			att_link->response_length = pos;
			goto BLE_ATT_PROCESSING_END;
		}
	}
	if(opcode == ATT_READ_BY_GROUP_TYPE_REQ) //service discovery
	{
		handled = 1;
		uint16_t start_handle = pdu[1] | (pdu[2]<<8);
		uint16_t end_handle = pdu[3] | (pdu[4]<<8);
		if(start_handle > end_handle || start_handle == 0)
		{
			ble_ATT_error_respond(ATT_READ_BY_GROUP_TYPE_REQ, start_handle, ATT_ERROR_INVALID_HANDLE, att_link);
			goto BLE_ATT_PROCESSING_END;
		}
		uint16_t group_type = pdu[5] | (pdu[6]<<8);
		att_link->response_buf[0] = ATT_READ_BY_GROUP_TYPE_RSP;
		if(group_type == UUID_PRIMARY_SERVICE)
		{
			int pos = 1;
			int service_type_used = -1;
			for(int s = 0; s < service_count; s++)
			{
				if(all_services_idx[s]->handle >= start_handle && all_services_idx[s]->handle <= end_handle)
				{
					if(all_services_idx[s]->uuid_16 > 0 && service_type_used != 2)
					{
						if(pos + 6 > att_link->mtu) break;
						if(service_type_used < 0)
						{
							service_type_used = 1;
							att_link->response_buf[pos++] = 6;
						}
						att_link->response_buf[pos++] = all_services_idx[s]->handle;
						att_link->response_buf[pos++] = all_services_idx[s]->handle>>8;
						att_link->response_buf[pos++] = all_services_idx[s]->group_end;
						att_link->response_buf[pos++] = all_services_idx[s]->group_end>>8;
						att_link->response_buf[pos++] = all_services_idx[s]->uuid_16;
						att_link->response_buf[pos++] = all_services_idx[s]->uuid_16>>8;
					}
					if(all_services_idx[s]->uuid_16 == 0 && service_type_used != 1)
					{
						if(pos + 20 > att_link->mtu) break;
						if(service_type_used < 0)
						{
							service_type_used = 2;
							att_link->response_buf[pos++] = 20;
						}
						att_link->response_buf[pos++] = all_services_idx[s]->handle;
						att_link->response_buf[pos++] = all_services_idx[s]->handle>>8;
						att_link->response_buf[pos++] = all_services_idx[s]->group_end;
						att_link->response_buf[pos++] = all_services_idx[s]->group_end>>8;
						for(int x = 0; x < 16; x++)
							att_link->response_buf[pos++] = all_services_idx[s]->uuid_128[x];
					}
				}
			}
			if(pos < 6) //no service fits criteria
			{
				ble_ATT_error_respond(ATT_READ_BY_GROUP_TYPE_REQ, start_handle, ATT_ERROR_ATTRIBUTE_NOT_FOUND, att_link);
			}
			else
			{
				att_link->response_pending = 1;
				att_link->response_length = pos;
#ifdef BLE_DEBUG_PRINTS
				uprintf("ATT PR SVC %d\n", start_handle);
#endif				
			}

			goto BLE_ATT_PROCESSING_END;
		}
	}
	if(opcode == ATT_READ_BY_TYPE_REQ) //service discovery or reading value by UUID
	{
		handled = 1;
		uint16_t start_handle = pdu[1] | (pdu[2]<<8);
		uint16_t end_handle = pdu[3] | (pdu[4]<<8);
		if(start_handle > end_handle || start_handle == 0)
		{
#ifdef BLE_DEBUG_PRINTS
			uprintf("ATT_READ_BY_TYPE handle err %d\n", start_handle);
#endif				
			ble_ATT_error_respond(ATT_READ_BY_TYPE_REQ, start_handle, ATT_ERROR_INVALID_HANDLE, att_link);
			goto BLE_ATT_PROCESSING_END;
		}
		uint16_t type = pdu[5] | (pdu[6]<<8);
		uint8_t type_128[16];
		if(length > 16) //128-bit UUID
			for(int n = 0; n < 16; n++)
				type_128[n] = pdu[5+n];
			
		att_link->response_buf[0] = ATT_READ_BY_TYPE_RSP;

		if(type == UUID_INCLUDE)
		{
#ifdef BLE_DEBUG_PRINTS
			uprintf("ATT_READ_BY_TYPE err include\n");
#endif							
			ble_ATT_error_respond(ATT_READ_BY_TYPE_REQ, start_handle, ATT_ERROR_ATTRIBUTE_NOT_FOUND, att_link);
			goto BLE_ATT_PROCESSING_END;
		}
		
		if(type == UUID_CHARACTERISTIC) //discovery
		{
			int pos = 1;
			int char_type_used = -1;
			for(int c = 0; c < characteristics_count; c++)
			{
				if(all_characteristics_idx[c]->handle >= start_handle && all_characteristics_idx[c]->handle <= end_handle)
				{
					if(all_characteristics_idx[c]->uuid_16 > 0)
						att_link->response_buf[pos++] = 2+3+2;
					else
						att_link->response_buf[pos++] = 2+3+16;
					att_link->response_buf[pos++] = all_characteristics_idx[c]->handle;
					att_link->response_buf[pos++] = all_characteristics_idx[c]->handle>>8;
						
					att_link->response_buf[pos++] = all_characteristics_idx[c]->properties;
					att_link->response_buf[pos++] = all_characteristics_idx[c]->value_handle;
					att_link->response_buf[pos++] = all_characteristics_idx[c]->value_handle>>8;
					if(all_characteristics_idx[c]->uuid_16 > 0)
					{
						att_link->response_buf[pos++] = all_characteristics_idx[c]->uuid_16;
						att_link->response_buf[pos++] = all_characteristics_idx[c]->uuid_16>>8;
					}
					else
					{
						for(int x = 0; x < 16; x++)
							att_link->response_buf[pos++] = all_characteristics_idx[c]->uuid_128[x];
					}
//					int len = fill_characteristics_value_to_buf(all_characteristics_idx[c], att_link->response_buf+pos);
//					if(pos+len > att_link->mtu) len = att_link->mtu - pos; //truncate - for debug only
//					pos += len;
					break;
				}
			}
			if(pos < 3) //no characteristics fits criteria
				ble_ATT_error_respond(ATT_READ_BY_TYPE_REQ, start_handle, ATT_ERROR_ATTRIBUTE_NOT_FOUND, att_link);
			else
			{
				att_link->response_pending = 1;
				att_link->response_length = pos;
#ifdef BLE_DEBUG_PRINTS
//				uprintf("ATT CHR %d\n", start_handle);
#endif
			}

			goto BLE_ATT_PROCESSING_END;
		}
		else //reading by UUID
		{
			int pos = 1;
			for(int c = 0; c < characteristics_count; c++)
			{
				if(all_characteristics_idx[c]->value_handle >= start_handle && all_characteristics_idx[c]->value_handle <= end_handle)
				{
					if(length > 16 && all_characteristics_idx[c]->uuid_16 > 0) continue;
					if(length <= 16 && all_characteristics_idx[c]->uuid_16 == 0) continue;
					
					if(length <= 16 && all_characteristics_idx[c]->uuid_16 != type) continue;
					if(length > 16)
						for(int n = 0; n < 16; n++)
							if(type_128[n] != all_characteristics_idx[c]->uuid_128[n]) continue;

					sCharacteristic *chr = all_characteristics_idx[c];
					att_link->response_buf[pos++] = 2 + chr->val_length;
					att_link->response_buf[pos++] = chr->value_handle;
					att_link->response_buf[pos++] = chr->value_handle>>8;
/*					if(!chr->value_not_ready)
					{
						chr->val_send_cache_length = chr->val_length;
						for(int n = 0; n < chr->val_length; n++)
							chr->value_send_cache[n] = chr->value[n];
					}
					for(int n = 0; n < chr->val_send_cache_length; n++)
						att_link->response_buf[pos++] = chr->value_send_cache[n];*/
					for(int n = 0; n < chr->val_length; n++)
						att_link->response_buf[pos++] = chr->value[n];
					break;
				}
			}
			if(pos < 3) //no characteristics fits criteria
				ble_ATT_error_respond(ATT_READ_BY_TYPE_REQ, start_handle, ATT_ERROR_ATTRIBUTE_NOT_FOUND, att_link);
			else
			{
				att_link->response_pending = 1;
				att_link->response_length = pos;
#ifdef BLE_DEBUG_PRINTS
				uprintf("ATT CHR %d\n", start_handle);
#endif
			}
			goto BLE_ATT_PROCESSING_END;
		}
	}
	if(opcode == ATT_FIND_INFORMATION_REQ) //descriptor discovery
	{
		handled = 1;
		uint16_t start_handle = pdu[1] | (pdu[2]<<8);
		uint16_t end_handle = pdu[3] | (pdu[4]<<8);
		if(start_handle > end_handle || start_handle == 0)
		{
#ifdef BLE_DEBUG_PRINTS
			uprintf("ATT_FIND_INFORMATION_REQ handle err %d\n", start_handle);
#endif				
			ble_ATT_error_respond(ATT_FIND_INFORMATION_REQ, start_handle, ATT_ERROR_INVALID_HANDLE, att_link);
			goto BLE_ATT_PROCESSING_END;
		}
			
		att_link->response_buf[0] = ATT_FIND_INFORMATION_RSP;

		int pos = 1;
		for(int c = 0; c < characteristics_count; c++)
		{
			sCharacteristic *chr = all_characteristics_idx[c];
			for(int d = 0; d < chr->descriptor_count; d++)
			{
				if(chr->descriptor_handles[d]  >= start_handle && chr->descriptor_handles[d] <= end_handle)
				{
					att_link->response_buf[pos++] = 1; //only 16-bit descriptors currently supported
					att_link->response_buf[pos++] = chr->descriptor_handles[d];
					att_link->response_buf[pos++] = chr->descriptor_handles[d]>>8;
					att_link->response_buf[pos++] = chr->descriptor_uuids[d];
					att_link->response_buf[pos++] = chr->descriptor_uuids[d]>>8;
					break;
				}
			}
			if(pos > 1) break;
		}
		
		if(pos < 3) //no characteristics fits criteria
			ble_ATT_error_respond(ATT_FIND_INFORMATION_REQ, start_handle, ATT_ERROR_ATTRIBUTE_NOT_FOUND, att_link);
		else
		{
			att_link->response_pending = 1;
			att_link->response_length = pos;
#ifdef BLE_DEBUG_PRINTS
			uprintf("ATT FIND INFO %d\n", start_handle);
#endif
		}
		goto BLE_ATT_PROCESSING_END;
	}
	if(opcode == ATT_READ_REQ) //simple read
	{
		handled = 1;
		uint16_t handle = pdu[1] | (pdu[2]<<8);
#ifdef BLE_DEBUG_PRINTS
		uprintf("ATT READ %d\n", handle);
#endif
		sCharacteristic *chr;
		int is_good = 0;
		int read_descr = -1;
		for(int c = 0; c < characteristics_count; c++)
		{
			if(handle == all_characteristics_idx[c]->value_handle)
			{
				chr = all_characteristics_idx[c];
				is_good = 1;
				break;
			}
			for(int d = 0; d < all_characteristics_idx[c]->descriptor_count; d++)
				if(handle == all_characteristics_idx[c]->descriptor_handles[d])
				{
					chr = all_characteristics_idx[c];
					is_good = 1;
					read_descr = d;
					break;
				}
			if(read_descr >= 0) break;
		}
		if(!is_good)
		{
			ble_ATT_error_respond(ATT_READ_REQ, handle, ATT_ERROR_ATTRIBUTE_NOT_FOUND, att_link);
			goto BLE_ATT_PROCESSING_END;
		}
		att_link->response_buf[0] = ATT_READ_RSP;
		int pos = 1;
		if(read_descr < 0)
		{
			pos += fill_characteristics_value_to_buf(chr, att_link->response_buf+1);
			chr->had_read = 1;
		}
		else
		{
			att_link->response_buf[1] = chr->descriptor_values[read_descr];
			att_link->response_buf[2] = chr->descriptor_values[read_descr]>>8;
			pos += 2;
		}

		att_link->response_pending = 1;
		att_link->response_length = pos;
		
		goto BLE_ATT_PROCESSING_END;
	}	
	if(opcode == ATT_WRITE_REQ) //simple write
	{
		handled = 1;
		uint16_t handle = pdu[1] | (pdu[2]<<8);
#ifdef BLE_DEBUG_PRINTS
		uprintf("ATT WRITE %d\n", handle);
#endif
		sCharacteristic *chr;
		int is_good = 0;
		int read_descr = -1;
		for(int c = 0; c < characteristics_count; c++)
		{
			if(handle == all_characteristics_idx[c]->value_handle)
			{
				chr = all_characteristics_idx[c];
				is_good = 1;
				break;
			}
			for(int d = 0; d < all_characteristics_idx[c]->descriptor_count; d++)
				if(handle == all_characteristics_idx[c]->descriptor_handles[d])
				{
					chr = all_characteristics_idx[c];
					is_good = 1;
					read_descr = d;
					break;
				}
			if(read_descr >= 0) break;
		}
		if(!is_good)
		{
#ifdef BLE_DEBUG_PRINTS
			uprintf("ATT WRITE: handle not found\n");
#endif
			ble_ATT_error_respond(ATT_WRITE_REQ, handle, ATT_ERROR_ATTRIBUTE_NOT_FOUND, att_link);
			goto BLE_ATT_PROCESSING_END;
		}
		att_link->response_buf[0] = ATT_WRITE_RSP;
		
		if(read_descr < 0)
		{		
			int val_len = length - 3;
			if(val_len > 24) val_len = 24;
			for(int x = 0; x < val_len; x++)
			{
				chr->value[x] = pdu[3+x];
				chr->value_write_cache[x] = pdu[3+x];
			}
			chr->val_length = val_len;
			chr->val_cached_length = val_len;
			chr->had_write = 1;
#ifdef BLE_DEBUG_PRINTS
			uprintf("ATT WRITE: filled len %d\n", val_len);
#endif
		}
		else
		{
			chr->descriptor_values[read_descr] = pdu[3] | (pdu[4]<<8);
		}

		att_link->response_pending = 1;
		att_link->response_length = 1;
		
		goto BLE_ATT_PROCESSING_END;
	}	

	if(opcode == ATT_WRITE_CMD)
	{
		handled = 1;
		att_link->response_pending = 0;
		att_link->response_length = 0;
		uint16_t handle = pdu[1] | (pdu[2]<<8);
#ifdef BLE_DEBUG_PRINTS
		uprintf("ATT WRITE CMD %d\n", handle);
#endif
		sCharacteristic *chr;
		int is_good = 0;
		int read_descr = -1;
		for(int c = 0; c < characteristics_count; c++)
		{
			if(handle == all_characteristics_idx[c]->value_handle)
			{
				chr = all_characteristics_idx[c];
				is_good = 1;
				break;
			}
			for(int d = 0; d < all_characteristics_idx[c]->descriptor_count; d++)
				if(handle == all_characteristics_idx[c]->descriptor_handles[d])
				{
					chr = all_characteristics_idx[c];
					is_good = 1;
					read_descr = d;
					break;
				}
			if(read_descr >= 0) break;
		}
		//no response is needed either for error or success
		if(!is_good)
		{
#ifdef BLE_DEBUG_PRINTS
			uprintf("ATT WRITE CMD: handle not found\n");
#endif
			goto BLE_ATT_PROCESSING_END;
		}
//		att_link->response_buf[0] = ATT_WRITE_RSP;
		
		if(read_descr < 0)
		{		
			int val_len = length - 3;
			if(val_len > 24) val_len = 24;
			for(int x = 0; x < val_len; x++)
			{
				chr->value[x] = pdu[3+x];
				chr->value_write_cache[x] = pdu[3+x];
			}
			chr->val_length = val_len;
			chr->val_cached_length = val_len;
			chr->had_write = 1;
#ifdef BLE_DEBUG_PRINTS
			uprintf("ATT WRITE CMD: filled len %d\n", val_len);
#endif
		}
		else
		{
			chr->descriptor_values[read_descr] = pdu[3] | (pdu[4]<<8);
		}
		
		goto BLE_ATT_PROCESSING_END;
	}	
	
BLE_ATT_PROCESSING_END:
	return;
}
void ble_clear_notifications(sATT_link *att_link)
{
	for(int c = 0; c < characteristics_count; c++)
	{
		sCharacteristic *chr = all_characteristics_idx[c];
		if(chr->descriptor_count > 0)
			for(int d = 0; d < chr->descriptor_count; d++)
				if(chr->descriptor_uuids[d] == 0x2902)
					chr->descriptor_values[d] = 0;
	}
}
int ble_check_notifications(sATT_link *att_link) //check if any characteristics changed and send notification if required
{
	int had_change = 0;
	for(int c = 0; c < characteristics_count; c++)
	{
		if(all_characteristics_idx[c]->changed)
		{
			sCharacteristic *chr = all_characteristics_idx[c];
			att_link->response_buf[0] = ATT_HANDLE_VALUE_NTF;
			att_link->response_buf[1] = chr->value_handle;
			att_link->response_buf[2] = chr->value_handle>>8;
			int pos = 3;
			pos += fill_characteristics_value_to_buf(chr, att_link->response_buf+3);

			att_link->response_pending = 1;
			att_link->response_length = pos;
			chr->changed = 0;
			had_change = 1;
			break;
		}
	}
	return had_change;
}
//check if for any scheduled ATT transactions
int ble_check_ATT(sATT_link *att_link)
{
	//for now, only notifications are implemented
	return ble_check_notifications(att_link);
}
