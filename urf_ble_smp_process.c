#include "urf_ble_smp_process.h"
#include "urf_ble_encryption.h"

void ble_smp_pair_request(uint8_t *pdu, sSMP_link *smp_link, sLLData *ll_link)
{
	uint8_t io_caps = pdu[1];
	uint8_t oob_flag = pdu[2];
	uint8_t bonding = pdu[3] & 0b11;
	uint8_t mitm = (pdu[3]&0b100)>0;
	uint8_t key_size = pdu[4];
	uint8_t init_key_distrib = pdu[5];
	//what we have & what is expected
	smp_link->expected_key_mask = pdu[6] & (SMP_KEY_MASK_LTK | SMP_KEY_MASK_IRK | SMP_KEY_MASK_CSRK);
	smp_link->in_pairing = 1;
			
	int rpos = 0;
	smp_link->response_buf[rpos++] = SMP_PAIR_RESPONSE;
	smp_link->response_buf[rpos++] = 0x03; //no input no output
	smp_link->response_buf[rpos++] = 0; //no OOB
	smp_link->response_buf[rpos++] = 0b01; //bonding, no mitm protection
	smp_link->response_buf[rpos++] = 16; //key size
	smp_link->response_buf[rpos++] = 0; //we don't need anything - no DB on the device
	smp_link->response_buf[rpos++] = smp_link->expected_key_mask;

	for(int n = 0; n < 7; n++)
	{
		smp_link->pair_req[n] = pdu[n];
		smp_link->pair_rsp[n] = smp_link->response_buf[n];
	}
	
	smp_link->response_pending = 1;
	smp_link->response_length = rpos;
#ifdef BLE_DEBUG_PRINTS
	uprintf("pairing req ks %d %02X %02X %02X %02X\n", key_size, io_caps, bonding, init_key_distrib, smp_link->expected_key_mask);
#endif	
}
void ble_smp_pair_confirm(uint8_t *pdu, sSMP_link *smp_link, sLLData *ll_link)
{
	for(int n = 0; n < 16; n++)
		smp_link->tgt_confirm[n] = pdu[n+1];
	
	uint8_t key[16];
	for(int n = 0; n < 16; n++)
		key[n] = 0;

	ble_rand_fill8(smp_link->our_pairing_random);
	ble_rand_fill8(smp_link->our_pairing_random+8);
	
	ble_encr_fill_key(key);
	ble_encr_c1(smp_link->our_pairing_random, smp_link->pair_req, smp_link->pair_rsp, ll_link->tgt_addr_type, ll_link->tgt_mac, ll_link->our_addr_type, ll_link->our_mac, smp_link->our_confirm);
	
	int rpos = 0;
	smp_link->response_buf[rpos++] = SMP_PAIR_CONFIRM; //pairing confirm
	for(int n = 0; n < 16; n++)
		smp_link->response_buf[rpos++] = smp_link->our_confirm[n];

	smp_link->response_pending = 1;
	smp_link->response_length = rpos;
	
#ifdef BLE_DEBUG_PRINTS
	uprintf("pairing conf\n");
#endif	
}
void ble_smp_pair_random(uint8_t *pdu, sSMP_link *smp_link, sLLData *ll_link)
{
	for(int n = 0; n < 16; n++)
		smp_link->tgt_pairing_random[n] = pdu[n+1];
	
	uint8_t key[16];
	for(int n = 0; n < 16; n++)
		key[n] = 0;
	ble_encr_fill_key(key);
	uint8_t confirm_chk[16];
	ble_encr_c1(smp_link->tgt_pairing_random, smp_link->pair_req, smp_link->pair_rsp, ll_link->tgt_addr_type, ll_link->tgt_mac, ll_link->our_addr_type, ll_link->our_mac, confirm_chk);

	int conf_ok = 1;
	for(int n = 0; n < 16; n++)
		if(confirm_chk[n] != smp_link->tgt_confirm[n]) conf_ok = 0;
	
	if(!conf_ok)
	{
		smp_link->response_buf[0] = SMP_PAIR_FAILED;
		smp_link->response_buf[1] = 0x04; //value doesn't match
		smp_link->response_pending = 1;
		smp_link->response_length = 2;
#ifdef BLE_DEBUG_PRINTS
		uprintf("pairing random fail\n");
		uhex_print(ll_link->our_mac, 6, 0);
		uprintf(" our mac\n");
		uhex_print(ll_link->tgt_mac, 6, 0);
		uprintf(" tgt mac\n %d %d\n", ll_link->our_addr_type, ll_link->tgt_addr_type);
		uhex_print(smp_link->pair_req, 7, 1);
		uprintf(" pair req\n");
		uhex_print(smp_link->pair_rsp, 7, 1);
		uprintf(" pair rsp\n");
		
		for(int n = 0; n < 16; n++)
			uprintf("%02X", smp_link->tgt_confirm[n]);
		uprintf("\n");
		for(int n = 0; n < 16; n++)
			uprintf("%02X", confirm_chk[n]);
		uprintf("\n");
		for(int n = 0; n < 16; n++)
			uprintf("%02X", smp_link->our_pairing_random[n]);
		uprintf("\n");
		for(int n = 0; n < 16; n++)
			uprintf("%02X", smp_link->tgt_pairing_random[n]);
		uprintf("\n");
#endif
	}
	else
	{
		int rpos = 0;
		smp_link->response_buf[rpos++] = SMP_PAIR_RANDOM;
		for(int n = 0; n < 16; n++)
			smp_link->response_buf[rpos++] = smp_link->our_pairing_random[n];

		smp_link->response_pending = 1;
		smp_link->response_length = rpos;
		ble_encr_s1(smp_link->our_pairing_random, smp_link->tgt_pairing_random, ll_link->encr.LTK);
		smp_link->pause_cnt = 15; //small pause before sending keys
#ifdef BLE_DEBUG_PRINTS
		uprintf("pairing random ok\n");
#endif
	}
}

void ble_smp_encryption_information_process(uint8_t *pdu, sSMP_link *smp_link, sLLData *ll_link)
{
//	for(int n = 0; n < 16; n++) ll_link->encr.LTK[n] = pdu[n+1];
#ifdef BLE_DEBUG_PRINTS
	uprintf("smp: LTK got\n");
	for(int n = 0; n < 16; n++)
		uprintf("%02X", pdu[n+1]);
	uprintf("\n");
#endif
}

void ble_smp_master_identification_process(uint8_t *pdu, sSMP_link *smp_link, sLLData *ll_link)
{
//	ll_link->encr.EDIV = pdu[1] | (pdu[2]<<8);
//	for(int n = 0; n < 8; n++) ll_link->encr.er_rand[n] = pdu[n+3];
#ifdef BLE_DEBUG_PRINTS
	uint16_t ediv = pdu[1] | (pdu[2]<<8);
	uprintf("smp: MsID got, EDIV %d\n", ediv );
	for(int n = 0; n < 8; n++)
		uprintf("%02X", pdu[n+3]);
	uprintf("\n");
#endif
}


void ble_smp_encryption_information_send(sSMP_link *smp_link, sLLData *ll_link)
{
	smp_link->response_buf[0] = SMP_ENCRYPTION_INFO;
	int rpos = 1;
	ble_encr_generate_keys(ll_link);
	
	for(int n = 0; n < 16; n++)
		smp_link->response_buf[rpos++] = ll_link->encr.LTK[15-n];
	smp_link->response_pending = 1;
	smp_link->response_length = rpos;
	smp_link->pause_cnt = 3; //small pause
#ifdef BLE_DEBUG_PRINTS
	uprintf("smp: LTK sent\n");
//	for(int n = 0; n < 16; n++)
//		uprintf("%02X", ll_link->encr.LTK[n]);
//	uprintf("\n");
#endif
}
void ble_smp_master_identification_send(sSMP_link *smp_link, sLLData *ll_link)
{
	smp_link->response_buf[0] = SMP_MASTER_IDENT;
	int rpos = 1;
	smp_link->response_buf[rpos++] = ll_link->encr.EDIV;
	smp_link->response_buf[rpos++] = ll_link->encr.EDIV>>8;
	for(int n = 0; n < 8; n++)
		smp_link->response_buf[rpos++] = ll_link->encr.rr_rand[n];
	smp_link->response_pending = 1;
	smp_link->response_length = rpos;
	smp_link->pause_cnt = 2; //small pause
#ifdef BLE_DEBUG_PRINTS
	uprintf("smp: EDIV %d %02X%02X sent\n", ll_link->encr.EDIV, ll_link->encr.rr_rand[0], ll_link->encr.rr_rand[1]);
#endif
}
void ble_smp_identity_info_send(sSMP_link *smp_link, sLLData *ll_link)
{
	smp_link->response_buf[0] = SMP_IDENTITY_INFO;
	int rpos = 1;
	for(int n = 0; n < 16; n++)
		smp_link->response_buf[rpos++] = ll_link->encr.IRK[n];
	smp_link->response_pending = 1;
	smp_link->response_length = rpos;
	smp_link->pause_cnt = 2; //small pause
#ifdef BLE_DEBUG_PRINTS
	uprintf("smp: IRK sent\n");
#endif
}
void ble_smp_identity_address_info_send(sSMP_link *smp_link, sLLData *ll_link)
{
	smp_link->response_buf[0] = SMP_IDENTITY_ADDR_INFO;
	int rpos = 1;
	smp_link->response_buf[rpos++] = 0b01;
	for(int n = 0; n < 6; n++)
		smp_link->response_buf[rpos++] = ll_link->our_mac[n];
	smp_link->response_pending = 1;
	smp_link->response_length = rpos;
#ifdef BLE_DEBUG_PRINTS
	uprintf("smp: IAI sent\n");
#endif
}
void ble_smp_signing_info_send(sSMP_link *smp_link, sLLData *ll_link)
{
	smp_link->response_buf[0] = SMP_SIGNING_INFO;
	int rpos = 1;
	for(int n = 0; n < 16; n++)
		smp_link->response_buf[rpos++] = ll_link->encr.CSRK[n];
	smp_link->response_pending = 1;
	smp_link->response_length = rpos;
#ifdef BLE_DEBUG_PRINTS
	uprintf("smp: CSRK sent\n");
#endif
}

void ble_process_SMP(int length, volatile uint8_t *pdu, sSMP_link *smp_link, sLLData *ll_link)
{
	uint8_t opcode = pdu[0];
	int handled = 0;
	if(opcode == SMP_PAIR_REQUEST)
	{
		handled = 1;
		ble_smp_pair_request(pdu, smp_link, ll_link);
	}
	if(opcode == SMP_PAIR_CONFIRM) //pairing confirm
	{
		handled = 1;
		ble_smp_pair_confirm(pdu, smp_link, ll_link);
	}
	if(opcode == SMP_PAIR_RANDOM) //pairing random
	{
		handled = 1;
		ble_smp_pair_random(pdu, smp_link, ll_link);
	}
	if(opcode == SMP_ENCRYPTION_INFO)
	{
		handled = 1;
		ble_smp_encryption_information_process(pdu, smp_link, ll_link);
	}
	if(opcode == SMP_MASTER_IDENT)
	{
		handled = 1;
		ble_smp_master_identification_process(pdu, smp_link, ll_link);
	}
	if(!handled)
	{
#ifdef BLE_DEBUG_PRINTS
		uprintf("SMP unhandled: %02X\n", opcode);
#endif
	}
}

void ble_SMP_notify_security(sSMP_link *smp_link, int is_secure)
{
	smp_link->is_secure = is_secure;
}

int ble_check_SMP(sSMP_link *smp_link, sLLData *ll_link)
{
	if(smp_link->in_pairing && smp_link->is_secure)
	{
		if(smp_link->pause_cnt > 0)
		{
			smp_link->pause_cnt--;
			return 0;
		}
		if(smp_link->expected_key_mask & SMP_KEY_MASK_LTK)
		{
			smp_link->expected_key_mask |= SMP_KEY_MASK_EDIV;
			ble_smp_encryption_information_send(smp_link, ll_link);
			smp_link->expected_key_mask &= ~SMP_KEY_MASK_LTK;
			return 1;
		}
		if(smp_link->expected_key_mask & SMP_KEY_MASK_EDIV)
		{
			ble_smp_master_identification_send(smp_link, ll_link);
			smp_link->expected_key_mask &= ~SMP_KEY_MASK_EDIV;
			return 1;
		}
		if(smp_link->expected_key_mask & SMP_KEY_MASK_IRK)
		{
			smp_link->expected_key_mask |= SMP_KEY_MASK_IAI;
			ble_smp_identity_info_send(smp_link, ll_link);
			smp_link->expected_key_mask &= ~SMP_KEY_MASK_IRK;
			return 1;
		}
		if(smp_link->expected_key_mask & SMP_KEY_MASK_IAI)
		{
			ble_smp_identity_address_info_send(smp_link, ll_link);
			smp_link->expected_key_mask &= ~SMP_KEY_MASK_IAI;
			return 1;
		}
		if(smp_link->expected_key_mask & SMP_KEY_MASK_CSRK)
		{
			ble_smp_signing_info_send(smp_link, ll_link);
			smp_link->expected_key_mask &= ~SMP_KEY_MASK_CSRK;
			return 1;
		}
		//if nothing to send, then we have finished from our side
		smp_link->in_pairing = 0;
	}
	return 0; //by default, nothing to send
}

