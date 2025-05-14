#include "urf_ble_encryption.h"
#include "ble_const.h"
#include "nrf.h"


uint32_t ble_rand_cur = 12345678;

void ble_rand_init()
{
	NRF_RNG->SHORTS = 0;
	NRF_RNG->CONFIG = 1;
	NRF_RNG->TASKS_START = 1;
	volatile uint32_t good_rand = 0;
	good_rand <<= 8;
	NRF_RNG->EVENTS_VALRDY = 0;
	while(!NRF_RNG->EVENTS_VALRDY);
	good_rand |= NRF_RNG->VALUE;
	good_rand <<= 8;
	NRF_RNG->EVENTS_VALRDY = 0;
	while(!NRF_RNG->EVENTS_VALRDY);
	good_rand |= NRF_RNG->VALUE;
	good_rand <<= 8;
	NRF_RNG->EVENTS_VALRDY = 0;
	while(!NRF_RNG->EVENTS_VALRDY);
	good_rand |= NRF_RNG->VALUE;
	good_rand <<= 8;
	NRF_RNG->EVENTS_VALRDY = 0;
	while(!NRF_RNG->EVENTS_VALRDY);
	good_rand |= NRF_RNG->VALUE;
	
	ble_rand_cur = good_rand;
}

uint32_t ble_rand_num()
{
	if(NRF_RNG->EVENTS_VALRDY)
	{
		uint8_t rr = ble_rand_cur;
		ble_rand_cur &= 0xFFFFFF00;
		ble_rand_cur |= rr | NRF_RNG->VALUE;
		NRF_RNG->EVENTS_VALRDY = 0;
	}

	ble_rand_cur ^= ble_rand_cur << 13;
	ble_rand_cur ^= ble_rand_cur >> 17;
	ble_rand_cur ^= ble_rand_cur << 5;
	return ble_rand_cur;
}

void ble_rand_fill8(uint8_t *buf)
{
	uint32_t r = ble_rand_num();
	buf[0] = r;
	buf[1] = r>>8;
	buf[2] = r>>16;
	buf[3] = r>>24;
	r = ble_rand_num();
	buf[4] = r;
	buf[5] = r>>8;
	buf[6] = r>>16;
	buf[7] = r>>24;
}

struct
{
	volatile uint8_t key[16];
	volatile uint8_t plain[16];
	volatile uint8_t encr[16];
}nrf_aes_data;

void ble_encr_fill_key(uint8_t *key) //_inv versions for testing purpose
{
	for(int n = 0; n < 16; n++)
		nrf_aes_data.key[n] = key[n];
}

void ble_encr_e_inv(uint8_t *plain, uint8_t *encr)
{
	for(int n = 0; n < 16; n++)
		nrf_aes_data.plain[n] = plain[n];

	NRF_ECB->ECBDATAPTR = (uint32_t)&nrf_aes_data;
	NRF_ECB->EVENTS_ENDECB = 0;
	NRF_ECB->TASKS_STARTECB = 1;
	while(!NRF_ECB->EVENTS_ENDECB) ;
	for(int n = 0; n < 16; n++)
		encr[n] = nrf_aes_data.encr[n];
	return;
}


void ble_encr_fill_key_inv(uint8_t *key)
{
	for(int n = 0; n < 16; n++)
		nrf_aes_data.key[n] = key[15-n];
}

void ble_encr_e(uint8_t *plain, uint8_t *encr)
{
	for(int n = 0; n < 16; n++)
		nrf_aes_data.plain[n] = plain[15-n];

	NRF_ECB->ECBDATAPTR = (uint32_t)&nrf_aes_data;
	NRF_ECB->EVENTS_ENDECB = 0;
	NRF_ECB->TASKS_STARTECB = 1;
	while(!NRF_ECB->EVENTS_ENDECB) ;
	for(int n = 0; n < 16; n++)
		encr[n] = nrf_aes_data.encr[15-n];
	return;
}

uint32_t ble_encr_ah(uint8_t *key, uint32_t r)
{
	ble_encr_fill_key(key);
	uint8_t msg[16];
	uint8_t res[16];
	for(int n = 0; n < 13; n++) msg[n+3] = 0;
	msg[0] = r;
	msg[1] = r>>8;
	msg[2] = r>>16;
	ble_encr_e(msg, res);
	uint32_t r1 = 0;
	r1 = res[0] | (res[1]<<8) | (res[2]<<16);
	return r1;
}

void ble_encr_c1(uint8_t *r, uint8_t *preq, uint8_t *pres, uint8_t iat, uint8_t *ia, uint8_t rat, uint8_t *ra, uint8_t *out)
{
	uint8_t p1[16];
	p1[0] = iat;
	p1[1] = rat;
	for(int n = 0; n < 7; n++)
	{
		p1[2+n] = preq[n];
		p1[9+n] = pres[n];
	}
	uint8_t p2[16];
	for(int n = 0; n < 6; n++)
	{
		p2[n] = ra[n];
		p2[6+n] = ia[n];
	}
	for(int n = 0; n < 4; n++) p2[12+n] = 0;
	uint8_t xr1[16], xr2[16];
	for(int n = 0; n < 16; n++)
		xr1[n] = r[n] ^ p1[n];
	ble_encr_e(xr1, xr2);
	for(int n = 0; n < 16; n++)
		xr2[n] = xr2[n] ^ p2[n];
	
	ble_encr_e(xr2, out);
}

void ble_encr_s1(uint8_t *r1, uint8_t *r2, uint8_t *out)
{
	uint8_t r[16];
	for(int n = 0; n < 8; n++)
	{
//		r[n] = r2[n];
//		r[n+8] = r1[n];
		r[n] = r1[7-n];
		r[n+8] = r2[7-n];
	}
//	uprintf("rand: ");
//	for(int n = 0; n < 16; n++)
//		uprintf("%02X", r[n]);
//	uprintf("\n");
	ble_encr_e_inv(r, out);
}

void ble_encr_d1(uint8_t *k, uint16_t d, uint16_t r, uint8_t *out)
{
	ble_encr_fill_key(k);
	uint8_t arr[16];
	for(int n = 4; n < 16; n++)
		arr[n] = 0;
	arr[0] = d;
	arr[1] = d>>8;
	arr[2] = r;
	arr[3] = r>>8;
	ble_encr_e(arr, out);
}

uint16_t ble_encr_dm(uint8_t *k, uint8_t *r)
{
	ble_encr_fill_key(k);
	uint8_t arr[16];
	for(int n = 0; n < 8; n++)
	{
		arr[n] = r[n];
		arr[8+n] = 0;
	}
	uint8_t res[16];
	ble_encr_e(arr, res);
	return res[0] | (res[1]<<8);
}


void ble_encr_ccm_fill_array(sLLData *ll_link)
{
	int cpos = 0;
	uint8_t *ccmbuf = ll_link->encr.ccm_data.compr_array;
	for(int x = 0; x < 16; x++)
		ccmbuf[cpos++] = ll_link->encr.ccm_data.key[15-x];

	if(ll_link->encr.ccm_data.direction == 0)
	{
		ccmbuf[cpos++] = ll_link->encr.ccm_data.packet_counter_our;
		ccmbuf[cpos++] = ll_link->encr.ccm_data.packet_counter_our>>8;
		ccmbuf[cpos++] = ll_link->encr.ccm_data.packet_counter_our>>16;
		ccmbuf[cpos++] = ll_link->encr.ccm_data.packet_counter_our>>24;
		ccmbuf[cpos++] = ll_link->encr.ccm_data.packet_counter_msb_our;
	}
	else
	{
		ccmbuf[cpos++] = ll_link->encr.ccm_data.packet_counter_tgt;
		ccmbuf[cpos++] = ll_link->encr.ccm_data.packet_counter_tgt>>8;
		ccmbuf[cpos++] = ll_link->encr.ccm_data.packet_counter_tgt>>16;
		ccmbuf[cpos++] = ll_link->encr.ccm_data.packet_counter_tgt>>24;
		ccmbuf[cpos++] = ll_link->encr.ccm_data.packet_counter_msb_tgt;
	}
	ccmbuf[cpos++] = 0;
	ccmbuf[cpos++] = 0;
	ccmbuf[cpos++] = 0;
	ccmbuf[cpos++] = ll_link->encr.ccm_data.direction;
	for(int x = 0; x < 8; x++)
		ccmbuf[cpos++] = ll_link->encr.ccm_data.IV[x];

	NRF_CCM->CNFPTR = (uint32_t)ccmbuf;
}

void ble_encr_ccm_config(uint8_t mode)
{
	if(mode == 1) //rx
	{
		NRF_PPI->CHENSET = (1<<24) | (1<<25); //radio ready -> CCM->TASKS_KSGEN, address -> CCM->TASKS_CRYPT
		NRF_CCM->SHORTS = 0;
		NRF_CCM->ENABLE = 2;
	}
	else if(mode == 2) //tx
	{
		NRF_PPI->CHENSET = (1<<24);
		NRF_PPI->CHENCLR = (1<<25); //radio ready -> CCM->TASKS_KSGEN, key ready -> crypt via short
		NRF_CCM->SHORTS = 1;
		NRF_CCM->ENABLE = 2;
	}
	else if(mode == 0)
	{
		NRF_PPI->CHENCLR = (1<<24) | (1<<25);
		NRF_CCM->ENABLE = 0;
	}
}

void test_str_to_arr2(int len, char *txt, uint8_t *arr)
{
	for(int x = 0; x < len; x++)
	{
		int rx = len-1-x;
		int lU = txt[rx*2];
		int lL = txt[rx*2+1];
		int vU, vL;
		if(lU >= '0' && lU <= '9') vU = lU-'0';
		if(lU >= 'a' && lU <= 'f') vU = 10 + lU-'a';
		if(lU >= 'A' && lU <= 'F') vU = 10 + lU-'A';
		if(lL >= '0' && lL <= '9') vL = lL-'0';
		if(lL >= 'a' && lL <= 'f') vL = 10 + lL-'a';
		if(lL >= 'A' && lL <= 'F') vL = 10 + lL-'A';
		arr[x] = vU*16 + vL;
	}
}

void test_str_to_arr2_inv(int len, char *txt, uint8_t *arr)
{
	for(int x = 0; x < len; x++)
	{
		int rx = x;
		int lU = txt[rx*2];
		int lL = txt[rx*2+1];
		int vU, vL;
		if(lU >= '0' && lU <= '9') vU = lU-'0';
		if(lU >= 'a' && lU <= 'f') vU = 10 + lU-'a';
		if(lU >= 'A' && lU <= 'F') vU = 10 + lU-'A';
		if(lL >= '0' && lL <= '9') vL = lL-'0';
		if(lL >= 'a' && lL <= 'f') vL = 10 + lL-'a';
		if(lL >= 'A' && lL <= 'F') vL = 10 + lL-'A';
		arr[x] = vU*16 + vL;
	}
}

void ble_encr_set_ER(uint8_t *er_text, sLLData *ll_link)
{
	test_str_to_arr2(16, er_text, ll_link->encr.ER);
}
void ble_encr_set_IR(uint8_t *ir_text, sLLData *ll_link)
{
	test_str_to_arr2(16, ir_text, ll_link->encr.IR);
}
void ble_encr_generate_keys(sLLData *ll_link)
{
	//fast gen - test
	for(int n = 0; n < 16; n++)
	{
		ll_link->encr.LTK[n] = 151*(3*n+35);
		ll_link->encr.IRK[n] = 211*(11*n+37);
		ll_link->encr.CSRK[n] = 171*(5*n+43);
	}
	ll_link->encr.DIV = ble_rand_num()&0xFFFF;
	ll_link->encr.EDIV = ll_link->encr.DIV;
	ble_rand_fill8(ll_link->encr.rr_rand);
	return;
//	ll_link->encr.EDIV = 0x54FA; //"random number" - for now we don't support storing

	uint8_t dhk[16];
	for(int n = 0; n < 16; n++)
		dhk[n] = n*n + 15; //stupid way to make some key, need to replace
	ble_rand_fill8(ll_link->encr.rr_rand);
		
	uint16_t Y = ble_encr_dm(dhk, ll_link->encr.rr_rand);
	ll_link->encr.DIV = ble_rand_num()&0xFFFF;
	ll_link->encr.EDIV = ll_link->encr.DIV ^ Y;

	ble_encr_d1(ll_link->encr.ER, ll_link->encr.DIV, 0, ll_link->encr.LTK);
	ble_encr_d1(ll_link->encr.ER, ll_link->encr.DIV, 1, ll_link->encr.CSRK);
//	uint32_t gen_rand = 0x6812;
//	ble_encr_d1(ll_link->encr.ER, gen_rand, 0, ll_link->encr.LTK);
//	ble_encr_d1(ll_link->encr.ER, gen_rand, 1, ll_link->encr.CSRK);
	ble_encr_d1(ll_link->encr.IR, 1, 0, ll_link->encr.IRK);
	ble_encr_d1(ll_link->encr.IR, 3, 0, ll_link->encr.DHK);
}
void ble_encr_restore_keys(sLLData *ll_link)
{
	for(int n = 0; n < 16; n++)
	{
		ll_link->encr.LTK[n] = 151*(3*n+35);
		ll_link->encr.IRK[n] = 211*(11*n+37);
		ll_link->encr.CSRK[n] = 171*(5*n+43);
	}
	return;
	uint8_t dhk[16];
	for(int n = 0; n < 16; n++)
		dhk[n] = n*n + 15; //stupid way to make some key, need to replace

	uint16_t Y = ble_encr_dm(dhk, ll_link->encr.er_rand);
	ll_link->encr.DIV = ll_link->encr.EDIV ^ Y;
	ble_encr_d1(ll_link->encr.ER, ll_link->encr.DIV, 0, ll_link->encr.LTK);
	ble_encr_d1(ll_link->encr.ER, ll_link->encr.DIV, 1, ll_link->encr.CSRK);
}
