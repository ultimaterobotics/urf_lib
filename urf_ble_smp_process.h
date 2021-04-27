#include <stdint.h>
#include "ble_const.h"

#define SMP_KEY_MASK_LTK  0b0001
#define SMP_KEY_MASK_IRK  0b0010
#define SMP_KEY_MASK_CSRK 0b0100
#define SMP_KEY_MASK_EDIV 0x1000
#define SMP_KEY_MASK_IAI  0x2000

typedef struct sSMP_link
{
	uint8_t response_pending;
	uint16_t response_length;
	uint8_t response_buf[64];
	
	uint8_t pair_req[7];
	uint8_t pair_rsp[7];
	
	uint8_t our_pairing_random[16];
	uint8_t tgt_pairing_random[16];
	uint8_t our_confirm[16];
	uint8_t tgt_confirm[16];
	
	int expected_key_mask;
	uint8_t in_pairing;
	uint8_t is_secure;
	uint8_t pause_cnt;
}sSMP_link;

void ble_process_SMP(int length, volatile uint8_t *pdu, sSMP_link *smp_link, sLLData *ll_link);
int ble_check_SMP(sSMP_link *smp_link, sLLData *ll_link);
void ble_SMP_notify_security(sSMP_link *smp_link, int is_secure);
