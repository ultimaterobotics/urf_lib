#include <stdint.h>
#include "ble_const.h"
 
void ble_rand_init();
uint32_t ble_rand_num();
void ble_rand_fill8(uint8_t *buf);
void ble_encr_fill_key(uint8_t *key);
void ble_encr_e(uint8_t *plain, uint8_t *encr);
uint32_t ble_encr_ah(uint8_t *key, uint32_t r);
uint16_t ble_encr_dm(uint8_t *k, uint8_t *r);
void ble_encr_c1(uint8_t *r, uint8_t *preq, uint8_t *pres, uint8_t iat, uint8_t *ia, uint8_t rat, uint8_t *ra, uint8_t *out);
void ble_encr_s1(uint8_t *r1, uint8_t *r2, uint8_t *out);
void ble_encr_ccm_fill_array(sLLData *ll_link);
void ble_encr_ccm_config(uint8_t is_on);

void ble_encr_ccm_test();

void ble_encr_set_ER(uint8_t *er_text, sLLData *ll_link);
void ble_encr_set_IR(uint8_t *ir_text, sLLData *ll_link);
void ble_encr_generate_keys(sLLData *ll_link);
void ble_encr_restore_keys(sLLData *ll_link);