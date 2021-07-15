#ifndef BLE_CONST__H
#define BLE_CONST__H

#include <stdint.h>

//#define BLE_DEBUG_PRINTS

//=============ADVERTISING SECTION=============

#define BLE_ADV_ADDR 0x8E89BED6

#define BLE_ADV_IND_TYPE 			0
#define BLE_ADV_DIRECT_IND_TYPE		0b0001
#define BLE_ADV_NONCONN_IND_TYPE	0b0010
#define BLE_SCAN_REQ_TYPE			0b0011
#define BLE_SCAN_RSP_TYPE			0b0100
#define BLE_CONNECT_REQ_TYPE		0b0101
#define BLE_ADV_SCAN_IND_TYPE		0b0110

typedef struct {
	union
	{
		struct {
			unsigned type : 4;
			unsigned : 2;
			unsigned tx_addr : 1;
			unsigned rx_addr : 1;
		};
		uint8_t header;
	};
} ble_adv_header1;

//=============END OF ADVERTISING SECTION=============

//=============LINK LAYER SECTION=============

#define LL_CONNECTION_UPDATE_REQ 0x00
#define LL_CHANNEL_MAP_REQ 0x01
#define LL_TERMINATE_IND 0x02
#define LL_ENC_REQ 0x03
#define LL_ENC_RSP 0x04
#define LL_START_ENC_REQ 0x05
#define LL_START_ENC_RSP 0x06
#define LL_UNKNOWN_RSP 0x07
#define LL_FEATURE_REQ 0x08
#define LL_FEATURE_RSP 0x09
#define LL_PAUSE_ENC_REQ 0x0A
#define LL_PAUSE_ENC_RSP 0x0B
#define LL_VERSION_IND 0x0C
#define LL_REJECT_IND 0x0D
#define LL_SLAVE_FEATURE_REQ 0x0E
#define LL_CONNECTION_PARAM_REQ 0x0F
#define LL_CONNECTION_PARAM_RSP 0x10
#define LL_REJECT_IND_EXT 0x11
#define LL_PING_REQ 0x12
#define LL_PING_RSP 0x13
#define LL_LENGTH_REQ 0x14
#define LL_LENGTH_RSP 0x15

typedef struct {
	union
	{
		struct {
			unsigned LLID : 2;
			unsigned NESN : 1;
			unsigned SN : 1;
			unsigned MD : 1;
			unsigned CP : 1; //BLE 5.2
			unsigned : 2;
		};
		uint8_t header;
	};
} ble_LL_data_header1;

typedef struct sCCMconf
{
	volatile uint8_t key[16];
	volatile uint32_t packet_counter_our;
	volatile uint32_t packet_counter_msb_our; //must be 4 bytes, with ignored bytes 1-3 and bit 7 of byte 0
	volatile uint32_t packet_counter_tgt;
	volatile uint32_t packet_counter_msb_tgt; //must be 4 bytes, with ignored bytes 1-3 and bit 7 of byte 0
	volatile uint8_t direction;
	volatile uint8_t IV[8];
	volatile uint8_t compr_array[40];
}sCCMconf;

typedef struct sLLEncryption
{
	uint8_t enabled;
	uint8_t er_rand[8];
	uint8_t rr_rand[8];
	uint16_t EDIV;
	uint16_t DIV;
	uint8_t SKDm[8];
	uint8_t IVm[4];
	uint8_t SKDs[8];
	uint8_t IVs[4];
	uint8_t ER[16];
	uint8_t IR[16];
	uint8_t LTK[16];
	uint8_t IRK[16];
	uint8_t DHK[16];
	uint8_t CSRK[16];
	uint8_t key_length;
	sCCMconf ccm_data;
}sLLEncryption;

typedef struct sLLData
{
	uint8_t tgt_mac[6];
	uint8_t our_mac[6];
	uint8_t our_addr_type;
	uint8_t tgt_addr_type;
	
	uint32_t AA;
	uint32_t CRC;
	uint8_t win_size;
	uint16_t offset;
	volatile uint16_t interval;
	uint16_t latency;
	uint16_t timeout;
	uint8_t chan_map[5];
	uint8_t hop;
	uint8_t sca;
	
	uint8_t last_channel;
	uint8_t chan_cnt;
	
	uint16_t event_count;
	uint16_t instant;
	
	uint32_t last_rx_event_time;
	uint32_t link_start_time;
	uint8_t SN;
	uint8_t NESN;
	
	uint8_t our_params_requested;
	uint8_t reached_att_stage; //if connection failed to reach ATT soon enough, something is wrong
	//and we should drop it, also trying to switch the response mode
	
	sLLEncryption encr;
}sLLData;

typedef struct sL2CAP_header
{
	uint16_t length;
	uint16_t CID;
}sL2CAP_header;

//=============END OF LINK LAYER SECTION=============

//=============SMP SECTION=============

#define SMP_PAIR_REQUEST		0x01 
#define SMP_PAIR_RESPONSE		0x02
#define SMP_PAIR_CONFIRM		0x03
#define SMP_PAIR_RANDOM			0x04
#define SMP_PAIR_FAILED			0x05
#define SMP_ENCRYPTION_INFO		0x06
#define SMP_MASTER_IDENT		0x07
#define SMP_IDENTITY_INFO		0x08
#define SMP_IDENTITY_ADDR_INFO	0x09
#define SMP_SIGNING_INFO		0x0A
#define SMP_SEQURITY_REQ		0x0B
#define SMP_PAIRING_PUBKEY		0x0C
#define SMP_PAIRING_DHKEY		0x0D
#define SMP_PAIRING_KEYPRESS	0x0E

//=============GATT + ATT SECTION=============

#define ATT_ERROR_RSP 0x01
#define ATT_EXCHANGE_MTU_REQ 0x02
#define ATT_EXCHANGE_MTU_RSP 0x03 
#define ATT_FIND_INFORMATION_REQ 0x04
#define ATT_FIND_INFORMATION_RSP 0x05
#define ATT_FIND_BY_TYPE_VALUE_REQ 0x06
#define ATT_FIND_BY_TYPE_VALUE_RSP 0x07
#define ATT_READ_BY_TYPE_REQ 0x08
#define ATT_READ_BY_TYPE_RSP 0x09
#define ATT_READ_REQ 0x0A
#define ATT_READ_RSP 0x0B
#define ATT_READ_BLOB_REQ 0x0C
#define ATT_READ_BLOB_RSP 0x0D
#define ATT_READ_MULTIPLE_REQ 0x0E
#define ATT_READ_MULTIPLE_RSP 0x0F
#define ATT_READ_BY_GROUP_TYPE_REQ 0x10
#define ATT_READ_BY_GROUP_TYPE_RSP 0x11
#define ATT_WRITE_REQ 0x12
#define ATT_WRITE_RSP 0x13
#define ATT_WRITE_CMD 0x52
#define ATT_PREPARE_WRITE_REQ 0x16
#define ATT_PREPARE_WRITE_RSP 0x17
#define ATT_EXECUTE_WRITE_REQ 0x18
#define ATT_EXECUTE_WRITE_RSP 0x19
#define ATT_READ_MULTIPLE_VARIABLE_REQ 0x20
#define ATT_READ_MULTIPLE_VARIABLE_RSP 0x21
#define ATT_MULTIPLE_HANDLE_VALUE_NTF 0x23
#define ATT_HANDLE_VALUE_NTF 0x1B
#define ATT_HANDLE_VALUE_IND 0x1D
#define ATT_HANDLE_VALUE_CFM 0x1E
#define ATT_SIGNED_WRITE_CMD 0xD2

#define ATT_MTU_DEFAULT 23

#define ATT_ERROR_INVALID_HANDLE 0x01
#define ATT_ERROR_INVALID_PDU 0x04
#define ATT_ERROR_NOT_SUPPORTED 0x06
#define ATT_ERROR_ATTRIBUTE_NOT_FOUND 0x0A
#define ATT_ERROR_UNLIKELY 0x0E

#define UUID_PRIMARY_SERVICE 0x2800
#define UUID_SECONDARY_SERVICE 0x2801
#define UUID_INCLUDE 0x2802
#define UUID_CHARACTERISTIC 0x2803

#define CHARACTERISTIC_BROADCAST 0x01
#define CHARACTERISTIC_READ 0x02
#define CHARACTERISTIC_WRITE_NO_RESP 0x04
#define CHARACTERISTIC_WRITE 0x08
#define CHARACTERISTIC_NOTIFY 0x10
#define CHARACTERISTIC_INDICATE 0x20
#define CHARACTERISTIC_SIGNED_WRITES 0x40
#define CHARACTERISTIC_EXTENDED_PROPERTIES 0x80


#define VALUE_TYPE_UINT16 0x07
#define VALUE_TYPE_UINT32 0x08
#define VALUE_TYPE_UINT64 0x0A
#define VALUE_TYPE_UINT128 0x0B
#define VALUE_TYPE_SINT32 0x10
#define VALUE_TYPE_FLOAT32 0x14
#define VALUE_TYPE_UTF8 0x19
#define VALUE_TYPE_STRUCT 0x1B



#endif