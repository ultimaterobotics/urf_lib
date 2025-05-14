### BLE functions
This library implements basic BLE advertising and connected states, GATT discovery, characteristics reading/writing, notifications etc.
**Nost functions are defined in urf_ble_peripheral.h and urf_ble_att_process.h. Constants are defined there and in ble_const.h**

Makefile should include urf_ble_peripheral.c, urf_ble_att_process.c, urf_ble_smp_process.c and urf_ble_encryption.c 
Typical BLE workflow:

##### Declare services and characteristics:
```
sService GAP_service;
sCharacteristic gap_name_ch;
```
##### Fill their parameters:
```
GAP_service.handle = 0x01;
GAP_service.type = 0x2800;
GAP_service.group_end = 0x0F;
GAP_service.uuid_16 = 0x1800;
ble_add_service(&GAP_service);
  
gap_name_ch.handle = 0x02;
gap_name_ch.value_handle = 0x03;
gap_name_ch.uuid_16 = 0x2A00;
gap_name_ch.descriptor_count = 0;
gap_name_ch.val_length = sprintf(gap_name_ch.value, "Device name");
gap_name_ch.val_type = VALUE_TYPE_UTF8;
gap_name_ch.properties = CHARACTERISTIC_READ;
ble_add_characteristic(&gap_name_ch);
GAP_service.char_idx[0] = gap_name_ch.mem_idx;  
GAP_service.char_count = 1;
```
##### Init BLE (once in the beginning)
```
ble_init_radio();
```
##### Then send (properly filled) advertising packets
Here is an example of sending an advertising packet which tells that we are a discoverable BLE device with a name:
```
uint8_t pdu[40];
uint8_t payload[40];
		
for(int x = 0; x < 6; x++)
  payload[x] = ble_mac[x];
int pp = 6;
payload[pp++] = 0x02; //2 bytes of field length
payload[pp++] = 0x01; //field type: flags
payload[pp++] = 0b0110; //general discovery, br/edr not supported

uint8_t name[32];
int nlen = sprintf(name, "test");
payload[pp++] = nlen+1; //field length: name length + 1 byte for type
payload[pp++] = 0x08; //type: short device name
for(int x = 0; x < nlen; x++)
  payload[pp++] = name[x];
payload[pp++] = 0; //just in case
int adv_ch = 37; //37,38,39 are advertising channels
int len = ble_prepare_adv_pdu(pdu, 35, payload, BLE_ADV_IND_TYPE, 0, 1);
ble_LL_send_PDU(0x8E89BED6, len, pdu, adv_ch);
```
And when some device will respond to this advertising - a whole lot of code would be automatically called inside, establishing connection, answerind to discovery requests etc (if services/characteristics from above were filled), resulting in device being able to read/write/get notifications from defined characteristics.

When BLE connection is established, you can update characteristics values at any point - all subsequent reads would return updated value. 

#### For sending notifications, following procedure should be used:
1. Add notifications on/off descriptor (UUID 0x2902) during corresponding characteristic initialization:
```
activity_ch.descriptor_count = 1;
activity_ch.descriptor_uuids[0] = 0x2902;
activity_ch.descriptor_handles[0] = activity_ch.handle+2;
activity_ch.descriptor_values[0] = 0;
```
2. When you have a new value that needs to be sent via notification, check if notification is on, and write it:
```
if(activity_ch.descriptor_values[0] > 0)
{
  activity_ch.changed = 1;
  activity_ch.value[x] = some_new_value;
  activity_ch.val_length = length_of_that_value;
}
```
#### For catching characteristics write events:
When characteristics was written, its .had_write flag is set to 1. At any point in custom code, you can check it in the following way:
```
if(activity_ch.had_write)
{
  activity_ch.had_write = 0;
  process_in_any_way(activity_ch.value);
}
```

### Functions
Most functions declared in header file are not supposed to be called from the outside if you don't want to change default processing pipeline. Functions that should be called:

#### void ble_init_radio()
Initializes radio in BLE mode. Should be called at the beginning

#### int ble_add_service(sService* srv)
Adds service into list of services which would be discoverable via GATT requests. Library uses only pointer, object itself must be allocated and kept in memory by user code.

#### int ble_add_characteristic(sCharacteristic* chr)
Adds characteristics into list of characteristics which would be discoverable via GATT requests.
**Important:** function returns characteristics internal ID number. This ID number must be written into some sService object .char_idx[n] field (see example above) - only then it will be visible via GATT.
Library uses only pointer, object itself must be allocated and kept in memory by user code.

#### void ble_uuid_from_text(uint8_t* uuid, uint8_t * uuid_str)
Allows to set 128-bit UUID field from standard string form:
ble_uuid_from_text(activity_ch.uuid_128, "9314A400-1EA3-5BA0-B43A-35AC4F240E00") - fills activity_ch.uuid_128 with provided text UUID.

#### void ble_peripheral_set_ER(uint8_t * er_text)
Set ER key (for encryption purposes) from hex-formatted string (expects 32-character hex string like "000102030405060708090A0B10111213")

#### void ble_peripheral_set_IR(uint8_t * ir_text)
Set IR key (for encryption purposes), same format as ER key

#### void ble_peripheral_generate_keys()
Generates other encryption keys based on ER, IR

#### void ble_peripheral_generate_mac(uint8_t * res)
Generates 6-byte resolvable MAC address (according to BLE specs, random MAC is generated in a way that first 24 bits are linked to second 24 bits via encryption function). May be required by some devices to establish encrypted connection

#### void ble_set_our_mac(uint8_t * mac)
Set MAC address, expects 6 bytes with actual mac values, not hex string

#### int ble_prepare_adv_pdu(uint8_t * pdu, int payload_len, uint8_t * payload, uint8_t type, uint8_t rand_rx, uint8_t rand_tx)
Packs input data into advertising channel PDU for further sending. Result is packed into **pdu**, at least 3+payload_len bytes should be allocated for it. Returns packed size (3+payload_len). 
Input parameters: **payload_len** - length of payload, **payload** - array with payload, **type** - BLE packet type (check ble_const.h for standard values like BLE_ADV_IND_TYPE), **rand_rx** - if 1, intended for random receiver MAC, **rand_tx** - if 1, our MAC is random

#### void ble_LL_send_PDU(uint32_t addr, int pdu_length, uint8_t * pdu, int ble_channel)
Send previously prepared PDU over channel **ble_channel**, for advertising **addr** should be 0x8E89BED6

#### int ble_get_conn_state()
Returns 1 if some device is connected/trying to connect, 0 otherwise (should be used to decide whether we need to send advertisement packets: if connection is already running, sending advertisement may break it)

#### int ble_get_current_MTU()
Returns current MTU size (37 by default but could be increased per request from connected device)

#### int ble_channel_to_frequency(int channel)
Returns frequency code for NRF Radio hardware for given BLE channel
