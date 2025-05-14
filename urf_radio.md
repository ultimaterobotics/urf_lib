### urf_radio functions

Functions that are wrapping register settings, interrupts, shorts of NRF_RADIO hardware and handle rx/tx states

#### void rf_init(int channel, int speed, int crc_len)
Simple radio init on a fixed channel (range 0-100 corresponding to frequencies 2400...2500 MHz), with given radio speed (allowed values are 250, 1000 and 2000 KBit/s) and given CRC length (allowed values 0-3). If two devices are configured with the same parameters, they can send data to each other (as long as one is listening, another is sending in each of communication events - device can't simultaneously send and listen).

#### void rf_init_ext(int channel, int speed, int crc_len, int white_en, int s1_sz, int added_length, int max_length)
Same as **rf_init** but with additional parameters from PCNF0, PCNF1 of the NRF_RADIO: white_en - enable whitening, s1_sz - size of S1 field, added_length - number of bytes transmitted after packet length is sent, max_length - maximum transmitted length 

#### void rf_disable()
Disable radio (blocking but fast even in the worst case)

#### void rf_mode_rx_only()
Switch radio into continuous listening mode, if packet is received - continue listening (don't use without filling RX buffer pointer! **rf_listen** takes care of that and should be used by default)

#### void rf_mode_tx_only()
Send current packet and stop sending/listening (don't use without filling TX buffer pointer! **rf_send** takes care of that and should be used by default)

#### void rf_mode_tx_then_rx()
Send current packet and start listening, if packet is received - continue listening (don't use without filling RX buffer pointer! **rf_listen** takes care of that and should be used by default)

#### void rf_send(uint8_t * pack, int length)
Send packet with given length. pack[1] must be equal to packet length in order to be sent successfully: NRF radio hardware uses this byte as length field

#### void rf_listen()
Sets NRF radio hardware packet pointer to default temporary buffer and calls **rf_mode_rx_only**, should be used as default way to listen for a new packet

#### void rf_send_and_listen(uint8_t * pack, int length)
Sends packet in the same way as **rf_send** and then switched into listening mode in the same way as **rf_listen**

#### void rf_autorespond_on(uint8_t * pack_crc_ok, uint8_t * pack_crc_fail, int resp_pack_length)
#### void rf_autorespond_off()
#### int rf_get_ack_state();
Those 3 functions are not properly implemented, please don't use them for now.

#### void rf_override_irq(void (* new_irq))
Calls new_irq() function when RADIO_IRQHandler is called instead of default processing. This is used in BLE mode. Overall, don't use it if you are not sure how it works.

#### int rf_has_new_packet()
Indicates if any packet was received since last call of **rf_get_packet**. Returns 1 if something was received, 0 otherwise.

#### uint32_t rf_get_packet(uint8_t * pack)
Fills **pack** with the most recent received packet, returns packet length. **pack** must have 256 bytes of memory allocated.

#### int rf_is_busy()
Returns 1 if radio is currently busy (has packet to send, but didn't finish sending yet), 0 otherwise. Normally it should clear in less than 1 millisecond, but once in a rare while it doesn't happen, so can't count on that

#### void rf_clear_events()
Clear all radio events - useful if IRQ is called for unhandled reason
