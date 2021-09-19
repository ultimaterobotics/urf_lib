#include <stdint.h>

void time_start();
void time_pause();
void time_resume();
void time_stop();
void time_adjust(int ms_shift);
uint32_t micros();
uint32_t millis();
uint32_t seconds();
void delay_ms(uint32_t ms);
void delay_mcs(uint32_t mcs);
void schedule_event(uint32_t steps_dt, void (*tm_event)(void), int repeated);
void schedule_subevent(uint32_t steps_dt, void (*tm_event)(void));
void schedule_event_delayed(uint32_t delay, uint32_t steps_dt, void (*tm_event)(void), int repeated);
void schedule_event_adjust(int dt);
void schedule_event_stop();
