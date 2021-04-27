#include "urf_timer.h"
#include "nrf.h"

//once per TIME_RESOLUTION microseconds timer interrupt is called and all times are updated
//
#define TIME_RESOLUTION 250

volatile uint32_t mcs_time = 0;
volatile uint32_t ms_time = 0;
volatile uint32_t s_time = 0;
volatile uint32_t ms_counter = 0;
volatile uint32_t s_counter = 0;
volatile uint8_t timer_on = 0;

int timer_ints_per_ms = 1000 / TIME_RESOLUTION;

volatile uint8_t phase = 0;

void time_start()
{
	NRF_TIMER2->CC[0] = TIME_RESOLUTION;
	NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER2->BITMODE = 1; //8-bit
	NRF_TIMER2->PRESCALER = 4; //for 16MHz clock

	NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos); 
//	NRF_TIMER2->SHORTS = 0;
	NRF_TIMER2->SHORTS = 1; //COMPARE[0] -> CLEAR
	
//	NVIC_SetPriority(TIMER2_IRQn, 0);	
	NVIC_EnableIRQ(TIMER2_IRQn);
	NVIC_SetPriority(TIMER2_IRQn, 2);
	NRF_TIMER2->TASKS_START = 1;
	timer_on = 1;
}
void time_pause()
{
	NVIC_DisableIRQ(TIMER2_IRQn);
	timer_on = 0;
}
void time_resume()
{
	NVIC_EnableIRQ(TIMER2_IRQn);
	timer_on = 1;
}
void time_stop()
{
	NRF_TIMER2->TASKS_STOP = 1;
	timer_on = 0;
}

uint32_t micros()
{
	NRF_TIMER2->TASKS_CAPTURE[2] = 1;
	return mcs_time + NRF_TIMER2->CC[2];
}
uint32_t millis()
{
	return ms_time;
}
uint32_t seconds()
{
	return s_time;
}

void (*timed_event)(void) = 0;
void (*timed_subevent)(void) = 0;

int timed_event_repeat = 0;
int timed_event_delayed = 0;
uint32_t timed_event_period = 0;
int timed_event_adjustment = 0;
int timed_event_adjustment_state = 0;
int timed_subevent_enabled = 0;

void schedule_event(uint32_t steps_dt, void (*tm_event)(void), int repeated)
{
	timed_subevent_enabled = 0;
	timed_event_repeat = repeated;
	timed_event_period = steps_dt;
	NRF_TIMER3->TASKS_STOP = 1;
	
	timed_event = *tm_event;

	NRF_TIMER3->TASKS_CLEAR = 1;

	NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER3->BITMODE = 3; //32 bits
	NRF_TIMER3->PRESCALER = 0; //16 steps per uS
	NRF_TIMER3->CC[0] = steps_dt;

	NRF_TIMER3->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos); 
	NRF_TIMER3->SHORTS = 1<<8; //compare[0]->stop
	if(timed_event_repeat)
		NRF_TIMER3->SHORTS = 1; //compare[0]->clear
	NVIC_SetPriority(TIMER3_IRQn, 3);
	NVIC_EnableIRQ(TIMER3_IRQn);
	NRF_TIMER3->TASKS_START = 1;
}

void schedule_event_delayed(uint32_t delay, uint32_t steps_dt, void (*tm_event)(void), int repeated)
{
	timed_event_delayed = 1;
	schedule_event(delay, tm_event, 1);
	timed_event_period = steps_dt;
}
void schedule_subevent(uint32_t steps_dt, void (*tm_event)(void))
{
	timed_subevent = *tm_event;
	NRF_TIMER3->CC[1] = steps_dt;
	NRF_TIMER3->INTENSET |= (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos); 
	timed_subevent_enabled = 1;
}

void schedule_event_stop()
{
	NRF_TIMER3->TASKS_STOP = 1;
}
void schedule_event_adjust(int dt)
{
	timed_event_adjustment = dt;
	timed_event_adjustment_state = 1; //need to adjust
}

void TIMER3_IRQHandler(void)
{
	if(NRF_TIMER3->EVENTS_COMPARE[0])
	{
		NRF_TIMER3->EVENTS_COMPARE[0] = 0;
		if(timed_event_delayed)
		{
			NRF_TIMER3->CC[0] = timed_event_period;
			timed_event_delayed = 0;
		}
		timed_event();
		if(!timed_event_adjustment_state) return;
		if(timed_event_adjustment_state == 1)
		{
			uint32_t nt = timed_event_period + timed_event_adjustment;
			if(nt < 50) nt = 50; //too short will fail
			NRF_TIMER3->CC[0] = nt;
			timed_event_adjustment_state = 2;
		}
		else if(timed_event_adjustment_state == 2)
		{
			NRF_TIMER3->CC[0] = timed_event_period;
			timed_event_adjustment_state = 0;
		}
	}
	if(NRF_TIMER3->EVENTS_COMPARE[1]) //sub-event
	{
		NRF_TIMER3->EVENTS_COMPARE[1] = 0;
		if(timed_subevent_enabled)
			timed_subevent();
	}
}

void TIMER2_IRQHandler(void)
{
	NRF_TIMER2->EVENTS_COMPARE[0] = 0;
	mcs_time += TIME_RESOLUTION;
	ms_counter++;
	if(ms_counter >= timer_ints_per_ms)
	{
		ms_time++;
		ms_counter = 0;
		s_counter++;
		if(s_counter >= 1000)
		{
			s_time++;
			s_counter = 0;
		}
	}
}

void delay_ms(uint32_t ms)
{
	if(!timer_on) return;
	volatile uint32_t end = ms_time+ms;
	while(ms_time != end) ;
	return;
}

void delay_mcs(uint32_t mcs)
{
	if(!timer_on) return;
	volatile uint32_t end = micros() + mcs;
	if(mcs_time > end) return; //in case of timer overflow, just return early
		//if someone relies on precise behaviour of this code - it's their fault for not 
		//looking at implementation! (TODO: fix some day)
	while(micros() < end) ;
	return;
}

