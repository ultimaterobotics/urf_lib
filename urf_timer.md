### urf_timer functions

Time functions are using hardware timer TIMER2 for time counting and delays, and TIMER3 for scheduling. Both timers should not be used by other project code.
Time counting is based on calling interrupt once per 250 microseconds by default (can be changed by changing TIME_RESOLUTION define, but it must not exceed 1000 and preferably should be divisor of 1000)

#### void time_start()
Initializes TIMER2, sets time to zero, activates time interrupt. Should be called once at the project beginning, after HFO is initialized.

#### void time_pause()
Disables TIMER2 interrupt, so no time is counted - but timer is active (and consuming power).

#### void time_resume()
Enables back TIMER2 interrupt, but doesn't enable timer hardware itself if it was powered down. Intended to be used only in pair with time_pause().

#### void time_stop()
Disables TIMER2 interrupt and shuts it down. Intended to be used prior entering low power mode, followed by time_start() after wake-up. Such process resets all time variables to zero, so **time_adjust** may be used to restore them

#### void time_adjust(uint32_t ms_shift)
Adds ms_shift milliseconds to current time, adjusting milliseconds, microseconds and seconds counters correspondingly.

#### uint32_t micros()
Returns current time in microseconds based on last timer interrupt and current timer counter values, is 1 microsecond-precise.

#### uint32_t millis()
Returns current time in milliseconds based on last timer interrupt. Has precision of TIME_RESOLUTION microseconds (250 by default).

#### uint32_t seconds()
Returns current time in seconds

#### void delay_ms(uint32_t ms)
Blocking delay in milliseconds (has the same precision as millis())

#### void delay_mcs(uint32_t mcs)
Blocking delay in microseconds (close to 1-microsecond precise, may take a bit longer due to micros() function call overhead)

#### void schedule_event(uint32_t steps_dt, void (* tm_event)(void), int repeated)
Schedule event using TIMER3 interrupt. tm_event() function will be called in steps_dt/16 microseconds + interrupt overhead time (very few instructions). If repeated is >0, it will be called repeatedly with that interval regardless of how long it took to process, so need to be careful if long processing time is possible.

#### void schedule_subevent(uint32_t steps_dt, void (* tm_event)(void))
Schedule seconday event which will be called before main event scheduled by schedule_event(...), steps_dt must be smaller than scheduled period for the main event.

#### void schedule_event_delayed(uint32_t delay, uint32_t steps_dt, void (* tm_event)(void), int repeated)
Schedule event in the same manner as schedule_event(...), but first interval would be delay/16 microseconds, while subsequent ones would be repeated at steps_dt/16 microseconds interval. Makes no sense to use with repeated = 0.

#### void schedule_event_adjust(int dt)
Next scheduled event cycle after calling this function would be dt/16 microseconds longer/shorter (depending on sign) than requested period, without affecting timing of following cycles. Useful to adjust repeated events if they drift out of sync with something (used by BLE stack to keep connection frames synced with requests - when requests consistently come earlier / later than expected, timing is adjusted)

#### void schedule_event_stop()
Stop all scheduled events by stopping TIMER3
