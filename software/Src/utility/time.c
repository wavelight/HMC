#include "time.h"

#include "stm32f4xx_hal.h"

extern __IO uint32_t uwTick;

time time_get(void)
{
	return uwTick;
}

time time_elapsed(time old, time new)
{
	// I don't want to get 0xFFFFFFFF
	if(old > new) 
	{
		return 0;
	}

	return new - old;
}

time time_elapsed_now(time old)
{
	return time_elapsed(old, time_get());
}