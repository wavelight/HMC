#include "drv8323.h"

void drv8323_enable(void * motorPtr, bool enable);

bool drv8323_initialize(MOTOR * motor)
{
	// Link Functions and initialize DRV8323

	motor->driver.enable = drv8323_enable;
	
	return true;
}

void drv8323_enable(void * motorPtr, bool enable)
{
	MOTOR * motor = (MOTOR *)motorPtr;

	// TODO : GPIO Output/SPI Output

	motor->driver.enable_status = enable;
}
