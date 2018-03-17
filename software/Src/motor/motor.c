#include "motor.h"

void motor_initalize_driver(MOTOR * motor);

void motor_initialize(MOTOR * motor)
{
	motor->driver 		= MOTOR_DRIVER_DRV8323;
    motor->algorithm 	= ALGORITHM_TRAP_SENSORED;
	motor->direction 	= DIRECTION_FORWARD;

	motor_initalize_driver(motor);
}

void motor_initalize_driver(MOTOR * motor)
{
	switch(motor->driver)
	{
	case MOTOR_DRIVER_DRV8323:
		drv8323_initialize(motor);
		break;
	default:
		break;
	}
}

void readFault(MOTOR * motor)
{
	
}