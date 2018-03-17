#ifndef _DEFINITION_H_
#define _DEFINITION_H_

#include <stdio.h>

/* 
 *  General definitions
 */
#define true 	(1)
#define false 	(!true)

// UART
#define UART_BUFFER_SIZE (100)

/* 
 * Typedef definitions
 */
typedef uint8_t 	bool;
typedef uint32_t 	time;

/* 
 * enum definitions
 */
typedef enum {
	ALGORITHM_TRAP_SENSORED = 0,
	ALGORITHM_TRAP_SENSORLESS,
	ALGORITHM_FOC_SENSORED,
	ALGORITHM_FOC_SENSORLESS,
	
	ALGORITHM_MAX,
} CONTROL_ALGORITHM;

typedef enum {
	DIRECTION_FORWARD,
	DIRECTION_BACKWARD,

	DIRECTION_MAX,
} DIRECTION;

typedef enum {
	MOTOR_DRIVER_DRV8323 = 0,

	MOTOR_DRIVER_MAX,
} MOTOR_DRIVER;

/* 
 * Structure definitions
 */
typedef struct {
	CONTROL_ALGORITHM 		algorithm; 
	MOTOR_DRIVER			driver;
	DIRECTION				direction;
} MOTOR;

#endif