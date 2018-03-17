#ifndef _DEFINITION_H_
#define _DEFINITION_H_

#include <stdio.h>

/* 
 *  General definitions
 */
#define true 	(1)
#define false 	(!true)

// UART
#define UART_USB_BUFFER_SIZE (100)

#define UART_USB_PACKET_START_1 		(0x12)
#define UART_USB_PACKET_START_2 		(0x34)
#define UART_USB_PACKET_START_NULL 		(0xFFFF)

#define UART_USB_PACKET_END_1 			(0x0D)
#define UART_USB_PACKET_END_2 			(0x0A)
#define UART_USB_PACKET_END_NULL 		(0xFFFF)

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
} MOTOR_CONTROL_ALGORITHM;

typedef enum {
	DIRECTION_FORWARD,
	DIRECTION_BACKWARD,

	DIRECTION_MAX,
} DIRECTION;

typedef enum {
	MOTOR_DRIVER_DRV8323 = 0,

	MOTOR_DRIVER_MAX,
} MOTOR_DRIVER_LIST;

/* 
 * Structure definitions
 */
typedef struct {
	MOTOR_DRIVER_LIST 	ic;

	bool enable_status;
	void (*enable)(void*, bool);
} MOTOR_DRIVER;

typedef struct {
	MOTOR_DRIVER					driver;
	MOTOR_CONTROL_ALGORITHM 		algorithm; 
	DIRECTION						direction;
} MOTOR;

#endif