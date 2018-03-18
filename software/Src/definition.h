#ifndef _DEFINITION_H_
#define _DEFINITION_H_

#include <stdio.h>
#include <string.h>

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

typedef enum {
	PWM_CH_1	= 0,
	PWM_CH_1N,
	PWM_CH_2,
	PWM_CH_2N,
	PWM_CH_3,
	PWM_CH_3N,

	PWM_CH_MAX,
} PWM_CHANNEL;

/* 
 * Structure definitions
 */
// Motor driver(DRV8323/etc..)
typedef struct {
	MOTOR_DRIVER_LIST 	ic;

	bool enable_status;
	void (*enable)(void*, bool);
} MOTOR_DRIVER;

// Motor PWM 
typedef struct {
	bool 		(*set_duty)(void*, PWM_CHANNEL, uint16_t);
	uint16_t 	(*get_duty)(void*, PWM_CHANNEL);

	bool 		(*set_freq)(void*, PWM_CHANNEL, uint16_t);
	uint32_t	(*get_freq)(void*, PWM_CHANNEL);
} MOTOR_PWM;

typedef struct {
	MOTOR_DRIVER					driver;
	MOTOR_PWM						pwm;	
	MOTOR_CONTROL_ALGORITHM 		algorithm; 
	DIRECTION						direction;
} MOTOR;

#endif