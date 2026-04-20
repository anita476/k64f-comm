#ifndef _UART_NONBLOCKING_H_
#define _UART_NONBLOCKING_H_
#include <stdbool.h>
#include <stdint.h>
#include "gpio.h"

#define UART_NUM_MAX 4 // last uart
#define UART_COUNT 5

#define INVALID_UART 255
/******************** NON BLOCKING UART DRV IMPLEMENTATION ******************************/
/**
* @brief Initialize uart instance
* @returns A number representing uart id, or INVALID_UART if unsuccessful
* @param RX_pin Receiver pin
* @param TX_pin Transciever pin
**/
uint32_t UART_nonblocking_drv_instance_init (pin_t RX_pin, pin_t TX_pin);

/**
* @brief Query the read status of a uart
* @param uart_id The uart to query
* @returns True if theres pending data to be read, false if not
**/
bool UART_nonblocking_rstatus(uint32_t uart_id);

/**
*
* @brief Query the transmit status of a uart
* @param uart_id The uart id 
* @returns True if data can be transmitted, false if not.
*/
bool UART_nonblocking_tstatus(uint32_t uart_id);

/**
* @brief Set baudrate for a uart number (uarts go form 0 to 4)
**/
void UART_nonblocking_set_baudrate (uint32_t uart_id, uint32_t baudrate);

/**
* @brief Queue data to send. (Non blocking)
**/
void UART_nonblocking_data_transmit(uint32_t uart_id,unsigned char txdata);

/**
* @brief Receive data if available. This clears the read status
**/
unsigned char UART_nonblocking_data_receive(uint32_t uart_id);



#endif /*_UART_NONBLOCKING_H_*/