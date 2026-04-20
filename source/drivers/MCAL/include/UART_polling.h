#ifndef _UART_POLLING_H_
#define _UART_POLLING_H_
#include <stdint.h>
#include "gpio.h"
#define UART_NUM_MAX 4 // last uart
#define UART_COUNT 5

#define INVALID_UART 255
/******************** POLLING UART DRV IMPLEMENTATION ******************************/
/**
* @brief Initialize uart instance
* @returns A number representing uart id, or INVALID_UART if unsuccessful
* @param RX_pin Receiver pin
* @param TX_pin Transciever pin
**/
uint32_t UART_polling_drv_instance_init (pin_t RX_pin, pin_t TX_pin);

/**
* @brief Set baudrate for a uart number (uarts go form 0 to 4)
**/
void UART_polling_set_baudrate (uint32_t uart_id, uint32_t baudrate);

/**
* @brief Transmit data (function blocks until al bytes have been sent)
**/
void UART_polling_data_transmit(uint32_t uart_id,unsigned char txdata);

/**
* @brief Receive data (function blocks until all bytes have been received)
**/
unsigned char UART_polling_data_receive(uint32_t uart_id);


#endif /* _UART_POLLING_H_ */