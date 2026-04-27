#include "include/UART.h"
#if defined(UART_IMPL_POLLING)
#include "include/UART_polling.h"
#include "include/gpio.h"
#include "include/port.h"
#include <stdint.h>

#define UART_ALTS 5
#define UART_HAL_DEFAULT_BAUDRATE 9600

typedef struct {
	uint32_t uart_num;
	uint32_t uart_port;
	pin_t tx_pin;
	pin_t rx_pin;
	PORTMux_t alt;
} UARTConfig_t;

// @todo maybe add the other alternatives ?? preguntar
static UARTConfig_t uart_pin_map[UART_ALTS] = {{0, PB, PORTNUM2PIN(PB, 17), PORTNUM2PIN(PB, 16), PORT_mAlt3},
											   {1, PE, PORTNUM2PIN(PE, 0), PORTNUM2PIN(PE, 1), PORT_mAlt3},
											   {2, PD, PORTNUM2PIN(PD, 3), PORTNUM2PIN(PD, 2), PORT_mAlt3},
											   {3, PC, PORTNUM2PIN(PC, 17), PORTNUM2PIN(PC, 16), PORT_mAlt3},
											   {4, PE, PORTNUM2PIN(PE, 24), PORTNUM2PIN(PE, 25), PORT_mAlt3}};
// Index is uart number, value is the index in the map it refers to
static uint32_t uart_num_index[UART_COUNT] = {INVALID_UART, INVALID_UART, INVALID_UART, INVALID_UART, INVALID_UART};

static UART_Type *const uart_ptrs[] = UART_BASE_PTRS;

// Lookup for clock gating and nvic ints
uint32_t const uart_sim_masks[UART_COUNT] = {SIM_SCGC4_UART0_MASK, SIM_SCGC4_UART1_MASK, SIM_SCGC4_UART2_MASK,
											 SIM_SCGC4_UART3_MASK, SIM_SCGC1_UART4_MASK};
uint32_t const uart_nvim_ints[UART_COUNT] = {UART0_RX_TX_IRQn, UART1_RX_TX_IRQn, UART2_RX_TX_IRQn, UART3_RX_TX_IRQn,
											 UART4_RX_TX_IRQn};

static uint32_t find_uart_id(pin_t tx, pin_t rx);

uint32_t UART_polling_drv_instance_init(pin_t RX_pin, pin_t TX_pin) {
	uint32_t id = find_uart_id(TX_pin, RX_pin);
	if (id == INVALID_UART) {
		return INVALID_UART;
	}

	// enable clock for the pin port
	SIM->SCGC5 |= port_clock_masks[PIN2PORT(uart_pin_map[uart_num_index[id]].rx_pin)];

	// enable clock for uart module
	if (id != 4) {
		SIM->SCGC4 |= uart_sim_masks[id];
	} else {
		SIM->SCGC1 |= uart_sim_masks[id];
	}

	// enable nvic for that uart
	// NVIC_EnableIRQ(uart_nvim_ints[id]);

	UART_polling_set_baudrate(id, 9600);

	/* configure rx and tx pins */
	port_ptrs[uart_pin_map[uart_num_index[id]].uart_port]->PCR[PIN2NUM(uart_pin_map[uart_num_index[id]].tx_pin)] = 0x0;
	port_ptrs[uart_pin_map[uart_num_index[id]].uart_port]->PCR[PIN2NUM(uart_pin_map[uart_num_index[id]].tx_pin)] |=
		PORT_PCR_MUX(uart_pin_map[uart_num_index[id]].alt);
	port_ptrs[uart_pin_map[uart_num_index[id]].uart_port]->PCR[PIN2NUM(uart_pin_map[uart_num_index[id]].tx_pin)] |=
		PORT_PCR_IRQC(PORT_eDisabled);

	port_ptrs[uart_pin_map[uart_num_index[id]].uart_port]->PCR[PIN2NUM(uart_pin_map[uart_num_index[id]].rx_pin)] = 0x0;
	port_ptrs[uart_pin_map[uart_num_index[id]].uart_port]->PCR[PIN2NUM(uart_pin_map[uart_num_index[id]].rx_pin)] |=
		PORT_PCR_MUX(uart_pin_map[uart_num_index[id]].alt);
	port_ptrs[uart_pin_map[uart_num_index[id]].uart_port]->PCR[PIN2NUM(uart_pin_map[uart_num_index[id]].rx_pin)] |=
		PORT_PCR_IRQC(PORT_eDisabled);

	// enable uart transmission
	uart_ptrs[id]->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;

	return id;
}

void UART_polling_set_baudrate(uint32_t uart_id, uint32_t baudrate) {
	uint16_t sbr, brfa;
	uint32_t clock;

	clock = ((uart_id == 0) || (uart_id == 1)) ? (__CORE_CLOCK__) : (__CORE_CLOCK__ >> 1);
	baudrate = ((baudrate == 0) ? (UART_HAL_DEFAULT_BAUDRATE) :
								  ((baudrate > 0x1FFF) ? (UART_HAL_DEFAULT_BAUDRATE) : (baudrate)));

	sbr = clock / (baudrate << 4);				 // sbr = clock/(Baudrate x 16)
	brfa = (clock << 1) / baudrate - (sbr << 5); // brfa = 2*Clock/baudrate - 32*sbr

	uart_ptrs[uart_id]->BDH = UART_BDH_SBR(sbr >> 8);
	uart_ptrs[uart_id]->BDL = UART_BDL_SBR(sbr);
	uart_ptrs[uart_id]->C4 = (uart_ptrs[uart_id]->C4 & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
}

void UART_polling_data_transmit(uint32_t uart_id, unsigned char txdata) {
	while (((uart_ptrs[uart_id]->S1) & UART_S1_TDRE_MASK) == 0)
		;
	uart_ptrs[uart_id]->D = txdata;
}

unsigned char UART_polling_data_receive(uint32_t uart_id) {
	while (((uart_ptrs[uart_id]->S1) & UART_S1_RDRF_MASK) == 0)
		;
	return uart_ptrs[uart_id]->D;
}

static uint32_t find_uart_id(pin_t tx, pin_t rx) {
	int i = 0;
	while (i != UART_ALTS) {
		if (uart_pin_map[i].tx_pin == tx && uart_pin_map[i].rx_pin == rx) {
			uart_num_index[uart_pin_map[i].uart_num] = i;
			return uart_pin_map[i].uart_num;
		}
		i++;
	}
	return INVALID_UART;
}
#endif