#include "include/UART_nonblocking.h"
#include "include/port.h"
#define UART_ALTS 5
#define MAX_UART_USE 5 /* Max number of uarts that can be in use at any given time*/
#define BUFFER_CHAR_SIZE 32
#define UART_HAL_DEFAULT_BAUDRATE 9600
#define UART_SENTINEL 0xFF
typedef struct {
	uint32_t uart_num;
	uint32_t uart_port;
	pin_t tx_pin;
	pin_t rx_pin;
	PORTMux_t alt;
} UARTConfigNB_t;

// for now only one pin set per uart number
static UARTConfigNB_t uart_pin_map_NB[UART_COUNT] = {{0, PB, PORTNUM2PIN(PB, 17), PORTNUM2PIN(PB, 16), PORT_mAlt3},
													 {1, PE, PORTNUM2PIN(PE, 0), PORTNUM2PIN(PE, 1), PORT_mAlt3},
													 {2, PD, PORTNUM2PIN(PD, 3), PORTNUM2PIN(PD, 2), PORT_mAlt3},
													 {3, PC, PORTNUM2PIN(PC, 17), PORTNUM2PIN(PC, 16), PORT_mAlt3},
													 {4, PE, PORTNUM2PIN(PE, 24), PORTNUM2PIN(PE, 25), PORT_mAlt3}};

static UART_Type *const uart_ptrs_NB[] = UART_BASE_PTRS;

// Lookup for clock gating and nvic ints
uint32_t const uart_sim_masks_NB[UART_COUNT] = {SIM_SCGC4_UART0_MASK, SIM_SCGC4_UART1_MASK, SIM_SCGC4_UART2_MASK,
												SIM_SCGC4_UART3_MASK, SIM_SCGC1_UART4_MASK};
uint32_t const uart_nvim_ints_NB[UART_COUNT] = {UART0_RX_TX_IRQn, UART1_RX_TX_IRQn, UART2_RX_TX_IRQn, UART3_RX_TX_IRQn,
												UART4_RX_TX_IRQn};

/********** Store buffers for all uarts ************/
typedef struct {
	uint8_t id;
	/***************** CRITICAL ACCESS ZONE ************/
	uint8_t buffer[BUFFER_CHAR_SIZE];
	uint8_t _head; /* WRITE HERE*/
	uint8_t _tail; /* READ FROM HERE */
				   /***************************************************/
} UartBuffer_t;

typedef struct {
	UartBuffer_t tx;
	UartBuffer_t rx;
	bool active; // true once init'd
} UartState_t;

/* uart states ! */
static volatile UartState_t uart_state[MAX_UART_USE];
volatile uint8_t uart_count;

/* forward declarations */
static uint8_t buf_next(uint8_t idx);
static bool buf_is_empty(const UartBuffer_t *b);
static bool buf_is_full(const UartBuffer_t *b);
static bool buf_enqueue(UartBuffer_t *b, uint8_t byte);
static uint8_t buf_dequeue(UartBuffer_t *b);
static uint32_t find_uart_id(pin_t tx, pin_t rx);

uint32_t UART_nonblocking_drv_instance_init(pin_t RX_pin, pin_t TX_pin) {
	uint8_t id = find_uart_id(TX_pin, RX_pin);
	if (id == INVALID_UART || uart_count >= MAX_UART_USE) {
		return INVALID_UART;
	}

	// enable clock for the pin port
	SIM->SCGC5 |= port_clock_masks[PIN2PORT(uart_pin_map_NB[id].rx_pin)];

	// enable clock for uart module
	if (id != 4) {
		SIM->SCGC4 |= uart_sim_masks_NB[id];
	} else {
		SIM->SCGC1 |= uart_sim_masks_NB[id];
	}

	// enable nvic for that uart
	NVIC_EnableIRQ(uart_nvim_ints_NB[id]);

	UART_nonblocking_set_baudrate(id, 9600);

	/* configure rx and tx pins */
	port_ptrs[uart_pin_map_NB[id].uart_port]->PCR[PIN2NUM(uart_pin_map_NB[id].tx_pin)] = 0x0;
	port_ptrs[uart_pin_map_NB[id].uart_port]->PCR[PIN2NUM(uart_pin_map_NB[id].tx_pin)] |=
		PORT_PCR_MUX(uart_pin_map_NB[id].alt);
	port_ptrs[uart_pin_map_NB[id].uart_port]->PCR[PIN2NUM(uart_pin_map_NB[id].tx_pin)] |= PORT_PCR_IRQC(PORT_eDisabled);

	port_ptrs[uart_pin_map_NB[id].uart_port]->PCR[PIN2NUM(uart_pin_map_NB[id].rx_pin)] = 0x0;
	port_ptrs[uart_pin_map_NB[id].uart_port]->PCR[PIN2NUM(uart_pin_map_NB[id].rx_pin)] |=
		PORT_PCR_MUX(uart_pin_map_NB[id].alt);
	port_ptrs[uart_pin_map_NB[id].uart_port]->PCR[PIN2NUM(uart_pin_map_NB[id].rx_pin)] |= PORT_PCR_IRQC(PORT_eDisabled);

	// enable uart transmission &&& rie interrupt
	/*****
	 * Remember that: RIE is Receive Interrupt Enable
	 * So an interrupt is triggered when RDRF goes HIGH (8bit buff is full)
	 */
	uart_ptrs_NB[id]->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK;

	uart_state[id].tx.id = id;
	uart_state[id].tx._head = 0;
	uart_state[id].tx._tail = 0;
	uart_state[id].rx.id = id;
	uart_state[id].rx._head = 0;
	uart_state[id].rx._tail = 0;
	uart_state[id].active = true;
	uart_count++;
	return id;
}

void UART_nonblocking_set_baudrate(uint8_t uart_id, uint32_t baudrate) {
	uint16_t sbr, brfa;
	uint32_t clock;

	clock = ((uart_id == 0) || (uart_id == 1)) ? (__CORE_CLOCK__) : (__CORE_CLOCK__ >> 1);
	baudrate = ((baudrate == 0) ? (UART_HAL_DEFAULT_BAUDRATE) :
								  ((baudrate > 0x1FFF) ? (UART_HAL_DEFAULT_BAUDRATE) : (baudrate)));

	sbr = clock / (baudrate << 4);				 // sbr = clock/(Baudrate x 16)
	brfa = (clock << 1) / baudrate - (sbr << 5); // brfa = 2*Clock/baudrate - 32*sbr

	uart_ptrs_NB[uart_id]->BDH = UART_BDH_SBR(sbr >> 8);
	uart_ptrs_NB[uart_id]->BDL = UART_BDL_SBR(sbr);
	uart_ptrs_NB[uart_id]->C4 = (uart_ptrs_NB[uart_id]->C4 & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
}

uint8_t UART_nonblocking_data_transmit(uint8_t uart_id, unsigned char *txdata, uint8_t length) {
	if (!uart_state[uart_id].active)
		return 0;

	// producer - consumer -> protect buffer HEAD
	NVIC_DisableIRQ(uart_nvim_ints_NB[uart_id]);
	uint8_t sent = 0;
	for (uint8_t i = 0; i < length; i++) {
		if (!buf_enqueue((UartBuffer_t *) &uart_state[uart_id].tx, txdata[i]))
			break;
		sent++;
	}
	if (sent > 0)
		uart_ptrs_NB[uart_id]->C2 |= UART_C2_TIE_MASK;
	NVIC_EnableIRQ(uart_nvim_ints_NB[uart_id]);
	return sent; // caller knows if bytes were dropped
}

bool UART_nonblocking_data_receive(uint8_t uart_id, uint8_t *out) {
	// CONSUMER -> protect _tail
	NVIC_DisableIRQ(uart_nvim_ints_NB[uart_id]);
	if (!uart_state[uart_id].active || buf_is_empty((UartBuffer_t *) &uart_state[uart_id].rx)) {
		return false;
	}
	*out = buf_dequeue((UartBuffer_t *) &uart_state[uart_id].rx);
	NVIC_EnableIRQ(uart_id);
	return true;
}

static uint32_t find_uart_id(pin_t tx, pin_t rx) {
	int i = 0;
	while (i != UART_ALTS) {
		if (uart_pin_map_NB[i].tx_pin == tx && uart_pin_map_NB[i].rx_pin == rx) {
			return uart_pin_map_NB[i].uart_num;
		}
		i++;
	}
	return INVALID_UART;
}

bool UART_nonblocking_tstatus(uint8_t uart_id) {
	if (!uart_state[uart_id].active)
		return false;
	return buf_is_empty((UartBuffer_t *) &uart_state[uart_id].tx);
}

bool UART_nonblocking_rstatus(uint8_t uart_id) {
}

void UART_RX_TX_ISR(uint8_t id) {
	UART_Type *uart = uart_ptrs_NB[id];
	uint8_t s1 = uart->S1; // reading status register to as  rdrf clear seq

	// if data register is full
	if (s1 & UART_S1_RDRF_MASK) {
		uint8_t byte = uart->D; // reading Data  clears RDRF
		buf_enqueue((UartBuffer_t *) &uart_state[id].rx, byte);
	}

	/* if TX data register empty and interrupts is active  */
	if ((s1 & UART_S1_TDRE_MASK) && (uart->C2 & UART_C2_TIE_MASK)) {
		if (!buf_is_empty((UartBuffer_t *) &uart_state[id].tx)) {
			uart->D = buf_dequeue((UartBuffer_t *) &uart_state[id].tx); // writing D clears TDRE
		} else {
			/* disable transmit interrupt enable ! !!s*/
			uart->C2 &= ~UART_C2_TIE_MASK;
		}
	}
}
/************* UART INTERRUPT HANDLERS ********************/
__ISR__ UART0_RX_TX_IRQHandler(void) {
	UART_RX_TX_ISR(0);
}
__ISR__ UART1_RX_TX_IRQHandler(void) {
	UART_RX_TX_ISR(1);
}
__ISR__ UART2_RX_TX_IRQHandler(void) {
	UART_RX_TX_ISR(2);
}
__ISR__ UART3_RX_TX_IRQHandler(void) {
	UART_RX_TX_ISR(3);
}
__ISR__ UART4_RX_TX_IRQHandler(void) {
	UART_RX_TX_ISR(4);
}

/**s tatics */
static uint8_t buf_next(uint8_t idx) {
	return (idx + 1) % BUFFER_CHAR_SIZE;
}

static bool buf_is_empty(const UartBuffer_t *b) {
	return b->_head == b->_tail;
}

static bool buf_is_full(const UartBuffer_t *b) {
	return buf_next(b->_head) == b->_tail;
}

/** Returns false if buffer was full and byte was dropped) */
static bool buf_enqueue(UartBuffer_t *b, uint8_t byte) {
	if (buf_is_full(b))
		return false;
	b->buffer[b->_head] = byte;
	b->_head = buf_next(b->_head);
	return true;
}

static uint8_t buf_dequeue(UartBuffer_t *b) {
	if (buf_is_empty(b))
		return UART_SENTINEL; /* should never get to this */
	uint8_t byte = b->buffer[b->_tail];
	b->_tail = buf_next(b->_tail);
	return byte;
}