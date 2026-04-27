#ifndef UART_H
#define UART_H

#if defined(UART_IMPL_POLLING)
    #include "UART_polling.h"

#elif defined(UART_IMPL_NONBLOCKING)
    #include "UART_nonblocking.h"

#elif defined(UART_IMPL_FIFO)
    #include "UART_nonblocking_fifo.h"

#else
    #error "No UART implementation selected. Define UART_IMPL_POLLING, UART_IMPL_NONBLOCKING, or UART_IMPL_FIFO"

#endif
#endif /* UART_H */