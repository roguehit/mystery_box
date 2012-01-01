/* CPU frequency */
//#define F_CPU 16000000UL
//#define F_CPU 8000000UL

#define F_CPU 16000000	// 1 MHz oscillator.
#define BaudRate 4800
#define MYUBRR (F_CPU / 16 / BaudRate ) - 1 

/* UART baud rate */
//#define UART_BAUD  9600

/* HD44780 LCD port connections */
#define HD44780_RS D, 2 
#define HD44780_RW D, 3 
#define HD44780_E  D, 4 
/* The data bits have to be not only in ascending order but also consecutive. */
#define HD44780_D4 B, 0

/* Whether to read the busy flag, or fall back to
   worst-time delays. */
#define USE_BUSY_BIT 1
