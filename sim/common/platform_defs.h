#ifndef _PLATFORM_DEFS_H
#define _PLATFORM_DEFS_H

#ifndef CLK_SYS_KHZ
#define CLK_SYS_KHZ 12000
#endif

#ifndef UART_BAUD
#define UART_BAUD 115200
#endif

#define N_LEDS 5

#define FLASH_BASE  0x400000
#define FLASH_SIZE  0x400000

#define SRAM_BASE   0x800000
#define SRAM_SIZE     0x1000

#define LED_BASE   0x1000000
#define UART_BASE  0x1001000

#endif
