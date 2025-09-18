/*
 * Copyright 2021, Axel Heider <axelheider@gmx.de>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <config.h>
#include <machine/io.h>
#include <drivers/uart.h>

int early_boot = 1;

#ifdef CONFIG_PRINTING
void kernel_putDebugChar(unsigned char c)
{
    if (early_boot) {
        uart_console_early_putchar(c);
    } else {
        uart_console_putchar(c);
    }
}
#endif /* CONFIG_PRINTING */

#ifdef CONFIG_DEBUG_BUILD
unsigned char kernel_getDebugChar(void)
{
    return uart_drv_getchar();
}
#endif /* CONFIG_DEBUG_BUILD */
