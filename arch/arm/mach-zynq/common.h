/*
 * This file contains common function prototypes to avoid externs
 * in the c files.
 *
 *  Copyright (C) 2011 Xilinx
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MACH_ZYNQ_COMMON_H__
#define __MACH_ZYNQ_COMMON_H__

#include <asm/mach/time.h>
#include <mach/slcr.h>

void secondary_startup(void);

extern struct sys_timer xttcpss_sys_timer;

void platform_device_init(void);

void xilinx_init_machine(void);
void xilinx_irq_init(void);
void xilinx_map_io(void);
void xilinx_memory_init(void);


static inline void xilinx_system_reset(char mode, const char *cmd)
{
	xslcr_system_reset();
}

#endif
