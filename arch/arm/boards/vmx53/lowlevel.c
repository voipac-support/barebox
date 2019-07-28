/*
 * (c) 2014 Voipac <support@voipac.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <common.h>
#include <mach/esdctl.h>
#include <asm/barebox-arm-head.h>
#include <asm/barebox-arm.h>

extern char __dtb_imx53_vmx53_start[];

void __naked barebox_arm_reset_vector(void)
{
	uint32_t fdt;

	arm_cpu_lowlevel_init();

	fdt = (uint32_t)__dtb_imx53_vmx53_start - get_runtime_offset();

	imx53_barebox_entry(fdt);
}
