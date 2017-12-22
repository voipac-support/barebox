/*
 * Freescale i.MX28 SPL functions
 *
 * Copyright (C) 2011 Marek Vasut <marek.vasut@gmail.com>
 * on behalf of DENX Software Engineering GmbH
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef	__M28_INIT_H__
#define	__M28_INIT_H__

void mxs_early_delay(int delay);

void mx23_power_init(int __has_battery, int __use_battery_input,
		int __use_5v_input);
void mx28_power_init(int __has_battery, int __use_battery_input,
		int __use_5v_input);
void mxs_power_wait_pswitch(void);

extern const uint32_t mx28_dram_vals_default[190];
extern uint32_t mx23_dram_vals[];

void mx23_mem_init(void);
void mx28_mem_init(const uint32_t dram_vals[190]);
void mxs_mem_setup_cpu_and_hbus(void);
void mxs_mem_setup_vdda(void);
void mxs_mem_init_clock(unsigned char divider);

void mxs_lradc_init(void);
void mxs_lradc_enable_batt_measurement(void);

#endif	/* __M28_INIT_H__ */
