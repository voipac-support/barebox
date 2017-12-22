/*
 * Copyright (C) 2013 Lucas Stach <l.stach@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <common.h>
#include <mach/lowlevel.h>

extern char __dtb_tegra20_colibri_iris_start[];

static void common_toradex_colibri_t20_iris_start(void)
{
	tegra_cpu_lowlevel_setup(__dtb_tegra20_colibri_iris_start);

	tegra_avp_reset_vector();
}

ENTRY_FUNCTION(start_colibri_t20_256_usbload, r0, r1, r2)
{
	common_toradex_colibri_t20_iris_start();
}

ENTRY_FUNCTION(start_colibri_t20_256_hsmmc, r0, r1, r2)
{
	common_toradex_colibri_t20_iris_start();
}

ENTRY_FUNCTION(start_colibri_t20_256_v11_nand, r0, r1, r2)
{
	common_toradex_colibri_t20_iris_start();
}

ENTRY_FUNCTION(start_colibri_t20_256_v12_nand, r0, r1, r2)
{
	common_toradex_colibri_t20_iris_start();
}

ENTRY_FUNCTION(start_colibri_t20_512_usbload, r0, r1, r2)
{
	common_toradex_colibri_t20_iris_start();
}

ENTRY_FUNCTION(start_colibri_t20_512_hsmmc, r0, r1, r2)
{
	common_toradex_colibri_t20_iris_start();
}

ENTRY_FUNCTION(start_colibri_t20_512_v11_nand, r0, r1, r2)
{
	common_toradex_colibri_t20_iris_start();
}

ENTRY_FUNCTION(start_colibri_t20_512_v12_nand, r0, r1, r2)
{
	common_toradex_colibri_t20_iris_start();
}
