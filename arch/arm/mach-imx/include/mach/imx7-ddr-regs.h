/*
 * Copyright (C) 2017 Pengutronix, Fridolin Tux <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define MX7_DDRC_MSTR			0x307a0000
#define MX7_DDRC_STAT			0x307a0004
#define MX7_DDRC_MRCTRL0		0x307a0010
#define MX7_DDRC_MRCTRL1		0x307a0014
#define MX7_DDRC_MRSTAT			0x307a0018
#define MX7_DDRC_DERATEEN		0x307a0020
#define MX7_DDRC_DERATEINT		0x307a0024
#define MX7_DDRC_PWRCTL			0x307a0030
#define MX7_DDRC_PWRTMG			0x307a0034
#define MX7_DDRC_HWLPCTL		0x307a0038
#define MX7_DDRC_RFSHCTL0		0x307a0050
#define MX7_DDRC_RFSHCTL1		0x307a0054
#define MX7_DDRC_RFSHCTL3		0x307a0060
#define MX7_DDRC_RFSHTMG		0x307a0064
#define MX7_DDRC_INIT0			0x307a00d0
#define MX7_DDRC_INIT1			0x307a00d4
#define MX7_DDRC_INIT2			0x307a00d8
#define MX7_DDRC_INIT3			0x307a00dc
#define MX7_DDRC_INIT4			0x307a00e0
#define MX7_DDRC_INIT5			0x307a00e4
#define MX7_DDRC_RANKCTL		0x307a00f4
#define MX7_DDRC_DRAMTMG0		0x307a0100
#define MX7_DDRC_DRAMTMG1		0x307a0104
#define MX7_DDRC_DRAMTMG2		0x307a0108
#define MX7_DDRC_DRAMTMG3		0x307a010c
#define MX7_DDRC_DRAMTMG4		0x307a0110
#define MX7_DDRC_DRAMTMG5		0x307a0114
#define MX7_DDRC_DRAMTMG6		0x307a0118
#define MX7_DDRC_DRAMTMG7		0x307a011c
#define MX7_DDRC_DRAMTMG8		0x307a0120
#define MX7_DDRC_ZQCTL0			0x307a0180
#define MX7_DDRC_ZQCTL1			0x307a0184
#define MX7_DDRC_ZQCTL2			0x307a0188
#define MX7_DDRC_ZQSTAT			0x307a018c
#define MX7_DDRC_DFITMG0		0x307a0190
#define MX7_DDRC_DFITMG1		0x307a0194
#define MX7_DDRC_DFILPCFG0		0x307a0198
#define MX7_DDRC_DFIUPD0		0x307a01a0
#define MX7_DDRC_DFIUPD1		0x307a01a4
#define MX7_DDRC_DFIUPD2		0x307a01a8
#define MX7_DDRC_DFIUPD3		0x307a01ac
#define MX7_DDRC_DFIMISC		0x307a01b0
#define MX7_DDRC_ADDRMAP0		0x307a0200
#define MX7_DDRC_ADDRMAP1		0x307a0204
#define MX7_DDRC_ADDRMAP2		0x307a0208
#define MX7_DDRC_ADDRMAP3		0x307a020c
#define MX7_DDRC_ADDRMAP4		0x307a0210
#define MX7_DDRC_ADDRMAP5		0x307a0214
#define MX7_DDRC_ADDRMAP6		0x307a0218
#define MX7_DDRC_ODTCFG			0x307a0240
#define MX7_DDRC_ODTMAP			0x307a0244
#define MX7_DDRC_SCHED			0x307a0250
#define MX7_DDRC_SCHED1			0x307a0254
#define MX7_DDRC_PERFHPR1		0x307a025c
#define MX7_DDRC_PERFLPR1		0x307a0264
#define MX7_DDRC_PERFWR1		0x307a026c
#define MX7_DDRC_PERFVPR1		0x307a0274
#define MX7_DDRC_PERFVPW1		0x307a0278
#define MX7_DDRC_DBG0			0x307a0300
#define MX7_DDRC_DBG1			0x307a0304
#define MX7_DDRC_DBGCAM			0x307a0308
#define MX7_DDRC_DBGCMD			0x307a030c
#define MX7_DDRC_DBGSTAT		0x307a0310
#define MX7_DDRC_SWCTL			0x307a0320
#define MX7_DDRC_SWSTAT			0x307a0324

#define MX7_DDRC_MP_PSTAT		0x307a03fc
#define MX7_DDRC_MP_PCCFG		0x307a0400
#define MX7_DDRC_MP_PCFGR_0		0x307a0404
#define MX7_DDRC_MP_PCFGW_0		0x307a0408
#define MX7_DDRC_MP_PCFGIDMASKCH_00	0x307a0410
#define MX7_DDRC_MP_PCFGIDVALUECH_00	0x307a0414
#define MX7_DDRC_MP_PCFGIDMASKCH_10	0x307a0418
#define MX7_DDRC_MP_PCFGIDVALUECH_10	0x307a041c
#define MX7_DDRC_MP_PCFGIDMASKCH_20	0x307a0420
#define MX7_DDRC_MP_PCFGIDVALUECH_20	0x307a0424
#define MX7_DDRC_MP_PCFGIDMASKCH_30	0x307a0428
#define MX7_DDRC_MP_PCFGIDVALUECH_30	0x307a042c
#define MX7_DDRC_MP_PCFGIDMASKCH_40	0x307a0430
#define MX7_DDRC_MP_PCFGIDVALUECH_40	0x307a0434
#define MX7_DDRC_MP_PCFGIDMASKCH_50	0x307a0438
#define MX7_DDRC_MP_PCFGIDVALUECH_50	0x307a043c
#define MX7_DDRC_MP_PCFGIDMASKCH_60	0x307a0440
#define MX7_DDRC_MP_PCFGIDVALUECH_60	0x307a0444
#define MX7_DDRC_MP_PCFGIDMASKCH_70	0x307a0448
#define MX7_DDRC_MP_PCFGIDVALUECH_70	0x307a044c
#define MX7_DDRC_MP_PCFGIDMASKCH_80	0x307a0450
#define MX7_DDRC_MP_PCFGIDVALUECH_80	0x307a0454
#define MX7_DDRC_MP_PCFGIDMASKCH_90	0x307a0458
#define MX7_DDRC_MP_PCFGIDVALUECH_90	0x307a045c
#define MX7_DDRC_MP_PCFGIDMASKCH_100	0x307a0460
#define MX7_DDRC_MP_PCFGIDVALUECH_100	0x307a0464
#define MX7_DDRC_MP_PCFGIDMASKCH_110	0x307a0468
#define MX7_DDRC_MP_PCFGIDVALUECH_110	0x307a046c
#define MX7_DDRC_MP_PCFGIDMASKCH_120	0x307a0470
#define MX7_DDRC_MP_PCFGIDVALUECH_120	0x307a0474
#define MX7_DDRC_MP_PCFGIDMASKCH_130	0x307a0478
#define MX7_DDRC_MP_PCFGIDVALUECH_130	0x307a047c
#define MX7_DDRC_MP_PCFGIDMASKCH_140	0x307a0480
#define MX7_DDRC_MP_PCFGIDVALUECH_140	0x307a0484
#define MX7_DDRC_MP_PCFGIDMASKCH_150	0x307a0488
#define MX7_DDRC_MP_PCFGIDVALUECH_150	0x307a048c
#define MX7_DDRC_MP_PCTRL_0		0x307a0490
#define MX7_DDRC_MP_PCFGQOS0_0		0x307a0494
#define MX7_DDRC_MP_PCFGQOS1_0		0x307a0498
#define MX7_DDRC_MP_PCFGWQOS0_0		0x307a049c
#define MX7_DDRC_MP_PCFGWQOS1_0		0x307a04a0
#define MX7_DDRC_MP_SARBASE0		0x307a0f04
#define MX7_DDRC_MP_SARSIZE0		0x307a0f08
#define MX7_DDRC_MP_SARBASE1		0x307a0f0c
#define MX7_DDRC_MP_SARSIZE1		0x307a0f10
#define MX7_DDRC_MP_SARBASE2		0x307a0f14
#define MX7_DDRC_MP_SARSIZE2		0x307a0f18
#define MX7_DDRC_MP_SARBASE3		0x307a0f1c
#define MX7_DDRC_MP_SARSIZE3		0x307a0f20

#define MX7_DDR_PHY_PHY_CON0		0x30790000
#define MX7_DDR_PHY_PHY_CON1		0x30790004
#define MX7_DDR_PHY_PHY_CON2		0x30790008
#define MX7_DDR_PHY_PHY_CON3		0x3079000c
#define MX7_DDR_PHY_PHY_CON4		0x30790010
#define MX7_DDR_PHY_PHY_CON5		0x30790014
#define MX7_DDR_PHY_LP_CON0		0x30790018
#define MX7_DDR_PHY_RODT_CON0		0x3079001c
#define MX7_DDR_PHY_OFFSET_RD_CON0	0x30790020
#define MX7_DDR_PHY_OFFSET_WR_CON0	0x30790030
#define MX7_DDR_PHY_GATE_CODE_CON0	0x30790040
#define MX7_DDR_PHY_SHIFTC_CON0		0x3079004c
#define MX7_DDR_PHY_CMD_SDLL_CON0	0x30790050
#define MX7_DDR_PHY_LVL_CON0		0x3079006c
#define MX7_DDR_PHY_LVL_CON3		0x30790078
#define MX7_DDR_PHY_CMD_DESKEW_CON0	0x3079007c
#define MX7_DDR_PHY_CMD_DESKEW_CON1	0x30790080
#define MX7_DDR_PHY_CMD_DESKEW_CON2	0x30790084
#define MX7_DDR_PHY_CMD_DESKEW_CON3	0x30790088
#define MX7_DDR_PHY_CMD_DESKEW_CON4	0x30790094
#define MX7_DDR_PHY_DRVDS_CON0		0x3079009c
#define MX7_DDR_PHY_MDLL_CON0		0x307900b0
#define MX7_DDR_PHY_MDLL_CON1		0x307900b4
#define MX7_DDR_PHY_ZQ_CON0		0x307900c0
#define MX7_DDR_PHY_ZQ_CON1		0x307900c4
#define MX7_DDR_PHY_ZQ_CON2		0x307900c8
#define MX7_DDR_PHY_RD_DESKEW_CON0	0x30790190
#define MX7_DDR_PHY_RD_DESKEW_CON3	0x3079019c
#define MX7_DDR_PHY_RD_DESKEW_CON6	0x307901a8
#define MX7_DDR_PHY_RD_DESKEW_CON9	0x307901b4
#define MX7_DDR_PHY_RD_DESKEW_CON12	0x307901c0
#define MX7_DDR_PHY_RD_DESKEW_CON15	0x307901cc
#define MX7_DDR_PHY_RD_DESKEW_CON18	0x307901d8
#define MX7_DDR_PHY_RD_DESKEW_CON21	0x307901e4
#define MX7_DDR_PHY_WR_DESKEW_CON0	0x307901f0
#define MX7_DDR_PHY_WR_DESKEW_CON3	0x307901fc
#define MX7_DDR_PHY_WR_DESKEW_CON6	0x30790208
#define MX7_DDR_PHY_WR_DESKEW_CON9	0x30790214
#define MX7_DDR_PHY_WR_DESKEW_CON12	0x30790220
#define MX7_DDR_PHY_WR_DESKEW_CON15	0x3079022c
#define MX7_DDR_PHY_WR_DESKEW_CON18	0x30790238
#define MX7_DDR_PHY_WR_DESKEW_CON21	0x30790244
#define MX7_DDR_PHY_DM_DESKEW_CON	0x30790250
#define MX7_DDR_PHY_RDATA0		0x307903a0
#define MX7_DDR_PHY_STAT0		0x307903ac
