soc imx6
loadaddr 0x10000000
dcdofs 0x400

#include <mach/imx6-ddr-regs.h>
#include <mach/imx6q-ddr-regs.h>

wm 32 MX6_IOM_GRP_DDR_TYPE 0x000C0000
wm 32 MX6_IOM_GRP_DDRPKE 0x00000000
wm 32 MX6_IOM_DRAM_SDCLK_0 0x00000030
wm 32 MX6_IOM_DRAM_SDCLK_1 0x00000030
wm 32 MX6_IOM_DRAM_CAS 0x00000030
wm 32 MX6_IOM_DRAM_RAS 0x00000030
wm 32 MX6_IOM_GRP_ADDDS 0x00000030
wm 32 MX6_IOM_DRAM_RESET 0x00000030
wm 32 MX6_IOM_DRAM_SDBA2 0x00000000
wm 32 MX6_IOM_DRAM_SDODT0 0x00000030
wm 32 MX6_IOM_DRAM_SDODT1 0x00000030
wm 32 MX6_IOM_DRAM_SDCKE0 0x00003000
wm 32 MX6_IOM_DRAM_SDCKE1 0x00003000
wm 32 MX6_IOM_GRP_CTLDS 0x00000030
wm 32 MX6_IOM_DDRMODE_CTL 0x00020000
wm 32 MX6_IOM_DRAM_SDQS0 0x00000028
wm 32 MX6_IOM_DRAM_SDQS1 0x00000028
wm 32 MX6_IOM_DRAM_SDQS2 0x00000028
wm 32 MX6_IOM_DRAM_SDQS3 0x00000028
wm 32 MX6_IOM_DRAM_SDQS4 0x00000028
wm 32 MX6_IOM_DRAM_SDQS5 0x00000028
wm 32 MX6_IOM_DRAM_SDQS6 0x00000028
wm 32 MX6_IOM_DRAM_SDQS7 0x00000028
wm 32 MX6_IOM_GRP_DDRMODE 0x00020000
wm 32 MX6_IOM_GRP_B0DS 0x00000028
wm 32 MX6_IOM_GRP_B1DS 0x00000028
wm 32 MX6_IOM_GRP_B2DS 0x00000028
wm 32 MX6_IOM_GRP_B3DS 0x00000028
wm 32 MX6_IOM_GRP_B4DS 0x00000028
wm 32 MX6_IOM_GRP_B5DS 0x00000028
wm 32 MX6_IOM_GRP_B6DS 0x00000028
wm 32 MX6_IOM_GRP_B7DS 0x00000028
wm 32 MX6_IOM_DRAM_DQM0 0x00000028
wm 32 MX6_IOM_DRAM_DQM1 0x00000028
wm 32 MX6_IOM_DRAM_DQM2 0x00000028
wm 32 MX6_IOM_DRAM_DQM3 0x00000028
wm 32 MX6_IOM_DRAM_DQM4 0x00000028
wm 32 MX6_IOM_DRAM_DQM5 0x00000028
wm 32 MX6_IOM_DRAM_DQM6 0x00000028
wm 32 MX6_IOM_DRAM_DQM7 0x00000028
wm 32 MX6_MMDC_P0_MPZQHWCTRL 0xa1390003
wm 32 MX6_MMDC_P1_MPZQHWCTRL 0xa1380003
wm 32 MX6_MMDC_P0_MPWLDECTRL0 0x00140014
wm 32 MX6_MMDC_P0_MPWLDECTRL1 0x00230018
wm 32 MX6_MMDC_P1_MPWLDECTRL0 0x000A001E
wm 32 MX6_MMDC_P1_MPWLDECTRL1 0x000A0015
wm 32 MX6_MMDC_P0_MPDGCTRL0 0x43080314
wm 32 MX6_MMDC_P0_MPDGCTRL1 0x02680300
wm 32 MX6_MMDC_P1_MPDGCTRL0 0x430C0318
wm 32 MX6_MMDC_P1_MPDGCTRL1 0x03000254
wm 32 MX6_MMDC_P0_MPRDDLCTL 0x3A323234
wm 32 MX6_MMDC_P1_MPRDDLCTL 0x3E3C3242
wm 32 MX6_MMDC_P0_MPWRDLCTL 0x2A2E3632
wm 32 MX6_MMDC_P1_MPWRDLCTL 0x3C323E34
wm 32 MX6_MMDC_P0_MPRDDQBY0DL 0x33333333
wm 32 MX6_MMDC_P0_MPRDDQBY1DL 0x33333333
wm 32 MX6_MMDC_P0_MPRDDQBY2DL 0x33333333
wm 32 MX6_MMDC_P0_MPRDDQBY3DL 0x33333333
wm 32 MX6_MMDC_P1_MPRDDQBY0DL 0x33333333
wm 32 MX6_MMDC_P1_MPRDDQBY1DL 0x33333333
wm 32 MX6_MMDC_P1_MPRDDQBY2DL 0x33333333
wm 32 MX6_MMDC_P1_MPRDDQBY3DL 0x33333333
wm 32 MX6_MMDC_P0_MPMUR0 0x00000800
wm 32 MX6_MMDC_P1_MPMUR0 0x00000800
wm 32 MX6_MMDC_P0_MDPDC 0x00020036
wm 32 MX6_MMDC_P0_MDOTC 0x09444040

SETUP_MDCFG0

wm 32 MX6_MMDC_P0_MDCFG1 0xFF328F64
wm 32 MX6_MMDC_P0_MDCFG2 0x01FF00DB
wm 32 MX6_MMDC_P0_MDMISC 0x00011740
wm 32 MX6_MMDC_P0_MDSCR 0x00008000
wm 32 MX6_MMDC_P0_MDRWD 0x000026d2
wm 32 MX6_MMDC_P0_MDOR 0x003F1023

SETUP_MDASP_MDCTL

wm 32 MX6_MMDC_P0_MDSCR 0x04088032
wm 32 MX6_MMDC_P0_MDSCR 0x0408803a
wm 32 MX6_MMDC_P0_MDSCR 0x00008033
wm 32 MX6_MMDC_P0_MDSCR 0x0000803b
wm 32 MX6_MMDC_P0_MDSCR 0x00048031
wm 32 MX6_MMDC_P0_MDSCR 0x00048039
wm 32 MX6_MMDC_P0_MDSCR 0x09408030
wm 32 MX6_MMDC_P0_MDSCR 0x09408038
wm 32 MX6_MMDC_P0_MDSCR 0x04008040
wm 32 MX6_MMDC_P0_MDSCR 0x04008048
wm 32 MX6_MMDC_P0_MDREF 0x00007800
wm 32 MX6_MMDC_P0_MPODTCTRL 0x00011117
wm 32 MX6_MMDC_P1_MPODTCTRL 0x00011117
wm 32 MX6_MMDC_P0_MDPDC 0x00025576
wm 32 MX6_MMDC_P0_MAPSR 0x00011006
wm 32 MX6_MMDC_P0_MDSCR 0x00000000
wm 32 0x020e0010 0xf00000ff
wm 32 0x020e0018 0x007F007F
wm 32 0x020e001c 0x007F007F
wm 32 0x020c8000 0x80002021
