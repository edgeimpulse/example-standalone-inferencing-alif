/* Copyright (c) 2021 ALIF SEMICONDUCTOR
#include <display_cfg.h>

All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
- Neither the name of ALIF SEMICONDUCTOR nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.
*
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------------------*/

/**************************************************************************//**
* @file     i3c_camera_testApp.c
* @author   Tanay Rami
* @email    tanay@alifsemi.com
* @version  V1.0.0
* @date     30-April-2021
* @brief    TestApp to verify i2c over i3c using Threadx as an operating system.
* @bug      None.
* @Note     None.
******************************************************************************/

/* Includes ----------------------------------------------------------------- */
/* System Includes */
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Project Includes */
#include "RTE_Components.h"
#include CMSIS_device_header

#include "display_cfg.h"
extern uint8_t lcd_image[DIMAGE_Y][DIMAGE_X][PIXEL_BYTES];

#define HW_REG_WORD(base,offset) *((volatile uint32_t *) (base + offset))
#define HW_REG_BYTE(base,offset) *((volatile uint8_t *) (base + offset))

/* Define main entry point.  */
static void DSI_DPHY_REG_WRITE (uint16_t,uint8_t);
static uint8_t DSI_DPHY_REG_READ (uint16_t);
#define cdc_base_addr      0x49031000
#define mipi_dsi_base_addr 0x49032000
#define mipi_csi_base_addr 0x49033000
#define expslv1_base_addr  0x4903F000

//Display
static void DCSWN_S(uint8_t);			//cmd
static void DCSW1_S(uint8_t,uint8_t);	//cmd, data
static void DCSRN_S (uint8_t); //cmd
static void SMRPS_S (uint8_t);
static void DCSW_L (uint8_t,uint8_t, uint8_t, uint8_t, uint8_t); //cmd, data1, data2, data3, data4

void hw_disp_init()
{
	uint32_t rd_data;
	uint8_t r_data_8;

#if defined(E50RA_MW550_N)
#define RATE_464M
	int VACT = 854;
	int VSA = 4;
	int VBP = 30;
	int VFP = 20;
	int HACT = 480;
	int HSA = 4;
	int HBP = 30;
	int HFP = 18;
#elif defined(E43RB_FW405_C)
#define RATE_415M
	int VACT = 800;
	int VSA = 2;
	int VBP = 10;
	int VFP = 10;
	int HACT = 480;
	int HSA = 4;
	int HBP = 5;
	int HFP = 5;
#elif defined(E43GB_MW405_C)
#define RATE_415M
	int VACT = 800;
	int VSA = 2;
	int VBP = 10;
	int VFP = 10;
	int HACT = 480;
	int HSA = 4;
	int HBP = 5;
	int HFP = 5;
#else
#error
#endif

#if defined(RATE_415M)
	int hsfreqrange = 0x5;
	int pll_soc_m_7_0 = 0x7;
	int pll_soc_m_9_8 = 0x2;
	uint8_t vco_cntrl = 0x18;	//011-000
#elif defined(RATE_464M)
	int hsfreqrange = 0x16;
	int pll_soc_m_7_0 = 0x44;
	int pll_soc_m_9_8 = 0x2;
	uint8_t vco_cntrl = 0x18;	//011-000
#else
#error
#endif

	int pll_soc_n = 0x2;
	int below_450Mbps = 1;

	/* Switch to 38.4MHz Crystal OSC */
	HW_REG_WORD(0x71007410,0x00) |= (1U << 1);

	/* Disable MIPI PHY LDO */
	HW_REG_WORD(0x7004001C,0x00) &= ~(1U << 10);

	PMU_delay_loop_us(250 * 1000);	// 100 milisec

	/* Enable MIPI PHY LDO */
	HW_REG_WORD(0x7004001C,0x00) |= (1U << 10);

	PMU_delay_loop_us(100 * 1000);	// 100 milisec


	////////////////////////////////////////////////////////////////
	// LCD Controller Setup
	////////////////////////////////////////////////////////////////

	HW_REG_WORD(expslv1_base_addr,0x04) = (14 << 16) | 1;	// pixel clock = 400MHz / 14 = 28.57MHz

	HW_REG_WORD(cdc_base_addr,0x18) = 0;

	HW_REG_WORD(cdc_base_addr,0x08) = (HSA-1)<<16 | (VSA-1);
	HW_REG_WORD(cdc_base_addr,0x0C) = (HSA+HBP-1)<<16 | (VSA+VBP-1);
	HW_REG_WORD(cdc_base_addr,0x10) = (HSA+HBP+HACT-1)<<16 | (VSA+VBP+VACT-1);
	HW_REG_WORD(cdc_base_addr,0x14) = (HSA+HBP+HACT+HFP-1)<<16 | (VSA+VBP+VACT+VFP-1);

	HW_REG_WORD(cdc_base_addr,0x10C) = 0;
	HW_REG_WORD(cdc_base_addr,0x110) = (HSA+HBP+HACT-1)<<16 | (HSA+HBP);
	HW_REG_WORD(cdc_base_addr,0x114) = (VSA+VBP+VACT-1)<<16 | (VSA+VBP);
	HW_REG_WORD(cdc_base_addr,0x11C) = PIXEL_BYTES == ARGB_BYTES ? 0 : 1;	// 0: 32-bit ARGB, 1: 24-bit RGB
	HW_REG_WORD(cdc_base_addr,0x134) = (uint32_t) &lcd_image[0][0][0];
	HW_REG_WORD(cdc_base_addr,0x138) = (HACT*PIXEL_BYTES)<<16 | (HACT*PIXEL_BYTES+7);
	HW_REG_WORD(cdc_base_addr,0x13C) = VACT;
	HW_REG_WORD(cdc_base_addr,0x10C) = 1;

	HW_REG_WORD(cdc_base_addr,0x24) = 1;


	////////////////////////////////////////////////////////////////
	// DSI D-PHY Setup
	////////////////////////////////////////////////////////////////

	HW_REG_WORD(mipi_dsi_base_addr,0x4) = 0;

	HW_REG_WORD(mipi_dsi_base_addr,0xA0) = 0;
	//number of lines
	HW_REG_WORD(mipi_dsi_base_addr,0xA4) = 1;

	HW_REG_WORD(expslv1_base_addr,0x30) =  0x00000100;
	HW_REG_WORD(mipi_dsi_base_addr,0xB4) = 0x00000000;//test_clr=0
	HW_REG_WORD(mipi_dsi_base_addr,0xB4) = 0x00000001;//test_clr=1
	HW_REG_WORD(mipi_dsi_base_addr,0xB4) = 0x00000000;//test_clr=0
	HW_REG_WORD(expslv1_base_addr,0x30) =  0x00000110;
	HW_REG_WORD(mipi_dsi_base_addr,0xB4) = 0x00000000;//test_clr=0
	HW_REG_WORD(mipi_dsi_base_addr,0xB4) = 0x00000001;//test_clr=1
	HW_REG_WORD(mipi_dsi_base_addr,0xB4) = 0x00000000;//test_clr=0
	HW_REG_WORD(expslv1_base_addr,0x30) =  0x00000100;

	//7. Set hsfreqrange[6:0] and cfgclkfreqrange[7:0]
	//[22:16] - hsfreqrange[6:0]
	//[31:24] - cfgclkfreqrange[7:0] = round[(cfgclk-17)*4] = (25-17)*4 = 8'b001000000
	//cfgclk = 25MHz

	//7.1 TX
	HW_REG_WORD(expslv1_base_addr,0x30) = (0x20 << 24) | (hsfreqrange << 16) | (1U << 8);

	//8. Configure TX register 0x16A to set pll_mpll_prog_rw (bits1:0) to 2'b11.
	//Before D-PHY power, set TX register 0x16A bits1:0 to 2'b11 (pll_mpll_prog_rw).
	r_data_8 = DSI_DPHY_REG_READ(0x16A);
	DSI_DPHY_REG_WRITE(0x16A, r_data_8 | 3);

	//9. Configure TX register 0x1AB to set cb_sel_vref_lprx_rw (bits 1:0) to 2'b10.
	r_data_8 = DSI_DPHY_REG_READ(0x1AB);
	DSI_DPHY_REG_WRITE(0x1AB, (r_data_8 & ~3) | 2);

	//10. Configure TX register 0x1AA to set cb_sel_vrefcd_lprx_rw (bits 6:5) to 2'b10.
	r_data_8 = DSI_DPHY_REG_READ(0x1AA);
	DSI_DPHY_REG_WRITE(0x1AA, (r_data_8 & ~0x60) | 0x40);

	//When operating as master or when in High-Speed BIST modes,
	//for datarates below 450 Mbps, clkdiv_clk_en must be enabled.
	//To do this, write 1'b1 in TX configuration register with address 0x1AC bit [4].
	r_data_8 = DSI_DPHY_REG_READ(0x1AC);
	DSI_DPHY_REG_WRITE(0x1AC, r_data_8 | (below_450Mbps << 4));

	//11. Configure TX register 0x402 to set txclk_term_lowcap_lp00_en_ovr_en_rw and
	//txclk_term_lowcap_lp00_en_ovr_rw(bits 1:0) to 2'b10;
	DSI_DPHY_REG_WRITE(0x402,0x2);

	//12. Refer to table "Supported rise/fall time limits" and configure TX test control registers
	//with appropriate values for the specified rise/fall time.
	//between 500 Mbps and 1 Gbps, 1 ns, 150 ps - 300 ps, 1, 12'd657, 225
	//under 500 Mbps, 2 ns, 150 ps - 300 ps, 1, 12'd657, 225
	//dphy4txtester_DIG_RDWR_TX_SLEW_5
	DSI_DPHY_REG_WRITE(0x270,0x91);

	//dphy4txtester_DIG_RDWR_TX_SLEW_6
	DSI_DPHY_REG_WRITE(0x271,0x2);

	//dphy4txtester_DIG_RDWR_TX_SLEW_7
	//13. Set bits [5:4] of TX control register with address 0x272 to 2'b01 to enable slew rate calibration. (Lanes
	//that are disabled should not run slew-rate calibration algorithm. For the disabled lanes, set
	//sr_finished_ovr_en = 1??b1, sr_finished_ovr = 1??b1, srcal_en_ovr_en = 1??b1 through test control
	//registers 0x310, 0x50b, 0x70b);
	DSI_DPHY_REG_WRITE(0x272,0x11);

	//For the disabled lanes, set
	//sr_finished_ovr_en = 1??b1, sr_finished_ovr = 1??b1, srcal_en_ovr_en = 1??b1 through test control registers 0x310, 0x50b, 0x70b, 0x90b, 0xbob);
	//bit2, bit1, bit0 are given as "reserved" in the Databook; but program as per the requirement.
	//Since you are not using lane2, lane3 - you can disable those bits in the registers for lane2 and lane3.

	//dphy4txtester_DIG_RDWR_TX_LANE2_SLEWRATE_0
	//lane 2
	DSI_DPHY_REG_WRITE(0x90b,0x0e);

	//lane 3
	DSI_DPHY_REG_WRITE(0xb0b,0x0e);

	//14. Set cfgclkfreqrange[7:0] = round[(Fcfg_clk(MHz)-17)*4] = 8'b00101000;
	//already set
	//15. Apply cfg_clk signal with the appropriate frequency with 25 Mhz frequency;
	//16. Configure PLL operating frequency

	//pll_soc_clksel [21:20]	RW	0x0	clkext div selection - clksel
	//should be set to 2'b01
	rd_data = HW_REG_WORD(expslv1_base_addr,0x10);
	rd_data |= 1UL << 20;
	HW_REG_WORD(expslv1_base_addr,0x10) = rd_data;

	//When the PHY is configured as a master (tx_rxz=1'b1) the PLL needs to always be properly configured for
	//the desired operating frequency before D-PHY Start-up.
	//Use pll shadow control (set to 0 if using SoC registers, 1 for D-PHY test interface)
	rd_data = HW_REG_WORD(expslv1_base_addr,0x10);
	rd_data |= 1UL << 4;
	HW_REG_WORD(expslv1_base_addr,0x10) = rd_data;

	//m[11:0] - pll_m_ovr_rw[7:0], pll_m_ovr_rw[11:8], pll_m_ovr_en_rw ?? 0x179, 0x17a, 0x17b
	//n[3:0] - pll_n_ovr_rw[3:0], pll_n_ovr_en_rw ?? 0x178
	//Foutmax [MHz] Foutmin [MHz] F VCO max [MHz] F VCO min [MHz] Output division factor P vco_cntrl[5:3]
	//1000          500           4000            2000            4                        001
	//500           250           4000            2000            8                        010
	//250           125           4000            2000            16                       011
	//125           62.5          4000            2000            32                       100
	//62.5          40            4000            2000            64                       101
	//24Mhz > fclkin/N > 8 Mhz
	//24Mhz > 38.4Mhz/N > 8 Mhz
	//N = 4
	//n = 4 - 1

	//fout = bit rate / 2
	//M/N*1/2*fclkin*1/P = M/3*1/2*38.4Mhz*1/64 = 40Mhz = 400 = 0x190
	//M = 400 = 0x190

	//MIPI-DPHY PLL Control Register 1
	//pll_soc_m [9:0]	RW	0x0	Control of the feedback multiplication ratio M (40 to 625) for SoC direct PLL control	( dphy_pll_ctrl1.pll_soc_m = 0x0 )
	//pll_soc_n [15:12]	RW	0x0	Control of the input frequency division ratio N (1 to 16) for SoC direct PLL control	( dphy_pll_ctrl1.pll_soc_n = 0x0 )
	//HW_REG_WORD(expslv1_base_addr,0x14) = 0x031A1;

	//dphy4txtester_DIG_RDWR_TX_PLL_28
	//7:0 pll_m_ovr_rw__7__0__ R/W Description: PLL feedback divider override
	DSI_DPHY_REG_WRITE(0x179,pll_soc_m_7_0);

	//dphy4txtester_DIG_RDWR_TX_PLL_29
	//1:0 pll_m_ovr_rw__9__8__ R/W Description: PLL feedback divider override
	DSI_DPHY_REG_WRITE(0x17a,pll_soc_m_9_8);

	//dphy4txtester_DIG_RDWR_TX_PLL_30
	//0 pll_m_ovr_en_rw R/W Description: PLL feedback divider override enable
	DSI_DPHY_REG_WRITE(0x17b,0x1);

	//dphy4txtester_DIG_RDWR_TX_PLL_27
	//7 pll_n_ovr_en_rw R/W Description: PLL input divider override enable
	//6:3 pll_n_ovr_rw__3__0__ R/W Description: PLL input divider override
	DSI_DPHY_REG_WRITE(0x178,(0x80|(pll_soc_n<<3)));

	// vco_cntrl[5:0] - pll_vco_cntrl_ovr_rw[5:0], pll_vco_cntrl_ovr_en_rw ?? 0x17b
	//dphy4txtester_DIG_RDWR_TX_PLL_30
	//7 pll_vco_cntrl_ovr_en_rw R/W Description: PLL VCO control override enable
	//6:1 pll_vco_cntrl_ovr_rw__5__0__ R/W Description: PLL VCO control override
	//0 pll_m_ovr_en_rw R/W Description: PLL feedback divider override enable
	//Table 3-6
	DSI_DPHY_REG_WRITE(0x17b,(0x81|vco_cntrl<<1));

	// cpbias_cntrl[6:0] - pll_cpbias_cntrl_rw[6:0] ?? 0x15e
	// dphy4txtester_DIG_RDWR_TX_PLL_1
	// 6:0 pll_cpbias_cntrl_rw__6__0__ R/W Description: PLL Charge Pump Bias Control
	DSI_DPHY_REG_WRITE(0x15e,0x0);

	// gmp_cntrl[1:0] - pll_gmp_cntrl_rw[1:0] ?? 0x162
	// int_cntrl[5:0] - pll_int_cntrl_rw[5:0] ?? 0x162
	// dphy4txtester_DIG_RDWR_TX_PLL_5
	// 7:2 pll_int_cntrl_rw__5__0__ R/W Description: PLL Integral Charge Pump control
	// 1:0 pll_gmp_cntrl_rw__1__0__ R/W Description: PLL GMP Control
	DSI_DPHY_REG_WRITE(0x162,0x11);

	// prop_cntrl[5:0] - pll_prop_cntrl_rw[5:0] ?? 0x16e
	// dphy4txtester_DIG_RDWR_TX_PLL_17
	// 5:0 pll_prop_cntrl_rw__5__0__ R/W Description: PLL Proportional Charge Pump control
	DSI_DPHY_REG_WRITE(0x16e,0x10);

	//Output frequency [MHz] vco_cntrl [5:0] cpbias_cntrl [6:0] gmp_cntrl [1:0] int_cntrl[5:0] prop_cntrl[5:0] tx or rx_cb_vref_mpll_re g_rel_rw[2:0]
	//487.5-615              001111          0000000            01              000100         010000          010
	//40-46.44               101011          0000000            01              000100         010000          010
	//pll_soc_cpbias_cntrl [6:0]	RW	0x0	Charge Pump bias control, for SoC direct PLL control	( dphy_pll_ctrl2.pll_soc_cpbias_cntrl = 0x0 )
	//pll_soc_int_cntrl [13:8]	RW	0x0	Integral Charge Pump control for SoC direct PLL control	( dphy_pll_ctrl2.pll_soc_int_cntrl = 0x0 )
	//pll_soc_prop_cntrl [21:16]	RW	0x0	Proportional Charge Pump control for SoC direct PLL control	( dphy_pll_ctrl2.pll_soc_prop_cntrl = 0x0 )
	//pll_soc_vco_cntrl [29:24]	RW	0x0	VCO operating range for SoC direct PLL control	( dphy_pll_ctrl2.pll_soc_vco_cntrl = 0x0 )
	//HW_REG_WORD(expslv1_base_addr,0x18) = 0x18100400;

	//pll_soc_gmp_cntrl [17:16]	RW	0x0	Controls the effective loop-filter resistance (=1/gmp) to increase/decrease MPLL bandwidth
	//rd_data = HW_REG_WORD(expslv1_base_addr,0x10) & ~(3UL << 16);
	//HW_REG_WORD(expslv1_base_addr,0x10) = rd_data | (1UL << 16);

	//SNPS: dphy4txtester_DIG_RDWR_TX_CB_3
	//2:0 cb_vref_mpll_reg_sel_rw__2__0__R/W Description: PLL reference voltage control
	DSI_DPHY_REG_WRITE(0x1AD,0x02);

	//17. Set basedir_n = 1'b0;
	//18. Set forcerxmode_n = 1'b0;
	HW_REG_WORD(expslv1_base_addr,0x34)=0x00000000;

	//19. Set all requests inputs to zero;
	//20. Wait for 15 ns;
	//PMU_delay_loop_ns(15);

	//21. Set enable_n and enableclk=1'b1;
	HW_REG_WORD(mipi_dsi_base_addr,0xa0)=0x00000004;

	//22. Wait 5 ns;
	//PMU_delay_loop_ns(5);

	//23. Set shutdownz=1'b1;
	HW_REG_WORD(mipi_dsi_base_addr,0xa0)=0x00000005;

	//24. Wait 5 ns;
	//PMU_delay_loop_ns(5);

	//25. Set rstz=1'b1;
	HW_REG_WORD(mipi_dsi_base_addr,0xa0)=0x00000007;

	//dphy4txtester_DIG_RDWR_TX_PLL_17
	// 7 pll_pwron_ovr_rw R/W Description: PLL poweron override
	// 6 pll_pwron_ovr_en_rw R/W Description: PLL poweron override enable control
	DSI_DPHY_REG_WRITE(0x16e,0xD0);

	//26. Wait until stopstatedata_n and stopstateclk outputs are asserted indicating PHY is driving LP11 in
	//enabled datalanes and clocklane.
	do {
		PMU_delay_loop_us(20);
		rd_data = HW_REG_WORD(mipi_dsi_base_addr,0xB0);

	} while ((rd_data & 0x00000094) != 0x00000094);

	////////////////////////////////////////////////////////////////
	// DSI Controller Setup
	////////////////////////////////////////////////////////////////

	//1.1 configure Vdieo Mode type VID_MODE_CFG[1:0]
	//1:0 vid_mode_type R/W This field indicates the video mode transmission type
	//15 lp_cmd_en R/W When set to 1, this bit enables the command transmission only in low-power mode.
	HW_REG_WORD(mipi_dsi_base_addr,0x38) = 0x00003F02; //burst mode

	//The following are added for ESC mode LPDT
	//CLKMGR_CFG
	//7:0 tx_esc_clk_division R/W
	//  This field indicates the division factor for the TX Escape clock source (lanebyteclk).
	//  The values 0 and 1 stop the TX_ESC clock generation.
	HW_REG_WORD(mipi_dsi_base_addr,0x08) = 4;

	//CMD_MODE_CFG
	//24 max_rd_pkt_size
	//19 dcs_lw_tx
	//18 dcs_sr_0p_tx
	//17 dcs_sw_1p_tx
	//16 dcs_sw_0p_tx
	//14 gen_lw_tx
	//13 gen_sr_2p_tx
	//12 gen_sr_1p_tx R/W This bit configures the Generic short read packet with one parameter command transmission type:
	//11 gen_sr_0p_tx R/W This bit configures the Generic short read packet with zero parameter command transmission type:
	//10 gen_sw_2p_tx R/W This bit configures the Generic short write packet with two parameters command transmission type:
	//9 gen_sw_1p_tx R/W This bit configures the Generic short write packet with one parameter command transmission type:
	//8 gen_sw_0p_tx R/W This bit configures the Generic short write packet with zero parameter command transmission type:
	//low power = 1; high speed = 0;
	HW_REG_WORD(mipi_dsi_base_addr,0x68)=0x010F7F00;

	HW_REG_WORD(mipi_dsi_base_addr,0x4) = 1;

	//////////////////////////////////////////////////////////////////////////////
	//// LCD Initialization
	//////////////////////////////////////////////////////////////////////////////

	// P4 all outputs
	HW_REG_WORD(0x70000000,0x4) = 0x00;

	// P4_6 high
	HW_REG_WORD(0x70000000,0x0) = 0x40;

	PMU_delay_loop_us(5000);

	// P4_6 low
	HW_REG_WORD(0x70000000,0x0) = 0x00;

	PMU_delay_loop_us(50000);

	// P4_6 high
	HW_REG_WORD(0x70000000,0x0) = 0x40;

	PMU_delay_loop_us(150000);

#if defined(E50RA_MW550_N)
	// E50RA-MW550-N
	// Change to Page 1
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x01);
	DCSW1_S(0x08,0x10);

	DCSW1_S(0x20,0x00);
	DCSW1_S(0x21,0x01);

	//Resolution setting
	//000 480X864
	//001 480X854
	//010 480X800
	//011 480X640
	//100 480X720
	DCSW1_S(0x30,0x01);
	DCSW1_S(0x31,0x00);

	DCSW1_S(0x40,0x16);
	DCSW1_S(0x41,0x33);
	DCSW1_S(0x42,0x03);
	DCSW1_S(0x43,0x09);
	DCSW1_S(0x44,0x06);

	DCSW1_S(0x50,0x88);
	DCSW1_S(0x51,0x88);
	DCSW1_S(0x52,0x00);
	DCSW1_S(0x53,0x49);
	DCSW1_S(0x55,0x49);

	DCSW1_S(0x60,0x07);
	DCSW1_S(0x61,0x00);
	DCSW1_S(0x62,0x07);
	DCSW1_S(0x63,0x00);

	DCSW1_S(0xA0,0x00);
	DCSW1_S(0xA1,0x09);
	DCSW1_S(0xA2,0x11);
	DCSW1_S(0xA3,0x0B);
	DCSW1_S(0xA4,0x05);
	DCSW1_S(0xA5,0x08);
	DCSW1_S(0xA6,0x06);
	DCSW1_S(0xA7,0x04);
	DCSW1_S(0xA8,0x09);
	DCSW1_S(0xA9,0x0C);
	DCSW1_S(0xAA,0x15);
	DCSW1_S(0xAB,0x08);
	DCSW1_S(0xAC,0x0F);
	DCSW1_S(0xAD,0x12);
	DCSW1_S(0xAE,0x09);
	DCSW1_S(0xAF,0x00);

	DCSW1_S(0xC0,0x00);
	DCSW1_S(0xC1,0x09);
	DCSW1_S(0xC2,0x10);
	DCSW1_S(0xC3,0x0C);
	DCSW1_S(0xC4,0x05);
	DCSW1_S(0xC5,0x08);
	DCSW1_S(0xC6,0x06);
	DCSW1_S(0xC7,0x04);
	DCSW1_S(0xC8,0x08);
	DCSW1_S(0xC9,0x0C);
	DCSW1_S(0xCA,0x14);
	DCSW1_S(0xCB,0x08);
	DCSW1_S(0xCC,0x0F);
	DCSW1_S(0xCD,0x11);
	DCSW1_S(0xCE,0x09);
	DCSW1_S(0xCF,0x00);

	// Change to Page 6
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x06);

	DCSW1_S(0x00,0x20);
	DCSW1_S(0x01,0x0A);
	DCSW1_S(0x02,0x00);
	DCSW1_S(0x03,0x00);
	DCSW1_S(0x04,0x01);
	DCSW1_S(0x05,0x01);
	DCSW1_S(0x06,0x98);
	DCSW1_S(0x07,0x06);
	DCSW1_S(0x08,0x01);
	DCSW1_S(0x09,0x80);
	DCSW1_S(0x0A,0x00);
	DCSW1_S(0x0B,0x00);
	DCSW1_S(0x0C,0x01);
	DCSW1_S(0x0D,0x01);
	DCSW1_S(0x0E,0x05);
	DCSW1_S(0x0F,0x00);

	DCSW1_S(0x10,0xF0);
	DCSW1_S(0x11,0xF4);
	DCSW1_S(0x12,0x01);
	DCSW1_S(0x13,0x00);
	DCSW1_S(0x14,0x00);
	DCSW1_S(0x15,0xC0);
	DCSW1_S(0x16,0x08);
	DCSW1_S(0x17,0x00);
	DCSW1_S(0x18,0x00);
	DCSW1_S(0x19,0x00);
	DCSW1_S(0x1A,0x00);
	DCSW1_S(0x1B,0x00);
	DCSW1_S(0x1C,0x00);
	DCSW1_S(0x1D,0x00);

	DCSW1_S(0x20,0x01);
	DCSW1_S(0x21,0x23);
	DCSW1_S(0x22,0x45);
	DCSW1_S(0x23,0x67);
	DCSW1_S(0x24,0x01);
	DCSW1_S(0x25,0x23);
	DCSW1_S(0x26,0x45);
	DCSW1_S(0x27,0x67);

	DCSW1_S(0x30,0x11);
	DCSW1_S(0x31,0x11);
	DCSW1_S(0x32,0x00);
	DCSW1_S(0x33,0xEE);
	DCSW1_S(0x34,0xFF);
	DCSW1_S(0x35,0xBB);
	DCSW1_S(0x36,0xAA);
	DCSW1_S(0x37,0xDD);
	DCSW1_S(0x38,0xCC);
	DCSW1_S(0x39,0x66);
	DCSW1_S(0x3A,0x77);
	DCSW1_S(0x3B,0x22);
	DCSW1_S(0x3C,0x22);
	DCSW1_S(0x3D,0x22);
	DCSW1_S(0x3E,0x22);
	DCSW1_S(0x3F,0x22);
	DCSW1_S(0x40,0x22);

	// Change to Page 7
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x07);
	DCSW1_S(0x17,0x22);
	DCSW1_S(0x02,0x77);
	DCSW1_S(0x26,0xB2);

	// Change to Page 0
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x00);
	DCSWN_S(0x35);
	DCSW1_S(0x3A,0x70);

#elif defined(E43RB_FW405_C)
	// E43RB-FW405-C
	// Change to Page 1
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x01);
	DCSW1_S(0x08,0x00);

	DCSW1_S(0x20,0x00);
	DCSW1_S(0x21,0x01);

	//Resolution setting
	//0x00 480X864
	//0x01 480X854
	//0x02 480X800
	//0x03 480X640
	//0x04 480X720
	DCSW1_S(0x30,0x02);
	DCSW1_S(0x31,0x00);

	DCSW1_S(0x40,0x00);
	DCSW1_S(0x41,0x33);
	DCSW1_S(0x42,0x02);
	DCSW1_S(0x43,0x89);
	DCSW1_S(0x44,0x89);
	DCSW1_S(0x46,0x34);

	DCSW1_S(0x50,0xA8);
	DCSW1_S(0x51,0xA8);
	DCSW1_S(0x52,0x00);
	DCSW1_S(0x53,0x78);
	DCSW1_S(0x54,0x00);
	DCSW1_S(0x55,0x78);

	DCSW1_S(0x60,0x07);
	DCSW1_S(0x61,0x04);
	DCSW1_S(0x62,0x08);
	DCSW1_S(0x63,0x04);

	DCSW1_S(0xA0,0x00);
	DCSW1_S(0xA1,0x0B);
	DCSW1_S(0xA2,0x13);
	DCSW1_S(0xA3,0x0D);
	DCSW1_S(0xA4,0x07);
	DCSW1_S(0xA5,0x0B);
	DCSW1_S(0xA6,0x07);
	DCSW1_S(0xA7,0x06);
	DCSW1_S(0xA8,0x07);
	DCSW1_S(0xA9,0x0A);
	DCSW1_S(0xAA,0x12);
	DCSW1_S(0xAB,0x0D);
	DCSW1_S(0xAC,0x11);
	DCSW1_S(0xAD,0x0F);
	DCSW1_S(0xAE,0x0E);
	DCSW1_S(0xAF,0x0B);

	DCSW1_S(0xC0,0x00);
	DCSW1_S(0xC1,0x0B);
	DCSW1_S(0xC2,0x13);
	DCSW1_S(0xC3,0x0D);
	DCSW1_S(0xC4,0x06);
	DCSW1_S(0xC5,0x0B);
	DCSW1_S(0xC6,0x07);
	DCSW1_S(0xC7,0x06);
	DCSW1_S(0xC8,0x07);
	DCSW1_S(0xC9,0x0A);
	DCSW1_S(0xCA,0x12);
	DCSW1_S(0xCB,0x0D);
	DCSW1_S(0xCC,0x11);
	DCSW1_S(0xCD,0x0F);
	DCSW1_S(0xCE,0x0E);
	DCSW1_S(0xCF,0x0B);

	// Change to Page 6
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x06);

	DCSW1_S(0x00,0x21);
	DCSW1_S(0x01,0x09);
	DCSW1_S(0x02,0x00);
	DCSW1_S(0x03,0x00);
	DCSW1_S(0x04,0x01);
	DCSW1_S(0x05,0x01);
	DCSW1_S(0x06,0x98);
	DCSW1_S(0x07,0x06);
	DCSW1_S(0x08,0x02);
	DCSW1_S(0x09,0x00);
	DCSW1_S(0x0A,0x00);
	DCSW1_S(0x0B,0x00);
	DCSW1_S(0x0C,0x01);
	DCSW1_S(0x0D,0x01);
	DCSW1_S(0x0E,0x00);
	DCSW1_S(0x0F,0x00);

	DCSW1_S(0x10,0xE0);
	DCSW1_S(0x11,0xE0);
	DCSW1_S(0x12,0x00);
	DCSW1_S(0x13,0x00);
	DCSW1_S(0x14,0x00);
	DCSW1_S(0x15,0x43);
	DCSW1_S(0x16,0x0B);
	DCSW1_S(0x17,0x00);
	DCSW1_S(0x18,0x00);
	DCSW1_S(0x19,0x00);
	DCSW1_S(0x1A,0x00);
	DCSW1_S(0x1B,0x00);
	DCSW1_S(0x1C,0x00);
	DCSW1_S(0x1D,0x00);

	DCSW1_S(0x20,0x01);
	DCSW1_S(0x21,0x23);
	DCSW1_S(0x22,0x45);
	DCSW1_S(0x23,0x67);
	DCSW1_S(0x24,0x01);
	DCSW1_S(0x25,0x23);
	DCSW1_S(0x26,0x45);
	DCSW1_S(0x27,0x67);

	DCSW1_S(0x30,0x01);
	DCSW1_S(0x31,0x11);
	DCSW1_S(0x32,0x00);
	DCSW1_S(0x33,0x22);
	DCSW1_S(0x34,0x22);
	DCSW1_S(0x35,0xCB);
	DCSW1_S(0x36,0xDA);
	DCSW1_S(0x37,0xAD);
	DCSW1_S(0x38,0xBC);
	DCSW1_S(0x39,0x66);
	DCSW1_S(0x3A,0x77);
	DCSW1_S(0x3B,0x22);
	DCSW1_S(0x3C,0x22);
	DCSW1_S(0x3D,0x22);
	DCSW1_S(0x3E,0x22);
	DCSW1_S(0x3F,0x22);
	DCSW1_S(0x40,0x22);

	// Change to Page 7
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x07);
	DCSW1_S(0x18,0x1D);
	DCSW1_S(0x02,0x77);
	DCSW1_S(0xE1,0x79);

	// Change to Page 0
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x00);
	DCSW1_S(0x36,0x01);
	DCSW1_S(0x3A,0x70);

#elif defined(E43GB_MW405_C)
	// Change to Page 1 CMD
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x01);
	DCSW1_S(0x08,0x10);

	DCSW1_S(0x20,0x00);
	DCSW1_S(0x21,0x01);

	//Resolution setting
	//0x00 480X864
	//0x01 480X854
	//0x02 480X800
	//0x03 480X640
	//0x04 480X720
	DCSW1_S(0x30,0x02);
	DCSW1_S(0x31,0x00);

	DCSW1_S(0x40,0x14);
	DCSW1_S(0x41,0x22);
	DCSW1_S(0x42,0x02);
	DCSW1_S(0x43,0x84);
	DCSW1_S(0x44,0x8A);

	DCSW1_S(0x50,0x78);
	DCSW1_S(0x51,0x78);
	DCSW1_S(0x52,0x00);
	DCSW1_S(0x53,0x2B);
	DCSW1_S(0x54,0x00);
	DCSW1_S(0x55,0x2B);

	DCSW1_S(0x60,0x07);
	DCSW1_S(0x61,0x06);
	DCSW1_S(0x62,0x06);
	DCSW1_S(0x63,0x04);

	DCSW1_S(0xA0,0x00);
	DCSW1_S(0xA1,0x0B);
	DCSW1_S(0xA2,0x19);
	DCSW1_S(0xA3,0x10);
	DCSW1_S(0xA4,0x06);
	DCSW1_S(0xA5,0x0F);
	DCSW1_S(0xA6,0x09);
	DCSW1_S(0xA7,0x06);
	DCSW1_S(0xA8,0x0C);
	DCSW1_S(0xA9,0x0E);
	DCSW1_S(0xAA,0x16);
	DCSW1_S(0xAB,0x0D);
	DCSW1_S(0xAC,0x15);
	DCSW1_S(0xAD,0x0F);
	DCSW1_S(0xAE,0x11);
	DCSW1_S(0xAF,0x00);

	DCSW1_S(0xC0,0x00);
	DCSW1_S(0xC1,0x24);
	DCSW1_S(0xC2,0x29);
	DCSW1_S(0xC3,0x0C);
	DCSW1_S(0xC4,0x07);
	DCSW1_S(0xC5,0x03);
	DCSW1_S(0xC6,0x03);
	DCSW1_S(0xC7,0x03);
	DCSW1_S(0xC8,0x03);
	DCSW1_S(0xC9,0x09);
	DCSW1_S(0xCA,0x0D);
	DCSW1_S(0xCB,0x01);
	DCSW1_S(0xCC,0x06);
	DCSW1_S(0xCD,0x1B);
	DCSW1_S(0xCE,0x08);
	DCSW1_S(0xCF,0x00);

	// Change to Page 6
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x06);

	DCSW1_S(0x00,0x20);
	DCSW1_S(0x01,0x04);
	DCSW1_S(0x02,0x00);
	DCSW1_S(0x03,0x00);
	DCSW1_S(0x04,0x01);
	DCSW1_S(0x05,0x01);
	DCSW1_S(0x06,0x88);
	DCSW1_S(0x07,0x04);
	DCSW1_S(0x08,0x01);
	DCSW1_S(0x09,0x90);
	DCSW1_S(0x0A,0x03);
	DCSW1_S(0x0B,0x01);
	DCSW1_S(0x0C,0x01);
	DCSW1_S(0x0D,0x01);
	DCSW1_S(0x0E,0x00);
	DCSW1_S(0x0F,0x00);

	DCSW1_S(0x10,0x55);
	DCSW1_S(0x11,0x53);
	DCSW1_S(0x12,0x01);
	DCSW1_S(0x13,0x0D);
	DCSW1_S(0x14,0x0D);
	DCSW1_S(0x15,0x43);
	DCSW1_S(0x16,0x0B);
	DCSW1_S(0x17,0x00);
	DCSW1_S(0x18,0x00);
	DCSW1_S(0x19,0x00);
	DCSW1_S(0x1A,0x00);
	DCSW1_S(0x1B,0x00);
	DCSW1_S(0x1C,0x00);
	DCSW1_S(0x1D,0x00);

	DCSW1_S(0x20,0x01);
	DCSW1_S(0x21,0x23);
	DCSW1_S(0x22,0x45);
	DCSW1_S(0x23,0x67);
	DCSW1_S(0x24,0x01);
	DCSW1_S(0x25,0x23);
	DCSW1_S(0x26,0x45);
	DCSW1_S(0x27,0x67);

	DCSW1_S(0x30,0x02);
	DCSW1_S(0x31,0x22);
	DCSW1_S(0x32,0x11);
	DCSW1_S(0x33,0xAA);
	DCSW1_S(0x34,0xBB);
	DCSW1_S(0x35,0x66);
	DCSW1_S(0x36,0x00);
	DCSW1_S(0x37,0x22);
	DCSW1_S(0x38,0x22);
	DCSW1_S(0x39,0x22);
	DCSW1_S(0x3A,0x22);
	DCSW1_S(0x3B,0x22);
	DCSW1_S(0x3C,0x22);
	DCSW1_S(0x3D,0x22);
	DCSW1_S(0x3E,0x22);
	DCSW1_S(0x3F,0x22);
	DCSW1_S(0x40,0x22);

	// Change to Page 5
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x05);
	DCSW1_S(0x09,0xFC);
	DCSW1_S(0x07,0xBC);

	// Change to Page 0
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x00);
	DCSW1_S(0x3A,0x70);

#else
#error
#endif

	//sleep exit
	DCSWN_S(0x11);
	PMU_delay_loop_us(120000);

	DCSWN_S(0x29);
	PMU_delay_loop_us(25000);

	//05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	//Normal Display mode on
	DCSW1_S(0x05, 0x13);

	//05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	//All Pixel On
	DCSW1_S(0x05, 0x23);

	//05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	//Display On
	DCSW1_S(0x05, 0x29);

	////////////////////////////////////////////////////////////////////////////
	// Video Mode
	////////////////////////////////////////////////////////////////////////////

	//This register configures the possibility for using non continuous clock in the clock lane.
	//0 phy_txrequestclkhs R/W This bit controls the D-PHY PPI txrequestclkhs signal.
	//from Synopsys
	HW_REG_WORD(mipi_dsi_base_addr,0x94) = 1;

	//1. configure MODE_CFG register to enable video mode
	HW_REG_WORD(mipi_dsi_base_addr,0x34) = 0;

	//2. configure DPI_COLOR_CODING register
	// 0x5 (CC05): 24-bit
	HW_REG_WORD(mipi_dsi_base_addr,0x10) = 5;

	//3. vid_pkt_size
	// pixel perline
	HW_REG_WORD(mipi_dsi_base_addr,0x3c) = HACT;

	//4. vid_num_chunks
	// chunks per line
	HW_REG_WORD(mipi_dsi_base_addr,0x40) = 0; //no chunks

	//5. null packet
	//12:0 vid_null_size R/W This register configures the number of bytes inside a null packet. Setting to 0 disables null packets.
	HW_REG_WORD(mipi_dsi_base_addr,0x44) = 0; //disable

	//6. This register configures the video HSA time.
	//11:0 vid_hsa_time R/W This field configures the Horizontal Sync Active period in lane byte clock cycles.
	HW_REG_WORD(mipi_dsi_base_addr,0x48) = HSA<<1;

	//7. This register configures the video HBP time.
	//11:0 vid_hbp_time R/W This field configures the Horizontal Back Porch period in lane byte clock cycles
	HW_REG_WORD(mipi_dsi_base_addr,0x4C) = HBP<<1;

	//8. This register configures the overall time for each video line.
	//14:0 vid_hline_time R/W This field configures the size of the total line time (HSA+HBP+HACT+HFP) counted in lane byte clock cycles.
	HW_REG_WORD(mipi_dsi_base_addr,0x50) = (HSA+HBP+HACT+HFP)<<1;

	//9. This register configures the VSA period.
	//9:0 vsa_lines R/W This field configures the Vertical Sync Active period measured in number of horizontal lines.
	HW_REG_WORD(mipi_dsi_base_addr,0x54) = VSA;

	//10. This register configures the VBP period.
	//9:0 vbp_lines R/W This field configures the Vertical Back Porch period measured in number of horizontal lines.
	HW_REG_WORD(mipi_dsi_base_addr,0x58) = VBP;

	//11. This register configures the VFP period.
	//9:0 vfp_lines R/W This field configures the Vertical Front Porch period measured in number of horizontal lines.
	HW_REG_WORD(mipi_dsi_base_addr,0x5C) = VFP;

	//12. This register configures the vertical resolution of video.
	//13:0 v_active_lines R/W This field configures the Vertical Active period measured in number of horizontal lines.
	HW_REG_WORD(mipi_dsi_base_addr,0x60) = VACT;

	HW_REG_WORD(cdc_base_addr,0x18) = 1;

	//////////////////////////////////////////////////////////////////////////////
	//// Display LCD back light power
	//////////////////////////////////////////////////////////////////////////////

	// P4_4 high
	HW_REG_WORD(0x70000000,0x0) |= 0x10;

}

uint8_t DSI_DPHY_REG_READ(uint16_t reg_addr)
{

	//50
	//[0] phy_testclk R/W Clock to capture testdin bus contents into the macro, with
	//    testen signal controlling the operation selection.
	//[1] phy_testclr R/W When active, performs vendor specific interface initialization.
	//    Active High.
	//
	//54
	//[16] phy_testen R/W When asserted high, it configures an address write operation
	//     on the falling edge of testclk. When asserted low, it
	//     configures a data write operation on the rising edge of
	//     testclk
	//[15:8] phy_testdout R Vendor-specific 8-bit data output for reading data and other
	//      probing functionalities
	//[7:0] phy_testdin R/W Test interface 8-bit data input for programming internal
	//      registers and accessing test functionalities.

	uint32_t read_data;
	uint8_t r_data_8;

	//1.
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//a. testclk
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//a. testen
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000;//b. testen_high
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002;//c. testclk high
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000;//d. place testdin 0x00
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//e. testclk low
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//f. place testen low
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = reg_addr >> 8;//g. MSB testdin
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002;//h. set testclk high

	//2.
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//a. testclk low
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000 | (reg_addr >> 8);//b. testen_high
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002;//c. set testclk high
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000 | (reg_addr & 0xff); //d. LSB testdin
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//e. set testclk low
	read_data = HW_REG_WORD(mipi_dsi_base_addr,0x00b8);
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//f. testen_low

	r_data_8 = (read_data >> 8) & 0xff;

	return r_data_8;
}

void DSI_DPHY_REG_WRITE(uint16_t reg_addr,uint8_t write_data)
{

	//50
	//[1] phy_testclk R/W Clock to capture testdin bus contents into the macro, with
	//    testen signal controlling the operation selection.
	//[0] phy_testclr R/W When active, performs vendor specific interface initialization.
	//    Active High.
	//
	//54
	//[16] phy_testen R/W When asserted high, it configures an address write operation
	//     on the falling edge of testclk. When asserted low, it
	//     configures a data write operation on the rising edge of
	//     testclk
	//[15:8] phy_testdout R Vendor-specific 8-bit data output for reading data and other
	//      probing functionalities
	//[7:0] phy_testdin R/W Test interface 8-bit data input for programming internal
	//      registers and accessing test functionalities.

	//1.
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//a. testclk
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//a. testen
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000;//b. testen_high
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002;//c. testclk high
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000;//d. place testdin 0x00
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//e. testclk low
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//f. place testen low
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = reg_addr >> 8;//g. MSB testdin
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002;//h. set testclk high

	//2.
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//a. testclk low
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000 | (reg_addr >> 8);//b. testen_high
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002;//c. set testclk high
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000 | (reg_addr & 0xff); //d. LSB testdin
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//e. set testclk low
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000 | (reg_addr & 0xff);//b. testen_low

	//3.
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = write_data; //a.
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002; //b. testclk high

	//turn off for clean exit
	HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//a. testclk
	HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//a. testen
}

#define A_GEN_HDR			0x6C
#define A_GEN_PLD_DATA		0x70
#define DCS_DELAY			100

void DCSWN_S (uint8_t cmd)
{
	HW_REG_WORD(mipi_dsi_base_addr,A_GEN_HDR) = 0x05 | cmd<<8;
	PMU_delay_loop_us(DCS_DELAY);
}

void DCSW1_S (uint8_t cmd, uint8_t data)
{
	HW_REG_WORD(mipi_dsi_base_addr,A_GEN_HDR) = 0x15 | cmd<<8 | data<<16;
	PMU_delay_loop_us(DCS_DELAY);
}

void DCSRN_S (uint8_t cmd)
{
	HW_REG_WORD(mipi_dsi_base_addr,A_GEN_HDR) = 0x137;
	PMU_delay_loop_us(DCS_DELAY);
	HW_REG_WORD(mipi_dsi_base_addr,A_GEN_HDR) = 0x06 | cmd<<8;
	PMU_delay_loop_us(DCS_DELAY);
}

void SMRPS_S(uint8_t num_bytes)
{
	HW_REG_WORD(mipi_dsi_base_addr,A_GEN_HDR) = 0x37 | num_bytes<<8;
	PMU_delay_loop_us(DCS_DELAY);
}

void DCSW_L (uint8_t cmd, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{
	HW_REG_WORD(mipi_dsi_base_addr,A_GEN_PLD_DATA) = data2<<24 | data1<<16 | cmd<<8 | cmd;
	HW_REG_WORD(mipi_dsi_base_addr,A_GEN_PLD_DATA) = data4<<8 | data3;
	HW_REG_WORD(mipi_dsi_base_addr,A_GEN_HDR) = 0x39 | 0x6<<8;
	PMU_delay_loop_us(DCS_DELAY);
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
