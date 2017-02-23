/****************************************************************************
 *   FileName    : tcc353x_dpll_tcc3535.c
 *   Description : dpll table for tcc3535
 ****************************************************************************
 *
 *   TCC Version 1.0
 *   Copyright (c) Telechips Inc.
 *   All rights reserved 
 
This source code contains confidential information of Telechips.
Any unauthorized use without a written permission of Telechips including not limited to re-
distribution in source or binary form is strictly prohibited.
This source code is provided "AS IS" and nothing contained in this source code shall 
constitute any express or implied warranty of any kind, including without limitation, any warranty 
of merchantability, fitness for a particular purpose or non-infringement of any patent, copyright 
or other third party intellectual property right. No warranty is made, express or implied, 
regarding the information's accuracy, completeness, or performance. 
In no event shall Telechips be liable for any claim, damages or other liability arising from, out of 
or in connection with this source code or the use in the source code. 
This source code is provided subject to the terms of a Mutual Non-Disclosure Agreement 
between Telechips and Company.
*
****************************************************************************/

#include "../../PAL/tcpal_os.h"
#include "../../PAL/tcpal_i2c.h"
#include "../../PAL/tcpal_spi.h"
#include "../inc/tcc353x_dpll_26000osc.h"
#include "../inc/tcc353x_defines.h"
#include "../../common/tcc353x_common.h"


I32U OSC_26000_DpllTable_Partial1Seg_tcc3536 [_OSC_26000_MAX_1SEG_FREQ_NUM_ * 5] = {
	/* please align low frequency to high frequency */
	/* start frequency, Pll, RC STEP_H, RC_STEP_L, ADC Clk */
    473143, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09, 
    479143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    485143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    491143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    497143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    503143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    509143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    515143, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09, 
    521143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    527143, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09, 
    533143, 	OSC_260_PLL_ISDB_T_1SEG_2C, 	0x15, 	0x46A9BFF1, 	0x0B, 
    539143, 	OSC_260_PLL_ISDB_T_1SEG_26, 	0x09, 	0xC93A1D7A, 	0x0A, 
    545143, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09, 
    551143, 	OSC_260_PLL_ISDB_T_1SEG_29, 	0x04, 	0x82914DA7, 	0x0B, 
    557143, 	OSC_260_PLL_ISDB_T_1SEG_2C, 	0x15, 	0x46A9BFF1, 	0x0B, 
    563143, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09, 
    569143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    575143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    581143, 	OSC_260_PLL_ISDB_T_1SEG_29, 	0x04, 	0x82914DA7, 	0x0B, 
    587143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    593143, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09, 
    599143, 	OSC_260_PLL_ISDB_T_1SEG_26, 	0x09, 	0xC93A1D7A, 	0x0A, 
    605143, 	OSC_260_PLL_ISDB_T_1SEG_28, 	0x15, 	0xCBE61C0A, 	0x0A, 
    611143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    617143, 	OSC_260_PLL_ISDB_T_1SEG_2B, 	0x0F, 	0xF0FF0FF0, 	0x0B, 
    623143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    629143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    635143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    641143, 	OSC_260_PLL_ISDB_T_1SEG_26, 	0x09, 	0xC93A1D7A, 	0x0A, 
    647143, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09, 
    653143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    659143, 	OSC_260_PLL_ISDB_T_1SEG_28, 	0x15, 	0xCBE61C0A, 	0x0A, 
    665143, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09, 
    671143, 	OSC_260_PLL_ISDB_T_1SEG_2A, 	0x0A, 	0x5BCF699D, 	0x0B, 
    677143, 	OSC_260_PLL_ISDB_T_1SEG_27, 	0x0F, 	0xF0FF0FF0, 	0x0A, 
    683143, 	OSC_260_PLL_ISDB_T_1SEG_26, 	0x09, 	0xC93A1D7A, 	0x0A, 
    689143, 	OSC_260_PLL_ISDB_T_1SEG_29, 	0x04, 	0x82914DA7, 	0x0B, 
    695143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    701143, 	OSC_260_PLL_ISDB_T_1SEG_27, 	0x0F, 	0xF0FF0FF0, 	0x0A, 
    707143, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09, 
    713143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    719143, 	OSC_260_PLL_ISDB_T_1SEG_29, 	0x04, 	0x82914DA7, 	0x0B, 
    725143, 	OSC_260_PLL_ISDB_T_1SEG_26, 	0x09, 	0xC93A1D7A, 	0x0A, 
    731143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    737143, 	OSC_260_PLL_ISDB_T_1SEG_2C, 	0x15, 	0x46A9BFF1, 	0x0B, 
    743143, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09, 
    749143, 	OSC_260_PLL_ISDB_T_1SEG_26, 	0x09, 	0xC93A1D7A, 	0x0A, 
    755143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    761143, 	OSC_260_PLL_ISDB_T_1SEG_28, 	0x15, 	0xCBE61C0A, 	0x0A, 
    767143, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09, 
    773143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    779143, 	OSC_260_PLL_ISDB_T_1SEG_26, 	0x09, 	0xC93A1D7A, 	0x0A, 
    785143, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09, 
    791143, 	OSC_260_PLL_ISDB_T_1SEG_26, 	0x09, 	0xC93A1D7A, 	0x0A, 
    797143, 	OSC_260_PLL_ISDB_T_1SEG_25, 	0x03, 	0x4E85BFF0, 	0x0A, 
    803143, 	OSC_260_PLL_ISDB_T_1SEG_2B, 	0x0F, 	0xF0FF0FF0, 	0x0B, 
         0, 	OSC_260_PLL_ISDB_T_1SEG_24, 	0x16, 	0x6DF13FF1, 	0x09 
};

I32U OSC26000_DpllTable_FullSeg_tcc3536 [_OSC_26000_MAX_13SEG_FREQ_NUM_ * 5] = {
    /* please align low frequency to high frequency */
    /* start frequency, Pll, RC STEP_H, RC_STEP_L, ADC Clk */
    473143, 	OSC_260_PLL_ISDB_T_13SEG_3E, 	0x0C, 	0x21852483, 	0x03, 
    479143, 	OSC_260_PLL_ISDB_T_13SEG_35, 	0x2A, 	0x9D547FF2, 	0x03, 
    485143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    491143, 	OSC_260_PLL_ISDB_T_13SEG_3E, 	0x0C, 	0x21852483, 	0x03, 
    497143, 	OSC_260_PLL_ISDB_T_13SEG_31, 	0x19, 	0x8B324CBE, 	0x03, 
    503143, 	OSC_260_PLL_ISDB_T_13SEG_34, 	0x26, 	0x96A3608C, 	0x03, 
    509143, 	OSC_260_PLL_ISDB_T_13SEG_30, 	0x14, 	0xD72E1A10, 	0x03, 
    515143, 	OSC_260_PLL_ISDB_T_13SEG_33, 	0x22, 	0x684DE754, 	0x03, 
    521143, 	OSC_260_PLL_ISDB_T_13SEG_3E, 	0x0C, 	0x21852483, 	0x03, 
    527143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    533143, 	OSC_260_PLL_ISDB_T_13SEG_3E, 	0x0C, 	0x21852483, 	0x03, 
    539143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    545143, 	OSC_260_PLL_ISDB_T_13SEG_30, 	0x14, 	0xD72E1A10, 	0x03, 
    551143, 	OSC_260_PLL_ISDB_T_13SEG_3E, 	0x0C, 	0x21852483, 	0x03, 
    557143, 	OSC_260_PLL_ISDB_T_13SEG_32, 	0x1E, 	0x0FFF1E0F, 	0x03, 
    563143, 	OSC_260_PLL_ISDB_T_13SEG_32, 	0x1E, 	0x0FFF1E0F, 	0x03, 
    569143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    575143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    581143, 	OSC_260_PLL_ISDB_T_13SEG_2E, 	0x0A, 	0xD5716CE0, 	0x03, 
    587143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    593143, 	OSC_260_PLL_ISDB_T_13SEG_35, 	0x2A, 	0x9D547FF2, 	0x03, 
    599143, 	OSC_260_PLL_ISDB_T_13SEG_32, 	0x1E, 	0x0FFF1E0F, 	0x03, 
    605143, 	OSC_260_PLL_ISDB_T_13SEG_3E, 	0x0C, 	0x21852483, 	0x03, 
    611143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    617143, 	OSC_260_PLL_ISDB_T_13SEG_2E, 	0x0A, 	0xD5716CE0, 	0x03, 
    623143, 	OSC_260_PLL_ISDB_T_13SEG_3E, 	0x0C, 	0x21852483, 	0x03, 
    629143, 	OSC_260_PLL_ISDB_T_13SEG_30, 	0x14, 	0xD72E1A10, 	0x03, 
    635143, 	OSC_260_PLL_ISDB_T_13SEG_3E, 	0x0C, 	0x21852483, 	0x03, 
    641143, 	OSC_260_PLL_ISDB_T_13SEG_2E, 	0x0A, 	0xD5716CE0, 	0x03, 
    647143, 	OSC_260_PLL_ISDB_T_13SEG_30, 	0x14, 	0xD72E1A10, 	0x03, 
    653143, 	OSC_260_PLL_ISDB_T_13SEG_2E, 	0x0A, 	0xD5716CE0, 	0x03, 
    659143, 	OSC_260_PLL_ISDB_T_13SEG_30, 	0x14, 	0xD72E1A10, 	0x03, 
    665143, 	OSC_260_PLL_ISDB_T_13SEG_33, 	0x22, 	0x684DE754, 	0x03, 
    671143, 	OSC_260_PLL_ISDB_T_13SEG_31, 	0x19, 	0x8B324CBE, 	0x03, 
    677143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    683143, 	OSC_260_PLL_ISDB_T_13SEG_2E, 	0x0A, 	0xD5716CE0, 	0x03, 
    689143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    695143, 	OSC_260_PLL_ISDB_T_13SEG_32, 	0x1E, 	0x0FFF1E0F, 	0x03, 
    701143, 	OSC_260_PLL_ISDB_T_13SEG_2E, 	0x0A, 	0xD5716CE0, 	0x03, 
    707143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    713143, 	OSC_260_PLL_ISDB_T_13SEG_30, 	0x14, 	0xD72E1A10, 	0x03, 
    719143, 	OSC_260_PLL_ISDB_T_13SEG_2E, 	0x0A, 	0xD5716CE0, 	0x03, 
    725143, 	OSC_260_PLL_ISDB_T_13SEG_3E, 	0x0C, 	0x21852483, 	0x03, 
    731143, 	OSC_260_PLL_ISDB_T_13SEG_30, 	0x14, 	0xD72E1A10, 	0x03, 
    737143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    743143, 	OSC_260_PLL_ISDB_T_13SEG_2E, 	0x0A, 	0xD5716CE0, 	0x03, 
    749143, 	OSC_260_PLL_ISDB_T_13SEG_31, 	0x19, 	0x8B324CBE, 	0x03, 
    755143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    761143, 	OSC_260_PLL_ISDB_T_13SEG_32, 	0x1E, 	0x0FFF1E0F, 	0x03, 
    767143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    773143, 	OSC_260_PLL_ISDB_T_13SEG_3E, 	0x0C, 	0x21852483, 	0x03, 
    779143, 	OSC_260_PLL_ISDB_T_13SEG_32, 	0x1E, 	0x0FFF1E0F, 	0x03, 
    785143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    791143, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03, 
    797143, 	OSC_260_PLL_ISDB_T_13SEG_34, 	0x26, 	0x96A3608C, 	0x03, 
    803143, 	OSC_260_PLL_ISDB_T_13SEG_2E, 	0x0A, 	0xD5716CE0, 	0x03,
         0, 	OSC_260_PLL_ISDB_T_13SEG_2D, 	0x05, 	0x810A26E5, 	0x03 
};

