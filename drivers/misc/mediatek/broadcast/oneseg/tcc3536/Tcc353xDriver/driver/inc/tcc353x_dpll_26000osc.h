/****************************************************************************
 *   FileName    : tcc353x_dpll_19200osc.h
 *   Description : dpll table for 19200 osc
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

#ifndef __TCC353X_DPLL_26000_H__
#define __TCC353X_DPLL_26000_H__

#include "tcc353x_defines.h"

/* PLL SET for OSC 26000 */
#define OSC_260_PLL_ISDB_T_1SEG_24                  0xA416  /*  40.083MHz */
#define OSC_260_PLL_ISDB_T_1SEG_25                  0xA516  /*  41.167MHz */
#define OSC_260_PLL_ISDB_T_1SEG_26                  0xA616  /*  42.250MHz */
#define OSC_260_PLL_ISDB_T_1SEG_27                  0xA716  /*  43.333MHz */
#define OSC_260_PLL_ISDB_T_1SEG_28                  0xA816  /*  44.417MHz */
#define OSC_260_PLL_ISDB_T_1SEG_29                  0xA916  /*  45.500MHz */
#define OSC_260_PLL_ISDB_T_1SEG_2A                  0xAA16  /*  46.583MHz */
#define OSC_260_PLL_ISDB_T_1SEG_2B                  0xAB16  /*  47.667MHz */
#define OSC_260_PLL_ISDB_T_1SEG_2C                  0xAC16  /*  48.750MHz */
#define OSC_260_PLL_ISDB_T_1SEG_2D                  0xAD16  /*  49.833MHz */
#define OSC_260_PLL_ISDB_T_1SEG_2E                  0xAE16  /*  50.917MHz */
#define OSC_260_PLL_ISDB_T_1SEG_2F                  0xAF16  /*  52.000MHz */

#define OSC_260_PLL_ISDB_T_13SEG_2D                 0xAD14  /*  99.667MHz */
#define OSC_260_PLL_ISDB_T_13SEG_2E                 0xAE14  /* 101.833MHz */
#define OSC_260_PLL_ISDB_T_13SEG_2F                 0xAF14  /* 104.000MHz */
#define OSC_260_PLL_ISDB_T_13SEG_30                 0xB014  /* 106.167MHz */
#define OSC_260_PLL_ISDB_T_13SEG_31                 0xB114  /* 108.333MHz */
#define OSC_260_PLL_ISDB_T_13SEG_32                 0xB214  /* 110.500MHz */
#define OSC_260_PLL_ISDB_T_13SEG_33                 0xB314  /* 112.667MHz */
#define OSC_260_PLL_ISDB_T_13SEG_34                 0xB414  /* 114.833MHz */
#define OSC_260_PLL_ISDB_T_13SEG_35                 0xB514  /* 117.000MHz */
#define OSC_260_PLL_ISDB_T_13SEG_36                 0xB614  /* 119.167MHz */
#define OSC_260_PLL_ISDB_T_13SEG_3E                 0xBE8E  /* 102.375MHz */
#define OSC_260_PLL_ISDB_T_13SEG_3F                 0xBF8E  /* 104.000MHz */

/* MAX FREQUENCY NUMBER */
#define _OSC_26000_MAX_1SEG_FREQ_NUM_                 57
#define _OSC_26000_MAX_13SEG_FREQ_NUM_                57

#endif
