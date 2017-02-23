/****************************************************************************
 *   FileName    : tcc353x_Interrupt_Process.c
 *   Description : Interrupt process
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

#include "../../common/tcc353x_common.h"
#include "../../api/inc/tcc353x_api.h"
#include "../../PAL/tcpal_os.h"
#include "../main/tcc353x_user_defines.h"
#include "../../../tcc353x_ringbuffer.h"

// fixme : if you use _MINIMIZE_STREAM_MEMCPY_ option, please remove this variable for reduce memory size
//I08U Tcc353xStreamData[TCC353X_STREAM_BUFFER_SIZE + 188];

/* for overflow test */
#define _DBG_CHK_OVERFLOW_CNT_
I32U gOverflowcnt = 0;
I32U gDbgIsrCnt = 0;

void Tcc353xStreamBufferInit(I32S _moduleIndex)
{
	Tcc353xRbuffInit();
}

void Tcc353xStreamBufferClose(I32S _moduleIndex)
{
	Tcc353xRbuffInit();
}

void Tcc353xStreamBufferReset(I32S _moduleIndex)
{
	Tcc353xRbuffFlush();
}

void Tcc353xStreamBufferFlush(I32S _moduleIndex)
{
	Tcc353xRbuffFlush();
}

I32U Tcc353xGetStreamBuffer(I32S _moduleIndex, I08U * _buff, I32U _size)
{
	I32S tsSize = 0;
	I32S totalSize = 0;

	totalSize = Tcc353xRbuffGetReadableSize();

	if (totalSize < 188) {
		return 0;
	}

	if (!_buff) {
		TcpalPrintErr((I08S *) "[dtv][tcc3536][error][TCC353X] Tcc353xGetStreamBuffer NULL\n");
		return 0;
	}

	if(_size > totalSize) {
		tsSize = totalSize;
	} else {
		tsSize = _size;
	}

	tsSize=Tcc353xRbuffPop(_buff, tsSize);
	if(tsSize<0) {
	       tsSize = 0;
	       TcpalPrintErr((I08S *) "[dtv][tcc3536][error][TCC353X] Tcc353xRbuffPop fail\n");
	}

	return tsSize;
}

I32U Tcc353xInterruptProcess(void)
{
	I32U ret = 0;
	I08U irqStatus = 0;
	I32S moduleIndex = 0;
	I32U totalSize = 0;
	I08U data = 0x00;

	/* Read BB Interrupt Status */
	Tcc353xApiGetIrqStatus(moduleIndex, &irqStatus);

	/* Stream Interrupt */
	if (irqStatus&0x01) {
		TcpalPrintErr((I08S *)
			      "[dtv][tcc3536][error][TCC353X] FIFO overflow[0x%02X] flush!!!\n",
			      irqStatus);

		/* IRQ Disable - Prevent additional interrupt signal */
		data = 0x00;
		Tcc353xApiRegisterWrite(0,0, 0x03, &data, 1);

		/* Tcc353x IRQ Clear */
		Tcc353xApiIrqClear(moduleIndex, irqStatus);
		Tcc353xApiInterruptBuffClr(moduleIndex);
		gOverflowcnt ++;
		ret = 0;
	} else {
		/* Tcc353x IRQ Clear */
		Tcc353xApiIrqClear(moduleIndex, irqStatus);
		Tcc353xApiGetFifoStatus(moduleIndex, &totalSize);
		ret = totalSize;
		if(ret>=510*188)
			TcpalPrintErr((I08S *)"[dtv][tcc3536][error][TCC353X] FIFO stat size ret=%d\n",ret);
	}

	gDbgIsrCnt++;

	if(gDbgIsrCnt>100) {
		gDbgIsrCnt = 0;
#ifdef _DBG_CHK_OVERFLOW_CNT_
		TcpalPrintStatus((I08S *)
				  "[dtv][tcc3536][debug_info][TCC353X] CurrOverflow gOverflowcnt=%d\n",
				  gOverflowcnt);
#endif
	}
	
	return ret;
}

#define MAX_READ_ATTEMPT_TEST	/* add (n * burst read protocols) */
#define _MINIMIZE_STREAM_MEMCPY_

void Tcc353xInterruptGetStream(I32U _fifoSize)
{
/* multiple by 4 and over 1k for spi dma */
#define _MAX_TS_READ_SIZE_	(15792) /* 188*84 */
#define _MIN_TS_READ_SIZE_	(1504) /* 188*8 */

	I32S totalSize = 0;
	I32U freeSize = 0;
	I32U writeSize = 0;
	I32U i;
	I32S readSizes[7] = {0,0,0,0,0,0,0};
	I32S ret;

	totalSize = _fifoSize - (_fifoSize%188);

	//                                                                           

	totalSize = (totalSize/188/4)*188*4;

	for(i=0; i<7; i++) {
		if(totalSize > _MAX_TS_READ_SIZE_) {
			readSizes[i] = _MAX_TS_READ_SIZE_;
			totalSize -= _MAX_TS_READ_SIZE_;
		} else {
			I32S temp;
			temp = (totalSize/188/4)*188*4;
			if(temp<_MIN_TS_READ_SIZE_) {
				readSizes[i] = 0;
				break;
			} else {
				readSizes[i] = temp;
				totalSize -= temp;
			}
		}
	}
	//                                                                         
	totalSize = _fifoSize - (_fifoSize%188);
	totalSize = (totalSize/188/4)*188*4;

	/* save to ring buffer */
	ret = Tcc353xApiStreamRead(0, NULL, totalSize);

	if(ret!=TCC353X_RETURN_SUCCESS) {
		TcpalPrintErr((I08S *) "[dtv][tcc3536][error][TCC353X] SyncByte Error!\n");
		TcpalPrintErr((I08S *) "[dtv][tcc3536][error][TCC353X] Buff Flush for SyncByte matching\n");
		Tcc353xApiInterruptBuffClr(0);
	}
}
