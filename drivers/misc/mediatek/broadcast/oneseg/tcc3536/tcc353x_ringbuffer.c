/****************************************************************************
 *   FileName    : tcc353x_ringbuffer.c
 *   Description : TCC353X ring buffer control functions
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

#include <linux/uaccess.h>        /* copy_to_user */
#include "tcc353x_ringbuffer.h"
#include "Tcc353xDriver/PAL/tcpal_os.h"

//#define MAX_RING_BUFFER_SIZE (188*320*20)
#define MAX_RING_BUFFER_SIZE (1203200)	// 188*320*20
#define MAX_APP_BUFF_SIZE (192512)	// 188*1024
static unsigned char Ring_Buff_Tcc353x[MAX_RING_BUFFER_SIZE];

typedef struct {
	unsigned long wp;
	unsigned long rp;
} Tcc353xRingbuffStr;

Tcc353xRingbuffStr Tcc353xRb = {0,0};

/** macro to find number that is greater than or equal to another argued number */
#define MAX(x, y) ((x)>=(y))?(x):(y)
/** macro to find number that is less than or equal to another argued number */
#define MIN(x, y) ((x)<=(y))?(x):(y)

void Tcc353xRbuffInit()
{
	Tcc353xRb.wp = 0;
	Tcc353xRb.rp = 0;
	TcpalPrintErr((I08S *)"[dtv][tcc3536][debug_info] ring_buffer_size=%d\n", (unsigned int)(MAX_RING_BUFFER_SIZE));
}

void Tcc353xRbuffFlush()
{
	Tcc353xRb.rp = Tcc353xRb.wp;
}

signed long Tcc353xRbuffGetWritableSize()
{
	signed long size = 0;

	size = (signed long)(Tcc353xRb.rp - Tcc353xRb.wp - 1);
	if (size<0) {
		size += MAX_RING_BUFFER_SIZE;
	}

	return size;
}

signed long Tcc353xRbuffGetReadableSize()
{
	signed long size = 0;
	size = (signed long)(Tcc353xRb.wp - Tcc353xRb.rp);
	if (size<0) {
		size += MAX_RING_BUFFER_SIZE;
	}

	/* exception error handler */
	if(size<0)
		size = 0;
	else if(size>=MAX_RING_BUFFER_SIZE)
		size = MAX_APP_BUFF_SIZE;

	return size;
}

signed long Tcc353xRbuffPop(unsigned char *_buff, unsigned int _size)
{
	signed long temp = 0;
	signed long remain = 0;
	signed long acc = 0;
	unsigned long cpdcnt;

	unsigned long rp, wp;
	rp = Tcc353xRb.rp;
	wp = Tcc353xRb.wp;

	if(!_buff)
		return 0;

	if(rp>=MAX_RING_BUFFER_SIZE || wp>=MAX_RING_BUFFER_SIZE) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] rbuff wp=%d rp=%d\n", wp, rp);
		rp = (rp%MAX_RING_BUFFER_SIZE);
		wp = (wp%MAX_RING_BUFFER_SIZE);
		return 0;
	}

	temp = MIN(_size, (MAX_RING_BUFFER_SIZE-rp));

	/* exception error handler */
	if(temp<=0) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] rbuff temp=%d\n", temp);
		return 0;
	}

	if(rp+temp>MAX_RING_BUFFER_SIZE) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] rbuff rp+temp=%d\n", rp+temp);
		return 0;
	}

	cpdcnt=copy_to_user(&_buff[0],&Ring_Buff_Tcc353x[rp],temp);
	acc+=temp;

	remain = _size - temp;
	/* exception error handler */
	if(remain>MAX_RING_BUFFER_SIZE) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] rbuff remain=%d\n", remain);
		return acc;
	}

	if (remain > 0) {
		cpdcnt=copy_to_user(&_buff[temp],&Ring_Buff_Tcc353x[0],remain);
		acc+=remain;
	}

	Tcc353xRb.rp = ((rp+_size)%MAX_RING_BUFFER_SIZE);
	return (signed long)_size;
}

signed long Tcc353xRbuffPush(unsigned char *_buff, unsigned int _size)
{
	signed long temp = 0, remain = 0;
	unsigned long rp, wp;
	rp = Tcc353xRb.rp;
	wp = Tcc353xRb.wp;

	if(!_buff)
		return 0;

	if(rp>=MAX_RING_BUFFER_SIZE || wp>=MAX_RING_BUFFER_SIZE) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] rbuff wp=%d rp=%d\n", wp, rp);
		rp = (rp%MAX_RING_BUFFER_SIZE);
		wp = (wp%MAX_RING_BUFFER_SIZE);
		return 0;
	}

	temp = MIN(_size, (MAX_RING_BUFFER_SIZE-wp));
	TcpalMemcpy(&Ring_Buff_Tcc353x[wp], _buff, temp);
	remain = _size - temp;
	if (remain > 0)
		TcpalMemcpy(&Ring_Buff_Tcc353x[0], &_buff[temp], remain);

	Tcc353xRb.wp = ((wp+_size)%MAX_RING_BUFFER_SIZE);
	return (signed long)_size;
}

