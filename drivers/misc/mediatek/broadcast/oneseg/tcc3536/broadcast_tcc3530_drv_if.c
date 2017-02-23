#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

#include "Tcc353xDriver/PAL/tcpal_os.h"
#include "Tcc353xDriver/PAL/tcpal_spi.h"
#if !defined (_TCSPI_ONLY_)
#include "Tcc353xDriver/PAL/tcpal_i2c.h"
#endif

#include "Tcc353xDriver/common/tcc353x_common.h"
#include "Tcc353xDriver/api/inc/tcc353x_api.h"
#include "Tcc353xDriver/sample/monitoring/tcc353x_monitoring.h"
#include "Tcc353xDriver/sample/main/tcc353x_user_defines.h"

#include "broadcast_tcc353x.h"
#include "../broadcast_dmb_typedef.h"
#include "../broadcast_dmb_drv_ifdef.h"

#define _DISPLAY_MONITOR_DBG_LOG_
/*#define _CHECK_TS_ERROR_*/
/*#define _USE_ONSEG_SIGINFO_MODIFIED_MODE_*/

#define _USE_BRAZIL_FREQUENCY_
#define _1_SEG_FIFO_THR_	(188*16) /* 1seg interupt cycle 57ms, 416kbps, QPSK, Conv.Code:2/3, GI:1/8 */
#define _13_SEG_FIFO_THR_	(188*168)

#define _USE_MONITORING_TIME_GAP_
#define _FORCE_SEND_GOOD_SIGNAL_WHEN_CHANGING_FREQ_
#define _DBG_LOG_READ_DMB_DATA_

#define ONESEG_ANTENNA_LEVLE1 5
#define ONESEG_ANTENNA_LEVLE2 20
#define ONESEG_ANTENNA_LEVLE3 65

#define FULLSEG_ANTENNA_LEVLE1 15
#define FULLSEG_ANTENNA_LEVLE2 30
#define FULLSEG_ANTENNA_LEVLE3 70

static int full_seg_signal_strength_indicator = 0;
static int one_seg_signal_strength_indicator = 0;
static int oneseg_to_fullseg_value = 300;
static int fullseg_to_oneseg_value = 450;

#if defined(_CHECK_TS_ERROR_)
#define _MAX_CHECK_PIDS_	(20)
typedef struct {
	unsigned int Pids[_MAX_CHECK_PIDS_];
	unsigned char curr_continutity_cnt[_MAX_CHECK_PIDS_];
	unsigned int err_sync_cnt;
	unsigned int err_continurity_cnt;
	unsigned int err_tei;
} tcc353xCheckPids_st;
tcc353xCheckPids_st Tcc353xCheckPids;
#endif

/*----------------------------------------------------------------------------
 Test Defines
----------------------------------------------------------------------------*/
//#define _TEST_SEND_GOOD_SIGNAL_QUALITY_

typedef enum {
	TMM_13SEG = 0,
	TMM_1SEG,
	UHF_1SEG,
	UHF_13SEG,
} EnumIsdbType;

typedef enum {
	ENUM_GET_ALL = 1,
	ENUM_GET_BER,
	ENUM_GET_PER,
	ENUM_GET_CN,
	ENUM_GET_CN_PER_LAYER,
	ENUM_GET_LAYER_INFO,
	ENUM_GET_RECEIVE_STATUS,
	ENUM_GET_RSSI,
	ENUM_GET_SCAN_STATUS,
	ENUM_GET_SYS_INFO,
	ENUM_GET_TMCC_INFO,
	ENUM_GET_ONESEG_SIG_INFO
} EnumSigInfo;

extern Tcc353xOption_t Tcc353xOptionSingle;
extern TcpalSemaphore_t Tcc353xDrvSem;

static unsigned int frequencyTable[56] = {
	473143,479143,485143,491143,497143,
	503143,509143,515143,521143,527143,
	533143,539143,545143,551143,557143,
	563143,569143,575143,581143,587143,
	593143,599143,605143,611143,617143,
	623143,629143,635143,641143,647143,
	653143,659143,665143,671143,677143,
	683143,689143,695143,701143,707143,
	713143,719143,725143,731143,737143,
	743143,749143,755143,761143,767143,
	773143,779143,785143,791143,797143,
	803143,
};

#define MMBI_MAX_TABLE 33
I32S MMBI_FREQ_TABLE[MMBI_MAX_TABLE] = {
	207857, 208286, 208714, 209143, 209571,
	210000, 210429, 210857, 211286, 211714,
	212143, 212571, 213000, 213429, 213857,
	214286, 214714, 215143, 215571, 216000,
	216429, 216857, 217286, 217714, 218143,
	218571, 219000, 219429, 219857, 220286,
	220714, 221143, 221571
};

static int currentBroadCast = UHF_13SEG;
static int currentSelectedChannel = -1;

static unsigned char InterleavingLen[24] = {
	0,0,0, 4,2,1, 
	8,4,2, 16,8,4,
	32,16,8, 62,62,62,
	62,62,62, 63,63,63
};

static I32S TCC3536_OnAir = 0;
extern I32U gOverflowcnt;
static I32S IsOnesegOnlyMode = 0;

#if defined (_USE_MONITORING_TIME_GAP_)
#define _MON_TIME_INTERVAL_ 	500
TcpalTime_t CurrentMonitoringTime = 0;
#endif

void Tcc353xStreamBufferInit(I32S _moduleIndex);
void Tcc353xStreamBufferClose(I32S _moduleIndex);
void Tcc353xStreamBufferReset(I32S _moduleIndex);
I32U Tcc353xGetStreamBuffer(I32S _moduleIndex, I08U * _buff, I32U _size);

#if defined (_FORCE_SEND_GOOD_SIGNAL_WHEN_CHANGING_FREQ_)
#define _GOOD_SIGNAL_INTERVAL_MAX_ 	2000
#define _GOOD_SIGNAL_INTERVAL_MIN_ 	1000

static TcpalTime_t Time_channel_tune = 0;
static int Need_send_good_signal = 0;
#endif

extern I32U gOverflowcnt;

/*=======================================================
      Function          : broadcast_tcc353x_drv_get_antenna_level
      Description     : calurate next antenna level
      Parameter     : mode(1,13) Current cn, ber, antenna level
      Return Value     : int next_antenna_level

      when              model           who         edit history
  -------------------------------------------------------
      Aug.04.2013  1seg      taew00k.kang   created
======================================================== */

int broadcast_tcc353x_drv_get_antenna_level(const int segment, const int current_antenna_level, const int cn, const int ber)
{

    int         next_antenna_level = 0; //next_antenna_level

    next_antenna_level = current_antenna_level;

/* TCC 3535 Antanna level tune value 2013-10-24 [START] */
//#define FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_3_TO_2    ((ber > 320) || (cn < 1950 && ber > 300))
//#define FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_2_TO_1    (ber >= 450)
//#define FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_2_TO_3 ((ber <= 290 &&  cn >= 2000) || (ber < 250 && cn >= 1800))
//#define FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_1_TO_0    (ber >= 2000 && cn <= 1745)
//#define FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_1_TO_2 (ber < 380)
//#define FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_0_TO_1 (ber < 2000 && cn > 1750)
/* TCC 3535 Antanna level tune value 2013-10-24 [END] */

/* TCC 3535 Antanna level tune value 2015-01-30 [START] */
#define FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_2_TO_3 ((ber <= 290 &&  cn >= 2750) || (ber < 250 && cn >= 1350))
#define FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_3_TO_2 ((ber > 320) || (cn < 1300 && ber > 300))
#define FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_1_TO_2 (ber < 380)
#define FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_2_TO_1 (ber >= 450)
#define FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_0_TO_1 (ber < 1950 && cn > 1100  || ber < 1930)
#define FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_1_TO_0 (ber >= 1950)
/* TCC 3535 Antanna level tune value 2015-01-30 [END] */

/* TCC 3535 Antanna level tune value 2013-10-24 [START] */
#define ONESEG_ANTENNA_LEVEL_CONDITION_FROM_3_TO_2    (ber > 50) || (600 > cn)
#define ONESEG_ANTENNA_LEVEL_CONDITION_FROM_2_TO_1    (ber >= 200)
#define ONESEG_ANTENNA_LEVEL_CONDITION_FROM_2_TO_3    (ber <= 20 && 660 <= cn)
#define ONESEG_ANTENNA_LEVEL_CONDITION_FROM_1_TO_0    (ber >= 1950 && cn <= 300)
#define ONESEG_ANTENNA_LEVEL_CONDITION_FROM_1_TO_2    (ber < 100)
#define ONESEG_ANTENNA_LEVEL_CONDITION_FROM_0_TO_1    (ber < 1950 && cn > 300)
/* TCC 3535 Antanna level tune value 2013-10-24 [END] */

    if(segment==13)
    {
        /* CN - histerysis curve */
        if ( current_antenna_level == 3 )
        {
            if ( FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_3_TO_2 )
            {
                //TcpalPrintStatus((I08S *)"[ant_test][fulseg][3->2] ber(%d),cn(%d)\n", ber, cn);
                next_antenna_level = 2;
            }
        }
        else if ( current_antenna_level == 2 )
        {
            if ( FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_2_TO_1 )
            {
                //TcpalPrintStatus((I08S *)"[ant_test][fulseg][2->1] ber(%d),cn(%d)\n", ber, cn);
                next_antenna_level = 1;
            }
            if ( FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_2_TO_3 )
            {
                //TcpalPrintStatus((I08S *)"[ant_test][fulseg][2->3] ber(%d),cn(%d)\n", ber, cn);
                next_antenna_level = 3;
            }
        }
        else if ( current_antenna_level == 1 )
        {
            if( FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_1_TO_0 )
            {
                //TcpalPrintStatus((I08S *)"[ant_test][fulseg][1->0] ber(%d),cn(%d)\n", ber, cn);
                next_antenna_level = 0;
            }
            if ( FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_1_TO_2 )
            {
                //TcpalPrintStatus((I08S *)"[ant_test][fulseg][1->2] ber(%d),cn(%d)\n", ber, cn);
                next_antenna_level = 2;
            }
        }
        else //current_antenna_level == 0
        {
            if ( FULLSEG_ANTENNA_LEVEL_CONDITION_FROM_0_TO_1 )
            {
                //TcpalPrintStatus((I08S *)"[ant_test][fulseg][0->1] ber(%d),cn(%d)\n", ber, cn);
                next_antenna_level = 1;
            }
        }
    }
    else if(segment==1)
    {
        /* CN - histerysis curve */
        if ( current_antenna_level == 3 )
        {
            if ( ONESEG_ANTENNA_LEVEL_CONDITION_FROM_3_TO_2 )
            {
                //TcpalPrintStatus((I08S *)"[ant_test][1seg][3->2] ber(%d),cn(%d)\n", ber, cn);
                next_antenna_level = 2;
            }
        }
        else if ( current_antenna_level == 2 )
        {
            if ( ONESEG_ANTENNA_LEVEL_CONDITION_FROM_2_TO_1 )
            {
                //TcpalPrintStatus((I08S *)"[ant_test][1seg][2->1] ber(%d),cn(%d)\n", ber, cn);
                next_antenna_level = 1;
            }
            if ( ONESEG_ANTENNA_LEVEL_CONDITION_FROM_2_TO_3 )
            {
                //TcpalPrintStatus((I08S *)"[ant_test][1seg][2->3] ber(%d),cn(%d)\n", ber, cn);
                next_antenna_level = 3;
            }
        }
        else if ( current_antenna_level == 1 )
        {
            if( ONESEG_ANTENNA_LEVEL_CONDITION_FROM_1_TO_0 )
            {
                //TcpalPrintStatus((I08S *)"[ant_test][1seg][1->0]ber(%d),cn(%d)\n", ber, cn);
                next_antenna_level = 0;
            }
            if ( ONESEG_ANTENNA_LEVEL_CONDITION_FROM_1_TO_2 )
            {
                //TcpalPrintStatus((I08S *)"[ant_test][1seg][1->2]ber(%d),cn(%d)\n", ber, cn);
                next_antenna_level = 2;
            }
        }
        else
        {
            if ( ONESEG_ANTENNA_LEVEL_CONDITION_FROM_0_TO_1 )
            {
                //TcpalPrintStatus((I08S *)"[ant_test][1seg][0->1]ber(%d),cn(%d)\n", ber, cn);
                next_antenna_level = 1;
            }
        }
    }

    return next_antenna_level;

}

static unsigned int get_datarate(unsigned int mod, unsigned int cr, unsigned int gi, unsigned int segnum)
{
#define MOD_MAX	4
#define CR_MAX	5
#define GI_MAX	4

	unsigned int ret = 0;
	unsigned int qpskd[CR_MAX][GI_MAX]={
		{281, 312, 330, 340},
		{374, 416, 441, 454},
		{421, 468, 496, 511},
		{468, 520, 551, 567},
		{492, 546, 578, 596}
	};

	unsigned int qam16d[CR_MAX][GI_MAX] = {
		{562, 624, 661, 681},
		{749, 832, 881, 908},
		{843, 936, 991, 1021},
		{936, 1040, 1101, 1135},
		{983, 1092, 1156, 1192}
	};

	unsigned int qam64d[CR_MAX][GI_MAX] = {
		{843, 936, 991, 1021},
		{1123, 1248, 1322, 1362},
		{1264, 1404, 1487, 1532},
		{1404, 1560, 1652, 1702},
		{1475, 1638, 1735, 1787}
	};

	if(cr>=CR_MAX)
		return 0;
	if(gi>=GI_MAX)
		return 0;
	if(mod>=MOD_MAX)
		return 0;
	if(segnum==0 || segnum>13)
		return 0;


	if(mod==0 || mod==1)
		ret = qpskd[cr][gi]*segnum;
	else if(mod==2)
		ret = qam16d[cr][gi]*segnum;
	else
		ret = qam64d[cr][gi]*segnum;

	return ret;
}

/* Body of Internel function */
int	broadcast_tcc353x_drv_start(void)
{
	int rc;

	rc = OK;
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] broadcast_tcc353x_drv_start\n");
	return rc;
}

int	broadcast_tcc353x_get_stop_mode(void)
{
	int rc;

	rc = OK;
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] broadcast_tcc353x_get_stop_mode\n");
	return rc;
}

int	broadcast_tcc353x_drv_if_power_on(void)
{
	int rc = ERROR;

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] broadcast_tcc353x_drv_if_power_on\n");
	if (!tcc353x_is_power_on())
		rc = tcc353x_power_on();

#if defined(_CHECK_TS_ERROR_)
{
	int i;

	TcpalMemset(&Tcc353xCheckPids, 0x00, sizeof(tcc353xCheckPids_st));
	for(i=0;i<_MAX_CHECK_PIDS_; i++)
		Tcc353xCheckPids.Pids[i] = 0x1FFF;
}
#endif
	TcpalPrintErr((I08S *)"[dtv][tcc3536][debug_info] fifo_threshold=%d\n", (unsigned int)(_13_SEG_FIFO_THR_));
	return rc;
}

int	broadcast_tcc353x_drv_if_power_off(void)
{
	int rc = ERROR;

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] broadcast_tcc353x_drv_if_power_off\n");
	if (tcc353x_is_power_on()) {
		rc = tcc353x_power_off();
	} else {
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][exception] warning-already power off\n");
	}

#if defined(_CHECK_TS_ERROR_)
{
	int i;

	TcpalMemset(&Tcc353xCheckPids, 0x00, sizeof(tcc353xCheckPids_st));
	for(i=0;i<_MAX_CHECK_PIDS_; i++)
		Tcc353xCheckPids.Pids[i] = 0x1FFF;
}
#endif
	return rc;
}

static void Tcc353xWrapperSafeClose (void)
{
	/* close driver & power ctrl*/
	TCC3536_OnAir = 0;
	currentSelectedChannel = -1;
	currentBroadCast = UHF_13SEG;
	TcpalIrqDisable();

	Tcc353xApiClose(0);
	Tcc353xStreamBufferClose(0);
#if defined (_TCSPI_ONLY_)
	Tcc353xTccspiClose(0);
#else
	Tcc353xI2cClose(0);
#endif
	broadcast_tcc353x_drv_if_power_off();
}

int	broadcast_tcc353x_drv_if_open(void)
{
	int rc = ERROR;
	int ret = 0;
	Tcc353xStreamFormat_t streamFormat;

	TcpalSemaphoreLock(&Tcc353xDrvSem);

#if defined (_FORCE_SEND_GOOD_SIGNAL_WHEN_CHANGING_FREQ_)
	Time_channel_tune = 0;
	Need_send_good_signal = 0;
#endif

#if defined (_TCSPI_ONLY_)
	Tcc353xTccspiOpen(0);
#else
	Tcc353xI2cOpen(0);
#endif
	ret = Tcc353xApiOpen(0, &Tcc353xOptionSingle, sizeof(Tcc353xOption_t));
	if (ret < 0) {
		/* driver re-open routine */
		TcpalPrintErr((I08S *) "[dtv][tcc3536][error] TCC353x Re-init (close & open)...\n");
		Tcc353xWrapperSafeClose();

		/* re-open driver & power ctrl*/
		broadcast_tcc353x_drv_if_power_on();
#if defined (_TCSPI_ONLY_)
	Tcc353xTccspiOpen(0);
#else
	Tcc353xI2cOpen(0);
#endif
		ret = Tcc353xApiOpen(0, &Tcc353xOptionSingle, sizeof(Tcc353xOption_t));
		if (ret < 0) {
			TcpalPrintErr((I08S *) "[dtv][tcc3536][error] TCC353x Init Fail!!!\n");
			Tcc353xWrapperSafeClose();
			TcpalSemaphoreUnLock(&Tcc353xDrvSem);
			return ERROR;
		}
	}

	streamFormat.pidFilterEnable = 0;
	streamFormat.syncByteFilterEnable = 1;
	streamFormat.tsErrorFilterEnable = 1;
	streamFormat.tsErrorInsertEnable = 1;
	ret = Tcc353xApiInit(0, NULL, 0, &streamFormat);

	if (ret != TCC353X_RETURN_SUCCESS) {
		TcpalPrintErr((I08S *) "[dtv][tcc3536][error] TCC353x Init Fail!!!\n");
		Tcc353xWrapperSafeClose();
		rc = ERROR;
	} else {
		Tcc353xStreamBufferInit(0);
		TcpalIrqEnable();
		TcpalPrintStatus((I08S *) "[dtv][tcc3536][ok] TCC353x Init Success!!!\n");
		rc = OK;
	}

	TCC3536_OnAir = 1;
	TcpalSemaphoreUnLock(&Tcc353xDrvSem);
	return rc;
}

int	broadcast_tcc353x_drv_if_close(void)
{
	int rc = ERROR;	
	int ret = 0;

	TcpalSemaphoreLock(&Tcc353xDrvSem);
	TCC3536_OnAir = 0;
	currentSelectedChannel = -1;
	currentBroadCast = UHF_13SEG;

	TcpalIrqDisable();
	ret = Tcc353xApiClose(0);
	Tcc353xStreamBufferClose(0);
#if defined (_TCSPI_ONLY_)
	Tcc353xTccspiClose(0);
#else
	Tcc353xI2cClose(0);
#endif
	if(ret == TCC353X_RETURN_SUCCESS)
		rc = OK;

#if defined (_FORCE_SEND_GOOD_SIGNAL_WHEN_CHANGING_FREQ_)
	Time_channel_tune = 0;
	Need_send_good_signal = 0;
#endif

	TcpalSemaphoreUnLock(&Tcc353xDrvSem);
	return rc;
}

int	broadcast_tcc353x_drv_if_set_channel(struct broadcast_dmb_set_ch_info *udata)
{	
	Tcc353xTuneOptions tuneOption;
	signed long frequency = 214714; /*tmm*/
	int ret;
	int needLockCheck = 0;

	TcpalSemaphoreLock(&Tcc353xDrvSem);

	if(TCC3536_OnAir == 0 || udata == NULL) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] broadcast_tcc353x_drv_if_set_channel error [!TCC3536_OnAir]\n");
		TcpalSemaphoreUnLock(&Tcc353xDrvSem);
		return ERROR;
	}

	TcpalMemset (&tuneOption, 0x00, sizeof(tuneOption));

	/* uhf 1segment */
	currentSelectedChannel = udata->channel;

	if(udata->segment == 13) {
		currentBroadCast = UHF_13SEG;
		tuneOption.rfIfType = TCC353X_ZERO_IF;
		tuneOption.segmentType = TCC353X_ISDBT_13SEG;
#if defined (_TCSPI_ONLY_)
		tuneOption.userFifothr = _13_SEG_FIFO_THR_;
#else
		tuneOption.userFifothr = 0;
#endif
		IsOnesegOnlyMode = 0;
	} else {
		currentBroadCast = UHF_1SEG;
		tuneOption.rfIfType = TCC353X_LOW_IF;
		tuneOption.segmentType = TCC353X_ISDBT_1_OF_13SEG;
		IsOnesegOnlyMode = 1;
#if defined (_TCSPI_ONLY_)
		tuneOption.userFifothr = _1_SEG_FIFO_THR_;
#else
		tuneOption.userFifothr = 0;
#endif
	}

	needLockCheck = 1;

#if defined (_USE_BRAZIL_FREQUENCY_)
	if(udata->channel<14 || udata->channel>69) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] channel information error\n");
		TcpalSemaphoreUnLock(&Tcc353xDrvSem);
		return ERROR;
	}
	frequency = frequencyTable[udata->channel-14];
#else
	if(udata->channel<13 || udata->channel>62) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] channel information error\n");
		TcpalSemaphoreUnLock(&Tcc353xDrvSem);
		return ERROR;
	}
	frequency = frequencyTable[udata->channel-13];
#endif

	TcpalIrqDisable();
	gOverflowcnt = 0;

	if(needLockCheck && udata->mode == 1)	 /* Scan mode & need lock check */
		ret = Tcc353xApiChannelSearch(0, frequency, &tuneOption);
	else	 /* normal mode */
		ret = Tcc353xApiChannelSelect(0, frequency, &tuneOption);

#if defined(_CHECK_TS_ERROR_)
{
	int i;

	TcpalMemset(&Tcc353xCheckPids, 0x00, sizeof(tcc353xCheckPids_st));
	for(i=0;i<_MAX_CHECK_PIDS_; i++)
		Tcc353xCheckPids.Pids[i] = 0x1FFF;
}
#endif

#if defined (_FORCE_SEND_GOOD_SIGNAL_WHEN_CHANGING_FREQ_)
	Time_channel_tune = TcpalGetCurrentTimeCount_ms();
	Need_send_good_signal = 1;
#endif

	Tcc353xStreamBufferReset(0);
	Tcc353xMonitoringApiInit(0, 0);
	TcpalIrqEnable();

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][debug_info] broadcast_tcc353x_drv_if_set_channel frequency=%d, ret=%d\n", frequency, ret);

	if(ret!=TCC353X_RETURN_SUCCESS) {
		TcpalSemaphoreUnLock(&Tcc353xDrvSem);
		return ERROR;
	}

	TcpalSemaphoreUnLock(&Tcc353xDrvSem);
	return OK;
}

int	broadcast_tcc353x_drv_if_resync(void)
{
	int rc;

	/* TCC353x use auto-resync */
	rc = OK;
	return rc;
}

int	broadcast_tcc353x_drv_if_detect_sync(struct broadcast_dmb_sync_info *udata)
{
	IsdbLock_t lock;
	I08U reg;

	TcpalSemaphoreLock(&Tcc353xDrvSem);
	if(TCC3536_OnAir == 0) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] broadcast_tcc353x_drv_if_detect_sync error [!TCC3536_OnAir]\n");
		TcpalSemaphoreUnLock(&Tcc353xDrvSem);
		return ERROR;
	}

	Tcc353xApiRegisterRead(0, 0, 0x0B, &reg, 1);
	Tcc353xApiParseIsdbSyncStat(&lock, reg);

	if(lock.TMCC)
		udata->sync_status = 3;
	else if(lock.CFO)
		udata->sync_status = 1;
	else
		udata->sync_status = 0;

	udata->sync_ext_status = reg;
	TcpalSemaphoreUnLock(&Tcc353xDrvSem);
	return OK;
}

static int Tcc353xWrapperGetLayerInfo(int layer, Tcc353xStatus_t *st)
{
	int ret = 0;
	unsigned char modulation;
	unsigned char cr;
	unsigned char mode;
	unsigned int intLen,outIntLen;
	unsigned char segNo;
	unsigned int temp;

	if(((st->opstat.syncStatus>>8)&0x0F)<0x0C)
		return 0xFFFF;

	if(layer==0) {
		modulation = (st->opstat.AMod & 0x03);
		cr = (st->opstat.ACr & 0x07);
		mode = st->opstat.mode;
		intLen = st->opstat.AIntLen;
		segNo = st->opstat.ASegNo;
	} else if(layer==1) {
		modulation = (st->opstat.BMod & 0x03);
		cr = (st->opstat.BCr & 0x07);
		mode = st->opstat.mode;
		intLen = st->opstat.BIntLen;
		segNo = st->opstat.BSegNo;
	} else if(layer==2) {
		modulation = (st->opstat.CMod & 0x03);
		cr = (st->opstat.CCr & 0x07);
		mode = st->opstat.mode;
		intLen = st->opstat.CIntLen;
		segNo = st->opstat.CSegNo;
	} else {
		return 0xFFFF;
	}

	ret = (modulation << 13);
	ret |= (cr << 10);
	temp = intLen * 3 + mode;
	if(temp>23)
		outIntLen = 62; /*set others*/
	else
		outIntLen = InterleavingLen[temp];
	ret |= ((outIntLen & 0x3F)<<4);
	ret |=  (segNo & 0x0F);

	return ret;
}

static int Tcc353xWrapperGetTmccInfo(Tcc353xStatus_t *st)
{
	int ret = 0;

	if(!st->opstat.dataState)
		return 0xFF;

	ret = ((st->opstat.sysId & 0x03)<<6);
	ret |= ((st->opstat.tmccSwitchCnt & 0x0F)<<2);
	ret |= ((st->opstat.af & 0x01)<<1);
	ret |= (st->opstat.pr & 0x01);

	return ret;
}

static int Tcc353xWrapperGetReceiveStat(Tcc353xStatus_t *st)
{
	int ret = 0;

	if(st->opstat.dataState)
		ret = ((st->opstat.af & 0x01)<<7);

	if(st->opstat.dataState)
		ret |= 0x00;
	else if(st->status.isdbLock.TMCC)
		ret |= 0x01;
	else if(st->status.isdbLock.CFO)
		ret |= 0x02;
	else
		ret |= 0x03;

	return ret;
}

static int Tcc353xWrapperGetScanStat(Tcc353xStatus_t *st)
{
	int ret = 0;

	if(st->status.isdbLock.TMCC)
		return 2;
	else if(st->status.isdbLock.CFO)
		return 1;
	else
		return 3;

	return ret;
}

static int Tcc353xWrapperGetSysInfo(Tcc353xStatus_t *st)
{
	int ret = 0;
	int temp = 0;

	if((st->opstat.syncStatus>>8&0x0F)<0x0C)
		return 0xFF;

	if(st->opstat.gi==0)
		temp = 3;
	else if(st->opstat.gi==1)
		temp = 2;
	else if(st->opstat.gi==2)
		temp = 1;
	else
		temp = 0;

	ret = (temp<<4);
	ret |= ((st->opstat.mode&0x03)<<6);

	return ret;
}

#ifdef _DISPLAY_MONITOR_DBG_LOG_
char *cModulation[8] = {"DQPSK","QPSK","16QAM","64QAM","reserved","reserved","reserved","non hier"};
char *cCR[8] = {"1/2","2/3","3/4","5/6","7/8","reserved","reserved","non hier"};

char *cSysInd[8] = {"TV","Audio","reserved","reserved","reserved","reserved","reserved","reserved"};
char *cEmergency[8] = {"No Emergency","Emergency","reserved","reserved","reserved","reserved","reserved","reserved"};
char *cPartial[8] = {"No Partial reception","Partial reception","reserved","reserved","reserved","reserved","reserved","reserved"};

char *cEmergencyStart[8] = {"No Emergency Start","Emergency Start","reserved","reserved","reserved","reserved","reserved","reserved"};
char *cRcvStatus[8] = {"TS out","Frame Sync","Mode detect","mode searching","reserved","reserved","reserved","reserved"};

char *cScanStat[8] = {"Scan Fail","Scan decision...","Channel Exist","No channel","reserved","reserved","reserved","reserved"};
char *cMode[8] = {"Mode1","Mode2","Mode3","reserved","reserved","reserved","reserved","reserved"};
char *cGI[8] = {"1/32","1/16","1/8","1/4","reserved","reserved","reserved","reserved"};
#endif

#if defined (_USE_MONITORING_TIME_GAP_)
Tcc353xStatus_t SignalInfo;
#endif

static void broadcast_tcc353x_drv_if_get_oneseg_sig_info(Tcc353xStatus_t *pst, struct broadcast_dmb_control_info *pInfo)
{
	pInfo->sig_info.info.oneseg_info.lock = pst->status.isdbLock.TMCC;
	pInfo->sig_info.info.oneseg_info.cn = pst->status.snr.currentValue;
	pInfo->sig_info.info.oneseg_info.ber = pst->status.viterbiber[0].currentValue;
	pInfo->sig_info.info.oneseg_info.per = pst->status.tsper[0].currentValue;
	pInfo->sig_info.info.oneseg_info.agc = pst->bbLoopGain;
	pInfo->sig_info.info.oneseg_info.rssi = pst->status.rssi.currentValue/100;
	pInfo->sig_info.info.oneseg_info.ErrTSP = pst->opstat.ARsErrorCnt;
	pInfo->sig_info.info.oneseg_info.TotalTSP = pst->opstat.ARsCnt;

	if(pst->antennaPercent[0]>=80)
		pInfo->sig_info.info.oneseg_info.antenna_level = 3;
	else if(pst->antennaPercent[0]>=60)
		pInfo->sig_info.info.oneseg_info.antenna_level = 3;
	else if(pst->antennaPercent[0]>=40)
		pInfo->sig_info.info.oneseg_info.antenna_level = 2;
	else if(pst->antennaPercent[0]>=20)
		pInfo->sig_info.info.oneseg_info.antenna_level = 1;
	else
		pInfo->sig_info.info.oneseg_info.antenna_level = 0;

#if defined (_USE_ONSEG_SIGINFO_MODIFIED_MODE_)
	/*
	Num : Modulation
	Exp : CR
	mode : GI
	*/
	if((pst->opstat.syncStatus>>8&0x0F)<0x0C) {
		pInfo->sig_info.info.oneseg_info.Num = 0xFF;
		pInfo->sig_info.info.oneseg_info.Exp = 0xFF;
		pInfo->sig_info.info.oneseg_info.mode = 0xFF;
	} else {
		pInfo->sig_info.info.oneseg_info.Num = (pst->opstat.AMod & 0x03);
		pInfo->sig_info.info.oneseg_info.Exp = (pst->opstat.ACr & 0x07);

		if(pst->opstat.gi==0)
			pInfo->sig_info.info.oneseg_info.mode = 3; /* GI - 1/4 */
		else if(pst->opstat.gi==1)
			pInfo->sig_info.info.oneseg_info.mode = 2; /* GI - 1/8 */
		else if(pst->opstat.gi==2)
			pInfo->sig_info.info.oneseg_info.mode = 1; /* GI - 1/16 */
		else
			pInfo->sig_info.info.oneseg_info.mode = 0; /* GI - 1/32 */

#ifdef _DISPLAY_MONITOR_DBG_LOG_
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor] OneSeg Modulation [%s] CR[%s] GI[%s]\n",
			cModulation[(pInfo->sig_info.info.oneseg_info.Num)&0x07],
			cCR[(pInfo->sig_info.info.oneseg_info.Exp)&0x07],
			cGI[(pInfo->sig_info.info.oneseg_info.mode)&0x03] );
#endif
	}
#else
	pInfo->sig_info.info.oneseg_info.Num = 0;
	pInfo->sig_info.info.oneseg_info.Exp = 0;
	pInfo->sig_info.info.oneseg_info.mode = 0;
#endif /* _USE_ONSEG_SIGINFO_MODIFIED_MODE_ */

#ifdef _DISPLAY_MONITOR_DBG_LOG_
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor] OneSeg lock=%d, Antenna=%d, cn=%d, ber=%d, per=%d, rssi=%d, errTSP=%d, totalTSP=%d\n",
			pInfo->sig_info.info.oneseg_info.lock,
			pInfo->sig_info.info.oneseg_info.antenna_level, 
			pInfo->sig_info.info.oneseg_info.cn, 
			pInfo->sig_info.info.oneseg_info.ber, 
			pInfo->sig_info.info.oneseg_info.per, 
			pInfo->sig_info.info.oneseg_info.rssi,
			pInfo->sig_info.info.oneseg_info.ErrTSP,
			pInfo->sig_info.info.oneseg_info.TotalTSP);

#endif
}

int	broadcast_tcc353x_drv_if_get_sig_info(struct broadcast_dmb_control_info *pInfo)
{	
	int ret = 0;
	Tcc353xStatus_t st;
	int layer;
#if defined (_USE_MONITORING_TIME_GAP_)
	TcpalTime_t tempTime = 0;
	unsigned int timeGap = 0;
#endif

	pInfo->sig_info.info.mmb_info.oneseg_to_fullseg_value = oneseg_to_fullseg_value;
	pInfo->sig_info.info.mmb_info.fullseg_to_oneseg_value = fullseg_to_oneseg_value;

	TcpalSemaphoreLock(&Tcc353xDrvSem);
	if(TCC3536_OnAir == 0 || pInfo==NULL) {
		TcpalSemaphoreUnLock(&Tcc353xDrvSem);
		return ERROR;
	}

#if !defined (_TEST_SEND_GOOD_SIGNAL_QUALITY_)
#if defined (_USE_MONITORING_TIME_GAP_)
	tempTime = TcpalGetCurrentTimeCount_ms();
	timeGap = (unsigned int)(TcpalGetTimeIntervalCount_ms(CurrentMonitoringTime));

	if(timeGap > _MON_TIME_INTERVAL_) {
		CurrentMonitoringTime = tempTime;
		ret =Tcc353xMonitoringApiGetStatus(0, 0, &SignalInfo);
		if(ret != TCC353X_RETURN_SUCCESS) {
			TcpalSemaphoreUnLock(&Tcc353xDrvSem);
			return ERROR;
		}
		Tcc353xMonitoringApiAntennaPercentage(0, &SignalInfo, sizeof(Tcc353xStatus_t));
	}
	ret = 0;
	TcpalMemcpy(&st, &SignalInfo, sizeof(Tcc353xStatus_t));
#else
	ret =Tcc353xMonitoringApiGetStatus(0, 0, &st);
	if(ret != TCC353X_RETURN_SUCCESS) {
		TcpalSemaphoreUnLock(&Tcc353xDrvSem);
		return ERROR;
	}
	Tcc353xMonitoringApiAntennaPercentage(0, &st, sizeof(Tcc353xStatus_t));
#endif
#endif

	/* calculate ber, per
	ber = ((errorcount)+(overcount*8*8))/(totalpktcnt*204*8) * scale
	per = overcount/totalpktcnt*scale
	*/

#if defined (_FORCE_SEND_GOOD_SIGNAL_WHEN_CHANGING_FREQ_)
	if(Need_send_good_signal) {
		unsigned int timeGap = 0;
		timeGap = (unsigned int)(TcpalGetTimeIntervalCount_ms(Time_channel_tune));

		if(timeGap >= _GOOD_SIGNAL_INTERVAL_MAX_) {
			Need_send_good_signal = 0;
			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor] send current signal info [timeout]\n");
		} else {
			if (!st.status.isdbLock.TMCC){
				/* unlock status */
				Need_send_good_signal = 0;
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor] send current signal info [tmcc unlock]\n");
			} else if(st.opstat.ARsCnt>0 && timeGap >_GOOD_SIGNAL_INTERVAL_MIN_) {
				Need_send_good_signal = 0;
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor] send current signal info [pkt cnt exist]\n");
			} else {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor] Force good ber,per value [ARSCnt=%d]\n",st.opstat.ARsCnt);

				/* set good ber, per for full-segment */
				st.opstat.ARsCnt = 1;	/* indication of modified value & avoid max ber, per */
				st.opstat.BRsCnt = 1;	/* indication of modified value & avoid max ber, per */
				st.opstat.CRsCnt = 1;	/* indication of modified value & avoid max ber, per */
				st.opstat.ARsErrorCnt = 0;
				st.opstat.BRsErrorCnt = 0;
				st.opstat.CRsErrorCnt = 0;
				st.opstat.ARsOverCnt = 0;
				st.opstat.BRsOverCnt = 0;
				st.opstat.CRsOverCnt = 0;

				/* set good ber, per for one-segment */
				st.status.viterbiber[0].currentValue = 0;
				st.status.viterbiber[1].currentValue = 0;
				st.status.viterbiber[2].currentValue = 0;
				st.status.tsper[0].currentValue = 0;
				st.status.tsper[1].currentValue = 0;
				st.status.tsper[2].currentValue = 0;
			}
		}
	}
#endif

#if defined (_TEST_SEND_GOOD_SIGNAL_QUALITY_)
	/* set good ber, per for full-segment */
	st.opstat.ARsCnt = 1;	/* indication of modified value & avoid max ber, per */
	st.opstat.BRsCnt = 1;	/* indication of modified value & avoid max ber, per */
	st.opstat.CRsCnt = 1;	/* indication of modified value & avoid max ber, per */
	st.opstat.ARsErrorCnt = 0;
	st.opstat.BRsErrorCnt = 0;
	st.opstat.CRsErrorCnt = 0;
	st.opstat.ARsOverCnt = 0;
	st.opstat.BRsOverCnt = 0;
	st.opstat.CRsOverCnt = 0;

	/* set good ber, per for one-segment */
	st.status.viterbiber[0].currentValue = 0;
	st.status.viterbiber[1].currentValue = 0;
	st.status.viterbiber[2].currentValue = 0;
	st.status.tsper[0].currentValue = 0;
	st.status.tsper[1].currentValue = 0;
	st.status.tsper[2].currentValue = 0;

	pInfo->sig_info.info.mmb_info.antenna_level_oneseg = 3;
	pInfo->sig_info.info.mmb_info.antenna_level_fullseg = 3;
#endif

	layer = pInfo->cmd_info.layer;
	pInfo->cmd_info.over = gOverflowcnt;

	switch(pInfo->cmd_info.cmd)
	{
	case ENUM_GET_ALL:
		pInfo->sig_info.info.mmb_info.cn = 
			st.status.snr.currentValue;

		if(layer==0) {
			pInfo->sig_info.info.mmb_info.ber_a = 
				st.status.viterbiber[0].currentValue;
			pInfo->sig_info.info.mmb_info.per_a = 
				st.status.tsper[0].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_a = 
				st.opstat.ARsCnt;
			pInfo->sig_info.info.mmb_info.layerinfo_a =  
				Tcc353xWrapperGetLayerInfo(0, &st);
		} else if(layer==1) {
			pInfo->sig_info.info.mmb_info.ber_b = 
				st.status.viterbiber[1].currentValue;
			pInfo->sig_info.info.mmb_info.per_b = 
				st.status.tsper[1].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_b = 
				st.opstat.BRsCnt;
			pInfo->sig_info.info.mmb_info.layerinfo_b =  
				Tcc353xWrapperGetLayerInfo(1, &st);
		} else if(layer==2) {
			pInfo->sig_info.info.mmb_info.ber_c = 
				st.status.viterbiber[2].currentValue;
			pInfo->sig_info.info.mmb_info.per_c = 
				st.status.tsper[2].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_c = 
				st.opstat.CRsCnt;
			pInfo->sig_info.info.mmb_info.layerinfo_c =  
				Tcc353xWrapperGetLayerInfo(2, &st);
		} else {
			pInfo->sig_info.info.mmb_info.ber_a = 
				st.status.viterbiber[0].currentValue;
			pInfo->sig_info.info.mmb_info.per_a = 
				st.status.tsper[0].currentValue;
			pInfo->sig_info.info.mmb_info.ber_b = 
				st.status.viterbiber[1].currentValue;
			pInfo->sig_info.info.mmb_info.per_b = 
				st.status.tsper[1].currentValue;
			pInfo->sig_info.info.mmb_info.ber_c = 
				st.status.viterbiber[2].currentValue;
			pInfo->sig_info.info.mmb_info.per_c = 
				st.status.tsper[2].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_a = 
				st.opstat.ARsCnt;
			pInfo->sig_info.info.mmb_info.total_tsp_b = 
				st.opstat.BRsCnt;
			pInfo->sig_info.info.mmb_info.total_tsp_c = 
				st.opstat.CRsCnt;
			pInfo->sig_info.info.mmb_info.layerinfo_a =  
				Tcc353xWrapperGetLayerInfo(0, &st);
			pInfo->sig_info.info.mmb_info.layerinfo_b =  
				Tcc353xWrapperGetLayerInfo(1, &st);
			pInfo->sig_info.info.mmb_info.layerinfo_c =  
				Tcc353xWrapperGetLayerInfo(2, &st);
		}

		pInfo->sig_info.info.mmb_info.tmccinfo = 
			Tcc353xWrapperGetTmccInfo(&st);

		pInfo->sig_info.info.mmb_info.receive_status = 
			Tcc353xWrapperGetReceiveStat(&st);

		pInfo->sig_info.info.mmb_info.rssi = 
			(st.status.rssi.currentValue);

		pInfo->sig_info.info.mmb_info.scan_status =
			Tcc353xWrapperGetScanStat(&st);

		pInfo->sig_info.info.mmb_info.sysinfo =
			Tcc353xWrapperGetSysInfo(&st);
		if(IsOnesegOnlyMode)
		{
			pInfo->sig_info.info.mmb_info.layerinfo_b = 0xFFFF;
			pInfo->sig_info.info.mmb_info.layerinfo_c = 0xFFFF;
			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] (1) 1seg only mode : set layer info B,C as a default\n");
		}

		pInfo->sig_info.info.mmb_info.agc = st.rfLoopGain;
		pInfo->sig_info.info.mmb_info.ber_1seg = st.status.viterbiber[0].currentValue;
		pInfo->sig_info.info.mmb_info.per_1seg = st.status.tsper[0].currentValue;
		pInfo->sig_info.info.mmb_info.total_tsp_1seg = st.opstat.ARsCnt;
		pInfo->sig_info.info.mmb_info.err_tsp_1seg = st.opstat.ARsOverCnt;
		one_seg_signal_strength_indicator = broadcast_tcc353x_drv_get_antenna_level(1, one_seg_signal_strength_indicator, pInfo->sig_info.info.mmb_info.cn, pInfo->sig_info.info.mmb_info.ber_1seg);
		pInfo->sig_info.info.mmb_info.antenna_level_oneseg = one_seg_signal_strength_indicator;
		//TcpalPrintStatus((I08S *)"[bhj][tcc3536][monitor][1seg] cn(%d), ber(%d)\n", pInfo->sig_info.info.mmb_info.cn, pInfo->sig_info.info.mmb_info.ber_1seg);

		if((st.opstat.CSegNo>0) && (st.opstat.CSegNo != 0x0F)){
			/* wich layer is fullseg? */
			int maxdata_a, maxdata_b, maxdata_c;

			maxdata_a = get_datarate(st.opstat.AMod, st.opstat.ACr, st.opstat.gi, st.opstat.ASegNo);
			maxdata_b = get_datarate(st.opstat.BMod, st.opstat.BCr, st.opstat.gi, st.opstat.BSegNo);
			maxdata_c = get_datarate(st.opstat.CMod, st.opstat.CCr, st.opstat.gi, st.opstat.CSegNo);

			if(maxdata_c>=maxdata_a && maxdata_c>=maxdata_b) {
				pInfo->sig_info.info.mmb_info.ber_fullseg = st.status.viterbiber[2].currentValue;
				pInfo->sig_info.info.mmb_info.per_fullseg = st.status.tsper[2].currentValue;
				pInfo->sig_info.info.mmb_info.total_tsp_fullseg = st.opstat.CRsCnt;
				pInfo->sig_info.info.mmb_info.err_tsp_fullseg = st.opstat.CRsOverCnt;
			} else if(maxdata_b>=maxdata_a && maxdata_b>=maxdata_c) {
				pInfo->sig_info.info.mmb_info.ber_fullseg = st.status.viterbiber[1].currentValue;
				pInfo->sig_info.info.mmb_info.per_fullseg = st.status.tsper[1].currentValue;
				pInfo->sig_info.info.mmb_info.total_tsp_fullseg = st.opstat.BRsCnt;
				pInfo->sig_info.info.mmb_info.err_tsp_fullseg = st.opstat.BRsOverCnt;
			} else {
				pInfo->sig_info.info.mmb_info.ber_fullseg = st.status.viterbiber[0].currentValue;
				pInfo->sig_info.info.mmb_info.per_fullseg = st.status.tsper[0].currentValue;
				pInfo->sig_info.info.mmb_info.total_tsp_fullseg = st.opstat.ARsCnt;
				pInfo->sig_info.info.mmb_info.err_tsp_fullseg = st.opstat.ARsOverCnt;
			}

			full_seg_signal_strength_indicator = broadcast_tcc353x_drv_get_antenna_level(13, full_seg_signal_strength_indicator, pInfo->sig_info.info.mmb_info.cn, pInfo->sig_info.info.mmb_info.ber_fullseg);
			pInfo->sig_info.info.mmb_info.antenna_level_fullseg = full_seg_signal_strength_indicator;
			//TcpalPrintStatus((I08S *)"[bhj][tcc3536][monitor][fullseg] cn(%d), ber(%d)\n", pInfo->sig_info.info.mmb_info.cn, pInfo->sig_info.info.mmb_info.ber_fullseg);

			/* wich layer is oneseg? */
			if(st.opstat.BSegNo==1 && st.opstat.ASegNo!=1) {
				pInfo->sig_info.info.mmb_info.ber_1seg = st.status.viterbiber[1].currentValue;
				pInfo->sig_info.info.mmb_info.per_1seg = st.status.tsper[1].currentValue;
				pInfo->sig_info.info.mmb_info.total_tsp_1seg = st.opstat.BRsCnt;
				pInfo->sig_info.info.mmb_info.err_tsp_1seg = st.opstat.BRsOverCnt;
				one_seg_signal_strength_indicator = broadcast_tcc353x_drv_get_antenna_level(1, one_seg_signal_strength_indicator, pInfo->sig_info.info.mmb_info.cn, pInfo->sig_info.info.mmb_info.ber_1seg);
				pInfo->sig_info.info.mmb_info.antenna_level_oneseg = one_seg_signal_strength_indicator;
				//TcpalPrintStatus((I08S *)"[bhj][tcc3536][monitor][1seg] cn(%d), ber(%d)\n", pInfo->sig_info.info.mmb_info.cn, pInfo->sig_info.info.mmb_info.ber_1seg);
			} else {
				/* same as old values */
			}
		} else if((st.opstat.BSegNo > 0) && (st.opstat.BSegNo != 0x0F)){
			pInfo->sig_info.info.mmb_info.ber_fullseg = st.status.viterbiber[1].currentValue;
			pInfo->sig_info.info.mmb_info.per_fullseg = st.status.tsper[1].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_fullseg = st.opstat.BRsCnt;
			pInfo->sig_info.info.mmb_info.err_tsp_fullseg = st.opstat.BRsOverCnt;
			full_seg_signal_strength_indicator = broadcast_tcc353x_drv_get_antenna_level(13, full_seg_signal_strength_indicator, pInfo->sig_info.info.mmb_info.cn, pInfo->sig_info.info.mmb_info.ber_fullseg);
			pInfo->sig_info.info.mmb_info.antenna_level_fullseg = full_seg_signal_strength_indicator;
			//TcpalPrintStatus((I08S *)"[bhj][tcc3536][monitor][fullseg] cn(%d), ber(%d)\n", pInfo->sig_info.info.mmb_info.cn, pInfo->sig_info.info.mmb_info.ber_fullseg);
		} else {
			pInfo->sig_info.info.mmb_info.ber_fullseg = 
				pInfo->sig_info.info.mmb_info.ber_1seg;
			pInfo->sig_info.info.mmb_info.per_fullseg = 
				pInfo->sig_info.info.mmb_info.per_1seg;
			pInfo->sig_info.info.mmb_info.total_tsp_fullseg = 
				pInfo->sig_info.info.mmb_info.total_tsp_1seg;
			pInfo->sig_info.info.mmb_info.err_tsp_fullseg = 
				pInfo->sig_info.info.mmb_info.err_tsp_fullseg;
			full_seg_signal_strength_indicator = broadcast_tcc353x_drv_get_antenna_level(13, full_seg_signal_strength_indicator, pInfo->sig_info.info.mmb_info.cn, pInfo->sig_info.info.mmb_info.ber_fullseg);
				pInfo->sig_info.info.mmb_info.antenna_level_fullseg = full_seg_signal_strength_indicator;
			one_seg_signal_strength_indicator = broadcast_tcc353x_drv_get_antenna_level(1, one_seg_signal_strength_indicator, pInfo->sig_info.info.mmb_info.cn, pInfo->sig_info.info.mmb_info.ber_1seg);
				pInfo->sig_info.info.mmb_info.antenna_level_oneseg = one_seg_signal_strength_indicator;
		}

#ifdef _DISPLAY_MONITOR_DBG_LOG_
		{
			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] [ENUM_GET_ALL] layer=%d\n",layer);
			if(layer==0 || layer>=3) {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] Layer A-----------------\n");

				if(pInfo->sig_info.info.mmb_info.layerinfo_a ==0xFFFF) {
					TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] Layer info fail\n");
				} else {
					TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] Modulation [%s] CR[%s] TimeInterleave=%d, Segment=%d\n",
					cModulation[(pInfo->sig_info.info.mmb_info.layerinfo_a>>13)&0x07],
					cCR[(pInfo->sig_info.info.mmb_info.layerinfo_a>>10)&0x07],
					(pInfo->sig_info.info.mmb_info.layerinfo_a>>4)&0x1F,
					(pInfo->sig_info.info.mmb_info.layerinfo_a)&0x0F );
				}

				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] ber=%d, per=%d, totalTSP=%d\n",
					pInfo->sig_info.info.mmb_info.ber_a,
					pInfo->sig_info.info.mmb_info.per_a, 
					pInfo->sig_info.info.mmb_info.total_tsp_a);
			}

			if(layer==1 || layer>=3) {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] Layer B-----------------\n");

				if(pInfo->sig_info.info.mmb_info.layerinfo_b ==0xFFFF) {
					TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] Layer info fail\n");
				} else {
					TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] Modulation [%s] CR[%s] TimeInterleave=%d, Segment=%d\n",
					cModulation[(pInfo->sig_info.info.mmb_info.layerinfo_b>>13)&0x07],
					cCR[(pInfo->sig_info.info.mmb_info.layerinfo_b>>10)&0x07],
					(pInfo->sig_info.info.mmb_info.layerinfo_b>>4)&0x1F,
					(pInfo->sig_info.info.mmb_info.layerinfo_b)&0x0F );
				}

				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] ber=%d, per=%d, totalTSP=%d\n",
					pInfo->sig_info.info.mmb_info.ber_b,
					pInfo->sig_info.info.mmb_info.per_b, 
					pInfo->sig_info.info.mmb_info.total_tsp_b);
			}

			if(layer==2 || layer>=3) {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] Layer C-----------------\n");

				if(pInfo->sig_info.info.mmb_info.layerinfo_c ==0xFFFF) {
					TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] Layer info fail\n");
				} else {
					TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] Modulation [%s] CR[%s] TimeInterleave=%d, Segment=%d\n",
					cModulation[(pInfo->sig_info.info.mmb_info.layerinfo_c>>13)&0x07],
					cCR[(pInfo->sig_info.info.mmb_info.layerinfo_c>>10)&0x07],
					(pInfo->sig_info.info.mmb_info.layerinfo_c>>4)&0x1F,
					(pInfo->sig_info.info.mmb_info.layerinfo_c)&0x0F );
				}

				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] ber=%d, per=%d, totalTSP=%d\n",
					pInfo->sig_info.info.mmb_info.ber_c,
					pInfo->sig_info.info.mmb_info.per_c, 
					pInfo->sig_info.info.mmb_info.total_tsp_c);
			}
			if(IsOnesegOnlyMode)
			{
				pInfo->sig_info.info.mmb_info.layerinfo_b = 0xFFFF;
				pInfo->sig_info.info.mmb_info.layerinfo_c = 0xFFFF;
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] (2) 1seg only mode : set layer info B,C as a default\n");
			}

			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] -----------------------\n");

			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] rssi=%d, cn=%d\n",
				pInfo->sig_info.info.mmb_info.rssi,
				pInfo->sig_info.info.mmb_info.cn);

			if(pInfo->sig_info.info.mmb_info.tmccinfo==0xFF) {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] tmcc info fail\n");
			} else {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] System [%s] transParam=%d, %s %s\n",
					cSysInd[(pInfo->sig_info.info.mmb_info.tmccinfo>>6)&0x03],
					(pInfo->sig_info.info.mmb_info.tmccinfo>>2)&0x0F,
					cEmergency[(pInfo->sig_info.info.mmb_info.tmccinfo>>1)&0x01],
					cPartial[(pInfo->sig_info.info.mmb_info.tmccinfo)&0x01] );
			}


			if(pInfo->sig_info.info.mmb_info.receive_status==0xFF) {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] receive status fail\n");
			} else {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] %s %s\n",
					cEmergencyStart[(pInfo->sig_info.info.mmb_info.receive_status>>7)&0x01],
					cRcvStatus[(pInfo->sig_info.info.mmb_info.receive_status)&0x07] );
			}

			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] scan status [%s]\n",
				cScanStat[(pInfo->sig_info.info.mmb_info.scan_status)&0x03] );

			if(pInfo->sig_info.info.mmb_info.sysinfo==0xFF) {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] system info fail\n");
			} else {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] %s GI[%s]\n",
					cMode[(pInfo->sig_info.info.mmb_info.sysinfo>>6)&0x03],
					cGI[(pInfo->sig_info.info.mmb_info.sysinfo>>4)&0x03] );
			}
		}
#endif
	break;

	case ENUM_GET_BER:

		if(layer==0) {
			pInfo->sig_info.info.mmb_info.ber_a = 
					st.status.viterbiber[0].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_a = 
					st.opstat.ARsCnt;
		} else if(layer==1) {
			pInfo->sig_info.info.mmb_info.ber_b = 
					st.status.viterbiber[1].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_b = 
					st.opstat.BRsCnt;
		} else if(layer==2) {
			pInfo->sig_info.info.mmb_info.ber_c = 
					st.status.viterbiber[2].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_c = 
					st.opstat.CRsCnt;
		} else {
			pInfo->sig_info.info.mmb_info.ber_a = 
					st.status.viterbiber[0].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_a = 
					st.opstat.ARsCnt;
			pInfo->sig_info.info.mmb_info.ber_b = 
					st.status.viterbiber[1].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_b = 
					st.opstat.BRsCnt;
			pInfo->sig_info.info.mmb_info.ber_c = 
					st.status.viterbiber[2].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_c = 
					st.opstat.CRsCnt;
		}

#ifdef _DISPLAY_MONITOR_DBG_LOG_
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] [ENUM_GET_BER] Layer=%d\n",layer);
		if(layer==0 || layer>=3) {
			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] ber_a=%d, total_tsp_a=%d\n",
					pInfo->sig_info.info.mmb_info.ber_a,
					pInfo->sig_info.info.mmb_info.total_tsp_a);
		}
		if(layer==1 || layer>=3) {
			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] ber_b=%d, total_tsp_b=%d\n",
					pInfo->sig_info.info.mmb_info.ber_b,
					pInfo->sig_info.info.mmb_info.total_tsp_b);
		}
		if(layer==2 || layer>=3) {
			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] ber_c=%d, total_tsp_c=%d\n",
					pInfo->sig_info.info.mmb_info.ber_c,
					pInfo->sig_info.info.mmb_info.total_tsp_c);
		}
#endif
	break;

	case ENUM_GET_PER:
		if(layer==0) {
			pInfo->sig_info.info.mmb_info.per_a = 
					st.status.tsper[0].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_a = 
					st.opstat.ARsCnt;
		} else if(layer==1) {
			pInfo->sig_info.info.mmb_info.per_b = 
					st.status.tsper[1].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_b = 
					st.opstat.BRsCnt;
		} else if(layer==2) {
			pInfo->sig_info.info.mmb_info.per_c = 
					st.status.tsper[2].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_c = 
					st.opstat.CRsCnt;
		} else {
			pInfo->sig_info.info.mmb_info.per_a = 
					st.status.tsper[0].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_a = 
					st.opstat.ARsCnt;
			pInfo->sig_info.info.mmb_info.per_b = 
					st.status.tsper[1].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_b = 
					st.opstat.BRsCnt;
			pInfo->sig_info.info.mmb_info.per_c = 
					st.status.tsper[2].currentValue;
			pInfo->sig_info.info.mmb_info.total_tsp_c = 
					st.opstat.CRsCnt;
		}
#ifdef _DISPLAY_MONITOR_DBG_LOG_
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] [ENUM_GET_PER] Layer=%d\n",layer);
		if(layer==0 || layer>=3) {
			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] per_a=%d, total_tsp_a=%d\n",
					pInfo->sig_info.info.mmb_info.per_a,
					pInfo->sig_info.info.mmb_info.total_tsp_a);
		}
		if(layer==1 || layer>=3) {
			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] per_b=%d, total_tsp_b=%d\n",
					pInfo->sig_info.info.mmb_info.per_b,
					pInfo->sig_info.info.mmb_info.total_tsp_b);
		}
		if(layer==2 || layer>=3) {
			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] per_c=%d, total_tsp_c=%d\n",
					pInfo->sig_info.info.mmb_info.per_c,
					pInfo->sig_info.info.mmb_info.total_tsp_c);
		}
#endif
	break;

	case ENUM_GET_CN:
		pInfo->sig_info.info.mmb_info.cn = 
			st.status.snr.currentValue;
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] cn=%d\n",
				pInfo->sig_info.info.mmb_info.cn);
	break;

	case ENUM_GET_CN_PER_LAYER:
		pInfo->sig_info.info.mmb_info.cn = 
			st.status.snr.currentValue;
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] cn=%d\n",
				pInfo->sig_info.info.mmb_info.cn);
	break;

	case ENUM_GET_LAYER_INFO:
		if(layer==0) {
			pInfo->sig_info.info.mmb_info.layerinfo_a =  
				Tcc353xWrapperGetLayerInfo(layer, &st);
		} else if(layer==1) {
			pInfo->sig_info.info.mmb_info.layerinfo_b =  
				Tcc353xWrapperGetLayerInfo(layer, &st);
		} else if(layer==2) {
			pInfo->sig_info.info.mmb_info.layerinfo_c =  
				Tcc353xWrapperGetLayerInfo(layer, &st);
		} else {
			pInfo->sig_info.info.mmb_info.layerinfo_a =  
				Tcc353xWrapperGetLayerInfo(0, &st);
			pInfo->sig_info.info.mmb_info.layerinfo_b =  
				Tcc353xWrapperGetLayerInfo(1, &st);
			pInfo->sig_info.info.mmb_info.layerinfo_c =  
				Tcc353xWrapperGetLayerInfo(2, &st);
		}
#ifdef _DISPLAY_MONITOR_DBG_LOG_
		{
			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] [ENUM_GET_LAYER_INFO] Layer=%d\n",layer);

			if(layer==0 || layer>=3) {
				if(pInfo->sig_info.info.mmb_info.layerinfo_a==0xFFFF) {
					TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] layerinfo_a fail\n");
				} else {
					TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] [A] Modulation [%s] CR[%s] TimeInterleave=%d, Segment=%d\n",
					cModulation[(pInfo->sig_info.info.mmb_info.layerinfo_a>>13)&0x07],
					cCR[(pInfo->sig_info.info.mmb_info.layerinfo_a>>10)&0x07],
					(pInfo->sig_info.info.mmb_info.layerinfo_a>>4)&0x1F,
					(pInfo->sig_info.info.mmb_info.layerinfo_a)&0x0F );
				}
			}
			if(layer==1 || layer>=3) {
				if(pInfo->sig_info.info.mmb_info.layerinfo_b==0xFFFF) {
					TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] layerinfo_b fail\n");
				} else {
					TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] [B] Modulation [%s] CR[%s] TimeInterleave=%d, Segment=%d\n",
					cModulation[(pInfo->sig_info.info.mmb_info.layerinfo_b>>13)&0x07],
					cCR[(pInfo->sig_info.info.mmb_info.layerinfo_b>>10)&0x07],
					(pInfo->sig_info.info.mmb_info.layerinfo_b>>4)&0x1F,
					(pInfo->sig_info.info.mmb_info.layerinfo_b)&0x0F );
				}
			}
			if(layer==2 || layer>=3) {
				if(pInfo->sig_info.info.mmb_info.layerinfo_c==0xFFFF) {
					TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] layerinfo_c fail\n");
				} else {
					TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] [C] Modulation [%s] CR[%s] TimeInterleave=%d, Segment=%d\n",
					cModulation[(pInfo->sig_info.info.mmb_info.layerinfo_c>>13)&0x07],
					cCR[(pInfo->sig_info.info.mmb_info.layerinfo_c>>10)&0x07],
					(pInfo->sig_info.info.mmb_info.layerinfo_c>>4)&0x1F,
					(pInfo->sig_info.info.mmb_info.layerinfo_c)&0x0F );
				}
			}
		}
#endif
	break;

	case ENUM_GET_RECEIVE_STATUS:
		pInfo->sig_info.info.mmb_info.receive_status = 
				Tcc353xWrapperGetReceiveStat(&st);
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] rcv status=%d\n",
				pInfo->sig_info.info.mmb_info.receive_status);
		/* for debugging log */
#ifdef _DISPLAY_MONITOR_DBG_LOG_
		{
			if(pInfo->sig_info.info.mmb_info.receive_status==0xFF) {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] receive status fail\n");
			} else {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] %s %s\n",
					cEmergencyStart[(pInfo->sig_info.info.mmb_info.receive_status>>7)&0x01],
					cRcvStatus[(pInfo->sig_info.info.mmb_info.receive_status)&0x07] );
			}
		}
#endif
	break;

	case ENUM_GET_RSSI:
		pInfo->sig_info.info.mmb_info.rssi = 
			(st.status.rssi.currentValue);
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] rssi=%d\n",
				pInfo->sig_info.info.mmb_info.rssi);
	break;

	case ENUM_GET_SCAN_STATUS:
		pInfo->sig_info.info.mmb_info.scan_status =
			Tcc353xWrapperGetScanStat(&st);
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] scan statu=%d\n",
				pInfo->sig_info.info.mmb_info.scan_status);
		/* for debugging log */
#ifdef _DISPLAY_MONITOR_DBG_LOG_
		{
			char *cScanStat[8] = {"Scan Fail","Scan decision...","Channel Exist","No channel","reserved","reserved","reserved","reserved"};
			TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] scan status [%s]\n",
				cScanStat[(pInfo->sig_info.info.mmb_info.scan_status)&0x03] );
		}
#endif
	break;

	case ENUM_GET_SYS_INFO:
		pInfo->sig_info.info.mmb_info.sysinfo =
			Tcc353xWrapperGetSysInfo(&st);
		TcpalPrintStatus((I08S *)"[fullseg][monitor]  sys info=%d\n",
				pInfo->sig_info.info.mmb_info.sysinfo);
		/* for debugging log */
#ifdef _DISPLAY_MONITOR_DBG_LOG_
		{
			if(pInfo->sig_info.info.mmb_info.sysinfo==0xFF) {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] system info fail\n");
			} else {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] %s GI[%s]\n",
					cMode[(pInfo->sig_info.info.mmb_info.sysinfo>>6)&0x03],
					cGI[(pInfo->sig_info.info.mmb_info.sysinfo>>4)&0x03] );
			}
		}
#endif
	break;

	case ENUM_GET_TMCC_INFO:
		pInfo->sig_info.info.mmb_info.tmccinfo = 
			Tcc353xWrapperGetTmccInfo(&st);
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] tmcc info=%d\n",
				pInfo->sig_info.info.mmb_info.tmccinfo);
		/* for debugging log */
#ifdef _DISPLAY_MONITOR_DBG_LOG_
		{
			if(pInfo->sig_info.info.mmb_info.tmccinfo==0xFF) {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] tmcc info fail\n");
			} else {
				TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] System [%s] transParam=%d, %s %s\n",
					cSysInd[(pInfo->sig_info.info.mmb_info.tmccinfo>>6)&0x03],
					(pInfo->sig_info.info.mmb_info.tmccinfo>>2)&0x0F,
					cEmergency[(pInfo->sig_info.info.mmb_info.tmccinfo>>1)&0x01],
					cPartial[(pInfo->sig_info.info.mmb_info.tmccinfo)&0x01] );
			}
		}
#endif
	break;

	case ENUM_GET_ONESEG_SIG_INFO:
		broadcast_tcc353x_drv_if_get_oneseg_sig_info(&st, pInfo);
	break;

	default:
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][monitor][fullseg] sig_info unknown command=%d\n",
				pInfo->cmd_info.cmd);
		TcpalSemaphoreUnLock(&Tcc353xDrvSem);
		return -1;
	break;
	}

	TcpalSemaphoreUnLock(&Tcc353xDrvSem);
	return OK;
}

int	broadcast_tcc353x_drv_if_get_ch_info(struct broadcast_dmb_ch_info *ch_info)
{
	int rc = ERROR;

	if(TCC3536_OnAir == 0) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] broadcast_tcc353x_drv_if_get_ch_info error [!TCC3536_OnAir]\n");
		return ERROR;
	}

	/* Unused function */
	rc = OK;
	return rc;
}

int	broadcast_tcc353x_drv_if_get_dmb_data(struct broadcast_dmb_data_info *pdmb_data)
{
	if(TCC3536_OnAir == 0 || pdmb_data==NULL) {
		return ERROR;
	}

	if(pdmb_data->data_buf_addr == 0) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] broadcast_tcc353x_drv_if_get_dmb_data[ERR] data_buf is null\n");
		return ERROR;
	}

	if(pdmb_data->data_buf_size < 188) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] broadcast_tcc353x_drv_if_get_dmb_data[ERR] buffsize < 188\n");
		return ERROR;
	}

	pdmb_data->copied_size = (unsigned int) Tcc353xGetStreamBuffer(0, (unsigned char*)((unsigned long)pdmb_data->data_buf_addr), pdmb_data->data_buf_size);
	pdmb_data->packet_cnt = pdmb_data->copied_size / 188;
#if defined (_DBG_LOG_READ_DMB_DATA_)
	TcpalPrintErr((I08S *)"[dtv][tcc3536][debug_info](2) dmb data_buf=0x%x, data_buf_size=%u, copied_size=%u, packet_cn=%u\n",
		(unsigned int)(pdmb_data->data_buf_addr), pdmb_data->data_buf_size, pdmb_data->copied_size, pdmb_data->packet_cnt);
#endif
#if defined(_CHECK_TS_ERROR_)
{
	int i,j;
	int idx=-1;
	int idx_z=-1;

	for(i=0; i<pdmb_data->packet_cnt; i++) {
		unsigned int pid;
		unsigned int data0, data1,data2,data3;
		data0 = *((unsigned char*)((unsigned long)pdmb_data->data_buf_addr)+(i*188));
		data1 = *((unsigned char*)((unsigned long)pdmb_data->data_buf_addr)+(i*188)+1);
		data2 = *((unsigned char*)((unsigned long)pdmb_data->data_buf_addr)+(i*188)+2);
		data3 = *((unsigned char*)((unsigned long)pdmb_data->data_buf_addr)+(i*188)+3);

		pid = (((data1<<8) | data2)&0x1FFF);
		if(data0!= 0x47) {
			Tcc353xCheckPids.err_sync_cnt++;
			TcpalPrintErr((I08S *)"[dtv][tcc3536][error][check ts] err_sync_cnt=%d\n", Tcc353xCheckPids.err_sync_cnt);
			continue;
		}

		if(pid==0x1FFF)
			continue;

		if(data1&0x80) {
			Tcc353xCheckPids.err_tei++;
			TcpalPrintErr((I08S *)"[dtv][tcc3536][error][check ts] err_tei=%d\n", Tcc353xCheckPids.err_tei);
		}

		for(j=0; j<_MAX_CHECK_PIDS_; j++) {
			if(Tcc353xCheckPids.Pids[j]==pid) {
				unsigned char next;
				idx = j;
				next =((Tcc353xCheckPids.curr_continutity_cnt[idx]+1)%0x10);  
				if(next != (data3&0x0F)) {
					TcpalPrintErr((I08S *)"[dtv][tcc3536][error][check ts] continuity pid=%d, curr=%d, next=%d\n", 
						pid, Tcc353xCheckPids.curr_continutity_cnt[idx], next);	
				}
				Tcc353xCheckPids.curr_continutity_cnt[idx] = next;
				break;
			}

			if(Tcc353xCheckPids.Pids[j]==0x1FFF && idx_z==-1) {
				idx_z = j;
			}
		}

		if(idx==-1 && idx_z!=-1)
			Tcc353xCheckPids.Pids[idx_z] = pid;
		}
}
#endif

	return OK;
}

int	broadcast_tcc353x_drv_if_reset_ch(void)
{
	int ret = 0;
	int rc = ERROR;

	TcpalSemaphoreLock(&Tcc353xDrvSem);

	if(TCC3536_OnAir == 0) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] broadcast_tcc353x_drv_if_reset_ch error [!TCC3536_OnAir]\n");
		TcpalSemaphoreUnLock(&Tcc353xDrvSem);
		return ERROR;
	}

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] broadcast_tcc353x_drv_if_reset_ch\n");
	ret = Tcc353xApiStreamStop(0);

	if (ret!=TCC353X_RETURN_SUCCESS)
		rc = ERROR;
	else
		rc = OK;

	TcpalSemaphoreUnLock(&Tcc353xDrvSem);
	return rc;
}

int	broadcast_tcc353x_drv_if_user_stop(int mode)
{
	int rc;

	rc = OK;
	if(TCC3536_OnAir == 0) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error][no semaphore] broadcast_tcc353x_drv_if_user_stop error [!TCC3536_OnAir]\n");
		return ERROR;
	}

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][no semaphore] broadcast_tcc353x_drv_if_user_stop\n");
	Tcc353xApiUserLoopStopCmd(0);
	return rc;
}

int	broadcast_tcc353x_drv_if_select_antenna(unsigned int sel)
{
	int rc;

	rc = OK;
	if(TCC3536_OnAir == 0) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error][no semaphore] broadcast_tcc353x_drv_if_select_antenna error [!TCC3536_OnAir]\n");
		return ERROR;
	}
	return rc;
}

int	broadcast_tcc353x_drv_if_isr(void)
{
	int rc;

	rc = OK;
	if(TCC3536_OnAir == 0) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] broadcast_tcc353x_drv_if_isr error [!TCC3536_OnAir]\n");
		return ERROR;
	}
	return rc;
}

int	broadcast_tcc353x_drv_if_read_control(char *buf, unsigned int size)
{
	int ret = 0;

	if(TCC3536_OnAir == 0 || buf == NULL)
		return 0;

	ret = (int)(Tcc353xGetStreamBuffer(0, buf, size));
	return ret;
}

int	broadcast_tcc353x_drv_if_get_mode (unsigned short *mode)
{
	int rc = ERROR;

	if(mode == NULL) {
		return ERROR;
	}

	if(TCC3536_OnAir == 0 || currentSelectedChannel== -1) {
		mode[0] = 0xFFFF;
	} else {
		unsigned short channel_num;
		unsigned short band = 0;
		unsigned short rcvSegment = 0;

		channel_num = currentSelectedChannel;

		if(currentBroadCast == UHF_1SEG) {
			band = 0;
			rcvSegment = 1;
		} else if (currentBroadCast == UHF_13SEG) {
			band = 0;
			rcvSegment = 0;
		} else if(currentBroadCast == TMM_1SEG) {
			band = 1;
			rcvSegment = 1;
		} else {
			band = 1;
			rcvSegment = 0;
		}

		mode[0] = ((channel_num&0xFF)<<8) | 
				((band&0x0F)<<4) | 
				(rcvSegment&0x0F);
	}

	rc = OK;
	return rc;
}

/* optional part when we include driver code to build-on
it's just used when we make device driver to module(.ko)
so it doesn't work in build-on */
MODULE_DESCRIPTION("TCC3536 ISDB-T device driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("TCC");
