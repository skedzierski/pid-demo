#include "vl6180.h"
#include "stdint.h"

int VL6180_WaitDeviceBooted(VL6180Dev_t dev)
{
    uint8_t FreshOutReset;
	int status;
	do {
		status = VL6180_RdByte(dev, SYSTEM_FRESH_OUT_OF_RESET, &FreshOutReset);
	} while (FreshOutReset != 1 && status == 0);
	return status;
}

#define Fix7_2_KCPs(x) ((((uint32_t)(x))*1000)>>7)
#define _DMaxData(field) VL6180DevDataGet(dev, DMaxData.field)

/*
 * 32 bit integer square root with not so bad precision (integer result) and is quite fast
 * see http://en.wikipedia.org/wiki/Methods_of_computing_square_roots
 */
uint32_t VL6180_SqrtUint32(uint32_t num)
{
	uint32_t res = 0;
	uint32_t bit = 1 << 30; /* The second-to-top bit is set: 1 << 30 for 32 bits */

	/* "bit" starts at the highest power of four <= the argument. */
	while (bit > num)
		bit >>= 2;

	while (bit != 0) {
		if (num >= res + bit) {
		    num -= res + bit;
		    res = (res >> 1) + bit;
		} else
		    res >>= 1;
		bit >>= 2;
	}
	return res;
}

static uint32_t _DMax_RawValueAtRateKCps(VL6180Dev_t dev, int32_t rate)
{
	uint32_t snrLimit_K;
	int32_t DMaxSq;
	uint32_t RawDMax;
	DMaxFix_t retSignalAt400mm;
	uint32_t ambTuningWindowFactor_K;


	ambTuningWindowFactor_K = _DMaxData(ambTuningWindowFactor_K);
	snrLimit_K              = _DMaxData(snrLimit_K);
	retSignalAt400mm        = _DMaxData(retSignalAt400mm);
	/* 12 to 18 bits Kcps */
	if (rate > 0) {
		DMaxSq = 400 * 400 * 1000 / rate - (400 * 400 / 330);
		/* K of (1/RtnAmb -1/330 )=> 30bit- (12-18)bit  => 12-18 bits*/
		if (DMaxSq <= 0) {
		    RawDMax = 0;
		} else {
		    /* value can be more 32 bit so base on raneg apply
			 * retSignalAt400mm before or after division to presevr accuracy */
		    if (DMaxSq < (2 << 12)) {
				DMaxSq = DMaxSq * retSignalAt400mm /
							(snrLimit_K + ambTuningWindowFactor_K);
				/* max 12 + 12 to 18 -10 => 12-26 bit */
		    } else {
				DMaxSq = DMaxSq / (snrLimit_K + ambTuningWindowFactor_K) * retSignalAt400mm;
				/* 12 to 18 -10 + 12 to 18 *=> 12-26 bit */
		    }
		    RawDMax = VL6180_SqrtUint32(DMaxSq);
		}
	} else {
		RawDMax = 0x7FFFFFFF; /* bigest possibmle 32bit signed value */
	}
	return RawDMax;
}

static int _DMax_InitData(VL6180Dev_t dev)
{
	int status, warning;
	uint8_t u8;
	uint16_t u16;
	uint32_t u32;
	uint32_t Reg2A_KCps;
	uint32_t RegB8;
	uint8_t  MaxConvTime;
	uint32_t XTalkCompRate_KCps;
	uint32_t RangeIgnoreThreshold;
	int32_t minSignalNeeded;
	uint8_t SysRangeCheckEn;
	uint8_t snrLimit;
	static const int MaxConvTimeAdjust = -4;

	warning = 0;

	do {
		status = VL6180_RdByte(dev, 0x02A, &u8);
		if (status) {
		    break;
		}

		if (u8 == 0) {
		    warning = CALIBRATION_WARNING;
		    u8 = 40; /* use a default average value */
		}
		Reg2A_KCps = Fix7_2_KCPs(u8); /* convert to KCPs */

		status = VL6180_RdByte(dev, SYSRANGE_RANGE_CHECK_ENABLES, &SysRangeCheckEn);
		if (status) {
		    break;
		}

		status = VL6180_RdByte(dev, SYSRANGE_MAX_CONVERGENCE_TIME, &MaxConvTime);
		if (status) {
			break;
		}

		status = VL6180_RdDWord(dev, 0x0B8, &RegB8);
		if (status) {
		    break;
		}

		status = VL6180_RdByte(dev, SYSRANGE_MAX_AMBIENT_LEVEL_MULT, &snrLimit);
		if (status) {
		    break;
		}
		_DMaxData(snrLimit_K) = (int32_t)16 * 1000 / snrLimit;
		XTalkCompRate_KCps =   VL6180DevDataGet(dev, XTalkCompRate_KCps);

		if (Reg2A_KCps >= XTalkCompRate_KCps) {
		    _DMaxData(retSignalAt400mm) = Reg2A_KCps;
		} else{
		    _DMaxData(retSignalAt400mm) = 0;
			/* Reg2A_K - XTalkCompRate_KCp <0 is invalid */
		}

		/* if xtalk range check is off omit it in snr clipping */
		if (SysRangeCheckEn&RANGE_CHECK_RANGE_ENABLE_MASK) {
		    status = VL6180_RdWord(dev, SYSRANGE_RANGE_IGNORE_THRESHOLD, &u16);
		    if (status) {

				break;
		    }
		    RangeIgnoreThreshold = Fix7_2_KCPs(u16);
		} else{
		    RangeIgnoreThreshold  = 0;
		}

		minSignalNeeded = (RegB8 * 256) / ((int32_t)MaxConvTime + (int32_t)MaxConvTimeAdjust);
		/* KCps 8+8 bit -(1 to 6 bit) => 15-10 bit */
		/* minSignalNeeded = max ( minSignalNeeded,  RangeIgnoreThreshold - XTalkCompRate_KCps) */
		if (minSignalNeeded  <= (int32_t)RangeIgnoreThreshold - (int32_t)XTalkCompRate_KCps)
		    minSignalNeeded  =  RangeIgnoreThreshold - XTalkCompRate_KCps;

		u32 = (minSignalNeeded*(uint32_t)snrLimit) / 16;
		_DMaxData(ClipSnrLimit) = _DMax_RawValueAtRateKCps(dev, u32);
		/* clip to dmax to min signal snr limit rate*/
	} while (0);
	if (!status)
		status = warning;
	return status;
}

int VL6180_InitData(VL6180Dev_t dev)
{
	int status, dmax_status ;
	int8_t offset;
	uint8_t FreshOutReset;
	uint32_t CalValue;
	uint16_t u16;
	uint32_t XTalkCompRate_KCps;

	VL6180DevDataSet(dev, EceFactorM, DEF_ECE_FACTOR_M);
	VL6180DevDataSet(dev, EceFactorD, DEF_ECE_FACTOR_D);

	_DMax_OneTimeInit(dev);
    /* backup offset initial value from nvm these must be done prior any over call that use offset */
    status = VL6180_RdByte(dev, SYSRANGE_PART_TO_PART_RANGE_OFFSET, (uint8_t *)&offset);
    if (status) {
        return -1;
    }
    VL6180DevDataSet(dev, Part2PartOffsetNVM, offset);

    status = VL6180_RdDWord(dev, SYSRANGE_RANGE_IGNORE_THRESHOLD, &CalValue);
    if (status) {
        return -1;
    }
    if ((CalValue&0xFFFF0000) == 0) {
        CalValue = 0x00CE03F8;
    }
    VL6180DevDataSet(dev, Part2PartAmbNVM, CalValue);

    status = VL6180_RdWord(dev, SYSRANGE_CROSSTALK_COMPENSATION_RATE , &u16);
    if (status) {
        return -1;
    }
    XTalkCompRate_KCps = Fix7_2_KCPs(u16);
    VL6180DevDataSet(dev, XTalkCompRate_KCps, XTalkCompRate_KCps);

    dmax_status = _DMax_InitData(dev);
    if (dmax_status < 0) {
        return -1;
    }

    /* Read or wait for fresh out of reset  */
    status = VL6180_RdByte(dev, SYSTEM_FRESH_OUT_OF_RESET, &FreshOutReset);
    if (status) {
        return -1;
    }
    if (FreshOutReset != 1 || dmax_status)
        status = CALIBRATION_WARNING;

	return status;
}

int VL6180_Prepare(VL6180Dev_t dev)
{
	int status;

    status = VL6180_StaticInit(dev);
    if (status < 0)
        return -1;

    /* set range InterruptMode to new sample */
    status = VL6180_RangeConfigInterrupt(dev, CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);
    if (status)
        return -1;

    /* set default threshold */
    status = VL6180_RangeSetRawThresholds(dev, 10, 200);
    if (status) {
        return -1;
    }

    /* make sure to reset any left previous condition that can hangs first poll */
    status = VL6180_ClearAllInterrupt(dev);

	return status;
}

int VL6180_RangeSetInterMeasPeriod(VL6180Dev_t dev, uint32_t  InterMeasTime_msec)
{
	uint8_t SetTime;
	int status;

    if (InterMeasTime_msec > 2550) {
        status = INVALID_PARAMS;
        return status;
    }
    /* doc in not 100% clear and confusing about the limit practically all value are OK but 0
        * that can hang device in continuous mode */
    if (InterMeasTime_msec < 10) {
        InterMeasTime_msec = 10;
    }
    SetTime = (uint8_t)(InterMeasTime_msec / 10);
    status = VL6180_WrByte(dev, SYSRANGE_INTERMEASUREMENT_PERIOD, SetTime);
    if (SetTime != InterMeasTime_msec / 10) {
        status = MIN_CLIPED;
    }  /* on success change status to clip if it did */
	return status;
}

int VL6180_SetupGPIO1(VL6180Dev_t dev, uint8_t IntFunction, int ActiveHigh)
{
	int status;
	status = VL6180_SetupGPIOx(dev, 1, IntFunction, ActiveHigh);
	return status;
}

int VL6180_RangeConfigInterrupt(VL6180Dev_t dev, uint8_t ConfigGpioInt)
{
	int status;

	if (ConfigGpioInt <= CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY) {
		status = VL6180_UpdateByte(dev, SYSTEM_INTERRUPT_CONFIG_GPIO,
									(uint8_t)(~CONFIG_GPIO_RANGE_MASK),
									ConfigGpioInt);
	} else {
		status = INVALID_PARAMS;
	}
	return status;
}

int VL6180_RangeStartContinuousMode(VL6180Dev_t dev)
{
	int status;
	status = VL6180_RangeSetSystemMode(dev, MODE_START_STOP | MODE_CONTINUOUS);
	return status;
}