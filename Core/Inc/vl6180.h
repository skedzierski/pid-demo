#ifndef VL6180_API_H_
#define VL6180_API_H_

#include "stdint.h"

/** After power up or reset this register will start reading 1 when device is ready */
#define SYSTEM_FRESH_OUT_OF_RESET             0x016

/** default value ECE factor Molecular */
#define DEF_ECE_FACTOR_M    85
/** default value ECE factor Denominator */
#define DEF_ECE_FACTOR_D    100

#define VL6180DevDataGet(dev, field) (dev->Data.field)
#define VL6180DevDataSet(dev, field, data) dev->Data.field = (data)

/**
 * @brief Minimum range value in mm to qualify for crosstalk compensation
 */
#define SYSRANGE_CROSSTALK_VALID_HEIGHT       0x021
#define SYSRANGE_EARLY_CONVERGENCE_ESTIMATE   0x022
#define SYSRANGE_PART_TO_PART_RANGE_OFFSET    0x024

#define SYSRANGE_RANGE_IGNORE_THRESHOLD       0x026
#define SYSRANGE_CROSSTALK_COMPENSATION_RATE  0x01E

#define SYSTEM_INTERRUPT_CONFIG_GPIO           0x014

#define CONFIG_GPIO_RANGE_SHIFT            0
/** RANGE bits mask in #SYSTEM_INTERRUPT_CONFIG_GPIO  (unshifted)*/
#define CONFIG_GPIO_RANGE_MASK             (0x7<<CONFIG_GPIO_RANGE_SHIFT)
/** interrupt is disabled */
#define CONFIG_GPIO_INTERRUPT_DISABLED         0x00
/** trigger when value < low threshold */
#define CONFIG_GPIO_INTERRUPT_LEVEL_LOW        0x01
/** trigger when value < low threshold */
#define CONFIG_GPIO_INTERRUPT_LEVEL_HIGH       0x02
/** trigger when outside range defined by high low threshold */
#define CONFIG_GPIO_INTERRUPT_OUT_OF_WINDOW    0x03
/** trigger when new sample are ready */
#define CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY 0x04

#define INTERRUPT_CLEAR_RANGING                0x01
/** clear error interrupt in write to #SYSTEM_INTERRUPT_CLEAR */
#define INTERRUPT_CLEAR_ERROR                  0x04

#define SYSRANGE_INTERMEASUREMENT_PERIOD      0x01B

#define SYSRANGE_START                        0x018
    /** mask existing bit in #SYSRANGE_START*/
    #define SYSRANGE_START_MODE_MASK          0x03
    /** bit 0 in #SYSRANGE_START write 1 toggle state in continuous mode and arm next shot in single shot mode */
    #define MODE_START_STOP                   0x01
    /** bit 1 write 1 in #SYSRANGE_START set continuous operation mode */
    #define MODE_CONTINUOUS                   0x02
    /** bit 1 write 0 in #SYSRANGE_START set single shot mode */
    #define MODE_SINGLESHOT                   0x00

#define SYSRANGE_RANGE_CHECK_ENABLES          0x02D
    #define RANGE_CHECK_ECE_ENABLE_MASK      0x01
    #define RANGE_CHECK_RANGE_ENABLE_MASK    0x02
    #define RANGE_CHECK_SNR_ENABLE           0x10

#define SYSRANGE_MAX_CONVERGENCE_TIME         0x01C
#define SYSRANGE_MAX_AMBIENT_LEVEL_MULT       0x02C


/** Error and warning code returned by API
 *
 * negative value are true error mostly fatal\n
 * positive value  are warning most of time it's ok to continue\n
 */
enum VL6180_ErrCode_t {
	API_NO_ERROR        = 0,
	CALIBRATION_WARNING = 1,  /*!< warning invalid calibration data may be in used \a  VL6180_InitData() \a VL6180_GetOffsetCalibrationData \a VL6180_SetOffsetCalibrationData*/
	MIN_CLIPED          = 2,  /*!< warning parameter passed was clipped to min before to be applied */
	NOT_GUARANTEED      = 3,  /*!< Correct operation is not guaranteed typically using extended ranging on vl6180 */

	API_ERROR      = -1,    /*!< Unqualified error */
	INVALID_PARAMS = -2,    /*!< parameter passed is invalid or out of range */
	NOT_SUPPORTED  = -3,    /*!< function is not supported in current mode or configuration */
	RANGE_ERROR    = -4,    /*!< device report a ranging error interrupt status */
	TIME_OUT       = -5,    /*!< aborted due to time out */
};

typedef int32_t DMaxFix_t;
struct DMaxData_t {
	uint32_t ambTuningWindowFactor_K; /*!<  internal algo tuning (*1000) */

	DMaxFix_t retSignalAt400mm;  /*!< intermediate dmax computation value caching @a #SYSRANGE_CROSSTALK_COMPENSATION_RATE and private reg 0x02A */
    /* int32_t RegB8; */             /*!< register 0xB8 cached to speed reduce i2c traffic for dmax computation */
    /* place all word data below to optimize struct packing */
    /* int32_t minSignalNeeded; */    /*!< optimized computation intermediate base on register cached value */
	int32_t snrLimit_K;         /*!< cached and optimized computation intermediate from  @a #SYSRANGE_MAX_AMBIENT_LEVEL_MULT */
	uint16_t ClipSnrLimit;      /*!< Max value for snr limit */
    /* place all byte data below to optimize packing */
    /* uint8_t MaxConvTime; */        /*!< cached max convergence time @a #SYSRANGE_MAX_CONVERGENCE_TIME*/
};

struct VL6180DevData_t {

	uint32_t Part2PartAmbNVM;  /*!< backed up NVM value */
	uint32_t XTalkCompRate_KCps; /*! Cached XTlak Compensation Rate */

	uint16_t EceFactorM;        /*!< Ece Factor M numerator  */
	uint16_t EceFactorD;        /*!< Ece Factor D denominator*/

	uint8_t UpscaleFactor;      /*!<  up-scaling factor*/

    struct DMaxData_t DMaxData;
    uint8_t DMaxEnable;
	int8_t  Part2PartOffsetNVM;     /*!< backed up NVM value */
};

typedef struct {
	int32_t range_mm;          /*!< range distance in mm. */
	int32_t signalRate_mcps;   /*!< signal rate (MCPS)\n these is a 9.7 fix point value, which is effectively a measure of target reflectance.*/
	uint32_t errorStatus;      /*!< Error status of the current measurement. \n see @a ::RangeError_u @a VL6180_GetRangeStatusErrString() */

uint32_t DMax;              /*!< DMax  when applicable */

} VL6180_RangeData_t;

struct MyDev_t {
    struct VL6180DevData_t Data;          /*!< embed ST VL6180 Dev  data as "Data"*/
    int     i2c_bus_num;                   /*!< i2c bus number user specific field */
    int     i2c_dev_addr;                  /*!< i2c devcie address user specific field */                /*!< mutex user specific field */
    int     i2c_file;                      /*!< sample i2c file handle */
};
typedef struct MyDev_t *VL6180Dev_t;

int VL6180_WaitDeviceBooted(VL6180Dev_t dev);

int VL6180_InitData(VL6180Dev_t dev);

int VL6180_SetupGPIO1(VL6180Dev_t dev, uint8_t IntFunction, int ActiveHigh);

int VL6180_Prepare(VL6180Dev_t dev);

int VL6180_RangeStartContinuousMode(VL6180Dev_t dev);

int VL6180_RangeStartSingleShot(VL6180Dev_t dev);

int VL6180_RangeSetMaxConvergenceTime(VL6180Dev_t dev, uint8_t  MaxConTime_msec);

int VL6180_RangePollMeasurement(VL6180Dev_t dev, VL6180_RangeData_t *pRangeData);

int VL6180_RangeGetMeasurementIfReady(VL6180Dev_t dev, VL6180_RangeData_t *pRangeData);

int VL6180_RangeGetMeasurement(VL6180Dev_t dev, VL6180_RangeData_t *pRangeData);

int VL6180_RangeGetResult(VL6180Dev_t dev, int32_t *pRange_mm);

int VL6180_RangeConfigInterrupt(VL6180Dev_t dev, uint8_t ConfigGpioInt);

#define VL6180_RangeClearInterrupt(dev) VL6180_ClearInterrupt(dev, INTERRUPT_CLEAR_RANGING)

int VL6180_RangeGetInterruptStatus(VL6180Dev_t dev, uint8_t *pIntStatus);



extern const char *VL6180_RangeStatusErrString[];

const char *VL6180_RangeGetStatusErrString(uint8_t RangeErrCode);

int VL6180_StaticInit(VL6180Dev_t dev);

int VL6180_RangeWaitDeviceReady(VL6180Dev_t dev, int MaxLoop);

int VL6180_RangeSetInterMeasPeriod(VL6180Dev_t dev, uint32_t  InterMeasTime_msec);

int VL6180_UpscaleSetScaling(VL6180Dev_t dev, uint8_t scaling);

int VL6180_UpscaleGetScaling(VL6180Dev_t dev);

#define VL6180_RangeIsFilteredMeasurement(pRangeData) ((pRangeData)->errorStatus == RangingFiltered)

uint16_t VL6180_GetUpperLimit(VL6180Dev_t dev);

int VL6180_RangeSetThresholds(VL6180Dev_t dev, uint16_t low, uint16_t high, int SafeHold);

int VL6180_RangeGetThresholds(VL6180Dev_t dev, uint16_t *low, uint16_t *high);

int VL6180_RangeSetRawThresholds(VL6180Dev_t dev, uint8_t low, uint8_t high);

int VL6180_RangeSetEceFactor(VL6180Dev_t dev, uint16_t  FactorM, uint16_t FactorD);

int VL6180_RangeSetEceState(VL6180Dev_t dev, int enable);

int VL6180_FilterSetState(VL6180Dev_t dev, int state);

int VL6180_FilterGetState(VL6180Dev_t dev);

int VL6180_DMaxSetState(VL6180Dev_t dev, int state);

int VL6180_DMaxGetState(VL6180Dev_t dev);

int VL6180_RangeSetSystemMode(VL6180Dev_t dev, uint8_t mode);

int VL6180_RangeIgnoreSetEnable(VL6180Dev_t dev, int EnableState);

int VL6180_RangeIgnoreConfigure(VL6180Dev_t dev, uint16_t ValidHeight_mm, uint16_t IgnoreThreshold);

int8_t VL6180_GetOffsetCalibrationData(VL6180Dev_t dev);

int  VL6180_SetOffsetCalibrationData(VL6180Dev_t dev, int8_t offset);

/** use where fix point 9.7 bit values are expected
 *
 * given a floating point value f it's .7 bit point is (int)(f*(1<<7))*/
typedef uint16_t FixPoint97_t;
int  VL6180_SetXTalkCompensationRate(VL6180Dev_t dev, FixPoint97_t Rate);

int VL6180_SetGroupParamHold(VL6180Dev_t dev, int Hold);

int VL6180_SetI2CAddress(VL6180Dev_t dev, uint8_t NewAddr);

int VL6180_SetupGPIOx(VL6180Dev_t dev, int pin, uint8_t IntFunction, int ActiveHigh);

int VL6180_SetGPIOxPolarity(VL6180Dev_t dev, int pin, int active_high);

int VL6180_SetGPIOxFunctionality(VL6180Dev_t dev, int pin, uint8_t functionality);

int VL6180_DisableGPIOxOut(VL6180Dev_t dev, int pin);


#define msec_2_i2cloop(time_ms, i2c_khz) (((time_ms) * (i2c_khz) / 49) + 1)

typedef enum {
	INTR_POL_LOW = 0, /*!< set active low polarity best setup for falling edge */
	INTR_POL_HIGH = 1, /*!< set active high polarity best setup for rising edge */
} IntrPol_e;

int VL6180_GetInterruptStatus(VL6180Dev_t dev, uint8_t *status);

int VL6180_ClearInterrupt(VL6180Dev_t dev, uint8_t IntClear);

#define VL6180_ClearErrorInterrupt(dev) VL6180_ClearInterrupt(dev, INTERRUPT_CLEAR_ERROR)

#define VL6180_ClearAllInterrupt(dev) VL6180_ClearInterrupt(dev, INTERRUPT_CLEAR_ERROR|INTERRUPT_CLEAR_RANGING)

int VL6180_WrByte(VL6180Dev_t dev, uint16_t index, uint8_t data);


int VL6180_UpdateByte(VL6180Dev_t dev, uint16_t index, uint8_t AndData, uint8_t OrData);

int VL6180_WrWord(VL6180Dev_t dev, uint16_t index, uint16_t data);

int VL6180_WrDWord(VL6180Dev_t dev, uint16_t index, uint32_t data);

int VL6180_RdByte(VL6180Dev_t dev, uint16_t index, uint8_t *data);

int VL6180_RdWord(VL6180Dev_t dev, uint16_t index, uint16_t *data);

int VL6180_RdDWord(VL6180Dev_t dev, uint16_t index, uint32_t *data);

int VL6180_RdMulti(VL6180Dev_t dev, uint16_t index, uint8_t *data, int nData);

#endif /* VL6180_API_H_ */
