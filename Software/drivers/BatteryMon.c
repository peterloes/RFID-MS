/***************************************************************************//**
 * @file
 * @brief	Battery Monitoring
 * @author	Ralf Gerhauser
 * @version	2020-06-18
 *
 * This module periodically reads status information from the battery pack
 * via its SMBus interface.  It also provides routines to access the registers
 * of the battery controller manually.
 *
 * @warning
 * The firmware on the battery controller (ATmega32HVB) is quite buggy!
 * When accessing a non-implemented register (e.g. 0x1D), the correct
 * response of the controller should be a Not-Acknowledge (NAK).  Instead
 * it enters clock-stretching mode (SCL is permanently driven low) and
 * also seems to hang-up internally (the LEDs reporting the capacity of the
 * battery are no more flashing).  The bus stalls with SCL low and SDA high
 * at this stage.  The EFM32 I2C-controller waits for SCL returning to high,
 * but this never happens.  Sometimes the bus is released after about 4 seconds,
 * but not always. The only work-around found for this situation is to pull
 * SDA low for approximately 3 seconds.  This is detected by the firmware and
 * SCL is released again.
 *
 ****************************************************************************//*
Revision History:
2020-06-18,rage LogBatteryInfo: Removed SBS_ManufacturerData.
		Disabled workaround for probing prototype battery packs.
2020-01-22,rage	Added support for battery controller TI bq40z50.
2018-03-25,rage	Set interrupt priority for SMB_IRQn.
		Added BatteryInfoReq() and BatteryInfoGet().
		BatteryCheck() also handles read requests from BatteryInfoReq().
		LogBatteryInfo() calls drvLEUART_sync() to prevent overflows.
2016-04-05,rage	Made global variables of type "volatile".
2016-02-26,rage	Implemented alarms to read battery status 2 times per day.
		BugFix: <SMB_Status> must be declared volatile.
		Removed unused variable <g_BattRunDays>.
		Do not install interval timer if BAT_MON_INTERVAL is 0.
		Changed LogBatteryInfo() to use BAT_LOG_INFO_LVL.
2016-02-14,rage	Show error message if no battery controller is connected.
		Display "More than 6 weeks" if runtime is above 65534min.
		Report actual current together with voltage.
2014-04-13,rage	Implemented error recovery mechanism via SMB_Reset().
2014-12-20,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include <string.h>
#include "em_cmu.h"
#include "em_i2c.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "AlarmClock.h"		// msDelay()
#include "LEUART.h"
#include "PowerFail.h"
#include "BatteryMon.h"
#include "Display.h"
#include "Logging.h"

/*=============================== Definitions ================================*/

    /*!@name Hardware Configuration: SMBus controller and pins. */
//@{
#define SMB_GPIOPORT		gpioPortA	//!< Port SMBus interface
#define SMB_SDA_PIN		0		//!< Pin PA0 for SDA signal
#define SMB_SCL_PIN		1		//!< Pin PA1 for SCL signal
#define SMB_I2C_CTRL		I2C0		//!< I2C controller to use
#define SMB_I2C_CMUCLOCK	cmuClock_I2C0	//!< Enable clock for I2C
#define SMB_LOC		I2C_ROUTE_LOCATION_LOC0 //!< Use location 0
#define SMB_IRQn		I2C0_IRQn	//!< I2C controller interrupt
#define SMB_IRQHandler		I2C0_IRQHandler	//!< SMBus interrupt handler
//@}

    /*!@brief I2C Transfer Timeout (500ms) in RTC ticks */
#define I2C_XFER_TIMEOUT	(RTC_COUNTS_PER_SEC / 2)

    /*!@brief I2C Recovery Timeout (5s) in RTC ticks */
#define I2C_RECOVERY_TIMEOUT	(RTC_COUNTS_PER_SEC * 5)

    /*!@brief Structure to hold Information about a Battery Controller */
typedef struct
{
    uint8_t	 addr;		//!< SMBus address of the battery controller
    BC_TYPE	 type;		//!< Corresponding controller type
    const char	*name;		//!< ASCII name of the controller
} BC_INFO;

    /*!@brief Format identifiers.
     *
     * These enumerations specify various data formats.  They are used as an
     * element in structure @ref ITEM to specify the data representation of
     * an item.  They are handled by a switch() statement in ItemDataString().
     */
typedef enum
{
    FRMT_STRING,	//!<  0: 0-terminated string
    FRMT_HEX,		//!<  1: H1,2,3,4 Hexadecimal data representation
    FRMT_HEXDUMP,	//!<  2: Show Hexdump for more than 4 bytes
    FRMT_INTEGER,	//!<  3: Integer value
    FRMT_SERNUM,	//!<  4: Serial Number
    FRMT_PERCENT,	//!<  5: U1 Amount in percent [%]
    FRMT_DURATION,	//!<  6: U2 Duration in [min]
    FRMT_OC_REATIME,	//!<  7: Overcurrent Reaction Time in 1/2[ms] units
    FRMT_HC_REATIME,	//!<  8: Highcurrent Reaction Time in 2[ms] units
    FRMT_VOLT,		//!<  9: U2 Voltage in [V]
    FRMT_MILLIVOLT,	//!< 10: U2 Voltage in [mV]
    FRMT_MILLIAMP,	//!< 11: I2 Current in [±mA], +:charging, -:discharging
    FRMT_MILLIAMPH,	//!< 12: U2 Capacity in [mAh]
    FRMT_MICROOHM,	//!< 13: Resistance in [uOhm]
    FRMT_DATE,		//!< 14: Date [15:9=Year|8:5=Month|4:0=Day]
    FRMT_TEMP,		//!< 15: U2 Temperature [0.1°K]
    FRMT_TYPE_CNT	//!< Format Type Count
} FRMT_TYPE;

/*================================== Macros ==================================*/

#ifndef LOGGING		// define as UART output, if logging is not enabled
    #define LogError(str)	drvLEUART_puts(str "\n")
#endif

/*========================= Global Data and Routines =========================*/

    /*!@brief I2C Device Address of the Battery Controller */
uint8_t g_BatteryCtrlAddr;

    /*!@brief Battery Controller Type */
BC_TYPE g_BatteryCtrlType = BCT_UNKNOWN;

    /*!@brief ASCII Name of the Battery Controller, or "" if no one found */
const char *g_BatteryCtrlName;

    /*!@brief Battery voltage in [mV] */
volatile int16_t   g_BattMilliVolt = (-1);   // show error message per default

    /*!@brief Remaining capacity of the Battery in [mAh] */
volatile uint16_t  g_BattCapacity;

/*================================ Local Data ================================*/

    /*!@brief Probe List of supported Battery Controllers */
static const BC_INFO l_ProbeList[] =
{  //  addr	type		name (maximum 10 characters!)
    {  0x0A,	BCT_ATMEL,	"ATMEL"		},
    {  0x16,	BCT_TI,		"TI bq40z50",	},
    {  0x00,	BCT_UNKNOWN,	""		}	// End of the list
};

    /* Defining the SMBus initialization data */
static I2C_Init_TypeDef smbInit =
{
  .enable   = true,		// Enable controller after initialization
  .master   = true,		// Configure for MASTER mode
  .refFreq  = 0,		// Get clock frequency from clock source
  .freq     = 10000,		// Low frequency because of long SMBus wires
  .clhr     = i2cClockHLRStandard,	// Set to use 4:4 low/high duty cycle
};

    /* Status of the last SMBus transaction */
volatile I2C_TransferReturn_TypeDef SMB_Status;

    /*!@brief Flag to trigger Battery Controller Probing. */
static volatile bool	 l_flgBatteryCtrlProbe = true;

    /*!@brief Flag to trigger battery monitoring measurement. */
static volatile bool	 l_flgBatMonTrigger;

#if BAT_MON_INTERVAL > 0
    /* Timer handle for the battery monitoring interval */
static TIM_HDL	l_thBatMon = NONE;
#endif

    /* Battery Info structure - may hold up to two info requests */
static BAT_INFO  l_BatInfo;

/*=========================== Forward Declarations ===========================*/

#if BAT_MON_INTERVAL > 0
static void	BatMonTrigger(TIM_HDL hdl);
#endif
static void	BatMonTriggerAlarm(int alarmNum);


/***************************************************************************//**
 *
 * @brief	Initialize the battery monitoring module
 *
 * This routine initializes the board-specific SMBus (I2C) interface, which
 * is connected to the battery pack.
 *
 ******************************************************************************/
void	 BatteryMonInit (void)
{
    /* Be sure to enable clock to GPIO (should already be done) */
    CMU_ClockEnable (cmuClock_GPIO, true);

    /* Enable clock for I2C controller */
    CMU_ClockEnable(SMB_I2C_CMUCLOCK, true);

    /* Configure GPIOs for SMBus (I2C) functionality with Pull-Ups */
    GPIO_PinModeSet (SMB_GPIOPORT, SMB_SCL_PIN, gpioModeWiredAndPullUp, 1);
    GPIO_PinModeSet (SMB_GPIOPORT, SMB_SDA_PIN, gpioModeWiredAndPullUp, 1);

    /* Route SMB signals to the respective pins */
    SMB_I2C_CTRL->ROUTE = I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN | SMB_LOC;

    /* Initialize SMBus (I2C) controller */
    I2C_Init (SMB_I2C_CTRL, &smbInit);

    /* Clear and enable SMBus interrupt */
    NVIC_SetPriority(SMB_IRQn, INT_PRIO_SMB);
    NVIC_ClearPendingIRQ (SMB_IRQn);
    NVIC_EnableIRQ (SMB_IRQn);

    /* Get a timer handle for the battery monitoring interval */
#if BAT_MON_INTERVAL > 0
    if (l_thBatMon == NONE)
    {
	l_thBatMon = sTimerCreate (BatMonTrigger);
	if (l_thBatMon != NONE)
	    sTimerStart (l_thBatMon, BAT_MON_INTERVAL);
    }
#endif

    /* Set up alarm times when to log the battery status */
    AlarmAction (ALARM_BATTERY_MON_1, BatMonTriggerAlarm);
    AlarmSet (ALARM_BATTERY_MON_1, ALARM_BAT_MON_TIME_1);
    AlarmEnable (ALARM_BATTERY_MON_1);

    AlarmAction (ALARM_BATTERY_MON_2, BatMonTriggerAlarm);
    AlarmSet (ALARM_BATTERY_MON_2, ALARM_BAT_MON_TIME_2);
    AlarmEnable (ALARM_BATTERY_MON_2);

    /* Initialize Battery Info structure */
    l_BatInfo.Req_1 = l_BatInfo.Req_2 = SBS_NONE;
}


/***************************************************************************//**
 *
 * @brief	De-Initialize the battery monitoring module
 *
 * This routine brings the SMBus (I2C) interface, which is connected to the
 * battery pack, into a quiescent state.
 *
 ******************************************************************************/
void	 BatteryMonDeinit (void)
{
    /* Disable SMBus interrupt */
    NVIC_DisableIRQ (SMB_IRQn);

    /* Reset SMBus controller */
    I2C_Reset (SMB_I2C_CTRL);

    /* Disable clock for I2C controller */
    CMU_ClockEnable (SMB_I2C_CMUCLOCK, false);

    /* Reset variables */
    g_BatteryCtrlAddr = 0x00;
    g_BatteryCtrlName = "";
    g_BatteryCtrlType = BCT_UNKNOWN;
}


/***************************************************************************//**
 *
 * @brief	Probe for Controller Type
 *
 * This routine probes the type of battery controller.  This is done by checking
 * dedicated I2C-bus addresses on the SMBus.  The following addresses and their
 * corresponding controller type are supported:
 * - 0x0A in case of Atmel, and
 * - 0x16 for the TI bq40z50.
 * The address is stored in @ref g_BatteryCtrlAddr, its ASCII name in @ref
 * g_BatteryCtrlName and the controller type is stored as bit definition
 * @ref BC_TYPE in @ref g_BatteryCtrlType.
 *
 ******************************************************************************/
static void BatteryCtrlProbe (void)
{
int	i;
int	status;


    for (i = 0;  l_ProbeList[i].addr != 0x00;  i++)
    {
	g_BatteryCtrlAddr = l_ProbeList[i].addr;	// try this address
	status = BatteryRegReadValue (SBS_ManufacturerAccess, NULL);
	if (status >= 0)
	{
	    /* Response from controller - battery found */
	    break;
	}
	else
	{
	    if (status != i2cTransferNack)
		LogError ("BatteryCtrlProbe: Unexpected error %d", status);
	}
    }

    g_BatteryCtrlAddr = l_ProbeList[i].addr;
    g_BatteryCtrlName = l_ProbeList[i].name;
    g_BatteryCtrlType = l_ProbeList[i].type;

#if 0
    /*
     * WORKAROUND: There may be some Battery Packs with the new TI
     * controller out in the field, that use I2C-bus address 0x0A.  These
     * would be detected as "Atmel" devices, which is wrong.
     * Therefore this workaround probes for register SBS_TurboPower (0x59)
     * which only exists in the TI controller.
     */
    status = BatteryRegReadValue (SBS_TurboPower, NULL);
    if (status >= 0)
    {
	/* Register exists - must be TI controller */
	g_BatteryCtrlName = l_ProbeList[1].name;
	g_BatteryCtrlType = l_ProbeList[1].type;
    }
#endif
}


/***************************************************************************//**
 *
 * @brief	SMBus Interrupt Handler
 *
 * This handler is executed for each byte transferred via the SMBus interface.
 * It calls the driver function I2C_Transfer() to prepare the next data byte,
 * or generate a STOP condition at the end of a transfer.
 *
 ******************************************************************************/
void	 SMB_IRQHandler (void)
{

    /* Update <SMB_Status> */
    SMB_Status = I2C_Transfer (SMB_I2C_CTRL);

}


/***************************************************************************//**
 *
 * @brief	SMBus Reset
 *
 * This internal routine aborts the current I2C-bus transfer and tries to
 * recover from a state where SCL is driven low by the battery controller.
 * It should be called if there occurs a timeout of a transfer.
 *
 ******************************************************************************/
static void  SMB_Reset (void)
{
    LogError("SMB_Reset: Try to recover from invalid state");

    /* abort the current transfer */
    SMB_I2C_CTRL->CMD = I2C_CMD_ABORT;
    msDelay(100);

    /* check if SCL is still low */
    if ((GPIO->P[SMB_GPIOPORT].DIN & (1 << SMB_SCL_PIN)) == 0)
    {
	/* drive SDA low */
	GPIO->P[SMB_GPIOPORT].DOUTCLR = (1 << SMB_SDA_PIN);
	SMB_I2C_CTRL->ROUTE = I2C_ROUTE_SCLPEN | SMB_LOC;

	/* wait until SCL returns to high */
	uint32_t start = RTC->CNT;
	while ((GPIO->P[SMB_GPIOPORT].DIN & (1 << SMB_SCL_PIN)) == 0)
	{
	    /* check for timeout */
	    if (((RTC->CNT - start) & 0x00FFFFFF) > I2C_RECOVERY_TIMEOUT)
	    {
		LogError("SMB_Reset: Recovery failed, giving up");
		break;
	    }
	}

	/* re-configure GPIO as SDA signal */
	SMB_I2C_CTRL->ROUTE = I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN | SMB_LOC;
    }
}


/***************************************************************************//**
 *
 * @brief	Read Word Register from the Battery Controller
 *
 * This routine reads two bytes from the register address specified by @p cmd,
 * assembles them to a signed 16bit value, and returns this.  If an error
 * occurred, a negative status code is returned instead.
 *
 * @param[in] cmd
 *	SBS command, i.e. the register address to be read.
 *
 * @return
 *	Requested 16bit signed data value, or a negative error code of type
 *	@ref I2C_TransferReturn_TypeDef.  Additionally to those codes, there is
 *	another error code defined, named @ref i2cTransferTimeout.
 *
 * @see
 *	BatteryRegReadBlock()
 *
 ******************************************************************************/
int	 BatteryRegReadWord (SBS_CMD cmd)
{
uint32_t value;
int	 status;

    status = BatteryRegReadValue (cmd, &value);

    if (status < 0)
	return status;

    return (int16_t)value;
}


/***************************************************************************//**
 *
 * @brief	Read Register Value from the Battery Controller
 *
 * This routine reads a value from the register address specified by @p cmd.
 * The size of the value is determined from @p cmd, it could be 1, 2, 3, or
 * 4 bytes.
 *
 * @param[in] cmd
 *	SBS command, i.e. the register address to be read.
 *
 * @param[in] pValue
 *	Address of 32bit variable where to store the value read from the
 *	register.  May be set to NULL, if value is omitted.
 *
 * @return
 *	Status code @ref i2cTransferDone (0), or a negative error code of type
 *	@ref I2C_TransferReturn_TypeDef.  Additionally to those codes, there is
 *	another error code defined, named @ref i2cTransferTimeout.
 *
 * @see
 *	BatteryRegReadBlock()
 *
 ******************************************************************************/
int	 BatteryRegReadValue (SBS_CMD cmd, uint32_t *pValue)
{
uint8_t  dataBuf[6];			// buffer for data read from register
uint32_t value = 0;
int	 status;
int	 i;


    /* Call block command to transfer data bytes into buffer */
    status = BatteryRegReadBlock (cmd, dataBuf, sizeof(dataBuf));

    if (status == i2cTransferDone  &&  pValue != NULL)
    {
	/* build value from data buffer (always little endian) */
	for (i = SBS_CMD_SIZE(cmd) - 1;  i >= 0;  i--)
	    value = (value << 8) | dataBuf[i];

	*pValue = value;
    }

    return status;
}


/***************************************************************************//**
 *
 * @brief	Read Data Block from the Battery Controller
 *
 * This routine reads an amount of bytes from the battery controller, as
 * specified by parameter cmd.  This contains the register address and number
 * of bytes to read.
 *
 * @param[in] cmd
 *	SBS command, i.e. the register address and number of bytes to read.
 *
 * @param[out] pBuf
 *	Address of a buffer where to store the data.
 *
 * @param[in] rdCnt
 *	Number of bytes to read.
 *
 * @return
 *	Status code @ref i2cTransferDone (0), or a negative error code of type
 *	@ref I2C_TransferReturn_TypeDef.  Additionally to those codes, there is
 *	another error code defined, named @ref i2cTransferTimeout.
 *
 * @see
 *	BatteryRegReadValue()
 *
 ******************************************************************************/
int	BatteryRegReadBlock (SBS_CMD cmd, uint8_t *pBuf, size_t rdCnt)
{
I2C_TransferSeq_TypeDef smbXfer;	// SMBus transfer data
uint8_t addrBuf[1];			// buffer for device address


    /* Check parameters */
    EFM_ASSERT (SBS_CMD_SIZE(cmd) != 0);// size field must not be 0
    EFM_ASSERT (pBuf != NULL);		// buffer address
    EFM_ASSERT (rdCnt >= SBS_CMD_SIZE(cmd));	// buffer size

    if (rdCnt < SBS_CMD_SIZE(cmd))	// if EFM_ASSERT() is empty
	return i2cInvalidParameter;
    
       /* Check for power-fail */
    if (IsPowerFail())
	return i2cPowerFail;

    /* Set up SMBus transfer S-Wr-Cmd-Sr-Rd-data1-P */
    smbXfer.addr  = g_BatteryCtrlAddr;	// I2C address of the Battery Controller
    smbXfer.flags = I2C_FLAG_WRITE_READ; // write address, then read data
    smbXfer.buf[0].data = addrBuf;	// first buffer (data to write)
    addrBuf[0] = cmd;			// register address (strip higher bits)
    smbXfer.buf[0].len  = 1;		// 1 byte for command
    smbXfer.buf[1].data = pBuf;		// second buffer to store bytes read
    smbXfer.buf[1].len  = rdCnt;	// number of bytes to read

    /* Start I2C Transfer */
    SMB_Status = I2C_TransferInit (SMB_I2C_CTRL, &smbXfer);

    /* Check early status */
    if (SMB_Status < 0)
	return SMB_Status;		// return error code

    /* Wait until data is complete or time out */
    uint32_t start = RTC->CNT;
    while (SMB_Status == i2cTransferInProgress)
    {
	/* Enter EM1 while waiting for I2C interrupt */
	EMU_EnterEM1();

	/* check for timeout */
	if (((RTC->CNT - start) & 0x00FFFFFF) > I2C_XFER_TIMEOUT)
	{
	    SMB_Reset();
	    SMB_Status = (I2C_TransferReturn_TypeDef)i2cTransferTimeout;
	}
    }

    /* Return final status */
    return SMB_Status;
}


/***************************************************************************//**
 *
 * @brief	Item Data String
 *
 * This routine returns a formatted data string of the specified item data.
 * It uses BatteryRegReadValue() and BatteryRegReadBlock() to read the data
 * directly from the battery controller.
 *
 * @param[in] cmd
 *	SBS command, i.e. the register address and number of bytes to read.
 *
 * @param[in] frmt
 *	Format specifier for data representation.
 *
 * @return
 * 	Static buffer that contains the formatted data string of the item,
 * 	or error message if there was an error, e.g. a read error from the
 * 	battery controller.
 *
 * @warning
 *	This routine is not MT-save (which should not be a problem for this
 *	application)!
 *
 ******************************************************************************/
static const char *ItemDataString (SBS_CMD cmd, FRMT_TYPE frmt)
{
static char	 strBuf[120];	// static buffer to return string into
uint8_t		 dataBuf[40];	// buffer for I2C data, read from the controller
uint32_t	 value;		// unsigned data variable
int		 data = 0;	// generic signed integer data variable
int		 d, h, m;	// FRMT_DURATION: days, hours, minutes


    /* Prepare check for string buffer overflow */
    strBuf[sizeof(strBuf)-1] = 0x11;

    if (cmd != SBS_NONE)
    {
	/* See how many bytes we need to read */
	data = SBS_CMD_SIZE(cmd);	// get object size
	if (data > 4)
	{
	    /* More than 32 bits - must be a block, e.g. a string */
	    EFM_ASSERT(data < (int)sizeof(dataBuf));

	    if (BatteryRegReadBlock (cmd, dataBuf, data) < 0)
		return "READ ERROR";	// READ ERROR
	}
	else
	{
	    /* Read data word - may be 1, 2, 3, or 4 bytes long */
	    if (BatteryRegReadValue (cmd, &value) < 0)
		return "READ ERROR";	// READ ERROR

	    data = (int)value;
	}
    }

    /* Variable <data> contains 16bit raw value, build formatted string */
    switch (frmt)
    {
	case FRMT_STRING:	// return string to be displayed
	    /*
	     * The first byte contains the number of ASCII characters WITHOUT
	     * a trailing 0 as EndOfString marker.  However, in some cases the
	     * specified byte count is larger than the string - then an EOS
	     * marker exists in the data read from the controller.
	     */
	    data = dataBuf[0];
	    EFM_ASSERT(data < (int)(sizeof(strBuf)-1));
	    strncpy (strBuf, (char *)dataBuf+1, data);
	    strBuf[data] = EOS;		// terminate string
	    break;

	case FRMT_HEXDUMP:	// prepare data as hexdump
	    data = dataBuf[0];
	    for (d = 0;  d < data;  d++)	// data = number of bytes
		sprintf (strBuf + 3*d, "%02X ", dataBuf[d+1]);
	    strBuf[3*d - 1] = EOS;
	    break;

	case FRMT_HEX:		// HEX Digits (8, 16, 24, or 32bit)
	    switch (SBS_CMD_SIZE(cmd))
	    {
		case 1:
		    sprintf (strBuf, "0x%02X", data);
		    break;

		case 2:
		    sprintf (strBuf, "0x%04X", data);
		    break;

		case 3:
		    sprintf (strBuf, "0x%06X", data);
		    break;

		case 4:
		default:
		    sprintf (strBuf, "0x%08lX", value);
		    break;
	    }
	    break;

	case FRMT_INTEGER:	// Integer Value
	    sprintf (strBuf, "%d", data);
	    break;

	case FRMT_SERNUM:	// 5-Digit Integer Value
	    sprintf (strBuf, "%0d", data);
	    break;

	case FRMT_PERCENT:	// Amount in percent
	    sprintf (strBuf, "%d%%", data);
	    break;

	case FRMT_DURATION:	// Duration in [min]
	    if (data > 65534)		// > 45d
	    {
		strcpy (strBuf, "> 45 days");
	    }
	    else
	    {
		d = data / 60 / 24;
		data -= (d * 60 * 24);
		h = data / 60;
		data -= (h * 60);
		m = data;
		sprintf (strBuf, "%2dd %2dh %2dm", d, h, m);
	    }
	    break;

	case FRMT_OC_REATIME:	// Overcurrent Reaction Time in 1/2[ms] units
	    sprintf (strBuf, "%dms", data/2);
	    break;

	case FRMT_HC_REATIME:	// Highcurrent Reaction Time in 2[ms] units
	    sprintf (strBuf, "%dms", data*2);
	    break;

	case FRMT_VOLT:		// Voltage in [V]
	    sprintf (strBuf, "%d.%03dV", data / 1000, data % 1000);
	    break;

	case FRMT_MILLIVOLT:	// Voltage in [mV]
	    sprintf (strBuf, "%dmV", data);
	    break;

	case FRMT_MILLIAMP:	// Current in [±mA], +:charging, -:discharging
	    sprintf (strBuf, "%dmA", (int16_t)data);
	    break;

	case FRMT_MILLIAMPH:	// Capacity in [mAh]
	    sprintf (strBuf, "%dmAh", data);
	    break;

	case FRMT_MICROOHM:	// Resistance in [uOhm]
	    sprintf (strBuf, "%duOhm", data);
	    break;

	case FRMT_DATE:		// Date [15:9=Year|8:5=Month|4:0=Day]
	    sprintf (strBuf, "%04d-%02d-%02d", 1980 + (data >> 9),
		     (data >> 5) & 0xF, data & 0x1F);
	    break;

	case FRMT_TEMP:		// Temperature in 1/10[K], convert to [°C]
	    {
	    data -= 2732;	// subtract base of 273.16K
	    int degC = data / 10;
	    if (data < 0)
		data = -data;
	    sprintf (strBuf, "%d.%d C", degC, data % 10);
	    }
	    break;

	default:		// unsupported format
	    return "ERROR Unsupported Format";

    }	// switch (pItem->Frmt)

    /* Perform check for string buffer overflow */
    if (strBuf[sizeof(strBuf)-1] != 0x11)
    {
	sprintf (strBuf, "ERROR strBuf Overflow, cmd=%d frmt=%d", cmd, frmt);
    }

    return strBuf;
}


/***************************************************************************//**
 *
 * @brief	Log Battery Information
 *
 * Depending on the @ref BAT_LOG_INFO_LVL, this routine reads miscellaneous
 * information from the battery controller via the SMBus and logs it:
 * - Manufacturer Name
 * - Manufacturer Data
 * - Device Name
 * - Device Type
 * - Serial Number
 * - Design Voltage and Capacity
 * - Actual Voltage and remaining Capacity
 * - Remaining Run Time
 *
 * @param[in] infoLvl
 *	Enum of type @ref BAT_LOG_INFO_LVL which specifies the level of
 *	information.
 *
 ******************************************************************************/
void	LogBatteryInfo (BAT_LOG_INFO_LVL infoLvl)
{
#ifdef LOGGING
uint32_t value;		// unsigned data variable

    /* Check if the Battery Controller Probe routine should be called (again) */
    if (l_flgBatteryCtrlProbe)
    {
	l_flgBatteryCtrlProbe = false;
	BatteryCtrlProbe();
    }

    /* Try to read a register from the battery controller */
    if (BatteryRegReadValue (SBS_Voltage, NULL) < 0)
    {
	/* ERROR */
	g_BattMilliVolt = (-1);
	LogError ("Battery Controller Read Error");
	return;
    }

    /* No get and log all the information */
    if (infoLvl == BAT_LOG_INFO_VERBOSE)
    {
	Log ("Battery Controller Type is \"%s\" at address 0x%02X",
	     g_BatteryCtrlName, g_BatteryCtrlAddr);

	Log ("Battery Manufacturer Name : %s",
	     ItemDataString(SBS_ManufacturerName, FRMT_STRING));

	Log ("Battery Device Name       : %s",
	     ItemDataString(SBS_DeviceName, FRMT_STRING));

	Log ("Battery Device Type       : %s",
	     ItemDataString(SBS_DeviceChemistry, FRMT_STRING));

	Log ("Battery Serial Number     : %s",	// display as Hex value now
	     ItemDataString(SBS_SerialNumber, FRMT_HEX));

	Log ("Battery Design Voltage    : %s%s",
	     ItemDataString(SBS_DesignVoltage, FRMT_VOLT),
	     g_BatteryCtrlType==BCT_ATMEL ? " per cell" : "");

	Log ("Battery Design Capacity   : %s",
	     ItemDataString(SBS_DesignCapacity, FRMT_MILLIAMPH));

	Log ("Battery Full Charge Capac.: %s",
	     ItemDataString(SBS_FullChargeCapacity, FRMT_MILLIAMPH));
    }

    drvLEUART_sync();	// to prevent UART buffer overflow

    if (BatteryRegReadValue(SBS_RemainingCapacity, &value) >= 0)
    {
	g_BattCapacity = (uint16_t)value;
	if (infoLvl != BAT_LOG_INFO_DISPLAY_ONLY)
	    Log ("Battery Remaining Capacity: %ldmAh", value);
    }

    if (infoLvl != BAT_LOG_INFO_DISPLAY_ONLY)
    {
	Log ("Battery Runtime to empty  : %s",
	     ItemDataString(SBS_RunTimeToEmpty, FRMT_DURATION));
    }

    if (BatteryRegReadValue(SBS_Voltage, &value) >= 0)
    {
	g_BattMilliVolt = (int16_t)value;
	if (infoLvl != BAT_LOG_INFO_DISPLAY_ONLY)
	    Log ("Battery Actual Voltage    : %2ld.%ldV",
		 value / 1000, (value % 1000) / 100);
        
        DisplayUpdateTrigger (LCD_BATTERY);
    }

    if (infoLvl != BAT_LOG_INFO_DISPLAY_ONLY)
    {
	Log ("Battery Actual Current    : %s",
	     ItemDataString(SBS_BatteryCurrent, FRMT_MILLIAMP));
    }

    drvLEUART_sync();	// to prevent UART buffer overflow
#endif
}


/***************************************************************************//**
 *
 * @brief	Battery Check
 *
 * This routine is called periodically to check the status of the battery.
 * It reads the voltage, the remaining capacity, and the remaining run time
 * from the battery controller via the SMBus.<br>
 * It also handles the requests for BatteryInfoReq(), resp. BatteryInfoGet().
 *
 ******************************************************************************/
void	BatteryCheck (void)
{
bool	flgBatteryCtrlProbe = l_flgBatteryCtrlProbe;

    /* Check if the Battery Controller Probe routine should be called (again) */
    if (l_flgBatteryCtrlProbe)
    {
	l_flgBatteryCtrlProbe = false;
	BatteryCtrlProbe();
    }

    /* Check for Battery Information Request */
    if (l_BatInfo.Req_1 != SBS_NONE)
    {
	if (SBS_CMD_SIZE(l_BatInfo.Req_1) > 4)
	    BatteryRegReadBlock (l_BatInfo.Req_1, l_BatInfo.Buffer, 18);
	else
	    l_BatInfo.Data_1 = BatteryRegReadWord (l_BatInfo.Req_1);
	l_BatInfo.Req_1  = SBS_NONE;
    }

    if (l_BatInfo.Req_2 != SBS_NONE)
    {
	msDelay(100);		// to prevent hang-up of battery controller

	if (SBS_CMD_SIZE(l_BatInfo.Req_2) > 4)
	    BatteryRegReadBlock (l_BatInfo.Req_2, l_BatInfo.Buffer, 18);
	else
	    l_BatInfo.Data_2 = BatteryRegReadWord (l_BatInfo.Req_2);
	l_BatInfo.Req_2  = SBS_NONE;
    }

    l_BatInfo.Done = true;

    /* see if to log battery status */
    if (l_flgBatMonTrigger)
    {
	l_flgBatMonTrigger = false;

	/* Log verbose information, if Battery Pack has changed */
	LogBatteryInfo (flgBatteryCtrlProbe ? BAT_LOG_INFO_VERBOSE
					    : BAT_LOG_INFO_SHORT);
    }
}


/***************************************************************************//**
 *
 * @brief	Battery Change Trigger
 *
 * This routine triggers the probing of a Battery Pack on the SMBus and logs
 * the respective information, e.g. the serial number.
 * If applicable, it should be called from PowerFailCheck() after power has
 * returned (i.e. Power Good).
 *
 ******************************************************************************/
void	BatteryChangeTrigger(void)
{
    /* Set trigger flags */
    l_flgBatMonTrigger = true;
    l_flgBatteryCtrlProbe = true;

    g_flgIRQ = true;	// keep on running
}


#if BAT_MON_INTERVAL > 0
/***************************************************************************//**
 *
 * @brief	Battery Monitoring Trigger
 *
 * This routine is called when the battery monitoring interval is over.  This
 * interval can be adjusted by @ref BAT_MON_INTERVAL.
 *
 ******************************************************************************/
static void	BatMonTrigger(TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    /* Restart the timer */
    if (l_thBatMon != NONE)
	sTimerStart (l_thBatMon, BAT_MON_INTERVAL);

    /* Set trigger flag */
    l_flgBatMonTrigger = true;

    g_flgIRQ = true;	// keep on running
}
#endif


/***************************************************************************//**
 *
 * @brief	Battery Monitoring Trigger Alarm
 *
 * This routine is called when a battery monitoring alarm time has been reached.
 * There are 2 alarm times: @ref ALARM_BAT_MON_TIME_1 and @ref
 * ALARM_BAT_MON_TIME_2.
 *
 ******************************************************************************/
static void	BatMonTriggerAlarm(int alarmNum)
{
    (void) alarmNum;	// suppress compiler warning "unused parameter"

    /* Set trigger flag */
    l_flgBatMonTrigger = true;

    g_flgIRQ = true;	// keep on running
}


/***************************************************************************//**
 *
 * @brief	Battery Information Request
 *
 * This routine can be called from interrupt context to introduce up to two
 * requests for battery information.  The requests will be handled by function
 * BatteryCheck().  Use BatteryInfoGet() to poll for finished requests and
 * get the data.  If only one request is used, set <i>req_2</i> to SBS_NONE.
 *
 ******************************************************************************/
void	 BatteryInfoReq (SBS_CMD req_1, SBS_CMD req_2)
{
    l_BatInfo.Req_1 = req_1;
    l_BatInfo.Req_2 = req_2;
    strcpy ((char *)l_BatInfo.Buffer, "ERROR");
    l_BatInfo.Done  = false;
}


/***************************************************************************//**
 *
 * @brief	Battery Information Get
 *
 * This routine can be called from interrupt context to get the results of
 * a battery information request.  The requests will be handled by function
 * BatteryCheck().  Use BatteryInfoReq() to introduce information requests.
 * The information request is finished, when both request elements are set
 * to SBS_NONE.
 *
 ******************************************************************************/
BAT_INFO *BatteryInfoGet (void)
{
    return &l_BatInfo;
}
