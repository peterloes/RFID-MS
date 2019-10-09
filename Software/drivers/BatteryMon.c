/***************************************************************************//**
 * @file
 * @brief	Battery Monitoring
 * @author	Ralf Gerhauser
 * @version	2016-02-26
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

#include "em_cmu.h"
#include "em_i2c.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "AlarmClock.h"		// msDelay()
#include "BatteryMon.h"
#include "Display.h"
#include "Logging.h"

/*=============================== Definitions ================================*/

    /*!@name Hardware Configuration: SMBus controller and pins. */
//@{
#define SMB_GPIOPORT		gpioPortA	//!< Port SMBus interface
#define SMB_SDA_PIN		0		//!< Pin for SDA signal
#define SMB_SCL_PIN		1		//!< Pin for SCL signal
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

/*================================== Macros ==================================*/

#ifndef LOGGING		// define as empty, if logging is not enabled
    #define LogError(str)
#endif

/*========================= Global Data and Routines =========================*/

    /*!@brief Battery voltage in [mV] */
volatile int16_t   g_BattMilliVolt = (-1);   // show error message per default

    /*!@brief Remaining capacity of the Battery in [mAh] */
volatile uint16_t  g_BattCapacity;

/*================================ Local Data ================================*/

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

/* Flag to trigger battery monitoring measurement, see BAT_MON_INTERVAL. */
static volatile bool l_flgBatMonTrigger;

#if BAT_MON_INTERVAL > 0
    /* Timer handle for the battery monitoring interval */
static TIM_HDL	l_thBatMon = NONE;
#endif

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
 * assembles them to a 16bit value, and returns this.  If an error occured,
 * a negative status code is returned instead.
 *
 * @param[in] cmd
 *	SBS command, i.e. the register address to be read.
 *
 * @return
 *	Value of the 16bit register, or a negative error code of type @ref
 *	I2C_TransferReturn_TypeDef.  Additionally to those codes, there is
 *	another error code defined, named @ref i2cTransferTimeout.
 *
 * @see
 *	BatteryRegReadBlock()
 *
 ******************************************************************************/
int	 BatteryRegReadWord (SBS_CMD cmd)
{
I2C_TransferSeq_TypeDef smbXfer;	// SMBus transfer data
uint8_t addrBuf[1];			// buffer for device address (0x0A)
uint8_t dataBuf[2];			// buffer for data read from the device


    /* Check parameter */
    EFM_ASSERT ((cmd & ~0xFF) == 0);	// size field must be 0

    /* Set up SMBus transfer */
    smbXfer.addr  = 0x0A;		// I2C address of the Battery Controller
    smbXfer.flags = I2C_FLAG_WRITE_READ; // need write and read
    smbXfer.buf[0].data = addrBuf;	// first write device I2C address
    addrBuf[0] = cmd;
    smbXfer.buf[0].len  = 1;		// 1 byte for I2C address
    smbXfer.buf[1].data = dataBuf;	// where to store read data
    smbXfer.buf[1].len  = 2;		// read 2 bytes from register

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

    /* Check final status */
    if (SMB_Status != i2cTransferDone)
    {
	return SMB_Status;
    }

    /* Assign data for return value in LSB/MSB manner */
    return (dataBuf[1] << 8) | dataBuf[0];	// return 16 bit data
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
 * @param[in] bufSize
 *	Buffer size in number of bytes.
 *
 * @return
 *	Status code @ref i2cTransferDone (0), or a negative error code of type
 *	@ref I2C_TransferReturn_TypeDef.  Additionally to those codes, there is
 *	another error code defined, named @ref i2cTransferTimeout.
 *
 * @see
 *	BatteryRegReadWord()
 *
 ******************************************************************************/
int	BatteryRegReadBlock (SBS_CMD cmd, uint8_t *pBuf, size_t bufSize)
{
I2C_TransferSeq_TypeDef smbXfer;	// SMBus transfer data
uint8_t addrBuf[1];			// buffer for device address (0x0A)


    /* Check parameters */
    EFM_ASSERT ((cmd & ~0xFF) != 0);	// size field must not be 0
    EFM_ASSERT (pBuf != NULL);		// buffer address
    EFM_ASSERT (bufSize >= SBS_CMD_SIZE(cmd));	// buffer size

    if (bufSize < SBS_CMD_SIZE(cmd))	// if EFM_ASSERT() is empty
	return i2cInvalidParameter;

    /* Set up SMBus transfer */
    smbXfer.addr  = 0x0A;		// I2C address of the Battery Controller
    smbXfer.flags = I2C_FLAG_WRITE_READ; // need write and read
    smbXfer.buf[0].data = addrBuf;	// first write device I2C address
    addrBuf[0] = cmd;
    smbXfer.buf[0].len  = 1;		// 1 byte for I2C address
    smbXfer.buf[1].data = pBuf;		// where to store read data
    smbXfer.buf[1].len  = SBS_CMD_SIZE(cmd);	// number of bytes to read

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
uint8_t	 buf[40];
int	 data, numDays;

    /* Try to read a register from the battery controller */
    data = BatteryRegReadWord (SBS_Voltage);
    if (data < 0)
    {
	/* ERROR */
	g_BattMilliVolt = data;
	LogError ("Battery Controller Read Error");
	return;
    }

    /* No get and log all the information */
    if (infoLvl == BAT_LOG_INFO_VERBOSE)
    {
	if (BatteryRegReadBlock (SBS_ManufacturerName, buf, sizeof(buf)) == 0)
	    Log ("Battery Manufacturer Name : %s", buf);

	if (BatteryRegReadBlock (SBS_ManufacturerData, buf, sizeof(buf)) == 0)
	    Log ("Battery Manufacturer Data : %s", buf);

	if (BatteryRegReadBlock (SBS_DeviceName, buf, sizeof(buf)) == 0)
	    Log ("Battery Device Name       : %s", buf);

	if (BatteryRegReadBlock (SBS_DeviceChemistry, buf, sizeof(buf)) == 0)
	    Log ("Battery Device Type       : %s", buf);

	data = BatteryRegReadWord (SBS_SerialNumber);
	if (data >= 0)
	    Log ("Battery Serial Number     : %05d", data);

	data = BatteryRegReadWord (SBS_DesignVoltage);
	if (data >= 0)
	    Log ("Battery Design Voltage    : %d.%03dV per cell",
		 data / 1000, data % 1000);

	data = BatteryRegReadWord (SBS_DesignCapacity);
	if (data >= 0)
	    Log ("Battery Design Capacity   : %dmAh", data);

	data = BatteryRegReadWord (SBS_FullChargeCapacity);
	if (data >= 0)
	    Log ("Battery Full Charge Capac.: %dmAh", data);
    }

    data = BatteryRegReadWord (SBS_RemainingCapacity);
    if (data >= 0)
    {
	g_BattCapacity = data;
	if (infoLvl != BAT_LOG_INFO_DISPLAY_ONLY)
	    Log ("Battery Remaining Capacity: %dmAh", data);
    }

    if (infoLvl != BAT_LOG_INFO_DISPLAY_ONLY)
    {
	data = BatteryRegReadWord (SBS_RunTimeToEmpty);
	if (data >= 0)
	{
	    if (data >= 65534)
	    {
		Log ("Battery Runtime to empty  : More than 6 weeks");
	    }
	    else
	    {
		numDays = data / 60 / 24;	// calculate run time in days
		Log ("Battery Runtime to empty  : %dmin (%dd)", data, numDays);
	    }
	}
    }

    data = BatteryRegReadWord (SBS_Voltage);
    if (data >= 0)
    {
	g_BattMilliVolt = data;
	if (infoLvl != BAT_LOG_INFO_DISPLAY_ONLY)
	    Log ("Battery Actual Voltage    : %2d.%dV",
		 data / 1000, (data % 1000) / 100);

	DisplayUpdateTrigger (LCD_BATTERY);
    }

    if (infoLvl != BAT_LOG_INFO_DISPLAY_ONLY)
    {
	data = BatteryRegReadWord (SBS_BatteryCurrent);
	if (data >= 0)
	{
	    Log ("Battery Actual Current    : %4dmA", data);
	}
    }
#endif
}


/***************************************************************************//**
 *
 * @brief	Battery Check
 *
 * This routine is called periodically to check the status of the battery.
 * It reads the voltage, the remaining capacity, and the remaining run time
 * from the battery controller via the SMBus.
 *
 ******************************************************************************/
void	BatteryCheck (void)
{
    if (l_flgBatMonTrigger)
    {
	l_flgBatMonTrigger = false;

	LogBatteryInfo (BAT_LOG_INFO_SHORT);
    }
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
