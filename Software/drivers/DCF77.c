/***************************************************************************//**
 * @file
 * @brief	DCF77 Atomic Clock Decoder
 * @author	Ralf Gerhauser
 * @version	2015-02-26
 *
 * This module implements an Atomic Clock Decoder for the signal of the
 * German-based DCF77 long wave transmitter.
 *
 * In detail, it includes:
 * - Initialization of the hardware (GPIOs that are connected to the DCF77
 *   receiver).
 * - A signal handler which is called whenever the logic level of the DCF77
 *   receiver changes.  It decodes the 59 bits of the DCF77 signal to ones
 *   and zeros, and stores them into a <b>tm</b> structure.  When the data is
 *   complete, the system clock is synchronized with this value.
 * - A signal supervisor that detects when the DCF77 signal does not change
 *   its level for more than 5 seconds.
 * - Automatic switching between winter time (MEZ) and summer time (daylight
 *   saving time, MESZ).  All alarm times will be changed plus/minus one hour
 *   to effectively occur at the time, regardless whether MEZ or MESZ.
 *
 * When initialized, the DCF77 hardware module is enabled to receive the time
 * information and set the system clock.  If a segment LCD is available, the
 * <b>Antenna</b> symbol is blinking during this first time.  The further
 * behavior depends on the settings in the <i>config.h</i> file.  If the
 * define @ref DCF77_ONCE_PER_DAY is 1, the receiver (and also the Antenna
 * symbol) will be switched off after time has been synchronized.  As the name
 * of the define suggests, it will be switched on once per day.  To properly
 * detect a change between winter and summer time, i.e. <i>normal time</i>
 * (MEZ) and <i>daylight saving time</i> (MESZ), this happens at 01:55 for MEZ,
 * and 02:55 for MESZ.<br> If the define is 0, the receiver remains switched on
 * and the Antenna symbol is permanently shown.  However, when no DCF77 signal
 * could be received for more than 5 seconds, the Antenna symbol will not be
 * shown to identify this kind of error.<br>
 * The <b>ARing</b> symbol is used to visualize the progress of receiving one
 * complete time frame.
 * For hardware platforms without a segment LCD, an LED can be used as
 * indicator for receiving a DCF77 signal, please refer to the configuration
 * parameters @ref DCF77_DISPLAY_PROGRESS and @ref DCF77_INDICATOR.
 *
 * @see
 * https://de.wikipedia.org/wiki/DCF77 for a description of the DCF77 signal,
 * and the <a href="../X200_DCF77.pdf">data sheet</a> of the DCF77
 * hardware module.
 *
 * @note
 * To make the @ref DCF77_ONCE_PER_DAY feature work, an enum @ref
 * ALARM_DCF77_WAKE_UP of type @ref ALARM_ID must be defined in <i>config.h</i>.
 *
 * @warning
 * Be aware that the DCF77 hardware module is very sensitive against high
 * frequency signals.  Cases have been observed where an SPI access to the
 * SD-Card disturbs the DCF77 signal in a way, that a re-synchronization is
 * necessary.
 *
 ****************************************************************************//*
Revision History:
2016-04-06,rage	Made local variables of type "volatile".
		BugFix: implicit MEZ to MESZ change during daylight saving time
		was misinterpreted during initial time synchronisation.
2016-02-15,rage	Implemented additional check for consecutive received DCF77
		time frames, see FRAME_SEQ_CNT.  Consider change of timezone.
2016-02-10,rage	Changed ShowDCF77Indicator() to ON for STATE_NO_SIGNAL to show
		loss of signal.
2015-05-08,rage	BugFixes: In DCF77Handler() STATE_UP_TO_DATE must be set before
		calling DCF77Disable().  DCF77Disable() must also disable the
		Signal Supervision timer.
2015-02-26,rage	Define DCF77_INDICATOR to call external ShowDCF77Indicator()
		routine, e.g. for using an LED as indicator for DCF77 signal.
2014-05-16,rage	DCF77Handler(): Changed switch() statement into separate case
		values because the IAR compiler cannot handle ranges.
2014-04-07,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include "em_device.h"
#include "em_assert.h"
#include "em_cmu.h"
#include "Logging.h"
#include "DCF77.h"
#include "ExtInt.h"
#include "AlarmClock.h"
#if DCF77_DISPLAY_PROGRESS
  #include "SegmentLCD.h"
#endif

/*=============================== Definitions ================================*/

    // Module Debugging
#define MOD_DEBUG	0	// set 1 to enable debugging of this module
#if ! MOD_DEBUG
    #undef  DBG_PUTC
    #undef  DBG_PUTS
    #define DBG_PUTC(ch)	// define as empty
    #define DBG_PUTS(str)
#endif

/*!@brief Frame sequence count (number of consecutive valid frames) */
#define FRAME_SEQ_CNT		2

/*!@brief Time when DCF77 will be activated for switching from MEZ to MESZ */
#define ALARM_MEZ_TO_MESZ   1, 55
/*!@brief Time when DCF77 will be activated for switching from MESZ to MEZ */
#define ALARM_MESZ_TO_MEZ   2, 55

/*=========================== Typedefs and Structs ===========================*/

    /*!@brief Local states of the DCF77 signal */
typedef enum
{
    STATE_OFF,			//!< 0: [A] Atomic Clock Decoder is OFF
    STATE_NO_SIGNAL,		//!< 1: [B] No signal detected
    STATE_SYNC_WAIT,		//!< 2: [C] Waiting for 2s SYNC pause
    STATE_RECV_DATA,		//!< 3: [D] Got SYNC pulse, receive data
    STATE_UP_TO_DATE,		//!< 4: [E] Received complete time information
} DCF_STATE;

/*======================== External Data and Routines ========================*/

#if DCF77_INDICATOR
    /* external routine to switch DCF77 signal indicator on or off */
    extern void ShowDCF77Indicator (bool enable);

    /* external routine to call when <l_State> reaches STATE_UP_TO_DATE */
    extern void DisplayUpdEnable (void);
#endif

/*========================= Global Data and Routines =========================*/

    /*!@brief DCF77 date and time structure. */
struct tm    dcf77;	// not intended to be used by other modules

/*================================ Local Data ================================*/

    /*!@brief Current DCF77 state */
static volatile DCF_STATE  l_State = STATE_OFF;

    /*!@brief sTimer handle for signal supervision. */
static volatile TIM_HDL	 l_TimHdl = NONE;

    /*!@brief Frame sequence counter (1..FRAME_SEQ_CNT) */
static volatile uint8_t	 l_FrameSeqCnt = 1;


/*=========================== Forward Declarations ===========================*/

static void	TimeSynchronize (struct tm *pTime);
static void	SignalSuperVisor (TIM_HDL hdl);
static void	StateChange (DCF_STATE newState);


/***************************************************************************//**
 *
 * @brief	Initialize DCF77 hardware
 *
 * This routine initializes the GPIO pins which are connected to the external
 * DCF77 module.  The GPIO ports and pins have been defined in the header file.
 * You additionally have to call DCF77Enable() after module ExtInt and the
 * LCD have been initialized to start the decoder.
 *
 ******************************************************************************/
void	DCF77Init (void)
{
    /* Be sure to enable clock to GPIO (should already be done) */
    CMU_ClockEnable (cmuClock_GPIO, true);

#if DCF77_HARDWARE_ENABLE
    /* The DCF77 module provides a low-active enable pin, set to 1 for OFF */
    GPIO_PinModeSet (DCF77_ENABLE_PORT, DCF77_ENABLE_PIN, gpioModePushPull, 1);
#endif

    /*
     * Configure DCF77 signal input pin and connect it to the external
     * interrupt (EXTI) facility.  At this stage, the interrupt is not
     * enabled, this is done later by calling ExtIntInit().
     */
    GPIO_PinModeSet (DCF77_SIGNAL_PORT, DCF77_SIGNAL_PIN, gpioModeInput, 0);
    GPIO_IntConfig  (DCF77_SIGNAL_PORT, DCF77_SIGNAL_PIN, false, false, false);

    /* When called for the first time, allocate timer handle */
    if (l_TimHdl == NONE)
	l_TimHdl = sTimerCreate (SignalSuperVisor);

#if DCF77_ONCE_PER_DAY
    /*
     * Set up wake-up time for DCF77 receiver and decoder. The initial time zone
     * is MEZ.  Alarm times will be converted during first time synchronization
     * if daylight saving time (MESZ) is active.
     */
    AlarmAction (ALARM_DCF77_WAKE_UP, (ALARM_FCT)DCF77Enable);
    AlarmSet (ALARM_DCF77_WAKE_UP, ALARM_MEZ_TO_MESZ);
    AlarmEnable (ALARM_DCF77_WAKE_UP);
#endif
}

/***************************************************************************//**
 *
 * @brief	Enable the DCF77 decoder
 *
 * This routine switches the DCF77 decoder ON, i.e. the respective external
 * interrupt will be enabled.  If the define @ref DCF77_HARDWARE_ENABLE is 1,
 * additionally the DCF77 receiver will be activated via a dedicated pin.
 *
 * @warning
 * Call this routine only after software module ExtInt and the LCD already
 * have been initialized.
 *
 ******************************************************************************/
void	DCF77Enable (void)
{

#ifdef LOGGING
    Log ("DCF77: Enabled");
#endif

#if DCF77_HARDWARE_ENABLE
    /* Set low-active enable pin of DCF77 module to 0 */
    GPIO->P[DCF77_ENABLE_PORT].DOUTCLR = (1 << DCF77_ENABLE_PIN);
#endif

    /* Reset frame counter */
    l_FrameSeqCnt = 1;

    /* Interrupt enable */
    ExtIntEnable (DCF77_SIGNAL_PIN);

    /* Change DCF state */
    StateChange (STATE_NO_SIGNAL);
}

/***************************************************************************//**
 *
 * @brief	Disable the DCF77 decoder
 *
 * This routine switches the DCF77 decoder OFF, i.e. the respective external
 * interrupt will be disabled.  If the define @ref DCF77_HARDWARE_ENABLE is 1,
 * additionally the DCF77 receiver will be shut off via a dedicated pin.
 *
 ******************************************************************************/
void	DCF77Disable (void)
{
    /* Disable Signal Supervision */
    if (l_TimHdl != NONE)
	sTimerCancel (l_TimHdl);

    /* Interrupt disable */
    ExtIntDisable (DCF77_SIGNAL_PIN);

#if DCF77_HARDWARE_ENABLE
    /* Set low-active enable pin of DCF77 module to 1 */
    GPIO->P[DCF77_ENABLE_PORT].DOUTSET = (1 << DCF77_ENABLE_PIN);
#endif

#if DCF77_INDICATOR
    /* be sure to switch indicator OFF */
    ShowDCF77Indicator (false);
#endif

    /* Change DCF state */
    StateChange (STATE_OFF);

#ifdef LOGGING
    Log ("DCF77: Disabled");
#endif
}


//#define DCF77_TEST_CODE	// UNCOMMENT TO INCLUDE TEST CODE
#ifdef	DCF77_TEST_CODE
/* test code for "change time zone" - MODIFY THESE VARIABLES VIA DEBUGGER! */
int testMode = 0;	// 0: no test, 1: test MEZ->MESZ, 2: test MESZ->MEZ
int testCountDown;	// count down in [min] when change should occur
void testChangeTZ (void)
{
    if (testMode == 1)			// MEZ -> MESZ
    {
	dcf77.tm_isdst = 0;		// MEZ
	if (--testCountDown > 0)
	    return;
	dcf77.tm_isdst = 1;		// now MESZ
	if (++dcf77.tm_hour > 23)
	    dcf77.tm_hour = 0;
    }
    else if (testMode == 2)		// MESZ -> MEZ
    {
	dcf77.tm_isdst = 1;		// MESZ
	if (--testCountDown > 0)
	    return;
	dcf77.tm_isdst = 0;		// now MEZ
	if (--dcf77.tm_hour < 0)
	    dcf77.tm_hour = 23;
    }
}
#endif

/***************************************************************************//**
 *
 * @brief	Signal handler
 *
 * This handler is called by the EXTI interrupt service routine whenever the
 * logical level of the DCF77 signal changes.
 *
 * @param[in] extiNum
 *	EXTernal Interrupt number of the DCF77 signal.  This is identical
 *	with the pin number, i.e. @ref DCF77_SIGNAL_PIN.
 *
 * @param[in] extiLvl
 *	EXTernal Interrupt level: 0 means falling edge, logic level is now 0,
 *	1 means rising edge, logic level is now 1.  The signal from the DCF77
 *	module uses positive logic, i.e. pulses of about 100ms or 200ms are
 *	received.
 *
 * @param[in] timeStamp
 *	Time stamp (24bit) when the signal has changed its level.  This is
 *	used to decode a one or zero bit, and the synchronization pause.
 *
 * @note
 * The time stamp is read from the Real Time Counter (RTC), so its resolution
 * depends on the RTC.  Use the define @ref RTC_COUNTS_PER_SEC to convert the
 * RTC value into a duration.
 *
 ******************************************************************************/
void	DCF77Handler (int extiNum, bool extiLvl, uint32_t timeStamp)
{
static int8_t	bitNum = NONE;	// bit number, or NONE if waiting for SYNC
static uint32_t	pulseLength=0;	// length of high-pulse in number of RTC tics
static uint32_t	tsRising;	// time-stamp of previous rising edge
static uint32_t	value;		// general purpose variable
static bool	flgMESZ;	// true for MESZ (daylight saving time)
#if DCF77_ONCE_PER_DAY
static bool	flgAwaitChange;	// true if awaiting MEZ/MESZ change
#endif
static time_t	prevTime;	// previous time in seconds (UNIX time)
static bool	parity;		// data parity bit
bool		bit;		// current data bit


    (void) extiNum;	// suppress compiler warning "unused parameter"

    /* Check state to see if decoder is enabled */
    if (l_State == STATE_OFF)
    {
	/* Be sure to disable DCF77 */
	DCF77Disable();
	return;
    }

    /* Received signal, verify current state */
    if (l_State == STATE_NO_SIGNAL)
    {
	/* We've got a signal now - wait for SYNC */
	StateChange (STATE_SYNC_WAIT);
	bitNum = NONE;		// be sure to activate SYNC mode
    }

    /* Start timer to detect future signal inactivity after 5s */
    if (l_TimHdl != NONE)
	sTimerStart (l_TimHdl, 5);

    /* See if rising or falling edge */
    if (extiLvl)
    {
	/*========== Rising edge ==========*/

	DBG_PUTC('#');

#if DCF77_INDICATOR
  #if DCF77_ONCE_PER_DAY
	/* switch indicator always ON */
	DBG_PUTC('*');
	ShowDCF77Indicator (true);
  #else
	/* switch indicator ON - except if time is already up-to-date */
	if (l_State != STATE_UP_TO_DATE)
	{
	    DBG_PUTC('*');
	    ShowDCF77Indicator (true);
	}
  #endif
#endif
	/* see if new DCF77 time was prepared for setting system clock */
	if (bitNum == 58)
	{
#ifdef	DCF77_TEST_CODE
	    testChangeTZ();	// test routine for "change time zone"
#endif

	    /*
	     * Since DCF77 data may be faulty due to signal weakness, we have
	     * to receive a sequence of valid frames.  These must show a time
	     * data difference of exactly 60s (except the timezone has changed).
	     * The number of frames to be received ist defined by FRAME_SEQ_CNT.
	     */
	    /* Convert <tm> structure to <time_t> */
	    struct tm	currTimeTM = dcf77;
	    time_t	currTime;
	    long	expDiff;	// expected time difference

	    currTimeTM.tm_isdst = 0;		// always 0 for mktime()
	    currTime = mktime (&currTimeTM);	// current time in seconds

	    /*
	     * Flag to detect whether MEZ<=>MESZ change occurred.  The system
	     * always starts up in MEZ (g_isdst is 0), so, when initial time
	     * has been received during daylight saving time (MESZ), this flag
	     * will always be <true> and the difference to the previous time
	     * frame is expected to be +3660 seconds - which is not correct!
	     */
	    bool changeOccurred = (g_isdst != (bool)dcf77.tm_isdst);

	    /* Consider timezone change only when really expected */
	    expDiff = (flgAwaitChange && changeOccurred ?
			(g_isdst ? -3540 : +3660) : +60);

	    if (currTime == prevTime + expDiff)
		l_FrameSeqCnt++;	// count valid frame
	    else
		l_FrameSeqCnt = 1;	// reset frame counter

	    prevTime = currTime;	// save time for next compare

#ifdef LOGGING
	    Log ("DCF77: Time Frame %d is 20%02d%02d%02d-%02d%02d%02d",
		 l_FrameSeqCnt,
		 dcf77.tm_year, dcf77.tm_mon + 1, dcf77.tm_mday,
		 dcf77.tm_hour, dcf77.tm_min, dcf77.tm_sec);
#endif

	    /* see if enough valid frames have been received */
	    if (l_FrameSeqCnt >= FRAME_SEQ_CNT)
	    {
		/*
		 * A sequence of enough consecutive frames has been
		 * received, we can trust the information and synchronize
		 * the local clock.
		 */
		if (l_FrameSeqCnt > 250)
		    l_FrameSeqCnt = 250;	// prevent counter from overflow

		/* set local time, show time on display */
		TimeSynchronize (&dcf77);

		/* RTC is 0, correct timeStamp and tsRising values */
		tsRising -= timeStamp;
		timeStamp = 0;

#if DCF77_ONCE_PER_DAY
		if (changeOccurred)
		{
		    /* change already occurred - clear flag */
		    flgAwaitChange = false;
		}
#endif

		/* received complete and valid data, set STATE_UP_TO_DATE */
		StateChange (STATE_UP_TO_DATE);

#if DCF77_ONCE_PER_DAY
		/* disable the DCF77 to save power, except if waiting for change */
		if (! flgAwaitChange  ||  dcf77.tm_min < 50)
		{
		    DCF77Disable();
		    return;
		}
#endif
	    }

	    /* enter SYNC mode again (without blinking antenna) */
	    bitNum = NONE;
	}	// if (bitNum == 58)

	/* check for SYNC mode */
	if (bitNum == NONE)
	{
	    /*
	     * Waiting for SYNC, verify that previous high-pulse was valid and
	     * calculate distance of rising edges.  In case of SYNC this should
	     * be about 2 seconds (2000ms)
	     */
	    if (MS2TICS(40) < pulseLength  &&  pulseLength < MS2TICS(230))
	    {
	    uint32_t pauseLen;

		/* Previous pulse was valid - check distance of rising edges */
		pauseLen = (timeStamp - tsRising) & 0x00FFFFFF;
		if (MS2TICS(1800) < pauseLen  &&  pauseLen < MS2TICS(2200))
		{
		    /* SYNC detected - bit 0 will follow, receive data */
		    if (l_State != STATE_UP_TO_DATE)
			StateChange (STATE_RECV_DATA);
		    bitNum = 0;
		}
	    }
	}
	else
	{
	    /* Regular mode, increase bit number */
	    bitNum++;
	}

	/* Save time stamp for rising edge */
	tsRising = timeStamp;

	return;		// DONE - return from interrupt
    }

    /*========== Falling edge ==========*/

    DBG_PUTC('_');
    DBG_PUTC('A' + l_State);	// A=OFF B=NoSig C=SYNC D=Data E=UpToDate

#if DCF77_INDICATOR
  #if DCF77_ONCE_PER_DAY
	/* switch indicator always OFF */
	DBG_PUTC('.');
	ShowDCF77Indicator (false);
  #else
	/* switch indicator OFF - except if time is already up-to-date */
	if (l_State != STATE_UP_TO_DATE)
	{
	    DBG_PUTC('.');
	    ShowDCF77Indicator (false);
	}
  #endif
#endif

    /* Measure pulse length (24bit) - consider wrap-around */
    pulseLength = (timeStamp - tsRising) & 0x00FFFFFF;

    /* Ignore pulse if still seeking for SYNC */
    if (bitNum == NONE)
	return;		// DONE - return from interrupt

    /* Decode value for received bit */
    if (MS2TICS(40) < pulseLength  &&  pulseLength < MS2TICS(130))
    {
	bit = 0;	// 40~130ms means 0
	DBG_PUTC('0');
    }
    else if (MS2TICS(140) < pulseLength  &&  pulseLength < MS2TICS(230))
    {
	bit = 1;	// 140~230ms means 1
	DBG_PUTC('1');
    }
    else
    {
	/* Invalid pulse width - return to SYNC mode */
	StateChange (STATE_SYNC_WAIT);
	bitNum = NONE;
	DBG_PUTC('X');
	return;
    }

    /* Perform data processing according to the bit number */
    switch (bitNum)
    {
	case 0:		/* Bit 0: Start of frame - must be 0 */
#if DCF77_DISPLAY_PROGRESS
	    SegmentLCD_ARing (7, SYM_ON);	// display first ARing symbol
	    SegmentLCD_Update();		// Update display
#endif
	    if (bit != 0)
		break;		// bit not 0 means error
	    return;

	case 1:		/* Bit 1~14: Weather information (encoded) */
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
#if DCF77_DISPLAY_PROGRESS
	    if (bitNum == 8)
	    {
		SegmentLCD_ARing (0, SYM_ON);	// display next ARing symbol
		SegmentLCD_Update();		// Update display
	    }
#endif
	    return;

	case 15:	/* Bit 15: Call-bit for PTB employees */
#if DCF77_DISPLAY_PROGRESS
	    SegmentLCD_ARing (1, SYM_ON);	// display next ARing symbol
	    SegmentLCD_Update();		// Update display
#endif
	    return;

	case 16:	/* Bit 16: 1 if MEZ/MESZ change at end of this hour */
#ifdef	DCF77_TEST_CODE
	    if (testMode)
		bit = 1;	// if test mode, set to 1 (await change)
#endif
#if DCF77_ONCE_PER_DAY
	    flgAwaitChange = bit;
#endif
	    return;

	case 17:	/* Bit 17: 0=MEZ, 1=MESZ - set MESZ flag */
	    flgMESZ = bit;	// Daylight saving time flag
	    return;

	case 18:	/* Bit 18: 0=MESZ, 1=MEZ - verify MESZ flag */
	    if (flgMESZ == bit)
		break;		// if both bits are 0 or 1, this means error

	    /* store flag for daylight saving time */
	    dcf77.tm_isdst = flgMESZ;
	    return;

	case 19:	/* Bit 19: 1 if additional second at end of hour */
	    return;

	case 20:	/* Bit 20: Start of time - must be 1 */
	    if (bit != 1)
		break;		// bit not 1 means error
	    value  = 0;		// value means "minutes" in this case
	    parity = 0;		// clear parity
	    return;

	case 21:	/* Bit 21~27: Minutes as BCD */
	case 22:
	case 23:
	case 24:
	case 25:
	case 26:
	case 27:
	    parity ^= bit;	// XOR parity bit
	    if (bit)
		value |= (1 << (bitNum - 21));

#if DCF77_DISPLAY_PROGRESS
	    if (bitNum == 22)
	    {
		SegmentLCD_ARing (2, SYM_ON);	// display next ARing symbol
		SegmentLCD_Update();		// Update display
	    }
#endif
	    return;

	case 28:	/* Bit 28: Parity bit for minutes */
	    if (parity != bit)
		break;		// parity error

	    /* calculate and store minutes value */
	    dcf77.tm_min = ((value >> 4) & 0x0F) * 10 + (value & 0x0F);

	    /* prepare parity and variable for hours value */
	    value  = 0;		// value means "hours" in this case
	    parity = 0;		// clear parity
	    return;

	case 29:	/* Bit 29~34: Hours as BCD */
	case 30:
	case 31:
	case 32:
	case 33:
	case 34:
	    parity ^= bit;	// XOR parity bit
	    if (bit)
		value |= (1 << (bitNum - 29));

#if DCF77_DISPLAY_PROGRESS
	    if (bitNum == 30)
	    {
		SegmentLCD_ARing (3, SYM_ON);	// display next ARing symbol
		SegmentLCD_Update();		// Update display
	    }
#endif
	    return;

	case 35:	/* Bit 35: Parity bit for hours */
	    if (parity != bit)
		break;		// parity error

	    /* calculate and store hours value */
	    dcf77.tm_hour = ((value >> 4) & 0x03) * 10 + (value & 0x0F);

	    /* prepare parity and variable for date value */
	    value  = 0;		// value means "date" in this case
	    parity = 0;		// clear parity
	    return;

	case 36:	/* Bit 36~57: Date (day, month, year) */
	case 37:
	case 38:
	case 39:
	case 40:
	case 41:
	case 42:
	case 43:
	case 44:
	case 45:
	case 46:
	case 47:
	case 48:
	case 49:
	case 50:
	case 51:
	case 52:
	case 53:
	case 54:
	case 55:
	case 56:
	case 57:
	    parity ^= bit;	// XOR parity bit
	    if (bit)
		value |= (1 << (bitNum - 36));

#if DCF77_DISPLAY_PROGRESS
	    if (bitNum == 37)
	    {
		SegmentLCD_ARing (4, SYM_ON);	// display next ARing symbol
		SegmentLCD_Update();		// Update display
	    }
	    else if (bitNum == 44)
	    {
		SegmentLCD_ARing (5, SYM_ON);	// display next ARing symbol
		SegmentLCD_Update();		// Update display
	    }
	    else if (bitNum == 52)
	    {
		SegmentLCD_ARing (6, SYM_ON);	// display next ARing symbol
		SegmentLCD_Update();		// Update display
	    }
#endif
	    return;

	case 58:	/* Bit 58: Parity bit for date */
	    if (parity != bit)
		break;		// parity error

	    /* calculate and store date values */
	    dcf77.tm_mday = ((value >> 4) & 0x03) * 10 + (value & 0x0F);
	    value >>= (45 - 36);		// align month value
	    dcf77.tm_mon  = ((value >> 4) & 0x01) * 10 + (value & 0x0F) - 1;
	    value >>= (50 - 45);		// align year value
	    dcf77.tm_year = ((value >> 4) & 0x0F) * 10 + (value & 0x0F);

#if DCF77_DISPLAY_PROGRESS
	    /* clear all ARing segments */
	    SegmentLCD_ARingSetAll (SYM_OFF);
	    SegmentLCD_Update();		// Update display
#endif
	    return;

	default:		// invalid bit number
	    EFM_ASSERT(0);	// stall if DEBUG_EFM is set
	    break;
    }

    /* When the code arrives here, invalid data has been received */
    StateChange (STATE_SYNC_WAIT);
    bitNum = NONE;
}

/***************************************************************************//**
 *
 * @brief	Time Synchronization
 *
 * This function is called from the DCF77 interrupt handler after a sequence
 * of consecutive valid frames has been received, so we can be sure the time
 * information is valid.  It sets the system clock via ClockSet() and updates
 * the display with the new time.
 * It also checks for a change of MEZ to MESZ and vice versa.  If this happens,
 * all configured alarm times will be adjusted accordingly.
 *
 * @note
 * Be aware, this function is called in interrupt context!
 *
 * @param[in] pTime
 *	Pointer to a <b>tm</b> structure that holds the current time.
 *
 ******************************************************************************/
static void	TimeSynchronize (struct tm *pTime)
{
    /* flag to detect whether MEZ<=>MESZ change occurred */
    bool changeOccurred = (g_isdst != (bool)pTime->tm_isdst);

    /* set system clock to DCF77 time */
    g_CurrDateTime = *pTime;
    g_isdst = pTime->tm_isdst;		// flag for daylight saving time
    ClockSet (&g_CurrDateTime, true);	// set milliseconds to zero

    /* show time on display */
    ClockUpdate (false);	// g_CurrDateTime is already up to date

#if DCF77_ONCE_PER_DAY  &&  defined(LOGGING)
    /* log current DCF77 time */
    Log("DCF77: Time Synchronization %02d:%02d:%02d (%s)",
	pTime->tm_hour, pTime->tm_min, pTime->tm_sec, g_isdst ? "MESZ" : "MEZ");
#endif

    /*
     * DCF77 may be activated once per day only.  To detect a change
     * between MEZ and MESZ properly, the DCF77 will be switched-on at
     * the right time.  When such a change is detected, all alarm times
     * must be corrected to still occur at the same effective time.
     * This includes the ALARM_DCF77_WAKE_UP, which is switched between
     * 01:55 (MEZ) and 02:55 (MESZ) properly.
     */
    if (changeOccurred)
    {
    int	    alarm;
    int8_t  hour, minute;

	if (g_isdst)
	    Log ("DCF77: Changing time zone from MEZ to MESZ");
	else
	    Log ("DCF77: Changing time zone from MESZ to MEZ");

	/* MEZ <-> MESZ change detected */
	for (alarm = 0;  alarm < MAX_ALARMS;  alarm++)
	{
	    AlarmGet (alarm, &hour, &minute);

	    hour += (g_isdst ? +1 : -1);
	    if (hour < 0)
		hour = 23;
	    else if (hour > 23)
		hour = 0;

	    AlarmSet (alarm, hour, minute);
	}
    }
}

/***************************************************************************//**
 *
 * @brief	Signal Supervision
 *
 * This function is called if the DCF77 signal is inactive (does not change
 * its state) for more than 5 seconds.  It clears the <i>Antenna</i> symbol
 * from the LC-Display (or the indicator LED) and sets the DCF state to
 * STATE_NO_SIGNAL.
 *
 * @note
 * Be aware, this function is called in interrupt context!
 *
 * @param[in] hdl
 *	Timer handle (not used here).
 *
 ******************************************************************************/
static void	SignalSuperVisor (TIM_HDL hdl)
{
    (void) hdl;

    /* No signal for more than 5 seconds - set state to "no signal" */
    StateChange (STATE_NO_SIGNAL);
}

/***************************************************************************//**
 *
 * @brief	State Change
 *
 * This routine must be called whenever the state of the DCF receiver has
 * changed.  It updates the <i>Antenna</i> symbol on the LCD (or the indicator
 * LED).
 *
 * @note
 * Be aware, this function can be called in interrupt context!
 *
 * @param[in] newState
 *	New state to establish.  The value is of type @ref DCF_STATE.
 *
 ******************************************************************************/
static void	StateChange (DCF_STATE newState)
{
    /* Check new state */
    if (newState == l_State)
	return;			// no change at all

    /* Handle antenna symbol according to the new state */
    switch (newState)
    {
	case STATE_OFF:		// decoder switched off
	case STATE_NO_SIGNAL:	// no DCF77 signal detected
#if DCF77_DISPLAY_PROGRESS
	    /* Clear antenna symbol */
	    SegmentLCD_Symbol (LCD_SYMBOL_ANT, SYM_OFF);

	    /* Clear all ARing symbols */
	    SegmentLCD_ARingSetAll (SYM_OFF);
#endif
#if DCF77_INDICATOR
	    if (newState == STATE_NO_SIGNAL)
	    {
		/* switch indicator ON for lost signal */
		ShowDCF77Indicator (true);
	    }
	    else
	    {
		/* switch indicator OFF when DCF77 disabled */
		ShowDCF77Indicator (false);
	    }
#endif
	    break;

	case STATE_SYNC_WAIT:	// waiting for SYNC
#if DCF77_DISPLAY_PROGRESS
	    /* Let antenna symbol blink */
	    SegmentLCD_Symbol (LCD_SYMBOL_ANT, SYM_BLINK);

	    /* Clear all ARing symbols */
	    SegmentLCD_ARingSetAll (SYM_OFF);
#endif
	    break;

	case STATE_RECV_DATA:	// got SYNC pulse, receive data
	    break;		// no changes

	case STATE_UP_TO_DATE:	// time completely received
#if DCF77_DISPLAY_PROGRESS
	    /* Switch antenna symbol on */
	    SegmentLCD_Symbol (LCD_SYMBOL_ANT, SYM_ON);
#endif
	    /*
	     * If DCF77 is enabled once per day, the indicator shall be active
	     * as long as the receiver is enabled.
	     * If the receiver is enabled permanently, the indicator shall be
	     * switched off as soon as time is up-to-date, otherwise it would
	     * consume too much power all the day.
	     */
#if DCF77_INDICATOR  &&  ! DCF77_ONCE_PER_DAY
	    /* switch indicator OFF */
	    ShowDCF77Indicator (false);
#endif
	    /* callback to allow LCD updates */
	    DisplayUpdEnable();
	    break;

	default:		// unhandled state
	    EFM_ASSERT(0);	// stall if DEBUG_EFM is set
	    break;
    }

    /* Establish new state */
    l_State = newState;

#if DCF77_DISPLAY_PROGRESS
    /* Update display */
    SegmentLCD_Update();
#endif
}
