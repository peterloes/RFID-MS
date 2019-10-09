/***************************************************************************//**
 * @file
 * @brief	Logging
 * @author	Ralf Gerhauser
 * @version	2015-04-02
 *
 * This module provides a logging facility to send messages to the LEUART and
 * store them into a file on the SD-Card.
 *
 ****************************************************************************//*
Revision History:
2016-09-27,rage	LogFlushCheck: Flush log buffer if threshold has been reached,
		even if LOG_FLUSH_PAUSE is not over.
		Print logging timestamp with milliseconds resolution.
		Use INT_En/Disable() instead of __en/disable_irq().
2015-07-09,rage	IAR Compiler: Use sprintf() instead siprintf().
2015-04-02,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "em_device.h"
#include "em_assert.h"
#include "em_int.h"
#include "AlarmClock.h"
#include "Display.h"
#include "Logging.h"
#include "ff.h"		// FS_FAT12/16/32
#include "diskio.h"	// DSTATUS
#include "microsd.h"

/*=============================== Definitions ================================*/

#if LOG_MONITOR_FUNCTION == NONE
    #undef LOG_MONITOR_FUNCTION
#endif


    /*!@name Hardware Configuration: Log Flush LED. */
//@{
    //! Get or set level of the Log Flush LED */
#define LOG_FLUSH_LED  IO_Bit(GPIO->P[LOG_FLUSH_LED_PORT].DOUT, LOG_FLUSH_LED_PIN)
#define LOG_FLASH_LED_DELAY	50	// Delay [ms] between toggling the LED
#define LOG_FLASH_LED_CNT	5	// How often the LED is flashing
//@}

/*========================= Global Data and Routines =========================*/

    /*!@brief Filename of the current Log File on the SD-Card */
char	g_LogFilename[14];

/*================================ Local Data ================================*/

/*
 * The Log Buffer and its indices.  To ensure efficient data handling, all log
 * messages are directly stored as a consecutive stream of characters into the
 * log buffer, terminated by 0.  If the remaining amount of bytes to the end of
 * the buffer is less than LOG_ENTRY_MAX_SIZE, the storage wraps around, and
 * <idxLogPut> is set to 0, i.e. the beginning of the buffer.  This is marked
 * by an extra 0 byte, directly after the terminating 0 of the previous string.
 */
static char	l_LogBuf[LOG_BUF_SIZE];
static int	idxLogPut, idxLogGet;

    /* Counter for lost log entries */
static uint32_t	l_LostEntryCnt;

    /* Counter how many error messages may still be generated */
static int	l_ErrMsgCnt;

    /* Flag to trigger a flush of the log buffer, see LOG_SAMPLE_TIMEOUT. */
static volatile bool l_flgLogFlushTrigger;

    /* Flag to inhibit flushing the log buffer, see LOG_FLUSH_PAUSE. */
static volatile bool l_flgLogFlushInhibit;

    /* Counter to specify how often the Log Flush LED will flash */
static volatile uint8_t l_LogFlushLED_FlashCnt;

    /* File handle for log file */
static FIL	l_fh;

    /* Timer handle for the log buffer flushing control */
static TIM_HDL	l_thLogFlushCtrl = NONE;

#if LOG_ALIVE_INTERVAL > 0
    /* Timer handle for the alive interval */
static TIM_HDL	l_thLogAliveIntvl = NONE;
#endif

/*=========================== Forward Declarations ===========================*/

static void	logMsg(const char *prefix, const char *frmt, va_list args);
static void	logFlushLED(void);
static void	logFlushCtrl(TIM_HDL hdl);
#if LOG_ALIVE_INTERVAL > 0
static void	logAliveMsg(TIM_HDL hdl);
#endif


/***************************************************************************//**
 *
 * @brief	Initialize the Logging Facility
 *
 * This routine must be called once to initialize the logging facility.
 * It sets up the log buffer and allocates timers.  Disk access is not done
 * here because it is not available at this early point, see function
 * @ref LogFlushCheck() for this.
 *
 ******************************************************************************/
void	 LogInit (void)
{
    /* initialize indices */
    idxLogGet = idxLogPut = 0;

    /* Get a timer handle for the log sample timeout */
    if (l_thLogFlushCtrl == NONE)
	l_thLogFlushCtrl = sTimerCreate (logFlushCtrl);

#if LOG_ALIVE_INTERVAL > 0
    /* Get a timer handle for the log alive interval */
    if (l_thLogAliveIntvl == NONE)
    {
	l_thLogAliveIntvl = sTimerCreate (logAliveMsg);
	if (l_thLogAliveIntvl != NONE)
	    sTimerStart (l_thLogAliveIntvl, LOG_ALIVE_INTERVAL);
    }
#endif
}


/***************************************************************************//**
 *
 * @brief	Open Log File
 *
 * This routine (re-)opens the log file for writing.
 *
 * @param[in] filepattern
 *	Filename to compare all file entries in the root directory of the disk
 *	with.  The filename must follow the DOS 8.3 notation, i.e. 8 characters
 *	for the basename and 3 characters extension, separated by a dot.
 *	An asterisk (*) at the end of the basename or/and extension is treated
 *	as wildcard, the further characters will not be compared.
 *
 * @param[in] filename
 *	Fall-back filename to use if no appropriate file pattern could be
 *	found on the disk.
 *
 ******************************************************************************/
void	 LogFileOpen (char *filepattern, char *filename)
{
FRESULT	 res;		// FatFs function common result code
char	*pStr;		// string pointer


    /* Parameter Check */
    EFM_ASSERT(filename != NULL);

    if (filepattern != NULL)
    {
	/* Find filename with specified pattern on the SD-Card */
	pStr = FindFile ("/", filepattern);
	if (pStr != NULL)
	    filename = pStr;	// found pattern on disk
    }

    /* Log filename change */
    if (g_LogFilename[0] != EOS)
	Log ("Media Change: %s -> %s", g_LogFilename, filename);
    else
	Log ("Using Filename %s", filename);

    strcpy (g_LogFilename, filename);

    /* Discard old file handle, open new file */
    res = f_open (&l_fh, filename,  FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
    if (res == FR_OK)
    {
	res = f_lseek (&l_fh, f_size(&l_fh));
    }

    if (res != FR_OK)
    {
	LogError ("LogFileOpen: Error Code %d", res);
	l_fh.fs = NULL;		// invalidate file handle
    }
    else
    {
	l_ErrMsgCnt = 2;
    }

    /* Power off the SD-Card Interface */
    MICROSD_PowerOff();
}


/***************************************************************************//**
 *
 * @brief	Log a Message
 *
 * This routine writes a log message into the buffer.  It may be called from
 * interrupt context.
 *
 * The format of a log message is:
 * 20151231-235900 \<message\>
 *
 ******************************************************************************/
void	 Log (const char *frmt, ...)
{
va_list	 args;


    /* disable interrupts to prevent interfering of other logs */
    INT_Disable();

    /* build variable argument list and call logMsg() */
    va_start(args, frmt);
    logMsg (NULL, frmt, args);
    va_end(args);

    /* enable interrupts again */
    INT_Enable();
}


/***************************************************************************//**
 *
 * @brief	Log an Error Message
 *
 * This routine writes an error log message into the buffer.  It may be called
 * from interrupt context.
 *
 * The format of an error log message is:
 * 20151231-235900 ERROR \<message\>
 *
 ******************************************************************************/
void	 LogError (const char *frmt, ...)
{
va_list	 args;


    /* disable interrupts to prevent interfering of other logs */
    INT_Disable();

    /* build variable argument list and call logMsg() */
    va_start(args, frmt);
    logMsg ("ERROR ", frmt, args);
    va_end(args);

    /* enable interrupts again */
    INT_Enable();
}


/***************************************************************************//**
 *
 * @brief	Flush Log Buffer
 *
 * This routine flushes the log buffer, i.e. its contents is written to disk.
 *
 ******************************************************************************/
void	 LogFlush (void)
{
FRESULT	 res;		// FatFs function common result code
int	 cnt;
UINT	 bytesWr;


    /* See if Log File is open */
    if (IsFileHandleValid(&l_fh) == false)
	return;			// no file open or invalid file handle

    /* Switch the SD-Card Interface on */
    MICROSD_PowerOn();

    /* Re-Initialize disk (mount is still the same!) */
    if (disk_initialize(0) != 0)
    {
	if (--l_ErrMsgCnt >= 0)
	    LogError ("LogFlush: Init Failed");
    }
    else
    {
	/* Write all log messages to disk */
	while (idxLogGet != idxLogPut)
	{
	    cnt = l_LogBuf[idxLogGet];	// get string length
	    if (cnt == 0)
	    {
		idxLogGet = 0;		// length of 0 indicates wrap-around
		cnt = l_LogBuf[idxLogGet];
	    }

	    /* write string to file without the terminating 0 (EOS) */
	    res = f_write (&l_fh, l_LogBuf + idxLogGet + 1, cnt, &bytesWr);
	    if (res != FR_OK)
	    {
		if (--l_ErrMsgCnt >= 0)
		    LogError ("LogFlush: Error Code %d", res);
		break;
	    }

	    if (bytesWr < cnt)
	    {
		if (--l_ErrMsgCnt >= 0)
		    LogError ("LogFlush: Disk Full");
		res = FR_DISK_ERR;
		break;
	    }

	    /* update index, consider <len> byte and EOS */
	    idxLogGet += (cnt + 2);
	}

	/* Synchronize file system */
	if (res == FR_OK)
	    f_sync (&l_fh);
    }

    /* Switch the SD-Card Interface off */
    MICROSD_PowerOff();

    /* Signal that Log Flushing is done by flashing the LED */
    l_LogFlushLED_FlashCnt = LOG_FLASH_LED_CNT;
    LOG_FLUSH_LED = 1;			// switch LED on
    msTimerAction (logFlushLED);
    msTimerStart (LOG_FLASH_LED_DELAY);	// flashing delay/frequency

    /* Start timer to handle log flushing pause */
    if (l_thLogFlushCtrl != NONE)
	sTimerStart (l_thLogFlushCtrl, LOG_FLUSH_PAUSE);

    /* Inhibit flushing the log buffer for that time */
    l_flgLogFlushInhibit = true;
    l_flgLogFlushTrigger = false;
}


/***************************************************************************//**
 *
 * @brief	Check if Log Buffer should be Flushed
 *
 * This routine is periodically called from the main loop to check if the log
 * buffer should be flushed, i.e. l_flgLogFlushTrigger is set, or more than
 * @ref LOG_SAMPLE_MAX_SIZE bytes have been stored in the log buffer.
 *
 ******************************************************************************/
void	 LogFlushCheck (void)
{
int	 cnt;			// allocated space in the log buffer


    cnt = idxLogPut - idxLogGet;	// calculate allocated space
    if (cnt < 0)
	cnt += LOG_BUF_SIZE;		// wrap around

    if (cnt > LOG_SAMPLE_MAX_SIZE	// always flush if threshold is reached
    ||  (l_flgLogFlushTrigger  &&  ! l_flgLogFlushInhibit))
    {
	l_flgLogFlushTrigger = false;

	LogFlush();
    }
}


/***************************************************************************//**
 *
 * @brief	Log Message
 *
 * This routine writes the current time stamp, an optional prefix, and the
 * specified log message into the buffer.
 *
 * The format of a log message is:
 * 20151231-235900 \<prefix\> \<message\>
 *
 ******************************************************************************/
static void	logMsg(const char *prefix, const char *frmt, va_list args)
{
char	 tmpBuffer[LOG_ENTRY_MAX_SIZE];	// use this if the log buffer is full
char	*pBuf;				// pointer to the buffer to use
int	 cnt, num;			// message length, available space
struct tm    time;			// current time (hh:mm:ss)
unsigned int ms;			// current [ms]


    /* Start timer to handle sample timeout */
    if (l_flgLogFlushInhibit)
	l_flgLogFlushTrigger = true;	// set flag for later
    else if (l_thLogFlushCtrl != NONE)
	sTimerStart (l_thLogFlushCtrl, LOG_SAMPLE_TIMEOUT);

    /* Check if there is enough space in the log buffer */
    num = LOG_BUF_SIZE - idxLogPut;	// distance to end of buffer
    if (num > LOG_ENTRY_MAX_SIZE)
	num = 0;			// enough space, no additional memory

    cnt = idxLogPut + num - idxLogGet;	// calculate allocated space
    if (cnt < 0)
	cnt += LOG_BUF_SIZE;		// wrap around

    cnt = LOG_BUF_SIZE - cnt - 1;	// calculate free space

    if (cnt < LOG_ENTRY_MAX_SIZE)
    {
	/* Not enough space in buffer - skip entry and count as "lost" */
	l_LostEntryCnt++;

#ifdef LOG_MONITOR_FUNCTION
	sprintf (tmpBuffer, "ERROR: Log Buffer Out of Memory"
			    " - lost %ld Messages\n", l_LostEntryCnt);
	LOG_MONITOR_FUNCTION (tmpBuffer);
#endif
	pBuf = tmpBuffer;		// use temporary buffer
    }
    else
    {
	/* There is enough memory in log buffer */
	if (num > 0)
	{
	    l_LogBuf[idxLogPut] = 0;	// mark wrap-around
	    idxLogPut = 0;		// adjust start of new log message
	}

	pBuf = l_LogBuf + idxLogPut;	// use standard log buffer
    }

    /* Reserve one byte for string length information */
    cnt = 1;

    /* Store timestamp */
    ClockGetMilliSec (&time, &ms);

    if (time.tm_year != 0)
    {
	cnt += sprintf (pBuf + cnt,
			"20%02d%02d%02d-%02d%02d%02d.%03d ",
			time.tm_year,
			time.tm_mon + 1,
			time.tm_mday,
			time.tm_hour,
			time.tm_min,
			time.tm_sec,
			ms);
    }
    else
    {
	strcpy (pBuf + cnt, "00000000-000000.000 ");
	cnt += 20;
    }

    /* Store optional prefix */
    if (prefix != NULL)
    {
	if (*prefix != EOS)
	{
	    strcpy (pBuf + cnt, prefix);
	    cnt += strlen(prefix);
	}
    }

    /* Build and store the log message */
    cnt += vsprintf(pBuf + cnt, frmt, args);

    /* add <CR><LF> */
    strcpy (pBuf + cnt, "\r\n");
    cnt += 3;			// <CR> <LF> EOS

    /* Immediately send the complete log message to the monitor output */
#ifdef LOG_MONITOR_FUNCTION
    LOG_MONITOR_FUNCTION (pBuf + 1);
#endif

    /* Check length */
    EFM_ASSERT(cnt <= LOG_ENTRY_MAX_SIZE);

    /* If used, update standard log buffer */
    if (pBuf != tmpBuffer)
    {
	/* Store string length */
	l_LogBuf[idxLogPut] = cnt - 2;	// no <len> byte, no EOS

	/* Update index */
	idxLogPut += cnt;
    }
}


/***************************************************************************//**
 *
 * @brief	Log Flushing Control
 *
 * This routine is used for two tasks:
 * # As retriggerable monoflop which is (re-)started whenever a new log
 *   message is generated.  It waits the amount of time specified by
 *   @ref LOG_SAMPLE_TIMEOUT before flushing the log buffer to the SD-Card.
 *   This allows a sequence of log messages to be written at once.
 * # For the pause between two log buffer flushes, as specified by
 *   @ref LOG_FLUSH_PAUSE.  This ensures that the user has the time to remove
 *   (exchange) the SD-Card in a save way.
 *
 ******************************************************************************/
static void	logFlushCtrl(TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    /* See if log flush pause (inhibit), or log flush sample timeout */
    if (l_flgLogFlushInhibit)
	l_flgLogFlushInhibit = false;	// reset inhibit flag
    else
	l_flgLogFlushTrigger = true;

    g_flgIRQ = true;	// keep on running
}


#if LOG_ALIVE_INTERVAL > 0
/***************************************************************************//**
 *
 * @brief	Log Alive Message
 *
 * This routine is called when the Log Alive interval is over, see
 * @ref LOG_ALIVE_INTERVAL.
 *
 ******************************************************************************/
static void	logAliveMsg(TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    /* Restart the timer */
    if (l_thLogAliveIntvl != NONE)
	sTimerStart (l_thLogAliveIntvl, LOG_ALIVE_INTERVAL);

    /* Write Alive Message */
    Log ("Alive");
}
#endif


/***************************************************************************//**
 *
 * @brief	Log Flush LED
 *
 * This routine controls the Log Flush LED.  The LED flashes for a short
 * time to indicate that the log buffer has been flushed now and will not
 * be flushed for the next @ref LOG_FLUSH_PAUSE seconds.  This gives the
 * user the ability to remove the SD-Card in a save way.
 *
 ******************************************************************************/
static void	logFlushLED(void)
{
    /* Start condition is LED on */
    if (LOG_FLUSH_LED)
    {
	LOG_FLUSH_LED = 0;
	l_LogFlushLED_FlashCnt--;
    }
    else
    {
	LOG_FLUSH_LED = 1;
    }

    if (l_LogFlushLED_FlashCnt > 0)
	msTimerStart(LOG_FLASH_LED_DELAY);
}
