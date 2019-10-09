/***************************************************************************//**
 * @file
 * @brief	Driver for the SD-Card interface
 * @author	Silicon Labs
 * @author	Ralf Gerhauser
 * @version	2015-03-30
 *
 * This is the driver for the SD-Card interface.  It provides all required
 * board-specific functionality to access an SD-Card via SPI.
 * Furthermore this file contains routines for high-level access to the
 * file system on the SD-Card.
 *
 * For a separate documentation of the FAT file system, see
 * <a href="../../fatfs/doc/00index_e.html">FAT File System Module</a>.
 *
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ****************************************************************************//*
Revision History:
2016-09-27,rage	Use INT_En/Disable() instead of __en/disable_irq().
2016-04-05,rage	Made local variables of type "volatile".
2016-02-21,rage	Added IsDiskRemoved() to query CF-Card removal.
2015-03-30,rage	Implemented DiskInit(), DiskInfo(), and FileFind().
2015-02-18,rage	Adapted board-specific parts for SNB_Heaven.
*/

/*=============================== Header Files ===============================*/

#include <string.h>
#include "em_cmu.h"
#include "em_int.h"
#include "em_usart.h"
#include "microsd.h"
#include "AlarmClock.h"
#include "Display.h"
#include "Logging.h"

/*=============================== Definitions ================================*/

    /*!@brief Display duration in seconds to show info on LCD */
#define DISP_DUR		10

/*================================== Macros ==================================*/

#ifndef LOGGING		// define as empty, if logging is not enabled
    #define Log(str)
    #define LogError(str)
#endif

    /*!@brief Set level of the power enable pin. */
#define SET_MICROSD_PWR_PIN(level)  IO_Bit(GPIO->P[MICROSD_PWR_GPIO_PORT].DOUT, \
					   MICROSD_PWR_PIN) = (level)

    /*!@brief Check CD-Pin if Disk (SD-Card) is present. */
#define IS_DISK_PRESENT		(IO_Bit(GPIO->P[MICROSD_SPI_GPIO_PORT].DIN, \
					MICROSD_CD_PIN) == 0)

/*=========================== Typedefs and Structs ===========================*/

/*!@brief Enumeration of Disk States
 *
 * This is the list of Disk States.  They are used by the state machine in
 * DiskCheck().
 */
typedef enum
{
    DS_UNKNOWN,		//!< Unknown State (after Power-Up or Reset)
    DS_INSERTED,	//!< An SD-Card has been inserted
    DS_REMOVED,		//!< The SD-Card has been removed
    DS_INITIALIZED,	//!< The SD-Card is initialized
    DS_MOUNTED,		//!< The File System on the SD-Card has been mounted
    DS_MOUNT_FAILED,	//!< Mounting the File System failed
    END_DISK_STATE
} DISK_STATE;

/*================================ Local Data ================================*/

static volatile uint32_t timeOut, xfersPrMsec;
static FATFS		 l_FatFS;
static DISK_STATE l_DiskState = DS_UNKNOWN;
static DISK_STATE l_PrevDiskState;


//==============================================================================
//
//		H I G H - L E V E L   R O U T I N E S
//
//==============================================================================

/***************************************************************************//**
 *
 * @brief	Initialize SD-Card Interface
 *
 * This routine must be called once to initialize this module.
 *
 ******************************************************************************/
void	 DiskInit (void)
{
    /* Initialize the SPI peripheral and GPIOs for microSD card usage */
    MICROSD_Init();
}


/***************************************************************************//**
 *
 * @brief	Disk Check
 *
 * This routine checks if an SD-Card has been inserted or removed.  This is
 * done in polling mode by switching on an external low impedance pull-up
 * resistor for the Card-Detect (CD) pin and reading the current state of
 * this signal.  It then takes the appropriate action to mount or invalidate
 * the file system of the media.  Finally the pull-up resistor is switched
 * off again for power saving reasons.
 *
 * @return
 *	Disk state: <b>true</b> if a new file system has been mounted,
 *	<b>false</b> otherwise.
 *
 * @note
 * 	This function may be called from standard program, usually the loop
 * 	in module "main.c" - it must not be called from interrupt routines!
 *
 ******************************************************************************/
bool	 DiskCheck (void)
{
bool	 state = false;


    /* Enable Card Detect (CD) Pin with Pull-Up */
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_CD_PULLUP_PIN,
                    gpioModePushPull, 1);

    /* Save current state for next time */
    l_PrevDiskState = l_DiskState;

    /* Always check for card removal */
    if (! IS_DISK_PRESENT)
    {
	l_DiskState = DS_REMOVED;
    }

    /*
     * Proceed the current Disk State
     */
    switch (l_DiskState)
    {
	case DS_REMOVED:	// SD-Card has been removed
	    /* Check if state is new or unchanged */
	    if (l_DiskState != l_PrevDiskState)
	    {
		/* State is new, the SD-Card interface must be shut down */
		Log ("SD-Card Removed");
		DisplayText (2, "SD-Card Removed");
		DisplayNext (DISP_DUR, NULL, 0);

		/* Invalidate current File System */
		l_FatFS.fs_type = 0;
		disk_ioctl(0, CTRL_INVALIDATE, NULL);

		/* Shut Down and Power Off the SD-Card */
		MICROSD_Deinit();
	    }
	    /* no break */

	case DS_UNKNOWN:	// Unknown state after power-up or reset
	    /* Check for card insertion */
	    if (IS_DISK_PRESENT)
	    {
		l_DiskState = DS_INSERTED;
	    }
	    else
	    {
		break;		// still no disk present, leave switch()
	    }
	    /* no break */

	case DS_INSERTED:	// SD-Card has been inserted
	    /* Check if state is new or unchanged */
	    if (l_DiskState != l_PrevDiskState)
	    {
		/* State is new, the SD-Card interface must be set up */
		Log ("SD-Card Inserted");
		DisplayText (2, "SD-Card Inserted");
		DisplayNext (DISP_DUR, NULL, 0);
		MICROSD_Init();
	    }
	    /* SD-Card is present, try to initialize it */
	    if (disk_initialize(0) == 0)
	    {
		l_DiskState = DS_INITIALIZED;
		Log ("SD-Card Initialized");
	    }
	    else
	    {
		break;	// initialization still fails, leave switch()
	    }
	    /* no break */

	case DS_INITIALIZED:	// The SD-Card is initialized
	    /* Try mounting the File System on the SD-Card */
	    if (f_mount(0, &l_FatFS) == FR_OK)
	    {
	    uint32_t	sizeMB;

		l_DiskState = DS_MOUNTED;
		Log ("SD-Card File System mounted");
		state = true;	// Inform caller about the new mount

		/* Log Disk Size and display it on the LCD */
		sizeMB = DiskSize();
		if (sizeMB > 0)
		{
		    Log ("SD-Card %ldMB free", sizeMB);
		    DisplayText (2, "SD: %ldMB free", sizeMB);
		    DisplayNext (DISP_DUR, NULL, 0);
		}
	    }
	    else
	    {
		l_DiskState = DS_MOUNT_FAILED;
		LogError ("SD-Card Mount Failed");
		DisplayText (2, "SD: Mount Failed");
		DisplayNext (DISP_DUR, NULL, 0);
	    }
	    break;

	case DS_MOUNTED:	// File System on the SD-Card has been mounted
	    /* Remain in this state until card removal */
	    break;

	case DS_MOUNT_FAILED:	// Mounting the File System failed
	    /* Remain in this state until card removal */
	    break;

	default:		// Invalid state code
	    EFM_ASSERT(false);
	    l_DiskState = DS_UNKNOWN;	// try to recover
    }

    /* See if Disk State has changed */
    if (l_DiskState != l_PrevDiskState)
	g_flgIRQ = true;		// immediately process the new state

    /* Disable Card Detect (CD) Pin again */
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_CD_PULLUP_PIN,
                    gpioModeDisabled, 0);

    return state;
}


/***************************************************************************//**
 *
 * @brief	Is Disk Removed
 *
 * This routine returns <b>true</b> if no SD-Card is present.
 *
 * @return
 *	State of the SD-Card, <b>true</b> if none is present, <b>false</b>
 *	otherwise.
 *
 ******************************************************************************/
bool	 IsDiskRemoved (void)
{
    return l_DiskState == DS_REMOVED ? true : false;
}


/***************************************************************************//**
 *
 * @brief	Is File Handle Valid
 *
 * This routine verifies if the specified file handle is valid, i.e. if it
 * refers to a valid @ref FATFS structure.
 *
 * @param[in] pHdl
 *	Pointer to file handle structure.
 *
 * @return
 *	State of the file handle, <b>true</b> if valid, <b>false</b> if not.
 *
 ******************************************************************************/
bool	 IsFileHandleValid (FIL *pHdl)
{
    if (pHdl == NULL)
	return false;

    if (pHdl->fs == NULL)
	return false;

    if (pHdl->fs->fs_type == 0)
	return false;

    return true;
}


/***************************************************************************//**
 *
 * @brief	Available Disk Size in MB
 *
 * This routine returns the free disk space in megabyte.
 *
 * @return
 *	Free disk space in MB.
 *
 ******************************************************************************/
uint32_t	 DiskSize (void)
{
DWORD	 clustCnt;		// Number of available cluster
FATFS	*pFAT;			// Pointer to FAT structure currently in use


    /* Get free space of the whole disk */
    if (f_getfree("/", &clustCnt, &pFAT) == FR_OK)
    {
	/*
	 * Calculate free space in MB.
	 * DIV by 2 to get KB, MUL <clustCnt> DIV 1024 to get MB.
	 */
	return (clustCnt / 2 * pFAT->csize / 1024);
    }

    return 0;
}


/***************************************************************************//**
 *
 * @brief	Find File
 *
 * This function compares all filenames in the specified @p dirpath with
 * the filename pattern specified by parameter @p filepattern.
 *
 * @param[in] dirpath
 *	Directory path to read filenames from.
 *
 * @param[in] filepattern
 *	Filename to compare all file entries of the specified directory path
 *	with.  The filename must follow the DOS 8.3 notation, i.e. 8 characters
 *	for the basename and 3 characters extension, separated by a dot.
 *	An asterisk (*) at the end of the basename or/and extension is treated
 *	as wildcard, the further characters will not be compared.
 *
 * @return
 *	String pointer to a static piece of memory where the matching filename
 *	is stored, or NULL if file was not found.
 *
 * @warning
 * 	All parameters must be specified in upper case characters!
 *
 ******************************************************************************/
char	*FindFile (char *dirpath, char *filepattern)
{
static char filefound[13];
DIR	 dir;		// File objects
FILINFO	 fileinfo;	// File info object
int	 i, j;


    /* check parameters */
    EFM_ASSERT (dirpath != NULL);
    EFM_ASSERT (filepattern != NULL);

    /* open the specified directory */
    if (f_opendir(&dir, dirpath) != FR_OK)
	return NULL;	// abort on error

    /* read directory contents name by name */
    while (1)
    {
	if (f_readdir(&dir, &fileinfo) != FR_OK)
	    return NULL;	// abort on error

	if (fileinfo.fname[0] == EOS)
	    return NULL;	// no  more files in current directory

	if (fileinfo.fattrib & (AM_DIR | AM_VOL | AM_SYS))
	    continue;	// ignore subdirectories, volume labels and system files

	/* compare basename */
	for (i=j=0;  (i < 8)  &&  fileinfo.fname[i] != '.'
			      &&  fileinfo.fname[i] != EOS;  i++, j++)
	{
	    if (filepattern[j] == '*')
		break;	// wildcard - ignore the rest of the basename

	    if (fileinfo.fname[i] != filepattern[j])
		break;	// not equal - file does not match
	}

	/* check for wildcard */
	if (filepattern[j] == '*')		// "base*[.ext]"
	{
	    j++;			// skip '*'

	    /* skip the rest of the basename */
	    for ( ;  (i < 8)  &&  fileinfo.fname[i] != '.'
			      &&  fileinfo.fname[i] != EOS;  i++)
		;
	}

	/*
	 * Basenames are equal:
	 * a) "basename"  == "basename"
	 * b) "basename"  == "base*"
	 * c) "basename." == "basename."
	 * d) "basename." == "base*."
	 */
	if (fileinfo.fname[i] != filepattern[j])
	    continue;	// file does not match, try the next file

	/* check for extension */
	if (fileinfo.fname[i] == EOS)
	    break;	// no extension - filename does match

	/* verify if a dot follows the basename */
	EFM_ASSERT (filepattern[j] == '.');	// dot must follow
	if (filepattern[j] != '.')
	    return NULL;			// abort on error

	/* skip dot, compare extension */
	for (i++, j++;  fileinfo.fname[i] != EOS;  i++, j++)
	{
	    if (filepattern[j] == '*')
		break;	// wildcard - ignore the rest of the extension

	    if (fileinfo.fname[i] != filepattern[j])
		break;	// not equal - file does not match
	}

	/* check for wildcard */
	if (filepattern[j] == '*')
	    break;	// wildcard - filename does match

	if (fileinfo.fname[i] != filepattern[j])
	    continue;	// file does not match, try the next file

	EFM_ASSERT (filepattern[j] == EOS);	// EOS must follow
	if (filepattern[j] != EOS)
	    return NULL;			// abort on error

	break;		// complete match - leave the loop
    }

    /* file name does match */
    strcpy (filefound, fileinfo.fname);
    return filefound;
}


//==============================================================================
//
//	H E R E   F O L L O W S   T H E   S I L A B S   C O D E
//
//==============================================================================

/***************************************************************************//**
 *
 * @brief	Initialize SD-Card Driver
 *
 * This routine initializes the board-specific hardware for the SD-Card
 * interface, i.e. GPIOs for Power-Enable and the SPI interface.
 *
 ******************************************************************************/
void MICROSD_Init(void)
{
USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

    /* Enabling clock to USART and GPIO */
    CMU_ClockEnable(MICROSD_CMUCLOCK, true);
    CMU_ClockEnable(cmuClock_GPIO, true);

    /* Initialize USART in SPI master mode. */
    xfersPrMsec   = MICROSD_LO_SPI_FREQ / 8000;
    init.baudrate = MICROSD_LO_SPI_FREQ;
    init.msbf     = true;	// Most Significant Bit First
    USART_InitSync(MICROSD_USART, &init);

    /* Enabling pins and setting location, SPI CS not enabled */
    MICROSD_USART->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN |
			   USART_ROUTE_CLKPEN | MICROSD_LOC;

#if defined( USART_CTRL_SMSDELAY )
    /* This will allow us to use higher baudrate. */
    MICROSD_USART->CTRL |= USART_CTRL_SMSDELAY;
#endif

    /* Configure Power Enable Pin for SD-Card interface (still OFF) */
    GPIO_PinModeSet (MICROSD_PWR_GPIO_PORT, MICROSD_PWR_PIN,
		     gpioModePushPull, MICROSD_PWR_OFF);

    /* IO configuration of the SPI */
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_SPI_MOSI_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_SPI_MISO_PIN, gpioModeInputPull, 1);
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_SPI_CS_PIN,   gpioModePushPull, 1);
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_SPI_CLK_PIN,  gpioModePushPull, 0);

   /*
    * Configure the Card-Detect (CD) pin as pure input.  There is an external
    * low impedance pull-up resistor which is switched on for 16us to check
    * the current state of the SD-Card socket (card removed or inserted).
    * This is done in polling mode by function DiskCheck().
    */
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_CD_PIN, gpioModeInput, 0);
}


/**************************************************************************//**
 * @brief
 *  Deinitialize SPI peripheral.
 *  Turn off the SPI peripheral and disable SPI GPIO pins.
 *****************************************************************************/
void MICROSD_Deinit(void)
{
    MICROSD_PowerOff();

    USART_Reset(MICROSD_USART);
}


/**************************************************************************//**
 * @brief Wait for micro SD card ready.
 * @return 0xff: micro SD card ready, other value: micro SD card not ready.
 *****************************************************************************/
static uint8_t WaitReady(void)
{
uint8_t res;
uint32_t retryCount;

    /* Wait for ready in timeout of 500ms */
    retryCount = 500 * xfersPrMsec;
    do
	res = MICROSD_XferSpi(0xff);
    while ((res != 0xFF) && --retryCount);

    return res;
}
/** @endcond */


/**************************************************************************//**
 * @brief
 *  Do one SPI transfer.
 *
 * @param data
 *  Byte to transmit.
 *
 * @return
 *  Byte received.
 *****************************************************************************/
uint8_t MICROSD_XferSpi(uint8_t data)
{
    if ( timeOut )
    {
	timeOut--;
    }

    return USART_SpiTransfer(MICROSD_USART, data);
}


/**************************************************************************//**
 * @brief Deselect the micro SD card and release the SPI bus.
 *****************************************************************************/
void MICROSD_Deselect(void)
{
    GPIO->P[ MICROSD_SPI_GPIO_PORT ].DOUTSET = 1 << MICROSD_SPI_CS_PIN; /* CS pin high. */
    MICROSD_XferSpi(0xff);
}


/**************************************************************************//**
 * @brief Select the micro SD card and wait for the card to become ready.
 * @return 1:Successful, 0:Timeout.
 *****************************************************************************/
int MICROSD_Select(void)
{
    GPIO->P[ MICROSD_SPI_GPIO_PORT ].DOUTCLR = 1 << MICROSD_SPI_CS_PIN; /* CS pin low. */
    if (WaitReady() != 0xFF)
    {
	MICROSD_Deselect();
	return 0;
    }
    return 1;
}


/**************************************************************************//**
 * @brief Turn on micro SD card power.
 *****************************************************************************/
void MICROSD_PowerOn(void)
{
    /* Enable SD-Card power */
    SET_MICROSD_PWR_PIN(MICROSD_PWR_ON);

    /* Enable SPI clock */
    CMU_ClockEnable(MICROSD_CMUCLOCK, true);

    /* IO configuration of the SPI */
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_SPI_MOSI_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_SPI_MISO_PIN, gpioModeInputPull, 1);
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_SPI_CS_PIN,   gpioModePushPull, 1);
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_SPI_CLK_PIN,  gpioModePushPull, 0);
}


/**************************************************************************//**
 * @brief Turn off micro SD card power.
 *****************************************************************************/
void MICROSD_PowerOff(void)
{
    /* Wait for micro SD card ready */
    MICROSD_Select();
    MICROSD_Deselect();    /* Wait for micro SD card ready */
    MICROSD_Select();
    MICROSD_Deselect();

    /* Disable SPI clock */
    CMU_ClockEnable(MICROSD_CMUCLOCK, false);

    /* Reset IO configuration - except the CD pin*/
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_SPI_MOSI_PIN, gpioModeDisabled, 0);
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_SPI_MISO_PIN, gpioModeDisabled, 0);
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_SPI_CS_PIN,   gpioModeDisabled, 0);
    GPIO_PinModeSet(MICROSD_SPI_GPIO_PORT, MICROSD_SPI_CLK_PIN,  gpioModeDisabled, 0);

    /* Disable SD-Card power */
    SET_MICROSD_PWR_PIN(MICROSD_PWR_OFF);
}


/**************************************************************************//**
 * @brief Receive a data block from micro SD card.
 * @param[out] buff
 *  Data buffer to store received data.
 * @param btr
 *  Byte count (must be multiple of 4).
 * @return
 *  1:OK, 0:Failed.
 *****************************************************************************/
int MICROSD_BlockRx(uint8_t *buff, uint32_t btr)
{
uint8_t token;
uint16_t val;
uint32_t retryCount, framectrl, ctrl;


    /* Wait for data packet in timeout of 100ms */
    retryCount = 100 * xfersPrMsec;
    do
    {
	token = MICROSD_XferSpi(0xff);
    } while ((token == 0xFF) && --retryCount);

    if (token != 0xFE)
    {
	/* Invalid data token */
	return 0;
    }

    /* Save current configuration. */
    framectrl = MICROSD_USART->FRAME;
    ctrl      = MICROSD_USART->CTRL;

    /* Set frame length to 16 bit. This will increase the effective data rate. */
    MICROSD_USART->FRAME = (MICROSD_USART->FRAME & (~_USART_FRAME_DATABITS_MASK))
			 | USART_FRAME_DATABITS_SIXTEEN;
    MICROSD_USART->CTRL |= USART_CTRL_BYTESWAP;

    /* Clear send and receive buffers. */
    MICROSD_USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;

    if ( timeOut >= btr + 2 )
    {
	timeOut -= btr + 2;
    }
    else
    {
	timeOut = 0;
    }

    /* Pipelining - The USART has two buffers of 16 bit in both
    * directions. Make sure that at least one is in the pipe at all
    * times to maximize throughput. */
    MICROSD_USART->TXDOUBLE = 0xffff;
    do
    {
	MICROSD_USART->TXDOUBLE = 0xffff;

	while (!(MICROSD_USART->STATUS & USART_STATUS_RXDATAV))
	    ;

	val = MICROSD_USART->RXDOUBLE;
	*buff++ = val;
	*buff++ = val >> 8;

	btr -= 2;
    } while (btr);

    /* Next two bytes is the CRC which we discard. */
    while (!(MICROSD_USART->STATUS & USART_STATUS_RXDATAV));
    MICROSD_USART->RXDOUBLE;

    /* Restore old settings. */
    MICROSD_USART->FRAME = framectrl;
    MICROSD_USART->CTRL  = ctrl;

    return 1;     /* Return with success */
}


/**************************************************************************//**
 * @brief Send a data block to micro SD card.
 * @param[in] buff 512 bytes data block to be transmitted.
 * @param token Data token.
 * @return 1:OK, 0:Failed.
 *****************************************************************************/
#if _READONLY == 0
int MICROSD_BlockTx(const uint8_t *buff, uint8_t token)
{
uint8_t resp;
uint16_t val;
uint32_t bc = 512;
uint32_t framectrl, ctrl;


    if (WaitReady() != 0xFF)
    {
	return 0;
    }

    MICROSD_XferSpi(token);         /* Xmit a token */

    if (token == 0xFD)
    {
	/* StopTran token */
	return 1;
    }

    /* Save current configuration. */
    framectrl = MICROSD_USART->FRAME;
    ctrl      = MICROSD_USART->CTRL;

    /* Set frame length to 16 bit. This will increase the effective data rate. */
    MICROSD_USART->FRAME = (MICROSD_USART->FRAME & (~_USART_FRAME_DATABITS_MASK))
			 | USART_FRAME_DATABITS_SIXTEEN;
    MICROSD_USART->CTRL |= USART_CTRL_BYTESWAP;

    /* Clear send and receive buffers. */
    MICROSD_USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;

    if ( timeOut >= bc + 2 )
    {
	timeOut -= bc + 2;
    }
    else
    {
	timeOut = 0;
    }

    do
    {
	/* Transmit a 512 byte data block to the SD-Card. */
	val  = *buff++;
	val |= *buff++ << 8;
	bc  -= 2;

	while (!(MICROSD_USART->STATUS & USART_STATUS_TXBL))
	    ;

	MICROSD_USART->TXDOUBLE = val;
    } while (bc);

    while (!(MICROSD_USART->STATUS & USART_STATUS_TXBL));

    /* Transmit two dummy CRC bytes. */
    MICROSD_USART->TXDOUBLE = 0xFFFF;

    while (!(MICROSD_USART->STATUS & USART_STATUS_TXC));

    /* Clear send and receive buffers. */
    MICROSD_USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;

    /* Restore old settings. */
    MICROSD_USART->FRAME = framectrl;
    MICROSD_USART->CTRL  = ctrl;

    resp = MICROSD_XferSpi(0xff); /* Receive a data response */

    if ((resp & 0x1F) != 0x05)    /* If not accepted, return with error */
    {
	return 0;
    }

    return 1;
}
#endif  /* _READONLY */


/**************************************************************************//**
 * @brief
 *  Send a command packet to micro SD card.
 * @param[in] cmd
 *  Command byte.
 * @param[in] arg
 *  Argument.
 * @return
 *  Response value.
 *****************************************************************************/
uint8_t MICROSD_SendCmd(uint8_t cmd, DWORD arg)
{
uint8_t  n, res;
uint32_t retryCount;


    if (cmd & 0x80)
    {
	/* ACMD<n> is the command sequense of CMD55-CMD<n> */
	cmd &= 0x7F;
	res  = MICROSD_SendCmd(CMD55, 0);
	if (res > 1)
	{
	    return res;
	}
    }

    /* Select the card and wait for ready */
    MICROSD_Deselect();
    if (!MICROSD_Select())
    {
	return 0xFF;
    }

    /* Send command packet */
    MICROSD_XferSpi(0x40 | cmd);            /* Start + Command index */
    MICROSD_XferSpi((uint8_t)(arg >> 24));  /* Argument[31..24] */
    MICROSD_XferSpi((uint8_t)(arg >> 16));  /* Argument[23..16] */
    MICROSD_XferSpi((uint8_t)(arg >> 8));   /* Argument[15..8] */
    MICROSD_XferSpi((uint8_t) arg);         /* Argument[7..0] */
    n = 0x01;                               /* Dummy CRC + Stop */
    if (cmd == CMD0)
    {
	n = 0x95;                             /* Valid CRC for CMD0(0) */
    }
    if (cmd == CMD8)
    {
	n = 0x87;                             /* Valid CRC for CMD8(0x1AA) */
    }
    MICROSD_XferSpi(n);

    /* Receive command response */
    if (cmd == CMD12)
    {
	MICROSD_XferSpi(0xff);                /* Skip a stuff byte when stop reading */
    }
    retryCount = 10;                        /* Wait for a valid response in timeout of 10 attempts */
    do
    {
	res = MICROSD_XferSpi(0xff);
    } while ((res & 0x80) && --retryCount);

    return res;             /* Return with the response value */
}

/**************************************************************************//**
 * @brief Set SPI clock to a low frequency suitable for initial
 *        card initialization.
 *****************************************************************************/
void MICROSD_SpiClkSlow(void)
{
    USART_BaudrateSyncSet(MICROSD_USART, 0, MICROSD_LO_SPI_FREQ);
    xfersPrMsec = MICROSD_LO_SPI_FREQ / 8000;
}


/**************************************************************************//**
 * @brief Set SPI clock to maximum frequency.
 *****************************************************************************/
void MICROSD_SpiClkFast(void)
{
    USART_BaudrateSyncSet(MICROSD_USART, 0, MICROSD_HI_SPI_FREQ);
    xfersPrMsec = MICROSD_HI_SPI_FREQ / 8000;
}


/**************************************************************************//**
 * @brief
 *  Set a timeout value. The timeout value will be decremented towards zero
 *  when SPI traffic to/from the micro SD card takes place. Use @ref
 *  MICROSD_TimeOutElapsed() to check if timeout has elapsed.
 * @param[in] msec
 *  Millisecond timeout value (very approximate).
 *****************************************************************************/
void MICROSD_TimeOutSet(uint32_t msec)
{
    timeOut = xfersPrMsec * msec;
}


/**************************************************************************//**
 * @brief
 *  Check if timeout value set with @ref MICROSD_TimeOutSet() has elapsed.
 * @return
 *  True if timeout has elapsed.
 *****************************************************************************/
bool MICROSD_TimeOutElapsed(void)
{
    return timeOut == 0;
}


/***************************************************************************//**
 * @brief
 *   This function is required by the FAT file system in order to provide
 *   timestamps for created files. Since this example does not include a
 *   reliable clock we hardcode a value here.
 *
 *   Refer to drivers/fatfs/doc/en/fattime.html for the format of this DWORD.
 * @return
 *    A DWORD containing the current time and date as a packed data structure.
 ******************************************************************************/
DWORD get_fattime(void)
{
struct tm CurrDateTime;
DWORD	  fatTimeDate;


    /* be sure that structure is not changed during copy */
    INT_Disable();
    CurrDateTime = g_CurrDateTime;
    INT_Enable();

    /* build FAT time stamp from current time and date */
    fatTimeDate = ((CurrDateTime.tm_year + 2000 - 1980) << 25)
		| ((CurrDateTime.tm_mon + 1) << 21)
		| (CurrDateTime.tm_mday <<  16)
		| (CurrDateTime.tm_hour <<  11)
		| (CurrDateTime.tm_min  <<   5)
		| (CurrDateTime.tm_sec / 2);

    return fatTimeDate;
}
