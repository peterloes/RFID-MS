/***************************************************************************//**
 * @file
 * @brief	SNB Heaven
 * @author	Ralf Gerhauser
 * @version	2016-02-27
 *
 * This application consists of the following modules:
 * - ExtInt.c - External interrupt handler.
 * - Keys.c - Key interrupt handling and translation.
 * - AlarmClock.c - Alarm clock and timers facility.
 * - DCF77.c - DCF77 Atomic Clock Decoder
 * - clock.c - An implementation of the POSIX time() function.
 * - LCD_DOGM162.c - Driver for the DOGM162 LC-Display.
 * - Display.c - Display manager for LCD.
 * - LightBarrier.c - Interrupt logic for the outer and inner light barrier,
 *   enables the RFID reader.
 * - RFID.c - RFID reader to receive transponder IDs.
 * - BatteryMon.c - Battery monitor, periodically reads the state of the
 *   battery via the SMBus.
 * - LEUART.c - The Low-Energy UART can be used as monitoring and debugging
 *   connection to a host computer.
 * - microsd.c - Together with the files "diskio.c" and "ff.c", this module
 *   provides an implementation of a FAT file system on the SD-Card.
 * - Logging.c - Logging facility to send messages to the LEUART and store
 *   them into a file on the SD-Card.
 * - PowerFail.c - Handler to switch off all loads in case of Power Fail.
 *
 * Parts of the code are based on the example code of AN0006 "tickless calender"
 * from Energy Micro AS.
 *
 ***************************************************************************//**
 *
 * Parts are Copyright 2013 Energy Micro AS, http://www.energymicro.com
 *
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ****************************************************************************//*
Revision History:
2020-09-18,rage	Version 1.2:
              - Added support for battery controller TI bq40z50.
                LogBatteryInfo: Removed SBS_ManufacturerData.
                Display SBS_RunTimeToEmpty in days/hours/min.
                Display SBS_SerialNumber as hex value.
              - Corrected decoding of Short-Range(SR) RFID Reader transponder ID
              - Added Power Fail Logic
              - Added Interrupt Priority Settings
2016-04-13,rage	Removed trigger mask from <l_ExtIntCfg>.
2016-02-27,rage	Changed project name to "APRDL".
2016-02-16,rage	Since ExtIntInit() no more enables the external interrupts,
		ExtIntEnableAll() has to be called afterwards.
		Reboot if "*.UPD" files have been found on SD-Card.
2015-05-11,rage	Finished code, completed documentation.
2014-11-11,rage	Initial version.
*/

/*!
 * @mainpage
 * <b>Description</b><br>
 * APRDL (code name SNB_Heaven) is an application to control an intelligent
 * nest box (SNB = Smart Nest Box).  It consists of the following components:
 *
 * <b>Microcontroller</b><br>
 * The heart of the board is an EFM32G230 microcontroller.  It provides two
 * different clock domains: All low-energy peripheral is clocked via a
 * 32.768kHz external XTAL.  The MCU and other high performance peripheral
 * uses a high-frequency clock.  The board can be configured to use the
 * internal RC-oscillator, or an external 32MHz XTAL for that purpose,
 * see define @ref USE_EXT_32MHZ_CLOCK.
 *
 * <b>DCF77 Atomic Clock</b><br>
 * When powered on, the DCF77 hardware module is enabled to receive the time
 * information and set the system clock.  During this phase, the Power-On LED
 * (i.e. LED1) shows the current state of the DCF signal.  The further behavior
 * depends on the settings in the <i>config.h</i> file.  If the define @ref
 * DCF77_ONCE_PER_DAY is 1, the receiver (and also the LED) will be switched
 * off after time has been synchronized.  As the name of the define suggests,
 * it will be switched on once per day.  To properly detect a change between
 * winter and summer time, i.e. <i>normal time</i> (MEZ) and <i>daylight saving
 * time</i> (MESZ), this happens at 01:55 for MEZ, and 02:55 for MESZ.<br>
 * If the define is 0, the receiver remains switched on, but the LED will only
 * get active again when the time signal gets out-of-sync.<br>
 *
 * <b>Light Barriers</b><br>
 * There is an inner and outer light barrier mounted on the front of the nest
 * box.  When activity is detected by one of the light barriers, the RFID
 * reader will be enabled.  The current state of the light barriers can be
 * viewed on the LC-Display by asserting the S1 button.
 *
 * <b>RFID Reader</b><br>
 * The RFID reader is used to receive the transponder number of the bird,
 * moving in or out of the nest box.  It is enabled by the light barriers
 * and shut down after @ref RFID_POWER_OFF_TIMEOUT seconds when there is no
 * more activity detected.
 *
 * <b>Keys (Push Buttons)</b><br>
 * There exist 3 push buttons on the board.  When asserted, the following
 * action will be taken:
 * - S1 displays the status of the light barriers (LBO is the outer, LBI the
 *   inner light barrier), and the battery voltage and remaining capacity.
 * - S2 displays the current date and time and the previously detected
 *   transponder number.  If this button is asserted for more than 5 seconds,
 *   the transponder number on the LCD will be cleared.
 * - S3 is the RESET button.
 *
 * The duration how long the respective information is displayed before
 * switching the LCD off again, can be adjusted by the define @ref
 * LCD_POWER_OFF_TIMEOUT.
 *
 * <b>LC-Display</b><br>
 * The display provides 2 lines á 16 characters.  It is connected to the
 * EFM32 microcontroller via a parallel bus interface.  To save power, the
 * whole display usually is switched off and only activated by asserting a
 * push button or exchanging the SD-Card.  However, after a power-up or reset,
 * the LCD remains powered on until the system clock is synchronized with
 * the DCF77 atomic clock signal.  This allows the user to read the firmware
 * version, the free space on the SD-Card, and the log filename.
 *
 * The visual contrast of the LCD can be adjusted via @ref l_Contrast.
 *
 * <b>LEDs</b><br>
 * There are two LEDs, one is red, the other one is green.
 * - The red LED is the Power-On LED.  It shows the current state of the
 *   DCF77 signal during receiving of the time information.  In normal
 *   condition the LED will be switched off.
 * - The green LED is the Log Flush LED.  It flashes whenever there are write
 *   accesses to SD-Card.
 *
 * <b>SD-Card</b><br>
 * The SD-Card is used to store logging information.  Only formatted cards
 * can be used, supported file systems are FAT12, FAT16, and FAT32.  The
 * filenames must follow the DOS schema 8+3, i.e. maximum 8 characters for
 * the basename, and 3 for the extension.  If the SD-Card contains a file
 * <b>BOX<i>nnnn</i>.TXT</b>, where <b><i>nnnn</i></b> can be any decimal
 * number, this will be used as the new log file.  In this way the filename
 * allows you to assign a specific box number to a dedicated unit.  If no
 * such file exists on the media, file "BOX0999.TXT" is created.
 * - Removing an SD-Card
 *   -# Generate a log message by triggering the light barriers, optionally
 *      with a reference transponder.
 *   -# Wait about 5 seconds (@ref LOG_SAMPLE_TIMEOUT) until the green LED
 *      starts flashing.  Log data is now written to the SD-Card.
 *   -# When the green LED stops flashing, you have a guaranteed duration
 *      of 15 seconds (@ref LOG_FLUSH_PAUSE) where no further data will be
 *      written to the SD-Card.
 *   -# Remove the SD-Card within this time.  The message "SD-Card removed"
 *      will be displayed on the LCD for 20 seconds.  All newly generated log
 *      messages will be stored in memory until another media is available.
 *
 * - Inserting an SD-Card
 *   -# Just insert the SD-Card into the slot.  The message "SD-Card inserted"
 *      will be displayed on the LCD.
 *   -# The file system on the SD-Card will be mounted and the free space of
 *      the media is reported.
 *   -# If a firmware update file (*.UPD) exists on the SD-Card, a reboot will
 *      be initiated to pass control to the booter.
 *   -# Otherwise the firmware looks for a "BOX<n>.TXT" file to use as new log
 *      file.
 *
 * <b>Battery Monitor</b><br>
 * The battery pack has its own controller.  It is connected to the EFM32
 * microcontroller via I2C-bus.  The battery monitor requests status
 * information from the battery pack and logs this on a regular basis, i.e.
 * every @ref BAT_MON_INTERVAL seconds.
 *
 * <b>Low-Energy UART</b><br>
 * The Low-Power UART (LEUART) provides a connection to a host computer (PC).
 * It can be used as monitoring and debugging interface.  All log messages,
 * written to the SD-Card, are sent through this interface also.  This behaviour
 * can be changed by defining @ref LOG_MONITOR_FUNCTION to @ref NONE.
 * The format of this UART is 9600 baud, 8 data bits, no parity.
 *
 * <b>Firmware</b><br>
 * The firmware consists of an initialization part and a main loop, also called
 * service execution loop.  The initialization part sets up all modules, enables
 * devices and interrupts.  The service execution loop handles all tasks that
 * must not be executed in interrupt context.
 *
 * After power-up or reset the following actions are performed:
 * -# Basic initialization of MCU and clocks
 * -# Low-Energy UART is set up
 * -# LEDs are switched on for test purposes (lamp test)
 * -# The logging facility is initialized and the firmware version is logged
 * -# Further hardware initialization (Keys, DCF77, RFID reader, Light
 *    Barriers, SD-Card interface, Interrupts, Alarm Clock)
 * -# The LC-Display is activated and the firmware version is shown
 * -# LEDs are switched off after 4 seconds
 * -# The Battery Monitor is initialized
 * -# The DCF77 Atomic Clock is enabled to receive the time signal
 *
 * The program then enters the Service Execution Loop which takes care of:
 * - Power management for the RFID reader
 * - Power management for the LC-Display
 * - SD-Card change detection and re-mounting of the filesystem
 * - Battery monitoring
 * - Writing the log buffer to the SD-Card
 * - Entering the right energy mode
 */
/*=============================== Header Files ===============================*/

#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_dma.h"
#include "config.h"		// include project configuration parameters
#include "ExtInt.h"
#include "DCF77.h"
#include "Keys.h"
#include "LightBarrier.h"
#include "RFID.h"
#include "AlarmClock.h"
#include "Display.h"
#include "LCD_DOGM162.h"
#include "LEUART.h"
#include "BatteryMon.h"
#include "Logging.h"
#include "PowerFail.h"

#include "ff.h"		// FS_FAT12/16/32
#include "diskio.h"	// DSTATUS
#include "microsd.h"

/*================================ Global Data ===============================*/

extern PRJ_INFO const  prj;		// Project Information


/*! @brief Global DMA Control Block.
 *
 * It contains the configuration for all 8 DMA channels which may be used by
 * various peripheral devices, e.g. ADC, DAC, USART, LEUART, I2C, and others.
 * The entries of this array will be set by the initialization routines of the
 * driver, which was assigned to the respective channel.  Unused entries remain
 * zero.  There is a total of 16 entries in the array.  The first 8 are used
 * for the primary DMA structures, the second 8 for alternate DMA structures
 * as used for DMA scatter-gather mode, where one buffer is still available,
 * while the other can be re-configured.  This application uses only the first
 * 8 entries.
 *
 * @see  DMA Channel Assignment
 *
 * @note This array must be aligned to 256!
 */
#if defined (__ICCARM__)
    #pragma data_alignment=256
    DMA_DESCRIPTOR_TypeDef g_DMA_ControlBlock[DMA_CHAN_COUNT * 2];
#elif defined (__CC_ARM)
    DMA_DESCRIPTOR_TypeDef g_DMA_ControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#elif defined (__GNUC__)
    DMA_DESCRIPTOR_TypeDef g_DMA_ControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#else
    #error Undefined toolkit, need to define alignment
#endif


/*! @brief Global DMA Callback Structure.
 *
 * This array contains the addresses of the DMA callback functions, which are
 * executed for a dedicated DMA channel at the end of a DMA transfer.
 * The entries of this array will be set by the initialization routines of the
 * driver, which was assigned to the respective channel.  Unused entries remain
 * zero.
 */
DMA_CB_TypeDef g_DMA_Callback[DMA_CHAN_COUNT];

/*! @brief Flag to indicate that an Interrupt occurred in the meantime.
 *
 * This flag must be set <b>true</b> by any interrupt service routine that
 * requires actions in the service execution loop of main().  This prevents
 * the system from entering sleep mode, so the action can be taken before.
 */
volatile bool		g_flgIRQ;

/*! @brief Modules that require EM1.
 *
 * This global variable is a bit mask for all modules that require EM1.
 * Standard peripherals would stop working in EM2 because clocks, etc. are
 * disabled.  Therefore it is required for software modules that make use
 * of such devices, to set the appropriate bit in this mask, as long as they
 * need EM1.  This prevents the power management of this application to enter
 * EM2.  The enumeration @ref EM1_MODULES lists those modules.
 * Low-Power peripherals, e.g. the LEUART still work in EM1.
 *
 * Examples:
 *
   @code
   // Module RFID requires EM1, set bit in bit mask
   Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 1;
   ...
   // Module RFID is no longer active, clear bit in bit mask
   Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 0;
   @endcode
 */
volatile uint16_t	g_EM1_ModuleMask;

/*================================ Local Data ================================*/

/*! EXTI initialization structure
 *
 * Connect the external interrupts of the push buttons to the key handler, the
 * DCF77 signal to the atomic clock module, the outer and inner light barrier
 * to their handler.
 */
static const EXTI_INIT  l_ExtIntCfg[] =
{   //	IntBitMask,	IntFct
    {	KEY_EXTI_MASK,	KeyHandler	 },	// Keys
    {	DCF_EXTI_MASK,	DCF77Handler	 },	// DCF77
    {	LB_EXTI_MASK,	LB_Handler	 },     // Light Barriers
    {	PF_EXTI_MASK,	PowerFailHandler },    // Power Fail
    {	0,		NULL	         }    
};

/*!
 * Initialization structure to define a function to be called for each
 * translated key.
 */
static const KEY_INIT  l_KeyInit =
{
    .KeyFct	= DisplayKeyHandler
};

/*! LCD field definitions
 *
 * This array specifies the location and size of fields on the LC-Display.
 *
 * @warning	Enum @ref LCD_FIELD_ID is used as index within this array,
 * 		therefore care has to be taken to keep it "in sync"!
 */
static const LCD_FIELD l_LCD_Field[LCD_FIELD_ID_CNT] =
{
    /* X,  Y,	Width	*/
    {  0,  0,	16	},	//!< 0: LCD_LINE1_BLANK
    {  0,  1,	16	},	//!< 1: LCD_LINE2_BLANK
    {  0,  0,	16	},	//!< 2: LCD_LINE1_TEXT
    {  0,  1,	16	},	//!< 3: LCD_LINE2_TEXT
    {  0,  0,	16	},	//!< 4: LCD_LIGHT_BARRIERS
    {  0,  1,	16	},	//!< 5: LCD_BATTERY
    {  0,  0,	16	},	//!< 6: LCD_CLOCK
    {  0,  1,	16	},	//!< 7: LCD_TRANSPONDER
};

/*!@brief Array of functions to be called in case of power-fail.
 *
 * Initialization array to define the power-fail handlers required for some
 * modules.
 * This array must be 0-terminated.
 */
static const POWER_FAIL_FCT l_PowerFailFct[] =
{
    RFID_PowerFailHandler,	// switch off RFID reader
    NULL
};

/*=========================== Forward Declarations ===========================*/

static void dispFilename (int arg);
static void cmuSetup(void);
static void Reboot(void);


/******************************************************************************
 * @brief  Main function
 *****************************************************************************/
int main( void )
{
    /* Initialize chip - handle erratas */
    CHIP_Init();

    /* Set up clocks */
    cmuSetup();

    /* Init Low Energy UART with 9600bd (this is the maximum) */
    drvLEUART_Init (9600);

#ifdef DEBUG
    dbgInit();
#endif

    /* Output version string to SWO or LEUART */
    drvLEUART_puts("\n***** APRDL V");
    drvLEUART_puts(prj.Version);
    drvLEUART_puts(" *****\n\n");

    /* Configure PA2 to drive the red Power-On LED (LED1) - show we are alive */
    GPIO_PinModeSet (POWER_LED_PORT, POWER_LED_PIN, gpioModePushPull, 1);

    /* Configure PA5 to drive the green Ready LED - show we are alive */
    GPIO_PinModeSet (LOG_FLUSH_LED_PORT, LOG_FLUSH_LED_PIN, gpioModePushPull, 1);

    /*
     * All modules that make use of external interrupts (EXTI) should be
     * initialized before calling ExtIntInit() because this enables the
     * interrupts, so IRQ handler may be executed immediately!
     */

    /* Initialize Logging (do this early) */
    LogInit();

    /* Log Firmware Revision */
    Log ("APRDL V%s (%s %s)", prj.Version, prj.Date, prj.Time);

    /* Initialize key hardware */
    KeyInit (&l_KeyInit);

    /* Initialize DCF77 hardware */
    DCF77Init();

    /* Initialize RFID reader */
    RFID_Init();

    /* Initialize light barrier hardware */
    LB_Init();

    /* Initialize SD-Card Interface */
    DiskInit();
    
    /* Introduce Power-Fail Handlers, configure Interrupt */
    PowerFailInit (l_PowerFailFct);

    /*
     * Initialize External Interrupts
     */
    ExtIntInit (l_ExtIntCfg);

    /* Initialize the Alarm Clock module */
    AlarmClockInit();

    /* Verify element count */
    EFM_ASSERT(ELEM_CNT(l_LCD_Field) == LCD_FIELD_ID_CNT);

    /* Initialize display - show firmware version */
    DisplayInit (l_LCD_Field);
    LCD_Printf (LCD_LINE1_TEXT, ">>>> APRDL <<<<");
    LCD_Printf (LCD_LINE2_TEXT, "V%s %s", prj.Version, prj.Date);

    msDelay(4000);	// show version for 4s

    /* Switch Log Flush LED OFF */
    LOG_FLUSH_LED = 0;

    /* Initialize Battery Monitor */
    BatteryMonInit();

    /* Enable the DCF77 Atomic Clock Decoder */
    DCF77Enable();

    /* Enable all other External Interrupts */
    ExtIntEnableAll();

    /* Once read Voltage and Battery Capacity for the LC-Display */
    LogBatteryInfo (BAT_LOG_INFO_DISPLAY_ONLY);


    /* ============================================ *
     * ========== Service Execution Loop ========== *
     * ============================================ */
    while (1)
    {
      /* Check for power-fail */
      if (! PowerFailCheck())
      {	
        /* Check if to power-on or off the RFID reader */
	RFID_Check();

	/* Update or power-off the LC-Display */
	DisplayUpdateCheck();

	/* Check if SD-Card has been inserted or removed */
	if (DiskCheck())
	{
	    /* First check if an "*.UPD" file exists on this SD-Card */
	    if (FindFile ("/", "*.UPD") != NULL)
	    {
		/*
		 * In this case the SD-Card contains update images.  We must
		 * pass control to the booter to perform a firmware upgrade.
		 */
		Reboot();
	    }

	    /* New File System mounted - (re-)open Log File */
	    LogFileOpen("BOX*.TXT", "BOX0999.TXT");

	    /* Display the (new) name after 5 seconds */
	    DisplayNext (5, dispFilename, 0);

	    /* Be sure to flush current log buffer */
	    LogFlush();

	    /* Log information about the MCU and the battery */
	    uint32_t uniquHi = DEVINFO->UNIQUEH;
	    Log ("MCU: %s HW-ID: 0x%08lX%08lX",
		 PART_NUMBER, uniquHi, DEVINFO->UNIQUEL);
	    LogBatteryInfo (BAT_LOG_INFO_VERBOSE);
	}

	/* Check Battery State */
	BatteryCheck();

	/* Check if to flush the log buffer */
	LogFlushCheck();
      }

      /*
       * Check for current power mode:  If a minimum of one active module
       * requires EM1, i.e. <g_EM1_ModuleMask> is not 0, this will be
       * entered.  If no one requires EM1 activity, EM2 is entered.
       */
	if (! g_flgIRQ)		// enter EM only if no IRQ occured
	{
	    if (g_EM1_ModuleMask)
		EMU_EnterEM1();		// EM1 - Sleep Mode
	    else
		EMU_EnterEM2(true);	// EM2 - Deep Sleep Mode
	}
	else
	{
	    g_flgIRQ = false;	// clear flag to enter EM the next time
	}
    }
}


/******************************************************************************
 * @brief   Display Filename on LCD
 *
 * This callback function displays the new filename, i.e. the BOX number on
 * the LCD.  It is called 5 seconds after the filesystem of an inserted SD-Card
 * is mounted.  The information will be cleared after another 5 seconds.
 *
 *****************************************************************************/
static void dispFilename (int arg)
{
    (void) arg;

    DisplayText (2, "SD: %s", g_LogFilename);
    DisplayNext (5, NULL, 0);
}


/******************************************************************************
 * @brief   Configure Clocks
 *
 * This local routine is called once from main() to configure all required
 * clocks of the EFM32 device.
 *
 *****************************************************************************/
static void cmuSetup(void)
{
    /* Start LFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

#if USE_EXT_32MHZ_CLOCK
    /* Start HFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

    /* Select HFXO as clock source for HFCLK */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

    /* Disable HFRCO */
    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
#endif

    /* Route the LFXO clock to the RTC and set the prescaler */
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);	// RTC, LETIMER
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);	// LEUART0/1
    CMU_ClockEnable(cmuClock_RTC, true);

    /* Prescaler of 1 = 30 us of resolution and overflow each 8 min */
    CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

    /* Enable clock to low energy modules */
    CMU_ClockEnable(cmuClock_CORELE, true);

    /* Enable clock for HF peripherals (ADC, DAC, I2C, TIMER, and USART) */
    CMU_ClockEnable(cmuClock_HFPER, true);

    /* Enable clock to GPIO */
    CMU_ClockEnable(cmuClock_GPIO, true);
}


/******************************************************************************
 * @brief   Reboot
 *
 * This local routine brings the system into a quiescent state and then
 * generates a reset.  It is typically used to transfer control from the
 * application to the booter for firmware upgrades.
 *
 *****************************************************************************/
static void Reboot(void)
{
int	n, i;

    LCD_Printf (LCD_LINE1_TEXT, "  R E B O O T");
    LCD_Printf (LCD_LINE2_TEXT, "");

    /* Disable external interrupts */
    ExtIntDisableAll();

    /* Shut down peripheral devices */
    BatteryMonDeinit();
    RFID_PowerOff();
    DCF77Disable();

    drvLEUART_puts ("Shutting down system for reboot\n");

    /*
     * Show LED Pattern before resetting:
     * 3x 5-short-pulses, separated by a pause,
     * finally a dimming LED from maximum brightness to off.
     */
    for (n = 0;  n < 3;  n++)		// 3x patterns
    {
	for (i = 0;  i < 5;  i++)	// a' 5 pulses
	{
	    POWER_LED = 1;
	    msDelay(100);
	    POWER_LED = 0;
	    msDelay(100);
	}
	msDelay(800);			// pause
    }

    for (n = 0;  n < 200;  n++)
    {
	POWER_LED = 1;
	for (i = 0;  i < (200 - n);  i++)
	    DelayTick();

	POWER_LED = 0;
	for (i = 0;  i < n;  i++)
	    DelayTick();
    }

    /* Perform RESET */
    NVIC_SystemReset();
}


/******************************************************************************
 * @brief   Show DCF77 Signal Indicator
 *
 * This routine is called by the DCF77 module during the synchronisation of
 * the clock to indicate the current state of the DCF77 signal.
 * On the APRDL board it sets the red power LED to the current state of
 * the DCF77 signal, LED on means high, LED off low level.
 *
 *****************************************************************************/
void	ShowDCF77Indicator (bool enable)
{
    POWER_LED = enable;
}
