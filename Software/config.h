/***************************************************************************//**
 * @file
 * @brief	Project configuration file
 * @author	Ralf Gerhauser
 * @version	2016-02-27
 *
 * This file allows to set miscellaneous configuration parameters.  It must be
 * included by all modules.
 *
 ****************************************************************************//*
Revision History:
2016-02-26,rage	Increased LOG_BUF_SIZE to 4KB.
2016-02-10,rage	Set RFID_POWER_OFF_TIMEOUT to 6 minutes.
2014-11-11,rage	Derived from project "AlarmClock".
*/

#ifndef __INC_config_h
#define __INC_config_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"

/*=============================== Definitions ================================*/

/*
 * Basic defines - should all be moved to Generic.h
 */
    /* terminators for lists and strings */

#define	EOL		NULL		/* EndOfList		*/
#define EOS		'\0'		/* EndOfString		*/
#define	NONE		(-1)

    /* macro to calculate the number of elements of an array */
#define ELEM_CNT(array)  (sizeof (array) / sizeof ((array)[0]))

/*
 * LED Definitions for this project
 */
    /*!@brief GPIO Port of the (red) Power-LED. */
#define POWER_LED_PORT		gpioPortA
    /*!@brief GPIO Pin of the (red) Power-LED. */
#define POWER_LED_PIN		2
    /*! @brief Macro to set or clear the Power-LED */
#define POWER_LED   IO_Bit(GPIO->P[POWER_LED_PORT].DOUT, POWER_LED_PIN)
    /*!@brief GPIO Port of the (green) Log Flush LED. */
#define LOG_FLUSH_LED_PORT	gpioPortA
    /*!@brief GPIO Pin of the (green) Log Flush LED. */
#define LOG_FLUSH_LED_PIN	5		//! State: 0=OFF, 1=ON
    /*! @brief Macro to set or clear the Log Flush LED */
#define LOG_FLUSH_LED IO_Bit(GPIO->P[LOG_FLUSH_LED_PORT].DOUT, LOG_FLUSH_LED_PIN)

/*!
 * @brief MPU Clock Configuration.
 *
 * Set to 0 to use the internal RC oscillator, if 1 the external 32MHz XTAL
 * is used.
 */
#define USE_EXT_32MHZ_CLOCK	0

/*
 * Configuration for module "AlarmClock"
 */
    /*!@brief RTC frequency in [Hz]. */
#define RTC_COUNTS_PER_SEC	32768


/*!
 * @brief Interrupt Priority Settings
 *
 * There are 8 priority levels 0 to 7 with 0 to be the highest and 7 to be the
 * lowest priority.  DCF77 (which is called from the EXTI handler) and RTC must
 * use the same priority level to lock-out each other, because they both use
 * function localtime() and this is not multithreading save.  Funktion
 * localtime_r() would be the right choice here, unfortunately it is not
 * available with the IAR compiler library.
 */
#define INT_PRIO_UART	2		//!< UART interrupts for the RFID reader
#define INT_PRIO_LEUART	2		//!< LEUART RX interrupt (not used)
#define INT_PRIO_DMA	2		//!< DMA is used for LEUART
#define INT_PRIO_SMB	2		//!< SMBus used by the battery monitor
#define INT_PRIO_RTC	3		//!< lower priority than others
#define INT_PRIO_EXTI	INT_PRIO_RTC	//!< must be the same as @ref INT_PRIO_RTC


/*
 * Configuration for Atomic Clock module "DCF77.c"
 */
    /*!@brief Use Hardware Enable pin of the DCF77 receiver. */
#define DCF77_HARDWARE_ENABLE	1

    /*!@brief Activate DCF77 only once per day. */
#define DCF77_ONCE_PER_DAY	1

    /*!@brief Call ShowDCF77Indicator() to flash red LED on DCF77 signal. */
#define DCF77_INDICATOR		1


/*
 * Configuration for module "RFID"
 */

    /*!@brief Time in [s] after which the RFID reader is powered-off. */
#define RFID_POWER_OFF_TIMEOUT	360	// 6min


/*
 * Configuration for module "Logging"
 */
    /*!@brief Size of the log buffer in bytes. */
#define LOG_BUF_SIZE	4096

    /*!@brief Use this define to specify a function to be called for monitoring
     * the log activity.  Here, monitoring is done via the LEUART interface.
     */
#define LOG_MONITOR_FUNCTION	drvLEUART_puts

/* forward declaration */
void    drvLEUART_puts(const char *str);

    /*!@brief Disable "alive" message by setting this interval to 0. */
#define LOG_ALIVE_INTERVAL	0


/*!@name DMA Channel Assignment
 *
 * The following definitions assign the 8 DMA channels to the respective
 * devices or drivers.  These defines are used as index within the global
 * @ref DMA_DESCRIPTOR_TypeDef structure @ref g_DMA_ControlBlock.
 */
//@{
#define DMA_CHAN_LEUART_RX	0	//! LEUART Rx uses DMA channel 0
#define DMA_CHAN_LEUART_TX	1	//! LEUART Tx uses DMA channel 1
//@}


/*================================== Macros ==================================*/

#ifdef DEBUG
    /*
     * Debugging output via ITM or LEUART
     */
    #if DEBUG_VIA_ITM
	#define DBG_PUTC(ch)	ITM_SendChar(ch)
	#define DBG_PUTS(str)	ITM_SendStr(str)
	uint32_t ITM_SendChar (uint32_t ch);
	void ITM_SendStr(const char *pStr);
    #else
	#define DBG_PUTC(ch)	drvLEUART_putc(ch)
	#define DBG_PUTS(str)	drvLEUART_puts(str)
	void	drvLEUART_putc(char ch);
	void	drvLEUART_puts(const char *str);
    #endif
    void dbgInit(void);
#else
    #define DBG_PUTC(ch)
    #define DBG_PUTS(str)
#endif

    /*! Macro to address a single bit in the I/O range (peripheral range) in
     *  an atomic manner.
     * @param address   I/O register address.
     * @param bitNum    Bit number within this register.
     */
#define IO_BIT_ADDR(address, bitNum)					\
	((__IO uint32_t *) (BITBAND_PER_BASE				\
			+ (((uint32_t)(address)) - PER_MEM_BASE) * 32	\
			+ (bitNum) * 4))

    /*! Shortcut to directly access an I/O-bit. */
#define IO_Bit(regName, bitNum)	*IO_BIT_ADDR(&regName, bitNum)

    /*! Macro to address a single bit in an SRAM variable in an atomic manner.
     * @param address   Address of the variable in SRAM.
     * @param bitNum    Bit number within this variable.
     */
#define SRAM_BIT_ADDR(address, bitNum)					\
	((__IO uint32_t *) (BITBAND_RAM_BASE				\
			+ (((uint32_t)(address)) - RAM_MEM_BASE) * 32	\
			+ (bitNum) * 4))

    /*! Shortcut to directly access a bit in a variable. */
#define Bit(varName, bitNum)	*SRAM_BIT_ADDR(&varName, bitNum)

/*=========================== Typedefs and Structs ===========================*/

/*!@brief Structure to hold Project Information */
typedef struct
{
    char const  ID[12];
    char const  Date[16];
    char const  Time[10];
    char const  Version[16];
} PRJ_INFO;


/*!@brief Enumeration of Alarm Identifiers
 *
 * This is the list of Alarm IDs used by this application.  They are used to
 * identify a particular alarm time entry via the <b>alarmNum</b> parameter
 * when calling alarm functions, e.g. AlarmSet().
 */
typedef enum
{
    ALARM_DCF77_WAKE_UP,    //!< Wake up DCF77 to synchronize the system clock
    ALARM_BATTERY_MON_1,    //!< Time #1 for logging battery status
    ALARM_BATTERY_MON_2,    //!< Time #2 for logging battery status
    END_ALARM_ID
} ALARM_ID;


/*!@brief Enumeration of the EM1 Modules
 *
 * This is the list of Software Modules that require EM1 to work, i.e. they
 * will not work in EM2 because clocks, etc. would be disabled.  These enums
 * are used to set/clear the appropriate bit in the @ref g_EM1_ModuleMask.
 */
typedef enum
{
    EM1_MOD_RFID,	//!<  0: The RFID Modules uses the UART
    END_EM1_MODULES
} EM1_MODULES;

/*======================== External Data and Routines ========================*/

extern volatile bool	 g_flgIRQ;		// Flag: Interrupt occurred
extern volatile uint16_t g_EM1_ModuleMask;	// Modules that require EM1


#endif /* __INC_config_h */
