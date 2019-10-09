/***************************************************************************//**
 * @file
 * @brief	Header file of module DCF77.c
 * @author	Ralf Gerhauser
 * @version	2014-11-21
 ****************************************************************************//*
Revision History:
2016-04-13,rage	Removed DCF_TRIG_MASK (no more required by EXTI module).
2016-04-05,rage	Reverted DCF77_ENABLE_PIN to 1 and DCF77_SIGNAL_PIN to 2.
2015-07-28,rage	Changed DCF77_ENABLE_PIN to 2 and DCF77_SIGNAL_PIN to 1.
2014-11-21,rage	Added DCF77_DISPLAY_PROGRESS and DCF77_INDICATOR.
2014-05-10,rage	Initial version.
*/

#ifndef __INC_DCF77_h
#define __INC_DCF77_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_gpio.h"
#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

#ifndef DCF77_HARDWARE_ENABLE
    /*!@brief Set to 1 if DCF77 hardware module needs enable via pin. */
    #define DCF77_HARDWARE_ENABLE	0
#endif

#ifndef DCF77_ONCE_PER_DAY
    /*!@brief Set 1 to activate DCF77 only once per day. */
    #define DCF77_ONCE_PER_DAY		0
#endif

#ifndef DCF77_DISPLAY_PROGRESS
    /*!@brief Set 1 to display receive progress on segment LCD.
     * If 1, functions SegmentLCD_ARing(), SegmentLCD_ARingSetAll(),
     * SegmentLCD_Symbol(), and SegmentLCD_Update() are called to display
     * the progress of receiving DCF77 information.
     */
    #define DCF77_DISPLAY_PROGRESS	0
#endif

#ifndef DCF77_INDICATOR
    /*!@brief Set 1 to use a DCF77 signal indicator, e.g. an LED.
     * If 1, function ShowDCF77Indicator() will be called during the
     * synchronization, whenever the DCF77 signal changes to high or low.
     */
    #define DCF77_INDICATOR		0
#endif

/*!@brief Here follows the definition of GPIO ports and pins used to connect
 * to the external DCF77 hardware module.
 */
#if DCF77_HARDWARE_ENABLE
    #define DCF77_ENABLE_PORT	gpioPortD
    #define DCF77_ENABLE_PIN	1
#endif

#define DCF77_SIGNAL_PORT	gpioPortD
#define DCF77_SIGNAL_PIN	2


/*!@brief Bit mask of the affected external interrupt (EXTI). */
#define DCF_EXTI_MASK		(1 << DCF77_SIGNAL_PIN)

/*================================ Prototypes ================================*/

/* Initialize DCF77 hardware */
void	DCF77Init (void);

/* Enable the DCF77 decoder */
void	DCF77Enable (void);

/* Disable the DCF77 decoder */
void	DCF77Disable (void);

/* Signal handler, called from interrupt service routine */
void	DCF77Handler	(int extiNum, bool extiLvl, uint32_t timeStamp);


#endif /* __INC_DCF77_h */
