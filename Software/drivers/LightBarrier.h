/***************************************************************************//**
 * @file
 * @brief	Header file of module LightBarrier.c
 * @author	Ralf Gerhauser
 * @version	2014-11-26
 ****************************************************************************//*
Revision History:
2016-04-13,rage	Removed LB_TRIG_MASK (no more required by EXTI module).
2016-04-05,rage	Made variable <g_LB_ActiveMask> of type "volatile".
2014-11-26,rage	Initial version.
*/

#ifndef __INC_LightBarrier_h
#define __INC_LightBarrier_h

/*=============================== Header Files ===============================*/

#include "em_device.h"
#include "em_gpio.h"
#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

/*!@brief Here follows the definition of the two light barriers and their
 * related hardware configuration.
 */
#define LB_OUTER_PORT	gpioPortD
#define LB_OUTER_PIN	6

#define LB_INNER_PORT	gpioPortD
#define LB_INNER_PIN	7


/*!@brief Bit mask of all affected external interrupts (EXTIs). */
#define LB_EXTI_MASK	((1 << LB_INNER_PIN) | (1 << LB_OUTER_PIN))

/*================================ Global Data ===============================*/

extern volatile uint32_t  g_LB_ActiveMask;

/*================================ Prototypes ================================*/

/* Initialize Light Barrier hardware */
void	LB_Init (void);

/* light Barrier handler, called from interrupt service routine */
void	LB_Handler	(int extiNum, bool extiLvl, uint32_t timeStamp);


#endif /* __INC_LightBarrier_h */
