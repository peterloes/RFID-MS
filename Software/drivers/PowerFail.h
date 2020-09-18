/***************************************************************************//**
 * @file
 * @brief	Header file of module PowerFail.c
 * @author	Ralf Gerhauser
 * @version	2017-01-27
 ****************************************************************************//*
Revision History:
2017-01-27,rage	Initial version.
*/

#ifndef __INC_PowerFail_h
#define __INC_PowerFail_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_gpio.h"
#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

/*!@brief Here follows the definition of the power-fail input pin. */
#define POWER_FAIL_PORT	gpioPortB
#define POWER_FAIL_PIN	11

/*!@brief Bit mask for the affected external interrupt (EXTI). */
#define PF_EXTI_MASK	(1 << POWER_FAIL_PIN)


/*=========================== Typedefs and Structs ===========================*/

/*!@brief Array of functions to be called in case of power-fail.
 *
 * Initialization array to define the power-fail handlers of all modules.
 * This array is must be 0-terminated.
 *
 * <b>Typical Example:</b>
 * @code
 * static const POWER_FAIL_FCT l_PowerFailFct[] =
 * {
 *     RFID_PowerFailHandler,
 *     NULL
 * };
 * @endcode
 */
typedef void	(* POWER_FAIL_FCT)(void);


/*================================ Prototypes ================================*/

/* Initialize Light Barrier hardware */
void	PowerFailInit (const POWER_FAIL_FCT *pPowerFailFct);

/* Check if power-fail happened and perform all required actions */
bool	PowerFailCheck (void);

/* Probe routine, true means power-fail is active */
bool	IsPowerFail (void);

/* Power-Fail Handler, called from interrupt service routine */
void	PowerFailHandler (int extiNum, bool extiLvl, uint32_t timeStamp);


#endif /* __INC_PowerFail_h */
