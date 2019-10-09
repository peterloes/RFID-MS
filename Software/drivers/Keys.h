/***************************************************************************//**
 * @file
 * @brief	Header file of module Keys.c
 * @author	Ralf Gerhauser
 * @version	2014-11-11
 ****************************************************************************//*
Revision History:
2016-04-13,rage	Removed KEY_TRIG_MASK (no more required by EXTI module).
2014-11-11,rage	Derived from project "AlarmClock".
*/

#ifndef __INC_Keys_h
#define __INC_Keys_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_gpio.h"
#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

/*!@brief Here follows the definition of all keys (push buttons) and their
 * related hardware configurations.
 */
#define KEY_S1_PORT	gpioPortA
#define KEY_S1_PIN	3

#define KEY_S2_PORT	gpioPortA
#define KEY_S2_PIN	4


/*!@brief Bit mask of all affected external interrupts (EXTIs). */
#define KEY_EXTI_MASK	((1 << KEY_S1_PIN) | (1 << KEY_S2_PIN))

/*=========================== Typedefs and Structs ===========================*/

/*!@brief Translated key codes. */
typedef enum
{
    KEYCODE_NONE,		//!< 0: No key code active
    KEYCODE_S1_ASSERT,		//!< 1: Key code for S1 once asserted
    KEYCODE_S1_RELEASE,		//!< 2: Key code for S1 released again
    KEYCODE_S2_ASSERT,		//!< 3: Key code for S2 once asserted
    KEYCODE_S2_RELEASE,		//!< 4: Key code for S2 released again
    KEYCODE_S1_S2_ASSERT,	//!< 5: Key code for S1+S2 once asserted
    KEYCODE_S1_S2_RELEASE,	//!< 6: Key code for S1+S2 released again
    END_KEYCODE			//!< End of key code definitions
} KEYCODE;

/*!@brief Offset to be added to the ASSERT key code */
#define KEYOFFS_RELEASE	(KEYCODE)1	// +1 for RELEASE code

/*!@brief Function to be called for each translated key code. */
typedef void	(* KEY_FCT)(KEYCODE keycode);

/*!@brief Key initialization structure.
 *
 * Initialization structure to define the timings for the autorepeat (AR)
 * threshold and rate (in milliseconds), and a function to be called for each
 * translated key.
 *
 * <b>Typical Example:</b>
 * @code
 * static const KEY_INIT l_KeyInit =
 * {
 *    DisplayKeyHandler		// Key handler of module "Display.c"
 * };
 * @endcode
 */
typedef struct
{
    KEY_FCT   KeyFct;		//!< Fct. to be called for each translated key
} KEY_INIT;

/*================================ Prototypes ================================*/

/* Initialize key hardware */
void	KeyInit (const KEY_INIT *pInitStruct);

/* Key handler, called from interrupt service routine */
void	KeyHandler	(int extiNum, bool extiLvl, uint32_t timeStamp);


#endif /* __INC_Keys_h */
