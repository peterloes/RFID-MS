/***************************************************************************//**
 * @file
 * @brief	Handling of Keys (push buttons)
 * @author	Ralf Gerhauser
 * @version	2014-11-11
 *
 * This module provides all the functionality to receive key events and
 * translate them into key codes.
 * In detail, this includes:
 * - Initialization of the hardware (GPIOs that are connected to keys).
 * - Receive key events and and translate them into key codes.
 * - Call an external function for each translated key code.
 *
 ****************************************************************************//*
Revision History:
2016-04-05,rage	Made local variable <l_KeyState> of type "volatile".
2014-11-11,rage	Derived from project "AlarmClock".
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include "em_device.h"
#include "em_assert.h"
#include "em_cmu.h"
#include "Keys.h"
#include "AlarmClock.h"

/*=============================== Definitions ================================*/


/*=========================== Typedefs and Structs ===========================*/

/*! Internal states for key handling */
typedef enum
{
    KEY_IDLE,		//!< IDLE state - no key active
    KEY_S1,		//!< S1 Key is active
    KEY_S2,		//!< S2 Key is active
    KEY_S1_S2,		//!< both Keys are active
    KEY_CNT		//!< total number of key states.
} KEY_STATE;

/*======================== External Data and Routines ========================*/


/*========================= Global Data and Routines =========================*/


/*================================ Local Data ================================*/

    /*! Pointer to module configuration */
static const KEY_INIT *l_pKeyInit;

    /*! Variable to keep current key state */
static volatile KEY_STATE	l_KeyState = KEY_IDLE;

/*=========================== Forward Declarations ===========================*/

#if KEY_AUTOREPEAT
static void  KeyTimerFct (void);
#endif


/***************************************************************************//**
 *
 * @brief	Initialize key hardware
 *
 * This routine initializes the board-specific hardware for the key (push
 * button) functionality.  This is restricted to the GPIO set up, NVIC
 * interrupts will be configured later by calling function ExtIntInit().
 *
 * There are 2 keys and states supported:
 * - S1
 * - S2
 *
 * Each key generates an <b>asserted</b> code when the key is hit, and a
 * <b>released</b> code when it is released again.
 *
 * The GPIO ports and pins have been defined in the header file.
 *
 * @param[in] pInitStruct
 *	Address of an initialization structure of type KEY_INIT that defines
 *	the timings for the autorepeat threshold and rate, and a function to
 *	be called for each translated key.
 *
 * @note
 *	Parameter <b>pInitStruct</b> must point to a persistent data structure,
 *	i.e. this must be valid over the whole life time of the program.
 *
 ******************************************************************************/
void	KeyInit (const KEY_INIT *pInitStruct)
{
    /* Parameter check */
    EFM_ASSERT(pInitStruct != NULL);
    EFM_ASSERT(pInitStruct->KeyFct != NULL);

    /* Save configuration */
    l_pKeyInit = pInitStruct;

    /* Be sure to enable clock to GPIO (should already be done) */
    CMU_ClockEnable (cmuClock_GPIO, true);

    /*
     * Initialize GPIOs for all keys.  The port pins must be configured for
     * input, and connected to the external interrupt (EXTI) facility.  At
     * this stage, the interrupts are not enabled, this is done later by
     * calling ExtIntInit().
     */
    GPIO_PinModeSet (KEY_S1_PORT, KEY_S1_PIN, gpioModeInput, 0);
    GPIO_IntConfig  (KEY_S1_PORT, KEY_S1_PIN, false, false, false);

    GPIO_PinModeSet (KEY_S2_PORT, KEY_S2_PIN, gpioModeInput, 0);
    GPIO_IntConfig  (KEY_S2_PORT, KEY_S2_PIN, false, false, false);
}

/***************************************************************************//**
 *
 * @brief	Key handler
 *
 * This handler is called by the EXTI interrupt service routine for each
 * key which is asserted or released.  Together with the autorepeat feature
 * via a high-resolution timer and KeyTimerFct(), it translates the interrupt
 * number into a @ref KEYCODE.  This is then passed to the @ref KEY_FCT
 * defined as part of the @ref KEY_INIT structure.
 *
 * @param[in] extiNum
 *	EXTernal Interrupt number of a key.  This is identical with the pin
 *	number, e.g. @ref KEY_S1_PIN.
 *
 * @param[in] extiLvl
 *	EXTernal Interrupt level: 0 means falling edge, logic level is now 0,
 *	1 means rising edge, logic level is now 1.  Since the keys are
 *	connected to ground, we have a negative logic, i.e. 0 means asserted!
 *
 * @param[in] timeStamp
 *	Time stamp when the key event has been received.  This may be used
 *	to distinguish between short and long time asserted keys.
 *
 * @note
 * The time stamp is read from the Real Time Counter (RTC), so its resolution
 * depends on the RTC.  Use the define @ref RTC_COUNTS_PER_SEC to convert the
 * RTC value into a duration.
 *
 ******************************************************************************/
void	KeyHandler (int extiNum, bool extiLvl, uint32_t timeStamp)
{
KEY_STATE  keyState;		// new key state
KEYCODE	   keyCode;		// translated key code


    (void) timeStamp;		// suppress compiler warning "unused parameter"

    /* map the EXTI (pin) number to a key ID */
    switch (extiNum)
    {
	case KEY_S1_PIN:	// S1
	    keyState = KEY_S1;
	    keyCode  = KEYCODE_S1_ASSERT;
	    break;

	case KEY_S2_PIN:	// S2
	    keyState = KEY_S2;
	    keyCode  = KEYCODE_S2_ASSERT;
	    break;

	default:		// unknown pin number - ignore
	    return;
    }

    /*
     * Check if a key has been asserted or released.  Since the keys are
     * connected to ground, we have a negative logic, i.e. 0 means asserted!
     */
    if (extiLvl)
    {
	/* Level is 1, key has been RELEASED */

	if (keyState != l_KeyState)
	    return;	// only release active key - ignore all others

	/* pass a KEYCODE_XXX_RELEASE code to the KEY_FCT */
	keyCode += KEYOFFS_RELEASE;

	/* set key state to IDLE again */
	l_KeyState = KEY_IDLE;
    }
    else
    {
	/* Level is 0, key has been ASSERTED */

	if (l_KeyState != KEY_IDLE)
	    return;	// a key is already asserted - ignore all further keys

	/* set new key state and code */
	l_KeyState = keyState;
    }

    /* call the specified KEY_FCT */
    l_pKeyInit->KeyFct (keyCode);
}
