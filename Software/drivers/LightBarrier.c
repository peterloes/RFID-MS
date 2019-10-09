/***************************************************************************//**
 * @file
 * @brief	Light Barrier Logic
 * @author	Ralf Gerhauser
 * @version	2014-11-26
 *
 * This module receives the interrupts from the light barrier logic and
 * triggers the associated actions.  It contains an initialization routine
 * to set up the GPIOs, and an interrupt handler which processes the events.
 *
 ****************************************************************************//*
Revision History:
2016-04-05,rage	Made variable <g_LB_ActiveMask> of type "volatile".
2014-11-26,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include "em_cmu.h"
#include "LightBarrier.h"
#include "Display.h"
#include "RFID.h"
#include "Logging.h"

/*========================= Global Data and Routines =========================*/

    /*!@brief Bit mask what Light Barriers are active. */
volatile uint32_t  g_LB_ActiveMask;


/***************************************************************************//**
 *
 * @brief	Initialize the light barrier hardware
 *
 * This routine initializes the board-specific hardware for the light
 * barriers.  This is restricted to the GPIO set up, NVIC interrupts will
 * be configured later by calling function ExtIntInit().
 *
 ******************************************************************************/
void	LB_Init (void)
{
    /* Be sure to enable clock to GPIO (should already be done) */
    CMU_ClockEnable (cmuClock_GPIO, true);

    /*
     * Initialize GPIOs for the light barriers.  The port pins must be
     * configured for input, and connected to the external interrupt (EXTI)
     * facility.  At this stage, the interrupts are not enabled, this is
     * done later by calling ExtIntInit().
     */
    GPIO_PinModeSet (LB_OUTER_PORT, LB_OUTER_PIN, gpioModeInput, 0);
    GPIO_IntConfig  (LB_OUTER_PORT, LB_OUTER_PIN, false, false, false);

    GPIO_PinModeSet (LB_INNER_PORT, LB_INNER_PIN, gpioModeInput, 0);
    GPIO_IntConfig  (LB_INNER_PORT, LB_INNER_PIN, false, false, false);
}


/***************************************************************************//**
 *
 * @brief	Light Barrier handler
 *
 * This handler is called by the EXTI interrupt service routine whenever the
 * state of a light barrier changes.  It controls the power state of the RFID
 * reader, i.e. this is enabled as long as a minimum of one light barrier
 * indicates an object.
 *
 * @param[in] extiNum
 *	EXTernal Interrupt number of a light barrier.  This is identical with
 *	the pin number, e.g. @ref LB_OUTER_PIN.
 *
 * @param[in] extiLvl
 *	EXTernal Interrupt level: 0 means falling edge, logic level is now 0,
 *	1 means rising edge, logic level is now 1.  The level of the light
 *	barriers shows if the light beam could be received, i.e. 1 means normal
 *	state, level 0 indicates an object.
 *
 * @param[in] timeStamp
 *	Time stamp when the event has been received.  This parameter is not
 *	used here.
 *
 ******************************************************************************/
void	LB_Handler (int extiNum, bool extiLvl, uint32_t timeStamp)
{
    (void) timeStamp;		// suppress compiler warning "unused parameter"

    /* Set or clear the corresponding bit in the activity mask */
    Bit(g_LB_ActiveMask, extiNum) = ! extiLvl;
    DisplayUpdateTrigger (LCD_LIGHT_BARRIERS);

    /* Generate Log Message */
#ifdef LOGGING
    Log ("LB%c:%s", extiNum == LB_OUTER_PIN ? 'O':'I',
		    extiLvl == 0 ? "ON":"off");
#endif

    /*
     * If one ore more Light Barriers are active, the RFID reader must be
     * enabled, otherwise it should be switched off after a while.
     */
    if (g_LB_ActiveMask)
	RFID_Enable();		// immediately power-on the RFID reader
    else
	RFID_Disable();		// initiate power-off after a while

    g_flgIRQ = true;		// keep on running
}
