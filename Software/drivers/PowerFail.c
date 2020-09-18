/***************************************************************************//**
 * @file
 * @brief	Power Fail Logic
 * @author	Ralf Gerhauser
 * @version	2020-05-12
 *
 * This module handles all actions required in case of a power-fail.
 *
 ****************************************************************************//*
Revision History:
2020-01-22,rage	PowerFailCheck: call BatteryChangeTrigger() when power good.
2017-01-31,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include "em_assert.h"
#include "em_cmu.h"
#include "ExtInt.h"
#include "PowerFail.h"
#include "BatteryMon.h"
#include "Logging.h"

/*================================ Local Data ================================*/

    /*! Local pointer to list of power-fail handlers */
static const POWER_FAIL_FCT *l_pPowerFailFct;

    /* Flag to save power-fail state */
static volatile bool l_flgPowerFail;


/***************************************************************************//**
 *
 * @brief	Initialize the power fail module
 *
 * This routine must be called once to introduce an array of power-fail
 * handlers, i.e. functions which are called in case of power-fail.
 *
 ******************************************************************************/
void	PowerFailInit (const POWER_FAIL_FCT *pPowerFailFct)
{
    /* Parameter check */
    EFM_ASSERT(pPowerFailFct != NULL);

    /* Save list of power-fail handlers */
    l_pPowerFailFct = pPowerFailFct;

    /* Be sure to enable clock to GPIO (should already be done) */
    CMU_ClockEnable (cmuClock_GPIO, true);

    /* Initialize the GPIO pin for power-fail detection, configure interrupt */
    GPIO_PinModeSet (POWER_FAIL_PORT, POWER_FAIL_PIN, gpioModeInput, 0);
    GPIO_IntConfig  (POWER_FAIL_PORT, POWER_FAIL_PIN, false, false, false);
}


/***************************************************************************//**
 *
 * @brief	Check for power-fail
 *
 * This routine is called from the main execution loop to check if power-fail
 * happened.  It then performs the required actions, i.e. it all handler, which
 * have been previously introduced via PowerFailInit() will be executed.
 *
 * @return
 * 	The value <i>true</i> if power-fail is active, <i>false</i> if not.
 *
 ******************************************************************************/
bool	PowerFailCheck (void)
{
const POWER_FAIL_FCT *pFct;


    if (IsPowerFail())
    {
	if (! l_flgPowerFail)
	{
	    /* Set flag to inhibit further executions */
	    l_flgPowerFail = true;

	    /* Replay external interrupts to consider new power state */
	    ExtIntReplay();

	    /* Call all power-fail handlers from the list */
	    for (pFct = l_pPowerFailFct;  *pFct != NULL;  pFct++)
		(*pFct)();
	}

	return true;
    }
    else
    {
	if (l_flgPowerFail)
	{
	    /* Reset flag */
	    l_flgPowerFail = false;

	    /* Replay external interrupts to consider new power state */
	    ExtIntReplay();

	    /* We assume the Battery Pack has been changed */
	    BatteryChangeTrigger();

	}

	return false;
    }
}


/***************************************************************************//**
 *
 * @brief	Probe for power-fail
 *
 * This routine should be called by critical parts of software that control
 * power consuming actions, so these can be aborted, in case of power-fail.
 *
 * @return
 * 	The value <i>true</i> if power-fail is active, <i>false</i> if not.
 *
 ******************************************************************************/
bool	IsPowerFail (void)
{
    return IO_Bit(GPIO->P[POWER_FAIL_PORT].DIN, POWER_FAIL_PIN) == 0;
}


/***************************************************************************//**
 *
 * @brief	Power Fail Handler
 *
 * This handler is called by the EXTI interrupt service routine whenever the
 * state of the power-fail input pin changes.  This pin is connected with
 * <i>Power Good</i> signal of the power regulator.
 * The handler just logs a message and wakes-up the main execution loop, so
 * that PowerFailCheck() will be called.
 *
 * @param[in] extiNum
 *	EXTernal Interrupt number of power-fail signal.  This is identical with
 *	the pin number, e.g. @ref POWER_FAIL_PIN.
 *
 * @param[in] extiLvl
 *	EXTernal Interrupt level: 0 means falling edge, logic level is now 0
 *	which means voltage is below 3V, 1 means rising edge, logic level is
 *	now 1, power is good.
 *
 * @param[in] timeStamp
 *	Time stamp when the event has been received.  This parameter is not
 *	used here.
 *
 ******************************************************************************/
void	PowerFailHandler (int extiNum, bool extiLvl, uint32_t timeStamp)
{
    (void) extiNum;		// suppress compiler warning "unused parameter"

    /* Generate Log Message for new state (not in case of "replay") */
#ifdef LOGGING
    if (timeStamp != 0)
	Log ("Power-Fail: Received interrupt (POWER %s)",
	     extiLvl == 0 ? "FAIL":"GOOD");
#endif

    g_flgIRQ = true;		// keep on running
}
