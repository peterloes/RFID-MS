/***************************************************************************//**
 * @file
 * @brief	Display Manager
 * @author	Ralf Gerhauser
 * @version	2014-11-25
 *
 * This is the Display Manager module.  It controls all the information on
 * the LC-Display.  Depending on the state of the keys S1 and S2 different
 * information shall be displayed on the LCD:
 * - When <b>S1</b> is asserted, the display shows
 *  - The state of the outer and inner light barrier.
 *  - The remaining battery capacity
 * - When <b>S2</b> is asserted, the display shows
 *  - Date and Time
 *  - The previously detected transponder nummer
 * - When none of the keys is asserted, the LCD is powered-off after one minute.
 *
 * The low-level display routines can be found in LCD_DOGM162.c.
 *
 ****************************************************************************//*
Revision History:
2016-04-05,rage	Made all local variables of type "volatile".
2015-07-05,rage	Adapted for SNB_Heaven.
2015-05-10,rage	Clear the transponder number on the LC-Display if the S2 button
		remains asserted for more than 5 seconds.
2014-11-25,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include "em_device.h"
#include "em_assert.h"
#include "Keys.h"
#include "AlarmClock.h"
#include "Display.h"
#include "LCD_DOGM162.h"
#include "LightBarrier.h"
#include "RFID.h"
#include "BatteryMon.h"

/*================================ Local Data ================================*/

    /*!@brief Timer handle for switching the display off after a time. */
static volatile TIM_HDL l_hdlLCD_Off = NONE;

    /*!@brief Timer handle for Display Next callback routine. */
static volatile TIM_HDL l_hdlDispNext = NONE;

    /*!@brief Timer handle to clear the transponder number. */
static volatile TIM_HDL l_hdlClearXp = NONE;

    /*!@brief Flag if Display is currently powered on. */
static volatile bool	l_flgDisplayIsOn;

    /*!@brief Bit mask variable specifies which fields must be updated, each
     * bit refers to another field, see @ref LCD_FIELD_ID.
     */
static uint16_t l_bitMaskFieldUpd;

    /*!@brief Bit mask variable, containing the currently active fields, i.e.
     * the fields should be displayed on the LCD, see @ref l_bitMaskFieldUpd.
     */
static uint16_t l_bitMaskFieldActive;

    /*!@brief Flag is set after DCF77 Date and Time has been displayed. */
static volatile bool	l_flgDisplayUpdEnabled;

    /*!@brief Flag triggers the execution of function @ref l_DispNextFct. */
static volatile bool	l_DispNextFctTrigger;

    /*!@brief Function pointer for a callback routine which is executed after
     * the specified amount of time has elapsed, see @ref DisplayNext().
     */
static volatile DISP_NEXT_FCT l_DispNextFct;

    /*!@brief User parameter for function @ref l_DispNextFct. */
static volatile int	l_DispNextUserParm;

/*=========================== Forward Declarations ===========================*/

static void DisplayUpdate (void);
static void DisplayUpdateClock (void);
static void SwitchLCD_Off(TIM_HDL hdl);
static void DispNextTrigger(TIM_HDL hdl);
static void ClearTransponderID(TIM_HDL hdl);


/***************************************************************************//**
 *
 * @brief	Initialize the Display
 *
 * This routine initializes the LC-Display and all the required functionality
 * around it, e.g. a timer to switch off the display when it is not in use.
 *
 ******************************************************************************/
void	DisplayInit (const LCD_FIELD *pField)
{
    /* Get a timer handle to switch the display off after a time */
    if (l_hdlLCD_Off == NONE)
	l_hdlLCD_Off = sTimerCreate (SwitchLCD_Off);

    /* Create timer to trigger a callback routine after duration is over */
    if (l_hdlDispNext == NONE)
	l_hdlDispNext = sTimerCreate (DispNextTrigger);

    /*
     * Create a timer to clear the transponder number if S2 is asserted
     * for more than 5 seconds.
     */
    if (l_hdlClearXp == NONE)
	l_hdlClearXp = sTimerCreate (ClearTransponderID);

    /* Connect the update function */
    DisplayUpdateFctInstall (DisplayUpdateClock);

    /* Set flags to active state */
    l_bitMaskFieldActive = LCD_FIELD_ID_BIT(LCD_LINE1_BLANK);	// pseudo field
    l_flgDisplayIsOn = true;

    /* Initialize the LCD module specific parts */
    LCD_Init (pField);
}


/***************************************************************************//**
 *
 * @brief	Display Key Handler
 *
 * This handler receives the translated key codes from the interrupt-driven
 * key handler, including autorepeat keys.  That is, whenever the user asserts
 * a key (push button), the resulting code is sent to this function.
 * The handler sets the local bit mask @ref l_bitMaskFieldActive to a dedicated
 * value, so one, or a combination of LCD fields is displayed.
 *
 * The following keys are recognized:
 * - <b>S1</b> shows the LCD fields specified by @ref LCD_FIELD_ID_MASK_S1.
 *   These are the current state of the light barriers and the battery status.
 * - <b>S2</b> shows the LCD fields specified by @ref LCD_FIELD_ID_MASK_S2.
 *   These are the current date and time and the previously detected transponder
 *   number.  If this key is asserted for more than 5 seconds, the field for
 *   the transponder number will be cleared.
 * When the keys are released, the LCD Power-Off timer is started.
 *
 * @warning
 * 	This function is called in interrupt context!
 *
 * @param[in] keycode
 *	Translated key code of type KEYCODE.
 *
 ******************************************************************************/
void	DisplayKeyHandler (KEYCODE keycode)
{
    switch (keycode)
    {
	case KEYCODE_S1_ASSERT:		// S1 was asserted
	    l_bitMaskFieldActive = LCD_FIELD_ID_MASK_S1;
	    break;

	case KEYCODE_S2_ASSERT:		// S2 was asserted
	    l_bitMaskFieldActive = LCD_FIELD_ID_MASK_S2;
	    /* start timer to clear transponder number after 5 seconds */
	    if (l_hdlClearXp != NONE)
		sTimerStart (l_hdlClearXp, 5);
	    break;

	case KEYCODE_S2_RELEASE:	// S2 was released
	    /* cancel timer if S2 has been released before 5 seconds */
	    if (l_hdlClearXp != NONE)
		sTimerCancel (l_hdlClearXp);
	    /* no break */
	case KEYCODE_S1_RELEASE:	// S1 was released
	    /* (re-)start timer to switch display OFF after time */
	    if (l_hdlLCD_Off != NONE)
		sTimerStart (l_hdlLCD_Off, LCD_POWER_OFF_TIMEOUT);
	    return;

	default:	// ignore all other key codes
	    return;
    }

    /* Common part of KEYCODE_Sx_ASSERT */
    if (l_hdlLCD_Off != NONE)
	sTimerCancel (l_hdlLCD_Off);	// inhibit power-off of LCD

    /* Initiate first update for all selected fields */
    l_bitMaskFieldUpd = l_bitMaskFieldActive;
}


/***************************************************************************//**
 *
 * @brief	Display Update Check
 *
 * This function checks if the information on the LC-Display needs to be
 * updated, or if the LCD is currently not used and can be switched off.
 *
 * @note
 * 	This function may be called from standard program, usually the loop
 * 	in module "main.c" - it must not be called from interrupt routines!
 *
 ******************************************************************************/
void	DisplayUpdateCheck (void)
{

    /*
     * Check for callback trigger first
     */
    if (l_DispNextFctTrigger)
    {
	DISP_NEXT_FCT fct = l_DispNextFct;

	/* Clear trigger flag */
	l_DispNextFctTrigger = false;

	/* See if a callback routine has been defined and call it */
	if (fct)
	{
	    l_DispNextFct = NULL;	// no NEW callback for default

	    fct (l_DispNextUserParm);	// call user routine
	}
	else
	{
	    /* No callback - switch LCD off */
	    SwitchLCD_Off((TIM_HDL)0);
	}

    }

    /*
     * Check if LC-Display should be powered-on or off.  This is executed
     * in this main loop since it must not happen in any interrupt service
     * routine (ISR) due to calling delay functions and other issues.
     * However, the reason when to do it is triggered via ISRs.
     */
    if (l_bitMaskFieldActive)
    {
	/* LCD should be powered ON */
	if (! l_flgDisplayIsOn)
	{
	    LCD_PowerOn();
	    l_flgDisplayIsOn = true;
	}

	/* LCD is ON - check if fields need to be updated */
	if (l_bitMaskFieldUpd & l_bitMaskFieldActive)
	    DisplayUpdate();
    }
    else
    {
	/* LCD should be powered OFF */
	if (l_flgDisplayIsOn)
	{
	    LCD_PowerOff();
	    l_flgDisplayIsOn = false;
	}
    }
}


/***************************************************************************//**
 *
 * @brief	Display Update
 *
 * This local function is called from DisplayUpdateCheck() whenever fields
 * need to be updated, i.e. when bits in @ref l_bitMaskFieldUpd are set.
 * It displays the respective data on the LCD and clears the associated bit.
 *
 ******************************************************************************/
static void DisplayUpdate (void)
{
LCD_FIELD_ID	id;

    for (id = LCD_LINE1_BLANK;  id < LCD_FIELD_ID_CNT;  id++)
    {
	if ((LCD_FIELD_ID_BIT_VAR(l_bitMaskFieldUpd,    id) == 0)
	||  (LCD_FIELD_ID_BIT_VAR(l_bitMaskFieldActive, id) == 0))
	    continue;		// nothing to be done for this field

	/* clear this bit */
	LCD_FIELD_ID_BIT_VAR(l_bitMaskFieldUpd, id) = 0;

	/* update the respective field */
	switch (id)
	{
	    case LCD_LINE1_BLANK:
	    case LCD_LINE2_BLANK:	// print empty line
		LCD_Printf (id, "");
		break;

	    case LCD_LIGHT_BARRIERS:	// current state of light barriers
		LCD_Printf (id, "LBO: %s  LBI: %s",
			    g_LB_ActiveMask & (1 << LB_OUTER_PIN) ? "XX":"--",
			    g_LB_ActiveMask & (1 << LB_INNER_PIN) ? "XX":"--");
		break;

	    case LCD_BATTERY:		// battery voltage, remaining capacity
		if (g_BattMilliVolt < 0)
		{
		    LCD_Printf (id, "BatteryCtrlERROR");
		}
		else
		{
		    LCD_Printf (id, "%2d.%dV  %6dmAh",
				(g_BattMilliVolt / 1000),
				(g_BattMilliVolt % 1000) / 100,
				g_BattCapacity);
		}
		break;

	    case LCD_CLOCK:		// current date and time
		LCD_Printf (id, "%02d%02d%02d %02d:%02d:%02d",
			    g_CurrDateTime.tm_year,
			    g_CurrDateTime.tm_mon + 1,
			    g_CurrDateTime.tm_mday,
			    g_CurrDateTime.tm_hour,
			    g_CurrDateTime.tm_min,
			    g_CurrDateTime.tm_sec);
		break;

	    case LCD_TRANSPONDER:	// current transponder number
		LCD_Printf (id, g_Transponder);
		break;

	    default:		// LCD_LINE1_TEXT, LCD_LINE2_TEXT, or unknown ID
		break;		// nothing to be done
	}
    }
}


/***************************************************************************//**
 *
 * @brief	Display Update Enable
 *
 * This routine will be called from the DCF77 module when the date and time
 * have been completely received and is valid, so it can be displayed.
 *
 ******************************************************************************/
void	DisplayUpdEnable (void)
{
    if (! l_flgDisplayUpdEnabled)
    {
	l_flgDisplayUpdEnabled = true;	// do this only once after RESET

	/* display date and time, and an empty line below */
	l_bitMaskFieldActive = l_bitMaskFieldUpd = LCD_FIELD_ID_MASK_DCF77;

	/* Start timer to switch OFF the display after 30 seconds */
	if (l_hdlLCD_Off != NONE)
	    sTimerStart (l_hdlLCD_Off, 30);
    }
}


/***************************************************************************//**
 *
 * @brief	Display Text
 *
 * This routine allows you to display text on the LCD.  If the LCD is off,
 * it will be powered-on.  To automatically switch it off after a specified
 * duration, or to display another text after this time, DisplayNext() can
 * be used.
 *
 * @param[in] lineNum
 *	The line number where to display the text.  Must be 1 or 2.
 *
 * @param[in] frmt
 *	Format string of the text to print - same as for printf().
 *
 * @see
 * 	DisplayNext()
 *
 ******************************************************************************/
void	DisplayText (int lineNum, const char *frmt, ...)
{
LCD_FIELD_ID	id;
va_list		args;


    /* Parameter check */
    if (lineNum < 1  ||  lineNum > 2)
    {
	EFM_ASSERT(false);
	return;
    }

    /* Specify field to use */
    id = (lineNum == 1 ? LCD_LINE1_TEXT : LCD_LINE2_TEXT);

    /* Activate LCD */
    LCD_FIELD_ID_BIT_VAR(l_bitMaskFieldActive, id) = 1;
    LCD_FIELD_ID_BIT_VAR(l_bitMaskFieldUpd, id) = 1;
    DisplayUpdateCheck();

    /* Print Text */
    va_start (args, frmt);
    LCD_vPrintf (id, frmt, args);
    va_end (args);

    /* Cancel possible running power-off timer to ensure LCD remains ON */
    if (l_hdlLCD_Off != NONE)
	sTimerCancel (l_hdlLCD_Off);
}


/***************************************************************************//**
 *
 * @brief	Display Next
 *
 * This function defines what should happen next on the LC-Display.  It is
 * typically used after calling DisplayText() in one of the following ways:
 *
 * - @p duration is specified and @p fct is NULL: the LCD is switched off
 *   after the amount of time.
 *
 * - @p duration is specified and @p fct is not NULL: the callback function is
 *   executed after the amount of time.  This function may call DisplayText()
 *   and DisplayNext() again.  In this way it is possible to realize a
 *   "ticker" on the LCD.  For a generic approach a user parameter @p userParm
 *   is passed to the function which can be used to specify the item to be
 *   displayed.
 *
 * - @p duration is 0 and @p fct is NULL: the LCD is switched off immediately.
 *
 * - @p duration is 0 and @p fct is not NULL: the callback function will be
 *   executed as soon as possible.
 *
 * @param[in] duration
 * 	Duration in seconds, <b>after</b> which the specified action should
 * 	occur.  If a duration of 0 is specified, this happens immediately!
 *
 * @param[in] fct
 *	Callback function to be executed after @p duration.  If NULL is
 *	specified, no function will be called, instead the LC-Display is
 *	powered-off.
 *
 * @param[in] userParm
 *	User parameter for function @p fct.
 *
 * @note
 * 	Only one callback function can be installed at a dedicated time, i.e.
 * 	they cannot be stacked.  The function is called in standard context
 * 	by DisplayUpdateCheck() (not by an ISR), so there are no limitations.
 *
 ******************************************************************************/
void	DisplayNext (unsigned int duration, DISP_NEXT_FCT fct, int userParm)
{
    /* Be sure to reset the trigger flag */
    l_DispNextFctTrigger = false;

    /* Cancel possible running timer */
    if (l_hdlDispNext != NONE)
	sTimerCancel (l_hdlDispNext);

    /* Verify if LCD is active */
    if (! l_flgDisplayIsOn)
	return;			// LCD is already OFF, nothing to display

    /* Store function with argument */
    l_DispNextFct = fct;
    l_DispNextUserParm = userParm;

    /* Start timer for duration, or trigger function immediately */
    if (duration > 0)
    {
	if (l_hdlDispNext != NONE)
	    sTimerStart (l_hdlDispNext, duration);
    }
    else
    {
	l_DispNextFctTrigger = true;
	g_flgIRQ = true;	// keep on running
    }
}


/***************************************************************************//**
 *
 * @brief	Display Update for Clock
 *
 * This is a small helper routine to update the display whenever the system
 * time has changed, i.e. every second.  It is usually bound to the AlarmClock
 * module via DisplayUpdateFctInstall().
 *
 ******************************************************************************/
static void DisplayUpdateClock (void)
{
    if (l_flgDisplayIsOn)
    {
	LCD_FIELD_ID_BIT_VAR(l_bitMaskFieldUpd, LCD_CLOCK) = 1;
    }
}


/***************************************************************************//**
 *
 * @brief	Trigger a Display Update
 *
 * This routine must be called when a field needs to be updated, i.e. its
 * contents has changed, so the field must be re-displayed on the LCD.  If
 * the specified field ist currently not displayed on the LCD, the update
 * will be ignored.
 *
 * @param[in] fieldID
 *	The LCD field to be updated.
 *
 ******************************************************************************/
void DisplayUpdateTrigger (LCD_FIELD_ID fieldID)
{
    LCD_FIELD_ID_BIT_VAR(l_bitMaskFieldUpd, fieldID) = 1;

    g_flgIRQ = true;	// keep on running
}


/***************************************************************************//**
 *
 * @brief	Switch LCD Off
 *
 * This routine is called from the RTC interrupt handler to trigger the
 * power-off of the LC-Display, after the specified amount of time has elapsed.
 *
 ******************************************************************************/
static void SwitchLCD_Off(TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    if (l_flgDisplayUpdEnabled)		// NOT in the very beginning
	l_bitMaskFieldActive = 0;

    g_flgIRQ = true;	// keep on running
}


/***************************************************************************//**
 *
 * @brief	Display Next Trigger
 *
 * This routine is called from the RTC interrupt handler to trigger a
 * @ref DISP_NEXT_FCT callback routine, after the specified amount of
 * time is over.  If no callback routine is installed, SwitchLCD_Off()
 * is called instead to switch the LCD off.
 *
 * @see
 * 	DisplayNext(), DisplayText()
 *
 ******************************************************************************/
static void DispNextTrigger(TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    /* See if callback routine has been specified */
    if (l_DispNextFct)
	l_DispNextFctTrigger = true;
    else
	SwitchLCD_Off((TIM_HDL)0);	// no further fct, switch LCD off

    g_flgIRQ = true;	// keep on running
}


/***************************************************************************//**
 *
 * @brief	Clear Transponder Number on the LCD
 *
 * This routine is called if the user asserts the S2 button for more than
 * 5 seconds.  It clears the field for the transponder number on the LCD
 * which is useful to test the RFID receiver with a dedicated reference
 * transponder.
 *
 ******************************************************************************/
static void ClearTransponderID(TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    g_Transponder[0] = EOS;
    DisplayUpdateTrigger (LCD_TRANSPONDER);
}
