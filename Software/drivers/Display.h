/***************************************************************************//**
 * @file
 * @brief	Header file of module Display.c
 * @author	Ralf Gerhauser
 * @version	2014-11-25
 ****************************************************************************//*
Revision History:
2014-11-25,rage	Initial version.
*/

#ifndef __INC_Display_h
#define __INC_Display_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "config.h"		// include project configuration parameters
#include "Keys.h"

/*=============================== Definitions ================================*/

    /*!@brief Time in [s] after which the LCD is powered-off. */
#ifndef LCD_POWER_OFF_TIMEOUT
    #define LCD_POWER_OFF_TIMEOUT	20
#endif

/*=========================== Typedefs and Structs ===========================*/

    /*!@brief LCD field identifiers.
     * These enumerations are used for two purposes:
     * -# As index within an array of type @ref LCD_FIELD, which specifies
     *    the position and width of a field on the LC-Display.
     * -# As identifier when such a field needs to be updated on the LCD,
     *    see @ref LCD_FIELD_ID_MASK_S1 and DisplayUpdateCheck() for example.
     */
typedef enum
{
    LCD_LINE1_BLANK,	//!< 0: Leave line 1 of the LCD blank
    LCD_LINE2_BLANK,	//!< 1: Leave line 2 of the LCD blank
    LCD_LINE1_TEXT,	//!< 2: Display text in line 1, see DisplayText()
    LCD_LINE2_TEXT,	//!< 3: Display text in line 2, see DisplayText()
    LCD_LIGHT_BARRIERS,	//!< 4: Current state of the light barriers
    LCD_BATTERY,	//!< 5: Show information about the battery
    LCD_CLOCK,		//!< 6: Display current date and time
    LCD_TRANSPONDER,	//!< 7: Show transponder number
    LCD_FIELD_ID_CNT	//!< LCD Field ID count
} LCD_FIELD_ID;

    /*!@name Macros and defines to deal with field ID bit masks */
//@{
    /*!@brief Bit associated with <b>id</b> in bit mask */
#define LCD_FIELD_ID_BIT(id)		(1UL << (id))

    /*!@brief Address of bit associated with <b>id</b> in variable */
#define LCD_FIELD_ID_BIT_VAR(var, id)	Bit(var, (id))

    /*!@brief Bit mask that specifies the fields to display when the DCF77
     * clock was synchronised after power-up.  Field ID @ref LCD_LINE2_BLANK
     * lets line 2 be cleared, i.e. it is empty.
     */
#define LCD_FIELD_ID_MASK_DCF77	  LCD_FIELD_ID_BIT(LCD_CLOCK)		\
				| LCD_FIELD_ID_BIT(LCD_LINE2_BLANK)

    /*!@brief Bit mask that specifies the fields to display when button S1
     * is asserted.
     */
#define LCD_FIELD_ID_MASK_S1	  LCD_FIELD_ID_BIT(LCD_LIGHT_BARRIERS)	\
				| LCD_FIELD_ID_BIT(LCD_BATTERY)

    /*!@brief Bit mask that specifies the fields to display when button S2
     * is asserted.
     */
#define LCD_FIELD_ID_MASK_S2	  LCD_FIELD_ID_BIT(LCD_CLOCK)		\
				| LCD_FIELD_ID_BIT(LCD_TRANSPONDER)
//@}

    /*!@brief LCD field definition.
     *
     * This structure defines the location of a field on the LC-Display, i.e.
     * the <b>X</b> and <b>Y</b> coordinates of the field, and its <b>Width</b>.
     */
typedef struct
{
    uint8_t	X;		//!< X position of the field , starting at 0
    uint8_t	Y;		//!< Y position of the field , starting at 0
    uint8_t	Width;		//!< Width of the field in characters
} LCD_FIELD;

    /*!@brief Function to be called when specified duration is over
     *
     * This type of function is used by DisplayNext() to install a callback
     * routine which is executed after the specified amount of time has elapsed.
     * The function argument <b>userParm</b> can be used to specify the next
     * item to be displayed.
     */
typedef void	(* DISP_NEXT_FCT)(int userParm);

/*================================ Prototypes ================================*/

void	DisplayInit (const LCD_FIELD *pField);
void	DisplayKeyHandler (KEYCODE keycode);
void	DisplayUpdateCheck (void);
void	DisplayUpdateTrigger (LCD_FIELD_ID fieldID);
void	DisplayUpdEnable (void);
void	DisplayText (int lineNum, const char *frmt, ...);
void	DisplayNext (unsigned int duration, DISP_NEXT_FCT fct, int userParm);


#endif /* __INC_Display_h */
