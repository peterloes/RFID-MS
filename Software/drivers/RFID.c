/***************************************************************************//**
 * @file
 * @brief	RFID Reader
 * @author	Peter Loes / Ralf Gerhauser
 * @version	2020-09-17 / 2015-04-22
 *
 * This module provides the functionality to communicate with the RFID reader.
 * It contains the following parts:
 * - Power management for RFID reader and UART
 * - UART driver to receive data from the RFID reader
 * - Decoder to handle the received data
 *
 * @see LightBarriers.c
 *
 ****************************************************************************//*
 *
 * Parts are Copyright 2013 Energy Micro AS, http://www.energymicro.com
 *
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ****************************************************************************//*
Revision History:
2020-06-03,rage	- BugFix: Corrected decoding of SR transponder ID.
2019-02-10,rage	- RFID_Decode: Put generic parts at the end of the routine,
		  added debug code to print the received data bytes.
2018-11-12,rage	- Set interrupt priority for UARTs.
2016-04-05,rage	Made all local variables of type "volatile".
2014-11-25,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <string.h>
#include "em_device.h"
#include "em_assert.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "AlarmClock.h"
#include "Display.h"
#include "RFID.h"
#include "Logging.h"

/*=============================== Definitions ================================*/

    /*!@name Hardware Configuration: Power Enable for the RFID Module. */
//@{
#define RFID_POWER_PORT		gpioPortA	//!< Port for power control
#define RFID_POWER_PIN		6		//!< Power Pin: 0=OFF, 1=ON
    //! Set level of the power enable pin
#define SET_RFID_POWER_PIN(level)  IO_Bit(GPIO->P[RFID_POWER_PORT].DOUT,    \
					  RFID_POWER_PIN) = (level)
//@}

    /*!@name Hardware Configuration: Serial Communication via UART. */
//@{
#define UART			USART1		//!< Device to use
#define cmuClock_UART		cmuClock_USART1	//!< CMU clock for thge UART
#define UART_BAUDRATE		9600		//!< Baudrate for RFID reader
#define UART_TX_IRQn		USART1_TX_IRQn	//!< Tx interrupt number
#define UART_TX_IRQHandler	USART1_TX_IRQHandler  //!< Tx interrupt handler
#define UART_RX_IRQn		USART1_RX_IRQn	//!< Rx interrupt number
#define UART_RX_IRQHandler	USART1_RX_IRQHandler  //!< Rx interrupt handler
#define UART_PORT		gpioPortC	//!< Port for TX pin
#define UART_PIN_TX		0		//!< Tx pin on this port
#define UART_PIN_RX		1		//!< Rx pin on this port
#define UART_ROUTE_LOCATION	USART_ROUTE_LOCATION_LOC0  //!< Route location
#define INCLUDE_UART_TX		0		//!< Set 1 to include Tx part
//@}

    /*!@name Hardware Configuration: Serial Communication via UART. */
//@{
#define LCD_CTRL_PORT		gpioPortF	//!< Port for control lines
#define LCD_CTRL_PIN_E		3		//!< Data Enable signal
#define LCD_CTRL_PIN_RW		4		//!< Read/Write signal
#define LCD_CTRL_PIN_RS		5		//!< Register Select signal
//@}

/*========================= Global Data and Routines =========================*/

    /*! Transponder number */
char	 g_Transponder[18];

/*================================ Local Data ================================*/

    /*! Timer handle for switching the RFID reader off after a time. */
static volatile TIM_HDL	l_hdlRFID_Off = NONE;

    /*! Flag if RFID reader should be powered on. */
static volatile bool	l_flgRFID_On;

    /*! Flag if RFID reader is currently powered on. */
static volatile bool	l_flgRFID_IsOn;

    /*! Flag if a new run has been started, i.e. the module is prepared to
     *  receive a transponder number. */
static volatile bool	l_flgNewRun;

    /*! State (index) variable for RFID_Decode. */
static uint8_t	l_State;

/*=========================== Forward Declarations ===========================*/

static void SwitchRFID_Off(TIM_HDL hdl);
static void uartSetup(void);


/***************************************************************************//**
 *
 * @brief	Initialize the RFID Reader
 *
 * This routine initializes the RFID reader and all the required functionality
 * around it, e.g. a timer to switch off the reader when it is not in use.
 *
 ******************************************************************************/
void	RFID_Init (void)
{
    /* Trigger new run flag */
    l_flgNewRun = true;

    /* Get a timer handle to switch the RFID reader off after a time */
    if (l_hdlRFID_Off == NONE)
	l_hdlRFID_Off = sTimerCreate (SwitchRFID_Off);
}


/***************************************************************************//**
 *
 * @brief	Enable RFID reader
 *
 * This routine enables the RFID reader, i.e. it notifies the RFID software
 * module to power up and initialize the reader and the related hardware.
 *
 * @see RFID_Disable(), RFID_Check()
 *
 ******************************************************************************/
void RFID_Enable (void)
{
    l_flgRFID_On = true;

    if (l_hdlRFID_Off != NONE)
	sTimerCancel (l_hdlRFID_Off);	// inhibit power-off of RFID reader
}


/***************************************************************************//**
 *
 * @brief	Disable RFID reader
 *
 * This routine disables the RFID reader, i.e. it notifies the RFID software
 * module to power down the reader after a delay of @ref RFID_POWER_OFF_TIMEOUT
 * seconds.  It also retriggers the @ref l_flgNewRun flag, so the transponder
 * number is logged when received.
 *
 * @see RFID_Enable(), RFID_Check()
 *
 ******************************************************************************/
void RFID_Disable (void)
{
    /* Re-trigger new run flag */
    l_flgNewRun = true;

/* (re-)start timer to switch the RFID reader OFF after time */
    if (l_hdlRFID_Off != NONE)
	sTimerStart (l_hdlRFID_Off, RFID_POWER_OFF_TIMEOUT);
}


/***************************************************************************//**
 *
 * @brief	Power RFID reader On
 *
 * This routine powers the RFID reader on and initializes the related hardware.
 *
 ******************************************************************************/
void RFID_PowerOn (void)
{
    /* Module RFID requires EM1, set bit in bit mask */
    Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 1;

    /* Prepare UART to receive Transponder ID */
    uartSetup();

    /* Configure Power Enable Pin for RFID Module, switch it ON */
    GPIO_PinModeSet (RFID_POWER_PORT, RFID_POWER_PIN, gpioModePushPull, 1);

    /* Reset index */
    l_State = 0;

    /* Generate Log Message */
#ifdef LOGGING
    Log ("RFID is powered ON");
#endif
}


/***************************************************************************//**
 *
 * @brief	Power RFID reader Off
 *
 * This routine powers the RFID reader immediately off.
 *
 ******************************************************************************/
void RFID_PowerOff (void)
{
    /* Set Power Enable Pin for the RFID receiver to OFF */
    SET_RFID_POWER_PIN(0);

    /* Disable clock for USART module */
    CMU_ClockEnable(cmuClock_UART, false);

    /* Disable GPIO pins */
    GPIO_PinModeSet(UART_PORT, UART_PIN_TX, gpioModeDisabled, 0);
    GPIO_PinModeSet(UART_PORT, UART_PIN_RX, gpioModeDisabled, 0);

    /* Module RFID is no longer active, clear bit in bit mask */
    Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 0;

    /* Reset index */
    l_State = 0;

    /* Generate Log Message */
#ifdef LOGGING
    Log ("RFID is powered off");
#endif
}


/***************************************************************************//**
 *
 * @brief	RFID Check
 *
 * This function checks if the RFID reader needs to be powered on or off,
 * and if a transponder number has been received.  If this happens, a
 * log-entry is written and the number is displayed on the LCD if this has
 * been enabled.
 *
 * @note
 * 	This function may be called from standard program, usually the loop
 * 	in module "main.c" - it must not be called from interrupt routines!
 *
 ******************************************************************************/
void	RFID_Check (void)
{
    if (l_flgRFID_On)
    {
	/* RFID reader should be powered ON */
	if (! l_flgRFID_IsOn)
	{
	    RFID_PowerOn();
	    l_flgRFID_IsOn = true;
	}
    }
    else
    {
	/* RFID reader should be powered OFF */
	if (l_flgRFID_IsOn)
	{
	    RFID_PowerOff();
	    l_flgRFID_IsOn = false;
	}
    }
}

/***************************************************************************//**
 *
 * @brief	RFID Power Fail Handler
 *
 * This function will be called in case of power-fail to bring the RFID
 * hardware into a quiescent, power-saving state.
 *
 ******************************************************************************/
void	RFID_PowerFailHandler (void)
{
    /* Cancel timers */
    if (l_hdlRFID_Off != NONE)
	sTimerCancel (l_hdlRFID_Off);

    /* Switch RFID reader off */
    l_flgRFID_On = false;

    if (l_flgRFID_IsOn)
    {
	RFID_PowerOff();
	l_flgRFID_IsOn = false;
    }
}


/***************************************************************************//**
 *
 * @brief	Switch RFID Reader Off
 *
 * This routine is called from the RTC interrupt handler, after the specified
 * amount of time has elapsed, to trigger the power-off of the RFID reader.
 *
 ******************************************************************************/
static void SwitchRFID_Off(TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    l_flgRFID_On = false;

    g_flgIRQ = true;	// keep on running
}


/***************************************************************************//**
 *
 * @brief	Decode RFID
 *
 * This routine is called from the UART interrupt handler, whenever a new
 * byte has been received.  It contains a state machine to extract a valid
 * transponder ID from the data stream, store it into the global variable
 * @ref g_Transponder, and initiate a display update.  The transponder number
 * is additionally logged, if logging is enabled.
 *
 * @note
 * The RFID reader permanently sends the ID as long as the transponder resides
 * in its range.  To prevent a huge amount of log messages, the received data
 * is compared with the previous ID.  It will only be logged if it differs,
 * or the flag @ref l_flgNewRun is set, which indicates a new assertion of the
 * light barriers.
 *
 ******************************************************************************/
static void RFID_Decode(uint32_t byte)
{
const  uint8_t	 v[5] = { 0x0E, 0x00, 0x11, 0x00, 0x05};
const  char	 HexChar[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
 				'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
static uint8_t	 xorsum;	// sum of XORed data bytes
static uint8_t	 w[14];		// buffer for storing received bytes
bool	 flgRecvdID = false;
char	 newTransponder[50]; // also used to store data in case of error message
int	 offs = 0;	// byte offset within the received transponder message
int	 i, pos;


    /* store current byte into receive buffer */
    byte &= 0xFF;		// only bit 7~0 contains the data
    DBG_PUTC('[');DBG_PUTC(HexChar[(byte >> 4) & 0xF]);
    DBG_PUTC(HexChar[byte & 0xF]);DBG_PUTC(']');

   w[l_State] = (uint8_t)byte;
    
    /*
     * NOTE: Most of the code has been taken from file "RFID_tag.c"
     */
 

    switch (l_State)	// the state machine!
    {
    case 0:
	xorsum = 0;
	/* no break */
    case 1:
    case 2:
    case 3:
    case 4:
	// Verify prefix
        if (byte != v[l_State])
        {
	    l_State = 0; // restart state machine
	    break;       // break!
	}
        /* no break */
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
	xorsum ^= byte;		// build checksum
	l_State++;		// go on
	break;			// break!

    case 13:
	if (w[13] != xorsum)	//handle ERROR case
	{
	    /* Print Hex Codes of the wrong message */
	    pos = 0;
            for (i=0; i<=13; i++)
	    {
		newTransponder[pos++] = ' ';
		newTransponder[pos++] = HexChar[(w[i] >> 4) & 0x0F];
		newTransponder[pos++] = HexChar[(w[i]) & 0x0F];
            }
	    newTransponder[pos] = '\0';            
            LogError("RFID_Decode(): recv.XOR=0x%02X, calc.XOR=0x%02X,"
		     " data is%s", w[13], xorsum, newTransponder);
                
            l_State = 0;	// restart state machine
            break;
                
        }
        flgRecvdID = true;	// ID has been received - set flag
        offs = 12;		// byte offset within the message
        break;
	
     default:
	 l_State = 0;	// restart state machine
	 break;
    }
          
    /* see if a transponder ID has been received */
    if (flgRecvdID)
    {
        l_State = 0;		// restart state machine
        
       for (i=0; i < 8; i++)	// copy w and convert to ASCII HEX
       {
	    newTransponder[2*i]	  = HexChar[(w[offs-i]>>4) & 0x0F];
	    newTransponder[2*i+1] = HexChar[(w[offs-i]) & 0x0F];
       }
       newTransponder[16] = '\0';
    
       /* see if a new run or Transponder Number has changed */
       if (l_flgNewRun  ||  strcmp (newTransponder, g_Transponder))
       {
          l_flgNewRun = false;	// clear flag

	  /* yes, store new Transponder Number and update Display */
	  strcpy (g_Transponder, newTransponder);

	  DisplayUpdateTrigger (LCD_TRANSPONDER);

	  /* Generate Log Message */
#ifdef LOGGING
	 Log ("Transponder: %s", g_Transponder);
#endif
      }
   }
}

/*============================================================================*/
/*=============================== UART Routines ==============================*/
/*============================================================================*/

/* Setup UART in async mode for RS232*/
static USART_TypeDef           * uart   = UART;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;


/******************************************************************************
* @brief  uartSetup function
*
******************************************************************************/
static void uartSetup(void)
{
  /* Enable clock for USART module */
  CMU_ClockEnable(cmuClock_UART, true);

  /* Configure GPIO pins */
  GPIO_PinModeSet(UART_PORT, UART_PIN_TX, gpioModePushPull, 1);
  GPIO_PinModeSet(UART_PORT, UART_PIN_RX, gpioModeInput, 0);


  /* Prepare struct for initializing UART in asynchronous mode */
  uartInit.enable       = usartDisable;   // Don't enable UART upon intialization
  uartInit.refFreq      = 0;              // Set to 0 to use reference frequency
  uartInit.baudrate     = UART_BAUDRATE;  // Baud rate
  uartInit.oversampling = usartOVS16;     // Oversampling. Range is 4x, 6x, 8x or 16x
  uartInit.databits     = usartDatabits8; // Number of data bits. Range is 4 to 10
  uartInit.parity       = usartEvenParity;// Parity mode
  uartInit.stopbits     = usartStopbits1; // Number of stop bits. Range is 0 to 2
#if defined( USART_INPUT_RXPRS ) && defined( USART_CTRL_MVDIS )
  uartInit.mvdis        = false;          // Disable majority voting
  uartInit.prsRxEnable  = false;          // Enable USART Rx via Peripheral Reflex System
  uartInit.prsRxCh      = usartPrsRxCh0;  // Select PRS channel if enabled
#endif

  /* Initialize USART with uartInit struct */
  USART_InitAsync(uart, &uartInit);

  /* Prepare UART Rx and Tx interrupts */
  USART_IntClear(uart, _USART_IF_MASK);
  USART_IntEnable(uart, USART_IF_RXDATAV);
  NVIC_SetPriority(UART_RX_IRQn, INT_PRIO_UART);
  NVIC_ClearPendingIRQ(UART_RX_IRQn);
  NVIC_EnableIRQ(UART_RX_IRQn);
#if INCLUDE_UART_TX
  NVIC_ClearPendingIRQ(UART_TX_IRQn);
  NVIC_EnableIRQ(UART_TX_IRQn);
#endif

  /* Enable I/O pins at UART location #2 */
  uart->ROUTE = USART_ROUTE_RXPEN
#if INCLUDE_UART_TX
	      | USART_ROUTE_TXPEN
#endif
	      | UART_ROUTE_LOCATION;

#if INCLUDE_UART_TX
  /* Enable UART Rx and Tx */
  USART_Enable(uart, usartEnable);
#else
  /* Enable UART receiver only */
  USART_Enable(uart, usartEnableRx);
#endif
}


/**************************************************************************//**
 *
 * @brief UART RX IRQ Handler
 *
 * This interrupt service routine is called whenever a byte has been received
 * from the RFID reader.  It calls RFID_Decode() to extract a valid ID from the
 * data stream.
 *
 *****************************************************************************/
void UART_RX_IRQHandler(void)
{
    /* Check for RX data valid interrupt */
    if (uart->STATUS & USART_STATUS_RXDATAV)
    {
	/* Decode data */
	RFID_Decode (uart->RXDATA);

	/* Clear RXDATAV interrupt */
	USART_IntClear(USART1, USART_IF_RXDATAV);
    }
}

#if INCLUDE_UART_TX
/**************************************************************************//**
 * @brief UART TX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 *****************************************************************************/
void UART_TX_IRQHandler(void)
{
  /* Clear interrupt flags by reading them. */
  USART_IntGet(USART1);

  /* Check TX buffer level status */
  if (uart->STATUS & USART_STATUS_TXBL)
  {
    if (txBuf.pendingBytes > 0)
    {
      /* Transmit pending character */
      USART_Tx(uart, txBuf.data[txBuf.rdI]);
      txBuf.rdI = (txBuf.rdI + 1) % BUFFERSIZE;
      txBuf.pendingBytes--;
    }

    /* Disable Tx interrupt if no more bytes in queue */
    if (txBuf.pendingBytes == 0)
    {
      USART_IntDisable(uart, USART_IF_TXBL);
    }
  }
}
#endif
