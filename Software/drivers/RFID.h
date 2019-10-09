/***************************************************************************//**
 * @file
 * @brief	Header file of module RFID.c
 * @author	Ralf Gerhauser
 * @version	2016-02-24
 ****************************************************************************//*
Revision History:
2016-02-24,rage	Added prototype for RFID_PowerOff().
2014-11-25,rage	Initial version.
*/

#ifndef __INC_RFID_h
#define __INC_RFID_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

    /*!@brief Time in [s] after which the RFID reader is powered-off. */
#ifndef RFID_POWER_OFF_TIMEOUT
    #define RFID_POWER_OFF_TIMEOUT	10
#endif

/*================================ Global Data ===============================*/

extern char	 g_Transponder[18];

/*================================ Prototypes ================================*/

    /* Initialize the RFID module */
void	RFID_Init (void);

    /* Enable RFID reader */
void	RFID_Enable (void);

    /* Disable RFID reader */
void	RFID_Disable (void);

    /* Check if to power-on/off RFID reader, get tranponder number */
void	RFID_Check (void);

    /* Power RFID reader Off */
void	RFID_PowerOff (void);

#endif /* __INC_RFID_h */
