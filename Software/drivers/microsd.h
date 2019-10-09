/***************************************************************************//**
 * @file
 * @brief	Header file of module microsd.c
 * @author	Silicon Labs
 * @author	Ralf Gerhauser
 * @version	2016-02-21
 *
 * This header file contains the configuration and prototypes for the
 * SD-Card interface.  The name "microsd.h" must not be changed, because the
 * file is included by "diskio.c" as part of the FAT filesystem driver.
 *
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ***************************************************************************//**
Revision History:
2016-02-21,rage	Added prototype for IsDiskRemoved().
2015-02-18,rage	Initial version, derived from EFM32GG_DK3750 development kit.
*/

#ifndef __INC_microsd_h
#define __INC_microsd_h

/*=============================== Header Files ===============================*/

#include "em_device.h"
#include "em_gpio.h"
#include "diskio.h"		// DSTATUS
#include "ff.h"			// FS_FAT12/16/32
#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

/*!@name Configuration of the SD-Card interface. */
//@{
#define MICROSD_PWR_GPIO_PORT	gpioPortD	//!< Port for Power Enable
#define MICROSD_PWR_PIN		0		//!< Power Enable Pin
#define MICROSD_PWR_ON		1		//!< Set 1 to enable power
#define MICROSD_PWR_OFF		0		//!< Set 0 to disable power

#define MICROSD_SPI_GPIO_PORT	gpioPortC	//!< GPIO port for SPI
#define MICROSD_SPI_MOSI_PIN	2		//!< SPI MOSI Pin
#define MICROSD_SPI_MISO_PIN	3		//!< SPI MISO Pin
#define MICROSD_SPI_CLK_PIN	4		//!< SPI Clock Pin
#define MICROSD_SPI_CS_PIN	5		//!< SPI Chip Select (CS) Pin
#define MICROSD_CD_PIN		6		//!< Card Detect Pin
#define MICROSD_WP_PIN		7		//!< Write Protect Pin
#define MICROSD_CD_PULLUP_PIN	12		//!< Pull-Up for CD Signal

#define MICROSD_USART		USART2		//!< Use USART2 for SPI
#define MICROSD_CMUCLOCK	cmuClock_USART2	//!< Enable clock for USART
#define MICROSD_LOC		USART_ROUTE_LOCATION_LOC0  //!< Use location 0

#define MICROSD_HI_SPI_FREQ	8000000		//!< High speed is 8MHz
#define MICROSD_LO_SPI_FREQ	 100000		//!< Low speed is 100kHz
//@}

/*!@name Definitions for MMC/SDC commands */
//@{
#define CMD0	(0)		//!< GO_IDLE_STATE
#define CMD1	(1)		//!< SEND_OP_COND
#define ACMD41	(41 | 0x80)	//!< SEND_OP_COND (SDC)
#define CMD8	(8)		//!< SEND_IF_COND
#define CMD9	(9)		//!< SEND_CSD
#define CMD10	(10)		//!< SEND_CID
#define CMD12	(12)		//!< STOP_TRANSMISSION
#define ACMD13	(13 | 0x80)	//!< SD_STATUS (SDC)
#define CMD16	(16)		//!< SET_BLOCKLEN
#define CMD17	(17)		//!< READ_SINGLE_BLOCK
#define CMD18	(18)		//!< READ_MULTIPLE_BLOCK
#define CMD23	(23)		//!< SET_BLOCK_COUNT
#define ACMD23	(23 | 0x80)	//!< SET_WR_BLK_ERASE_COUNT (SDC)
#define CMD24	(24)		//!< WRITE_BLOCK
#define CMD25	(25)		//!< WRITE_MULTIPLE_BLOCK
#define CMD41	(41)		//!< SEND_OP_COND (ACMD)
#define CMD55	(55)		//!< APP_CMD
#define CMD58	(58)		//!< READ_OCR
//@}

/*================================ Global Data ===============================*/

//extern uint32_t  g_LB_ActiveMask;

/*================================ Prototypes ================================*/

/* High Level Routines */
void	 DiskInit (void);
bool	 DiskCheck (void);
bool	 IsDiskRemoved (void);
bool	 IsFileHandleValid (FIL *pHdl);
void	 CD_Handler (int extiNum, bool extiLvl, uint32_t timeStamp);
uint32_t DiskSize (void);
char	*FindFile (char *dirpath, char *filename);

/* Initialize the SD-Card interface */
void      MICROSD_Init(void);
void      MICROSD_Deinit(void);

int       MICROSD_Select(void);
void      MICROSD_Deselect(void);

void      MICROSD_PowerOn(void);
void      MICROSD_PowerOff(void);

int       MICROSD_BlockRx(uint8_t *buff, uint32_t btr);
int       MICROSD_BlockTx(const uint8_t *buff, uint8_t token);

uint8_t   MICROSD_SendCmd(uint8_t cmd, DWORD arg);
uint8_t   MICROSD_XferSpi(uint8_t data);

void      MICROSD_SpiClkFast(void);
void      MICROSD_SpiClkSlow(void);

bool      MICROSD_TimeOutElapsed(void);
void      MICROSD_TimeOutSet(uint32_t msec);


#endif /* __INC_microsd_h */
