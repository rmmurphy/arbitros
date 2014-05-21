/*---------------------------------------------------------------------------*
 * Copyright (C) 2013 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * File Name   : drv_sd.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for controlling access to an
 *               external sd card via a spi interface. It is built upon the
 *               work of William Greiman.
 *
 * Last Update : June, 16, 2013
 *---------------------------------------------------------------------------*/
#ifndef drv_sd_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define drv_sd_h

   #define SD_CARD_SD1  (0)
   #define SD_CARD_SD2  (1)
   #define SD_CARD_SDHC (2)

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "hal_gpio.h"
   #include "hal_spi.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      SD_OPERATION_FAILED = -3, /*The requirest operation failed*/
      SD_CARD_INIT_FAILED = -2, /*SD card failed to initialize*/
      SD_OUT_OF_HEAP      = -1, /*No more memory.*/
      SD_PASSED           = 0   /*Configuration good.*/

   }t_sdError; /*Possible error conditions returned by the device's
                 ioctl routine*/

   typedef enum
   {
      SD_INIT,      /*Initialize an SD card*/
      SD_RMDASHR,   /*Remove all the files within, and directory itself*/
      SD_ERASE,     /*Erase the contents of the file*/
      SD_REWIND,    /*Sets the file position back to the beginning*/
      SD_CARD_INFO, /*Print the type and size of card*/
      SD_LS,        /*List the contents of the current working directory*/
      SD_MKDIR,     /*Make a new directory*/
      SD_RMDIR,     /*Remove a directory*/
      SD_RM,        /*Remove a file*/
      SD_CD,        /*Change working directory*/
      SD_GET_SIZE   /*Get the size of a file*/
   }t_sdCmd;

   typedef struct
   {

      /*---------------------------------------------------------------------*
       * The particular spi port connected to the sd card.
       *---------------------------------------------------------------------*/
      t_spiChanId t_spiChan;

      /*---------------------------------------------------------------------*
       * The port used for controlling the chip select pin.
       *---------------------------------------------------------------------*/
      t_gpioPort t_csPort;

      /*---------------------------------------------------------------------*
       * The CS pin on the port given by 't_csPort'.
       *---------------------------------------------------------------------*/
      uint8_t t_csPin;

      /*---------------------------------------------------------------------*
       * This value represents a unique number assigned to this driver and has
       * to be different from all others registered with the kernel.
       *---------------------------------------------------------------------*/
      uint8_t c_majorNum;

   }t_sdSetup;

   typedef struct
   {
      uint8_t c_cardType;

   }t_cardInfoMsg;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_error drv_sdInit( t_sdSetup t_setup);

   void drv_sdExit( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef drv_sd_h*/