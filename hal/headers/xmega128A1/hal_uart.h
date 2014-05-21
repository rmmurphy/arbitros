/*---------------------------------------------------------------------------*
 * Copyright (C) 2011-2012 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
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
 * File Name   : hal_uart.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for controlling
 *               the UART module.
 *
 * Last Update : Oct 26, 2011
 *---------------------------------------------------------------------------*/
#ifndef hal_uart_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define hal_uart_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      UART_BUSY                 = -15, /*Uart processing a transaction*/
      UART_INVALID_MODULE       = -14, /*Invalid UART module*/
      UART_MODE_NOT_SUPPORTED   = -13, /*The selected COM mode not supported*/
      UART_INVALID_BAUD_RATE    = -12, /*Invalid baud rate*/
      UART_INVALID_STOPBIT_MODE = -11, /*Invalid number of stop bits*/
      UART_INVALID_PARITY_MODE  = -10, /*Invalid parity mode*/
      UART_INVALID_CHAR_SIZE    = -9,  /*Invalid character size*/
      UART_INVALID_COM_MODE     = -8,  /*Invalid communication mode*/
      UART_CHANNEL_OPEN         = -7,  /*A uart channel is already open*/
      UART_NO_DMA               = -6,  /*No DMA channels available*/
      UART_INVALID_INT_TYPE     = -5,  /*Request of invalid interrupt*/
      UART_INT_NOT_OPEN         = -4,  /*Interrupt hasn't been mapped*/
      UART_NO_CONFIG            = -3,  /*UART config hasn't been called*/
      UART_INVALID_HANDLE       = -2,  /*Handle doesn't map to a UART*/
      UART_OUT_OF_HEAP          = -1,  /*No more memory.*/
      UART_PASSED               = 0    /*Configuration good.*/

   }t_uartError;

   typedef enum
   {
      UART_1 = 0,  /*UART C0*/
      UART_2 = 3,  /*UART C1*/
      UART_3 = 6,  /*UART D0*/
      UART_4 = 9,  /*UART D1*/
      UART_5 = 12, /*UART E0*/
      UART_6 = 15, /*UART E1*/
      UART_7 = 18, /*UART F0*/
      UART_8 = 21  /*UART F1*/

   }t_uartChanId;

   typedef enum
   {
      ASYNC = 0,
      SYNC,
      IRDA,
      MASTER_SPI

   }t_comMode;

   typedef enum
   {
      CHAR_5BIT = 0,
      CHAR_6BIT,
      CHAR_7BIT,
      CHAR_8BIT

   }t_charSize;

   typedef enum
   {
      NO_PARITY = 0,
      EVEN_PARITY,
      ODD_PARITY

   }t_parityMode;

   typedef enum
   {
      ONE_STOP_BIT = 0,
      TWO_STOP_BITS

   }t_stopBitMode;

   typedef struct
   {
      t_comMode t_comMd;
      t_charSize t_charSz;
      t_parityMode t_parityMd;
      t_stopBitMode t_stopBitMd;
      uint32_t i_baudRate;
      bool b_enTxDma;
      bool b_enRxDma;
      void (*pf_rxCallBack)( uint16_t s_data);
      void (*pf_txCallBack)( uint16_t s_size);

   }t_uartConfig; /*Configuration for a particular UART channel*/

   typedef volatile int16_t t_UARTHNDL; /*Handle to a particular UART*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * This function request access to the next available DMA channel.
    *------------------------------------------------------------------------*/
   t_UARTHNDL hal_requestUartChannel( t_uartChanId t_chanId);

   /*------------------------------------------------------------------------*
    * This function releases access over a particular DMA channel.
    *------------------------------------------------------------------------*/
   t_uartError hal_releaseUartChannel( t_UARTHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Configures the DMA transaction for the channel pointed to by
    * 't_handle'.
    *------------------------------------------------------------------------*/
   t_uartError hal_configureUartChannel( t_UARTHNDL t_handle,
                                         t_uartConfig t_conf);

   t_uartError hal_enableUartRxInt( t_UARTHNDL t_handle);

   t_uartError hal_disableUartRxInt( t_UARTHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Write a single byte to the DATA register of the particular uart
    * channel pointed to by 't_handle'.
    *------------------------------------------------------------------------*/
   t_uartError hal_uartWriteByte( t_UARTHNDL t_handle,
                                  uint8_t c_byte);

   /*------------------------------------------------------------------------*
    * Read a block of data from a particular uart channel.
    *------------------------------------------------------------------------*/
   t_uartError hal_uartReadBlock( t_UARTHNDL t_handle,
								          int8_t *pc_rxBuffer,
								          uint16_t s_numBytes);

   /*------------------------------------------------------------------------*
    * Write a block of data to a particular uart channel.
    *------------------------------------------------------------------------*/
   t_uartError hal_uartWriteBlock( t_UARTHNDL t_handle,
								           int8_t *pc_txBuffer,
								           uint16_t s_numBytes);

   /*------------------------------------------------------------------------*
    * This function returns the address of the RX/TX data register for the
    * particular uart pointed to by 't_handle'.
    *------------------------------------------------------------------------*/
   int32_t hal_uartGetDataAddress( t_UARTHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Return the current baud rate for a particular uart channel.
    *------------------------------------------------------------------------*/
   int32_t hal_uartGetBaudRate( t_UARTHNDL t_handle);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef hal_uart_h*/
