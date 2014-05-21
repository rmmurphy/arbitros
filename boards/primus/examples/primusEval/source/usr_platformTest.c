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
 * File Name   : usr_platformTest.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : Demostrates how to use a few of the peripherals available
 *               with the arbitros operating system.
 *
 * References  :
 *
 * Last Update : Jan, 1, 2013
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "arb_error.h"
#include "arb_device.h"
#include "arb_printf.h"
#include "arb_thread.h"
#include "arb_semaphore.h"
#include "arb_sysTimer.h"
#include "arb_mailbox.h"
#include "arb_scheduler.h"
#include "hal_dma.h"
#include "hal_uart.h"
#include "hal_gpio.h"
#include "hal_twi.h"
#include "hal_timer.h"
#include "hal_spi.h"
#include "usr_platformTest.h"
#include "drv_sd.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define MAX_BUFFER_SIZE   (24)
#define TWI_SLAVE_ADDRESS (0x7F)

/*---------------------------------------------------------------------------*
 * Private Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{

   /*------------------------------------------------------------------------*
    * The main platform test interface thread
    *------------------------------------------------------------------------*/
   t_THRDHANDLE t_platformTest;
t_THRDHANDLE t_platformTest2;
   /*------------------------------------------------------------------------*
    * Handle to a particular uart used during one of the tests.
    *------------------------------------------------------------------------*/
   t_UARTHNDL t_uHandle;

   /*------------------------------------------------------------------------*
    * The mailbox for incoming messages to the platform test interface.
    *------------------------------------------------------------------------*/
   t_MAILBOXHNDL t_inMbxHndl;

   /*------------------------------------------------------------------------*
    * The mailbox for outgoing messages from the platform test interface.
    *------------------------------------------------------------------------*/
   t_MAILBOXHNDL t_outMbxHndl;

   /*------------------------------------------------------------------------*
    * Handle to a particular slave TWI used during one of the tests.
    *------------------------------------------------------------------------*/
   t_TWIHNDL t_twiSlave;

   /*------------------------------------------------------------------------*
    * Handle to a particular master TWI used during one of the tests.
    *------------------------------------------------------------------------*/
   t_TWIHNDL t_twiMaster;

   /*------------------------------------------------------------------------*
    * Handle to a particular timer used during one of the tests.
    *------------------------------------------------------------------------*/
   t_TIMERHNDL t_tHandle;

   /*------------------------------------------------------------------------*
    * Handle to a particular gpio interrupt used during one of the tests.
    *------------------------------------------------------------------------*/
   t_GPIOHNDL t_gHandle;

   /*------------------------------------------------------------------------*
    * Handle to a particular master SPI used during one of the tests.
    *------------------------------------------------------------------------*/
   t_SPIHNDL t_spiMaster;

   /*------------------------------------------------------------------------*
    * The number of bytes transfered across a peripheral
    *------------------------------------------------------------------------*/
   uint8_t c_numBytesTrans;

   /*------------------------------------------------------------------------*
    * Buffer for sending data across a peripheral
    *------------------------------------------------------------------------*/
   int8_t ac_txData[MAX_BUFFER_SIZE];

   /*------------------------------------------------------------------------*
    * Buffer for receiving data from a peripheral
    *------------------------------------------------------------------------*/
   int8_t ac_rxData[MAX_BUFFER_SIZE];

}t_platformTestObject;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
t_platformTestObject gt_platTestObject;

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void usr_platformTest2( t_parameters t_param,
                              t_arguments  t_args)
{

   while( RUN_FOREVER)
   {
      arb_sleep(20);
   }
}

static void usr_platformTest( t_parameters t_param,
                              t_arguments  t_args)
{
   int16_t s_numBytes;
   int16_t s_numMessages;
   t_twiError t_tErr;
   t_spiError t_sErr;

   while( RUN_FOREVER)
   {

      /*---------------------------------------------------------------------*
       * Block until a message is received from the console.
       *---------------------------------------------------------------------*/
      s_numBytes = arb_mailboxRead( gt_platTestObject.t_inMbxHndl,
                                    gt_platTestObject.ac_txData,
                                    (uint16_t)sizeof( gt_platTestObject.
                                    ac_txData));

      if( s_numBytes > 0)
      {
         s_numMessages = arb_mailboxGetNumMessages( gt_platTestObject.
         t_inMbxHndl);

         switch( gt_platTestObject.ac_txData[0])
         {
            case PLAT_UART_TEST:

               /*------------------------------------------------------------*
                * Store the number of bytes received so we know how much
                * the DMA transfered when the message is sent back to the
                * console via the mailbox.
                *------------------------------------------------------------*/
               gt_platTestObject.c_numBytesTrans = s_numBytes;

               hal_uartReadBlock( gt_platTestObject.t_uHandle,
                                  gt_platTestObject.ac_rxData,
                                  s_numBytes);

               hal_uartWriteBlock( gt_platTestObject.t_uHandle,
                                   gt_platTestObject.ac_txData,
                                   s_numBytes);

            break; /*End case PLAT_UART_TEST:*/

            case PLAT_TWI_TEST:

               /*------------------------------------------------------------*
                * Write the message from the master TWI to the slave TWI.
                *------------------------------------------------------------*/
               t_tErr = hal_twiMasterWrite( gt_platTestObject.t_twiMaster,
                                            (uint8_t *)gt_platTestObject.
                                            ac_txData,
                                            s_numBytes,
                                            TWI_SLAVE_ADDRESS,
                                            0);

               if( t_tErr < 0)
                  exit(0);

            break;/*End case PLAT_TWI_TEST:*/

            case PLAT_TIMER_GPIO_TEST:

               if( gt_platTestObject.ac_txData[1] == 1)
                  hal_startTimer( gt_platTestObject.t_tHandle);
               else
                  hal_stopTimer( gt_platTestObject.t_tHandle);

            break;/*End case PLAT_TIMER_GPIO_TEST:*/

            case PLAT_SPI_TEST:

               t_sErr = hal_spiReadWrite( gt_platTestObject.t_spiMaster,
                                          false,
                                          gt_platTestObject.ac_txData,
                                          gt_platTestObject.ac_rxData,
                                          s_numBytes);

            break;/*End case PLAT_SPI_TEST;*/

         }/*End switch( gt_platTestObject.ac_txData[0])*/

      }/*End if( s_numBytes > 0)*/

      arb_sleep(10);

   }/*End while( RUN_FOREVER)*/

}/*End usr_platformTest*/

static int8_t usr_twiSlaveTransFinished( t_twiStatus t_status,
                                         int8_t c_data)
{
   int8_t c_return = 0;
   static int8_t c_rxDataPtr = 0;
   int16_t s_numBytes;

   switch( t_status)
   {
      case TWI_BUS_ERROR:

      break;/*End case TWI_BUS_ERROR:*/

      case TWI_COLLISION:

      break;/*End case TWI_COLLISION:*/

      case TWI_TRANS_COMPLETE:

         /*------------------------------------------------------------------*
          * Send the message back to the console...
          *------------------------------------------------------------------*/
         s_numBytes = arb_mailboxWrite( gt_platTestObject.t_outMbxHndl,
                                        gt_platTestObject.ac_rxData,
                                        (uint16_t)c_rxDataPtr);

         c_rxDataPtr = 0;
         memset( (void *)gt_platTestObject.ac_rxData, 0,
         sizeof( gt_platTestObject.ac_rxData));

      break;/*End case TWI_TRANS_COMPLETE:*/

      case TWI_SLAVE_READ: /*Send byte to master*/

      break;/*End TWI_SLAVE_READ*/

      case TWI_SLAVE_WRITE: /*Store byte sent from master*/

         gt_platTestObject.ac_rxData[c_rxDataPtr] = c_data;
         c_rxDataPtr++;
         if( c_rxDataPtr == MAX_BUFFER_SIZE)
            c_rxDataPtr = 0;

      break;/*End TWI_SLAVE_WRITE*/

      default:

      break;

   }/*End switch( t_status)*/

   return c_return; /*If a read operation from the master return valid data*/

}/*End usr_twiSlaveTransFinished*/

static void uartRxComplete( uint16_t s_size)
{
   int16_t s_numBytes;

   /*------------------------------------------------------------------------*
    * Send the message back to the console...
    *------------------------------------------------------------------------*/
   s_numBytes = arb_mailboxWrite( gt_platTestObject.t_outMbxHndl,
                                  gt_platTestObject.ac_rxData,
                                  s_size);

   memset( (void *)gt_platTestObject.ac_rxData, 0,
   sizeof( gt_platTestObject.ac_rxData));

}/*End uartRxComplete*/

static void usr_timerTestInterrupt( void)
{
   hal_gpioToggle( GPIO_PORTH,
                   PIN_4);

}/*End usr_timerTestInterrupt*/

static void usr_timerTestGpioInterrupt( t_gpioPort t_port,
                                        uint8_t c_pin)
{
   int16_t s_numBytes;
   uint8_t ac_temp[1];

   /*------------------------------------------------------------------------*
    * Send the message back to the console...
    *------------------------------------------------------------------------*/
   s_numBytes = arb_mailboxWrite( gt_platTestObject.t_outMbxHndl,
                                  (int8_t *)ac_temp,
                                  (uint16_t)sizeof( ac_temp));

}/*End usr_timerTestGpioInterrupt*/

static void spiMasterComplete( int8_t *pc_rxData,
                               uint16_t s_length)
{
   int16_t s_numBytes;

   s_numBytes = arb_mailboxWrite( gt_platTestObject.t_outMbxHndl,
                                  pc_rxData,
                                  (uint16_t)s_length);

}/*End spiMasterComplete*/

static void usr_uartLoopbackTestInit( void)
{

   t_uartError t_err;
   t_uartConfig t_uConf;

   /*------------------------------------------------------------------------*
    * Request access to UART 6
    *------------------------------------------------------------------------*/
   gt_platTestObject.t_uHandle = hal_requestUartChannel( UART_6); /*UART E1*/
   if( gt_platTestObject.t_uHandle < 0)
      exit(0);

   t_uConf.t_comMd       = ASYNC;
   t_uConf.t_charSz      = CHAR_8BIT;
   t_uConf.t_parityMd    = NO_PARITY;
   t_uConf.t_stopBitMd   = ONE_STOP_BIT;
   t_uConf.i_baudRate    = 112500;
   t_uConf.b_enRxDma     = false;
   t_uConf.b_enTxDma     = false;
   t_uConf.pf_rxCallBack = &uartRxComplete;
   t_uConf.pf_txCallBack = NULL;

   /*------------------------------------------------------------------------*
    * Configure UART 6
    *------------------------------------------------------------------------*/
   t_err = hal_configureUartChannel( gt_platTestObject.t_uHandle,
                                     t_uConf);

   if( t_err < 0)
      exit(0);

}/*End usr_uartLoopbackTestInit*/

static void usr_twiLoopbackTestInit( void)
{
   t_twiConfig t_tConfig;
   t_twiError t_err;

   /*------------------------------------------------------------------------*
    * Request access to TWI channel 2...
    *------------------------------------------------------------------------*/
   gt_platTestObject.t_twiMaster = hal_requestTwiChannel( TWI_2);

   if( gt_platTestObject.t_twiMaster < 0)
      exit(0);

   /*------------------------------------------------------------------------*
    * Configure TWI channel 2 as master...
    *------------------------------------------------------------------------*/
   t_tConfig.t_mode         = TWI_MASTER;
   t_tConfig.i_baud         = 400000;

   t_err = hal_configureTwiChannel( gt_platTestObject.t_twiMaster,
                                    t_tConfig);

   if( t_err < 0)
      exit(0);

   /*------------------------------------------------------------------------*
    * Request access to TWI channel 3...
    *------------------------------------------------------------------------*/
   gt_platTestObject.t_twiSlave = hal_requestTwiChannel( TWI_3);

   if( gt_platTestObject.t_twiSlave < 0)
      exit(0);

   t_tConfig.t_mode           = TWI_SLAVE;
   t_tConfig.c_slaveAddress   = TWI_SLAVE_ADDRESS;
   t_tConfig.pf_transComplete = &usr_twiSlaveTransFinished;

   /*------------------------------------------------------------------------*
    * Configure TWI channel 3 as slave
    *------------------------------------------------------------------------*/
   t_err = hal_configureTwiChannel( gt_platTestObject.t_twiSlave,
                                    t_tConfig);

   if( t_err < 0)
      exit(0);

}/*End usr_twiLoopbackTestInit*/

static void usr_timerTestInit( void)
{

   t_timerConfig t_config;
   t_intConf t_iConf;
   t_gpioConf t_gConf;

   gt_platTestObject.t_tHandle = hal_requestTimer( TIMER_1);

   if( gt_platTestObject.t_tHandle < 0)
   {
      exit(0);
   }/*End if( gt_platTestObject.t_tHandle < 0)*/

   t_config.t_mode   = NORMAL;
   t_config.t_dir    = DIRECTION_UP;
   t_config.f_period = 1.0f;

   if( hal_configureTimer( gt_platTestObject.t_tHandle, t_config) < 0)
   {
      exit(0);
   }

   if( hal_requestTimerInterrupt( gt_platTestObject.t_tHandle,
                                  OVERFLOW,
                                  &usr_timerTestInterrupt) < 0)
   {
      exit(0);
   }

   /*---------------------------------------------------------------*
    * Configure the output pins the test will use.
    *---------------------------------------------------------------*/
   t_gConf.c_inputMask  = PIN_3;
   t_gConf.c_outputMask = PIN_4;
   t_gConf.t_inConf     = PULLDOWN;
   t_gConf.t_outConf    = TOTEM;

   if( hal_configureGpioPort( GPIO_PORTH,
                              t_gConf) < 0)
   {
      exit(0);
   }

   t_iConf.c_pin     = PIN_3;
   t_iConf.t_inSense = GPIO_BOTH_EDGES;
   t_iConf.pf_funPtr = &usr_timerTestGpioInterrupt;

   gt_platTestObject.t_gHandle = hal_requestGpioInt( GPIO_PORTH,
                                                     t_iConf);

   if( gt_platTestObject.t_gHandle < 0)
      exit(0);

//hal_startTimer( gt_platTestObject.t_tHandle);

}/*End usr_timerTestInit*/

static void usr_spiLoopbackTestInit( void)
{
   t_spiConfig t_conf;

   t_conf.i_baudRate = 16000000;
   t_conf.t_spiMd    = SPI_MODE_3;
   t_conf.t_spiOp    = SPI_MASTER;
   t_conf.t_spiOrder = SPI_MSB_FIRST;
   t_conf.b_enDma    = false;

   if( hal_configureSpiChannel( SPI_4, t_conf) < 0)
      exit(0);

   gt_platTestObject.t_spiMaster = hal_requestSpiChannel( SPI_4,
                                                          spiMasterComplete,
                                                          GPIO_PORTF,
                                                          PIN_4);
   if( gt_platTestObject.t_spiMaster < 0)
      exit(0);

}/*End usr_spiLoopbackTestInit*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
t_MAILBOXHNDL usr_getPlatTestInMailbox( void)
{
   return gt_platTestObject.t_inMbxHndl;
}/*End usr_getPlatTestInMailbox*/

t_MAILBOXHNDL usr_getPlatTestOutMailbox( void)
{
   return gt_platTestObject.t_outMbxHndl;
}/*End usr_getPlatTestOutMailbox*/

void usr_platformTestInit( void)
{
   t_mailboxConfig t_mConfig;

   /*------------------------------------------------------------------------*
    * Create a new thread that will provide an interface to the console.
    *------------------------------------------------------------------------*/
   gt_platTestObject.t_platformTest = arb_threadCreate( usr_platformTest,
                                                        1,
                                                        0,
                                                        ARB_STACK_512B,
                                                        2);

   if( gt_platTestObject.t_platformTest < 0)
   {
      exit(0);

   }/*End if( gt_platTestObject.t_platformTest < 0)*/

gt_platTestObject.t_platformTest2 = arb_threadCreate( usr_platformTest2,
                                                      1,
                                                      0,
                                                      ARB_STACK_256B,
                                                      1);

if( gt_platTestObject.t_platformTest2 < 0)
{
   exit(0);

}/*End if( gt_platTestObject.t_platformTest2 < 0)*/

   /*------------------------------------------------------------------------*
    * Create a mailbox for receiving messages from the console.
    *------------------------------------------------------------------------*/
   t_mConfig.s_queueSize  = sizeof( gt_platTestObject.ac_txData);
   t_mConfig.s_queueDepth = 2;
   t_mConfig.t_writeMode  = BLOCKING;    /*Threads writing block*/
   t_mConfig.t_readMode   = NONBLOCKING; /*Threads reading poll*/
   t_mConfig.b_wrtFromInt = false;
   t_mConfig.b_multRdWr   = false;

   gt_platTestObject.t_inMbxHndl = arb_mailboxCreate( t_mConfig);

   if( gt_platTestObject.t_inMbxHndl < 0)
      exit(0);

   /*------------------------------------------------------------------------*
    * Create a mailbox for sending messages to the console.
    *------------------------------------------------------------------------*/
   t_mConfig.s_queueSize  = sizeof( gt_platTestObject.ac_rxData);
   t_mConfig.s_queueDepth = 2;
   t_mConfig.t_writeMode  = NONBLOCKING; /*Threads writing poll*/
   t_mConfig.t_readMode   = BLOCKING;    /*Threads reading block*/
   t_mConfig.b_wrtFromInt = true;
   t_mConfig.b_multRdWr   = false;

   gt_platTestObject.t_outMbxHndl = arb_mailboxCreate( t_mConfig);

   if( gt_platTestObject.t_outMbxHndl < 0)
      exit(0);

   /*------------------------------------------------------------------------*
    * Configure the peripherals needed to perform the various tests...
    *------------------------------------------------------------------------*/
   usr_uartLoopbackTestInit();

   usr_twiLoopbackTestInit();

   usr_timerTestInit();

   usr_spiLoopbackTestInit();

}/*End usr_platformTestInit*/
