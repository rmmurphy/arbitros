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
 * File Name   : drv_wifly.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This driver is responsible for providing a standard API to
 *               the Roving Networks Wifly module. This code was based upon
 *               exampled provided in the Sparkfun Wifly shield library.
 *
 * Last Update : July 8, 2013
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "arb_device.h"
#include "arb_semaphore.h"
#include "arb_sysTimer.h"
#include "arb_printf.h"
#include "arb_thread.h"
#include "drv_wifly.h"
#include "hal_uart.h"
#include "utl_buffer.h"
#include "hal_gpio.h"
#include "hal_clocks.h"
#include "utl_buffer.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define WIFLY_CMD_MODE_RETRIES (5)
#define WIFLY_REBOOT_RETRIES   (5)
#define WIFLY_RESP_STRING_MAX  (15)

/*---------------------------------------------------------------------------*
 * Private Data Types
 *---------------------------------------------------------------------------*/
typedef struct
{

   /*------------------------------------------------------------------------*
    * Accessing the receive WIFLY channel needs a lock, this semaphore is used
    * for just that purpose.
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_rxMutex;

   /*------------------------------------------------------------------------*
    * Accessing the transmit WIFLY channel needs a lock, this semaphore is used
    * for just that purpose.
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_txMutex;

   /*------------------------------------------------------------------------*
    * This semaphore is used for waking up the user-space thread once a
    * rx message has been received.
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_rxBlockingSem;

   /*------------------------------------------------------------------------*
    * Ptr to the user-space receive buffer
    *------------------------------------------------------------------------*/
   uint8_t *pc_rxBuffer;

   /*------------------------------------------------------------------------*
    * Ptr to the user-space transmit buffer
    *------------------------------------------------------------------------*/
   uint8_t *pc_txBuffer;

   /*------------------------------------------------------------------------*
    * Handle to the particular WIFLY this wifly driver is using.
    *------------------------------------------------------------------------*/
   t_UARTHNDL t_uHandle;

   /*------------------------------------------------------------------------*
    * Configures the appropriate PORT for reseting the WiFly module.
    *------------------------------------------------------------------------*/
   uint8_t c_resetPort;

   /*------------------------------------------------------------------------*
    * Configures the appropriate pin for reseting the WiFly module.
    *------------------------------------------------------------------------*/
   uint8_t c_resetPin;

   /*------------------------------------------------------------------------*
    * We are going to want to know how many 'handles' or users are attached to
    * this device driver.
    *------------------------------------------------------------------------*/
   uint8_t c_numUsers;

   /*------------------------------------------------------------------------*
    * This buffer contains the desired response from the Wifly module based
    * on a given command.
    *------------------------------------------------------------------------*/
   char ac_respString[WIFLY_RESP_STRING_MAX];

   /*------------------------------------------------------------------------*
    * If true, the desired response contained in 'ac_respString' for a given
    * command was found.
    *------------------------------------------------------------------------*/
   bool b_respFound;

   /*------------------------------------------------------------------------*
    * Next read location within 'ac_respString'
    *------------------------------------------------------------------------*/
   uint8_t c_respIndex;

   /*------------------------------------------------------------------------*
    * Size of the response string that is currently being searched.
    *------------------------------------------------------------------------*/
   uint8_t c_respSize;

   /*------------------------------------------------------------------------*
    * If true, the Wifly module is in command mode.
    *------------------------------------------------------------------------*/
   bool b_enCmdMode;

}t_wiflyDev;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_error wiflyOpen( t_DEVHANDLE t_handle);

static int16_t wiflyRead( t_DEVHANDLE t_handle,
                          int8_t *pc_buff,
                          uint16_t s_size);

static int16_t wiflyWrite( t_DEVHANDLE t_handle,
                           int8_t *pc_buff,
                           uint16_t s_size);

static int32_t wiflyIoctl( t_DEVHANDLE t_handle,
                           uint16_t s_command,
                           int32_t  i_arguments);

static t_error wiflyClose( t_DEVHANDLE t_handle);

static void rxComplete( uint16_t s_byte);

static bool wiflyEnterCommandMode( void);

static void wiflyPrepareForResponse( const char *pc_respString);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
t_deviceOperations gt_wiflyDevOps =
{
    wiflyOpen,
    wiflyRead,
    wiflyWrite,
    wiflyIoctl,
    wiflyClose

};

/*---------------------------------------------------------------------------*
 * This is the device's shared memory, all actions on this variable must be
 * mutually exclusive.
 *---------------------------------------------------------------------------*/
static t_wiflyDev gt_wiflyDev;

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void wiflyPrepareForResponse( const char *pc_respString)
{
   size_t t_numBytes = strlen( pc_respString);

   if( t_numBytes > (WIFLY_RESP_STRING_MAX - 1))
      t_numBytes = (WIFLY_RESP_STRING_MAX - 1);

   snprintf( (char *)gt_wiflyDev.ac_respString,
             WIFLY_RESP_STRING_MAX,
             pc_respString);

   gt_wiflyDev.c_respSize = (uint8_t)t_numBytes;
   gt_wiflyDev.c_respIndex = 0;

   gt_wiflyDev.b_respFound = false;

}/*End wiflyPrepareForResponse*/

static bool wiflyEnterCommandMode( void)
{
   uint8_t c_retryCount;
   int8_t ac_buff[10];
   size_t t_size;

   gt_wiflyDev.b_enCmdMode = true;
   for( c_retryCount = 0; c_retryCount < WIFLY_CMD_MODE_RETRIES; c_retryCount++)
   {
      wiflyPrepareForResponse( "CMD");
      arb_sleep( 25);

      t_size = sprintf_P( (char *)ac_buff, PSTR("$$$"));

      hal_uartWriteBlock( gt_wiflyDev.t_uHandle,
                          ac_buff,
                          3);

      arb_sleep( 25);

      //t_size = sprintf_P( (char *)ac_buff, PSTR("\n\r"));
      //hal_uartWriteBlock( gt_wiflyDev.t_uHandle,
      //                    ac_buff,
       //                   t_size);

      if( gt_wiflyDev.b_respFound == true)
      {
         t_size = sprintf_P( (char *)ac_buff, PSTR("ver\n\r"));

         hal_uartWriteBlock( gt_wiflyDev.t_uHandle,
                             ac_buff,
                             5);

         arb_sleep( 25);

         return true;
      }

   }

   gt_wiflyDev.b_enCmdMode = false;
   return false;

}/*End wiflyEnterCommandMode*/

static void rxComplete( uint16_t s_byte)
{

   /*------------------------------------------------------------------------*
    * Signal any waiting threads that a carriage return has been
    * received.
    *------------------------------------------------------------------------*/
   if( gt_wiflyDev.b_enCmdMode == true)
   {
      arb_sysPrintChar( (const char)s_byte);

      if( gt_wiflyDev.b_respFound == false)
      {
         if( s_byte == gt_wiflyDev.ac_respString[gt_wiflyDev.c_respIndex])
         {
            gt_wiflyDev.c_respIndex++;
            if( gt_wiflyDev.c_respIndex == gt_wiflyDev.c_respSize)
            {
               if( gt_wiflyDev.b_enCmdMode == true)
               {
                  arb_sysPrintChar( '\r');
               }

               gt_wiflyDev.c_respIndex = 0;
               gt_wiflyDev.b_respFound = true;
            }
         }
         else
         {
            /*---------------------------------------------------------------*
             * If one of the bytes are incorrect then we locked onto an
             * incorrect sequence.
             *---------------------------------------------------------------*/
            if( gt_wiflyDev.c_respIndex > 0)
               gt_wiflyDev.c_respIndex = 0; /*Reset*/
         }
      }
   }/*End if( gt_wiflyDev.b_enCmdMode == true)*/
   else
   {

   }

}/*End rxComplete*/

static t_error wiflyOpen( t_DEVHANDLE t_handle)
{

   t_uartError t_uErr;
   t_error t_err = ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_wiflyDev.t_txMutex,
             0);

   /*------------------------------------------------------------------------*
    * Keep track of the number of user-space applications using the driver.
    *------------------------------------------------------------------------*/
   gt_wiflyDev.c_numUsers++;

   /*------------------------------------------------------------------------*
    * If there is at least one user-space handle attached to this driver
    * than enable the receive interrupt.
    *------------------------------------------------------------------------*/
   if( gt_wiflyDev.c_numUsers == 1)
   {

      t_uErr = hal_enableUartRxInt( gt_wiflyDev.t_uHandle);

      if( t_uErr < 0)
         t_err = ARB_HAL_ERROR;

   }/*End if( gt_wiflyDev.c_numUsers == 1)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_wiflyDev.t_txMutex);

   return t_err;

}/*End wiflyOpen*/

static int16_t wiflyRead( t_DEVHANDLE t_handle,
                            int8_t *pc_buff,
                            uint16_t s_size)
{
   int16_t s_bufferLevel = 0;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_wiflyDev.t_rxMutex,
             0);

   /*------------------------------------------------------------------------*
    * Wait for data to be available in the RX buffer.
    *------------------------------------------------------------------------*/
   arb_wait( gt_wiflyDev.t_rxBlockingSem,
             0);

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_wiflyDev.t_rxMutex);

   return s_bufferLevel;

}/*End wiflyRead*/

static int16_t wiflyWrite( t_DEVHANDLE t_handle,
                           int8_t *pc_buff,
                           uint16_t s_size)
{

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_wiflyDev.t_txMutex,
             0);

   if( gt_wiflyDev.b_enCmdMode == false)
   {

      hal_uartWriteBlock( gt_wiflyDev.t_uHandle,
                          pc_buff,
                          s_size);
   }

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_wiflyDev.t_txMutex);

   return s_size;

}/*End wiflyWrite*/

static int32_t wiflyIoctl( t_DEVHANDLE t_handle,
                           uint16_t s_command,
                           int32_t i_arguments)
{
   int32_t i_return = (int32_t)WIFLY_PASSED;

   switch( (t_wiflyCmd)s_command)
   {
      case WIFLY_BEGIN:
      {

         /*------------------------------------------------------------------*
          * We are going to access global memory or a register, so perform lock
          *------------------------------------------------------------------*/
         arb_wait( gt_wiflyDev.t_txMutex,
                   0);

         /*------------------------------------------------------------------*
          * Put the device into reset.
          *------------------------------------------------------------------*/
         hal_gpioOff( gt_wiflyDev.c_resetPort,
                      gt_wiflyDev.c_resetPin);

         arb_sleep(1);

         /*------------------------------------------------------------------*
          * Take the device out of reset.
          *------------------------------------------------------------------*/
         hal_gpioOn( gt_wiflyDev.c_resetPort,
                     gt_wiflyDev.c_resetPin);

         /*------------------------------------------------------------------*
          * Wait for the WiFly module to send the *READY* message.
          *------------------------------------------------------------------*/
         arb_sleep(1);

         /*------------------------------------------------------------------*
          * Release the lock
          *------------------------------------------------------------------*/
         arb_signal( gt_wiflyDev.t_txMutex);

      }
      break;/*End case WIFLY_BEGIN:*/

      case WIFLY_ENTER_CMD:

         /*------------------------------------------------------------------*
          * We are going to access global memory or a register, so perform lock
          *------------------------------------------------------------------*/
         arb_wait( gt_wiflyDev.t_txMutex,
                   0);

         if( wiflyEnterCommandMode() == false)
         {
            i_return = (int32_t)WIFLY_CMD_MODE_FAIL;
            gt_wiflyDev.b_enCmdMode = false;
         }

         /*------------------------------------------------------------------*
          * Release the lock
          *------------------------------------------------------------------*/
         arb_signal( gt_wiflyDev.t_txMutex);

      break;/*End case WIFLY_ENTER_CMD:*/
      
      case WIFLY_EXIT_CMD:
         /*------------------------------------------------------------------*
          * We are going to access global memory or a register, so perform lock
          *------------------------------------------------------------------*/
         arb_wait( gt_wiflyDev.t_txMutex,
                   0);

         gt_wiflyDev.b_enCmdMode = false;

         /*------------------------------------------------------------------*
          * Release the lock
          *------------------------------------------------------------------*/
         arb_signal( gt_wiflyDev.t_txMutex);

      break;

      case WIFLY_SEND_CMD:
      {
         /*------------------------------------------------------------------*
          * We are going to access global memory or a register, so perform lock
          *------------------------------------------------------------------*/
         arb_wait( gt_wiflyDev.t_txMutex,
                   0);

         char *pc_cmd = (char *)((uint16_t)i_arguments);
         char ac_end[2] = {'\n','\r'};
         uint8_t c_retries = 0;
         uint16_t s_size;

         if( gt_wiflyDev.b_enCmdMode == true)
         {
            
            do 
            {
            
               /*------------------------------------------------------------*
                * Initialize the search string
                *------------------------------------------------------------*/
               wiflyPrepareForResponse( (const char *)pc_cmd); 
               s_size = strlen(pc_cmd);
               i_return = (int32_t)hal_uartWriteBlock( gt_wiflyDev.t_uHandle,
                                                      (int8_t *)pc_cmd,
                                                      s_size);
               hal_uartWriteBlock( gt_wiflyDev.t_uHandle,
                                   (int8_t *)ac_end,
                                   2);
               /*------------------------------------------------------------*
                * Wait for the WiFly module to send the response message.
                *------------------------------------------------------------*/
               arb_sleep( 25);

               if( gt_wiflyDev.b_respFound == true)
                  break;

               c_retries++;

            }while( (gt_wiflyDev.b_respFound == false) &&
                    (c_retries < WIFLY_CMD_MODE_RETRIES));

            if( (gt_wiflyDev.b_respFound == true) &&
                (strncmp_P( (const char *)pc_cmd, PSTR("exit"), 4) == 0))
            {
               gt_wiflyDev.b_enCmdMode = false;

            }
         }
         else
            i_return = (int32_t)WIFLY_NO_CMD_MODE;

         /*------------------------------------------------------------------*
          * Release the lock
          *------------------------------------------------------------------*/
         arb_signal( gt_wiflyDev.t_txMutex);
      }
      break;/*End case WIFLY_SEND_CMD:*/

      default:

         i_return = (int32_t)WIFLY_INVALID_CMD;

      break;

   }/*End switch( (t_wiflyCmd)s_command)*/

   return i_return;

}/*End wiflyIoctl*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
static t_error wiflyClose( t_DEVHANDLE t_handle)
{
   t_uartError t_uErr;
   t_error t_err = ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_wiflyDev.t_rxMutex,
             0);

   /*------------------------------------------------------------------------*
    * Keep track of the number of user-space applications using the driver.
    *------------------------------------------------------------------------*/
   gt_wiflyDev.c_numUsers--;

   /*------------------------------------------------------------------------*
    * If there are no more handles attached to this driver than disable the
    * receive interrupt.
    *------------------------------------------------------------------------*/
   if( gt_wiflyDev.c_numUsers == 0)
   {

      t_uErr = hal_disableUartRxInt( gt_wiflyDev.t_uHandle);

      if( t_uErr < 0)
         t_err = ARB_HAL_ERROR;

   }/*End if( gt_wiflyDev.c_numUsers == 0)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_wiflyDev.t_rxMutex);

   return t_err;

}/*End wiflyClose*/

t_error drv_wiflyInit( t_wiflySetup t_setup)
{
   t_error t_err = ARB_PASSED;
   t_uartConfig t_uConf;
   t_gpioConf t_gConf;

   /*------------------------------------------------------------------------*
    * Make sure the kernel is aware that a new device has been loaded.
    *------------------------------------------------------------------------*/
   t_err = arb_registerDevice( "wiflyDevice0",
                               arb_createDevId( t_setup.c_majorNum, 0),
                               &gt_wiflyDevOps);

   if( t_err < 0)
   {
      goto failed1;
   }

   /*------------------------------------------------------------------------*
    * Request a semaphore from the kernel. Since the signal port is a shared
    * resource we need to have all actions on it be mutually exclusive.
    *------------------------------------------------------------------------*/
   gt_wiflyDev.t_rxMutex = arb_semaphoreCreate( MUTEX);

   if( gt_wiflyDev.t_rxMutex < 0)
   {
      t_err = (t_error)gt_wiflyDev.t_rxMutex;
      goto failed2;

   }/*End if( gt_wiflyDev.t_rxMutex < 0)*/

   /*------------------------------------------------------------------------*
    * Request a semaphore from the kernel. We will use this semaphore for
    * signaling the user-space program when the RX buffer has data.
    *------------------------------------------------------------------------*/
   gt_wiflyDev.t_rxBlockingSem = arb_semaphoreCreate( COUNTING);

   if( gt_wiflyDev.t_rxBlockingSem < 0)
   {
      t_err = (t_error)gt_wiflyDev.t_rxBlockingSem;
      goto failed3;

   }/*End if( gt_wiflyDev.t_rxBlockingSem < 0)*/

   /*------------------------------------------------------------------------*
    * Grab handle to wifly
    *------------------------------------------------------------------------*/
   gt_wiflyDev.t_uHandle = hal_requestUartChannel( t_setup.c_uartId);
   if( gt_wiflyDev.t_uHandle < 0)
   {
      t_err = ARB_HAL_ERROR;
      goto failed4;
   }

   t_uConf.t_comMd = ASYNC;
   t_uConf.t_charSz = CHAR_8BIT;
   t_uConf.t_parityMd = NO_PARITY;
   t_uConf.t_stopBitMd = ONE_STOP_BIT;
   t_uConf.i_baudRate = t_setup.i_baudRate;
   t_uConf.b_enRxDma = false;
   t_uConf.b_enTxDma = false;
   t_uConf.pf_rxCallBack = &rxComplete;
   /*------------------------------------------------------------------------*
    * By setting the tx call-back to NULL, all data transfers over the WIFLY
    * are performed "in-place".
    *------------------------------------------------------------------------*/
   t_uConf.pf_txCallBack = NULL;

   /*------------------------------------------------------------------------*
    * Configure wifly WIFLY
    *------------------------------------------------------------------------*/
   if( hal_configureUartChannel( gt_wiflyDev.t_uHandle,
                                 t_uConf) < 0)
   {
      t_err = ARB_HAL_ERROR;
      goto failed5;
   }

   /*------------------------------------------------------------------------*
    * Request a semaphore from the kernel. Since the signal port is a shared
    * resource we need to have all actions on it be mutually exclusive.
    *------------------------------------------------------------------------*/
   gt_wiflyDev.t_txMutex = arb_semaphoreCreate( MUTEX);

   if( gt_wiflyDev.t_txMutex < 0)
   {
      t_err = (t_error)gt_wiflyDev.t_txMutex;
      goto failed5;

   }/*End if( gt_wiflyDev.t_txMutex < 0)*/

   t_gConf.c_inputMask = 0;
   t_gConf.c_outputMask = t_setup.c_resetPin;
   t_gConf.b_setOutputLow = true;
   t_gConf.t_outConf = TOTEM;

   if( hal_configureGpioPort( t_setup.c_resetPort, t_gConf))
   {
      t_err = ARB_HAL_ERROR;
      goto failed6;

   }/*End if( gt_wiflyDev.t_txMutex < 0)*/

   gt_wiflyDev.c_resetPort = t_setup.c_resetPort;
   gt_wiflyDev.c_resetPin = t_setup.c_resetPin;

   /*------------------------------------------------------------------------*
    * We don't have any users attached to this device
    *------------------------------------------------------------------------*/
   gt_wiflyDev.c_numUsers = 0;
   gt_wiflyDev.b_enCmdMode = false;

   return ARB_PASSED;

failed6:

   arb_semaphoreDestroy( gt_wiflyDev.t_txMutex);

failed5:

   hal_releaseUartChannel( gt_wiflyDev.t_uHandle);

failed4:

   arb_semaphoreDestroy( gt_wiflyDev.t_rxBlockingSem);

failed3:

   arb_semaphoreDestroy( gt_wiflyDev.t_rxMutex);

failed2:

   arb_destroyDevice( "wiflyDevice0");

failed1:

   return t_err;

}/*End drv_wiflyInit*/

/*---------------------------------------------------------------------------*
 * Remove the device from the system
 *---------------------------------------------------------------------------*/
void drv_wiflyExit( void)
{

   if( gt_wiflyDev.t_rxMutex != 0) /*If created... destroy*/
   {
      hal_releaseUartChannel( gt_wiflyDev.t_uHandle);
      arb_semaphoreDestroy( gt_wiflyDev.t_rxBlockingSem);
      arb_semaphoreDestroy( gt_wiflyDev.t_rxMutex);
      arb_semaphoreDestroy( gt_wiflyDev.t_txMutex);
      arb_destroyDevice( "wiflyDevice0");

      memset( (void *)&gt_wiflyDev, 0, sizeof( gt_wiflyDev));

   }/*End if( gt_wiflyDev.t_rxMutex != 0)*/

}/*End drv_wiflyExit*/
