/*---------------------------------------------------------------------------*
 * Copyright (C) 2014 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
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
 * File Name   : drv_arbComm.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This driver is responsible for communicating between arbitros 
 *               platforms.
 *
 * Last Update : Jan 19, 2014
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "arb_device.h"
#include "arb_semaphore.h"
#include "drv_arbComm.h"
#include "hal_uart.h"
#include "utl_buffer.h"
#include "hal_pmic.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Data Types
 *---------------------------------------------------------------------------*/
typedef struct
{

   /*------------------------------------------------------------------------*
    * Accessing the receive UART channel needs a lock, this semaphore is used
    * for just that purpose.
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_rxMutex;

   /*------------------------------------------------------------------------*
    * Accessing the transmit UART channel needs a lock, this semaphore is used
    * for just that purpose.
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_txMutex;

   /*------------------------------------------------------------------------*
    * Handle to the particular RX buffer this arbComm driver is using.
    *------------------------------------------------------------------------*/
   t_BUFFHANDLE t_rxBuffer;

   /*------------------------------------------------------------------------*
    * We are going to want to know how many 'handles' or users are attached to
    * this device driver.
    *------------------------------------------------------------------------*/
   uint8_t c_numUsers;

   /*------------------------------------------------------------------------*
    * Handle to the particular UART this arbComm driver is using.
    *------------------------------------------------------------------------*/
   t_UARTHNDL t_uHandle;

}t_arbCommDev;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_error arbCommOpen( t_DEVHANDLE t_handle);

static int16_t arbCommRead( t_DEVHANDLE t_handle,
                            int8_t *pc_buff,
                            uint16_t s_size);

static int16_t arbCommWrite( t_DEVHANDLE t_handle,
                             int8_t *pc_buff,
                             uint16_t s_size);

static int32_t arbCommIoctl( t_DEVHANDLE t_handle,
                             uint16_t s_command,
                             int32_t  i_arguments);

static t_error arbCommClose( t_DEVHANDLE t_handle);

static void rxComplete( uint16_t s_byte);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
t_deviceOperations gt_arbCommDevOps =
{
    arbCommOpen,
    arbCommRead,
    arbCommWrite,
    arbCommIoctl,
    arbCommClose

};

/*---------------------------------------------------------------------------*
 * This is the device's shared memory, all actions on this variable must be
 * mutually exclusive.
 *---------------------------------------------------------------------------*/
static t_arbCommDev gt_arbCommDev;

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void rxComplete( uint16_t s_byte)
{
   uint16_t s_bufferLevel = utl_getBufferFullLevel( gt_arbCommDev.t_rxBuffer);
   uint16_t s_bufferSize  = utl_getBufferSize( gt_arbCommDev.t_rxBuffer);

   /*------------------------------------------------------------------------*
    * Fill buffer till full...
    *------------------------------------------------------------------------*/
   if( s_bufferLevel <= s_bufferSize)
   {

      utl_writeByte( gt_arbCommDev.t_rxBuffer,
                     s_byte);

   }/*End if( s_bufferLevel <= s_bufferSize)*/

}/*End rxComplete*/

static t_error arbCommOpen( t_DEVHANDLE t_handle)
{

   t_uartError t_uErr;
   t_error t_err = ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_arbCommDev.t_txMutex,
             0);

   /*------------------------------------------------------------------------*
    * Keep track of the number of user-space applications using the driver.
    *------------------------------------------------------------------------*/
   gt_arbCommDev.c_numUsers++;

   /*------------------------------------------------------------------------*
    * If there is at least one user-space handle attached to this driver
    * than enable the receive interrupt.
    *------------------------------------------------------------------------*/
   if( gt_arbCommDev.c_numUsers == 1)
   {

      t_uErr = hal_enableUartRxInt( gt_arbCommDev.t_uHandle);

      if( t_uErr < 0)
         t_err = ARB_HAL_ERROR;

   }/*End if( gt_arbCommDev.c_numUsers == 1)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_arbCommDev.t_txMutex);

   return t_err;

}/*End arbCommOpen*/

static int16_t arbCommRead( t_DEVHANDLE t_handle,
                            int8_t *pc_buff,
                            uint16_t s_size)
{
   int16_t s_bufferLevel = 0;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_arbCommDev.t_rxMutex,
             0);

   s_bufferLevel = utl_getBufferFullLevel( gt_arbCommDev.t_rxBuffer);

   /*------------------------------------------------------------------------*
    * Is there enough data in the buffer?
    *------------------------------------------------------------------------*/
   if( s_size > s_bufferLevel)
   {
      /*---------------------------------------------------------------------*
       * Release the lock
       *---------------------------------------------------------------------*/
      arb_signal( gt_arbCommDev.t_rxMutex);
      return (int16_t)ARBCOMM_NOT_ENOUGH_DATA;

   }/*End if( s_size > s_bufferLevel)*/
   
   utl_readBlock( gt_arbCommDev.t_rxBuffer,
                  pc_buff,
                  s_size);

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_arbCommDev.t_rxMutex);

   return s_bufferLevel;

}/*End arbCommRead*/

static int16_t arbCommWrite( t_DEVHANDLE t_handle,
                             int8_t *pc_buff,
                             uint16_t s_size)
{
   int16_t s_numBytes;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_arbCommDev.t_txMutex,
             0);

   do 
   {

      s_numBytes = hal_uartWriteBlock( gt_arbCommDev.t_uHandle,
                                       pc_buff,
                                       s_size);

      if( s_numBytes == (int16_t)UART_BUSY)
      {
         arb_sleep(1);
      }

   }while( s_numBytes == (int16_t)UART_BUSY);

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_arbCommDev.t_txMutex);

   return s_numBytes;

}/*End arbCommWrite*/

static int32_t arbCommIoctl( t_DEVHANDLE t_handle,
                             uint16_t s_command,
                             int32_t i_arguments)
{
   int32_t i_return = (int32_t)ARBCOMM_PASSED;

   switch( (t_arbCommCmd)s_command)
   {
      case ARBCOMM_GET_RX_BUFFER_LEVEL:

         i_return = (int32_t)utl_getBufferFullLevel( gt_arbCommDev.t_rxBuffer);

      break; /*End case ARBCOMM_GET_RX_BUFFER_LEVEL:*/

      default:

         i_return = (int32_t)ARBCOMM_INVALID_CMD;

      break;

   }/*End switch( (t_arbCommCmd)s_command)*/

   return i_return;

}/*End arbCommIoctl*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
static t_error arbCommClose( t_DEVHANDLE t_handle)
{
   t_uartError t_uErr;
   t_error t_err = ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_arbCommDev.t_rxMutex,
             0);

   /*------------------------------------------------------------------------*
    * Keep track of the number of user-space applications using the driver.
    *------------------------------------------------------------------------*/
   gt_arbCommDev.c_numUsers--;

   /*------------------------------------------------------------------------*
    * If there are no more handles attached to this driver than disable the
    * receive interrupt.
    *------------------------------------------------------------------------*/
   if( gt_arbCommDev.c_numUsers == 0)
   {

      t_uErr = hal_disableUartRxInt( gt_arbCommDev.t_uHandle);

      if( t_uErr < 0)
         t_err = ARB_HAL_ERROR;

   }/*End if( gt_arbCommDev.c_numUsers == 0)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_arbCommDev.t_rxMutex);

   return t_err;

}/*End arbCommClose*/

t_error drv_arbCommInit( t_arbCommSetup t_setup)
{
   t_error t_err = ARB_PASSED;
   t_uartConfig t_uConf;

   /*------------------------------------------------------------------------*
    * Make sure the kernel is aware that a new device has been loaded.
    *------------------------------------------------------------------------*/
   t_err = arb_registerDevice( "arbCommDevice0",
                               arb_createDevId( t_setup.c_majorNum, 0),
                               &gt_arbCommDevOps);

   if( t_err < 0)
   {
      goto failed1;
   }

   /*------------------------------------------------------------------------*
    * Request a semaphore from the kernel. Since the signal port is a shared
    * resource we need to have all actions on it be mutually exclusive.
    *------------------------------------------------------------------------*/
   gt_arbCommDev.t_rxMutex = arb_semaphoreCreate( MUTEX);

   if( gt_arbCommDev.t_rxMutex < 0)
   {
      t_err = (t_error)gt_arbCommDev.t_rxMutex;
      goto failed2;

   }/*End if( gt_arbCommDev.t_rxMutex < 0)*/

   /*------------------------------------------------------------------------*
    * Grab handle to arbComm UART
    *------------------------------------------------------------------------*/
   gt_arbCommDev.t_uHandle = hal_requestUartChannel( t_setup.c_uartId);
   if( gt_arbCommDev.t_uHandle < 0)
   {
      t_err = ARB_HAL_ERROR;
      goto failed3;
   }

   t_uConf.t_comMd = ASYNC;
   t_uConf.t_charSz = CHAR_8BIT;
   t_uConf.t_parityMd = NO_PARITY;
   t_uConf.t_stopBitMd = ONE_STOP_BIT;
   t_uConf.i_baudRate = t_setup.i_baudRate;
   t_uConf.b_enRxDma = false;
   t_uConf.b_enTxDma = true;
   t_uConf.pf_rxCallBack = &rxComplete;
   /*------------------------------------------------------------------------*
    * By setting the tx call-back to NULL, all data transfers over the uart
    * are performed "in-place".
    *------------------------------------------------------------------------*/
   t_uConf.pf_txCallBack = NULL;

   /*------------------------------------------------------------------------*
    * Configure arbComm UART
    *------------------------------------------------------------------------*/
   if( hal_configureUartChannel( gt_arbCommDev.t_uHandle,
                                 t_uConf) < 0)
   {
      t_err = ARB_HAL_ERROR;
      goto failed4;
   }

   /*------------------------------------------------------------------------*
    * Allocate RX buffer of size = A*B1, where A = the max number of
    * possible tokens, B = the size of each token including a terminating
    * character, and 1 byte for the character that ends the string.
    *------------------------------------------------------------------------*/
   gt_arbCommDev.t_rxBuffer = utl_createBuffer( t_setup.s_rxBuffSize);
   if( gt_arbCommDev.t_rxBuffer < 0)
   {
      t_err = ARB_OUT_OF_HEAP;
      goto failed4;
   }/*End if( gt_arbCommDev.t_txBuffer < 0)*/

   /*------------------------------------------------------------------------*
    * Request a semaphore from the kernel. Since the signal port is a shared
    * resource we need to have all actions on it be mutually exclusive.
    *------------------------------------------------------------------------*/
   gt_arbCommDev.t_txMutex = arb_semaphoreCreate( MUTEX);

   if( gt_arbCommDev.t_txMutex < 0)
   {
      t_err = (t_error)gt_arbCommDev.t_txMutex;
      goto failed5;

   }/*End if( gt_arbCommDev.t_txMutex < 0)*/

   /*------------------------------------------------------------------------*
    * We don't have any users attached to this device
    *------------------------------------------------------------------------*/
   gt_arbCommDev.c_numUsers = 0;

   return ARB_PASSED;

failed5:

   utl_destroyBuffer( gt_arbCommDev.t_rxBuffer);

failed4:

   hal_releaseUartChannel( gt_arbCommDev.t_uHandle);

failed3:

   arb_semaphoreDestroy( gt_arbCommDev.t_rxMutex);

failed2:

   arb_destroyDevice( "arbCommDevice0");

failed1:

   return t_err;

}/*End drv_arbCommInit*/

/*---------------------------------------------------------------------------*
 * Remove the device from the system
 *---------------------------------------------------------------------------*/
void drv_arbCommExit( void)
{

   if( gt_arbCommDev.t_rxMutex != 0) /*If created... destroy*/
   {

      utl_destroyBuffer( gt_arbCommDev.t_rxBuffer);
      hal_releaseUartChannel( gt_arbCommDev.t_uHandle);
      arb_semaphoreDestroy( gt_arbCommDev.t_rxMutex);
      arb_semaphoreDestroy( gt_arbCommDev.t_txMutex);
      arb_destroyDevice( "arbCommDevice0");

      memset( (void *)&gt_arbCommDev, 0, sizeof( gt_arbCommDev));

   }/*End if( gt_arbCommDev.t_rxMutex != 0)*/

}/*End drv_arbCommExit*/