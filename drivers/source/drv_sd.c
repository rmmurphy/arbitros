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
 * References  : http://elm-chan.org/docs/mmc/mmc_e.html
 *
 * Last Update : June, 16, 2013
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "arb_device.h"
#include "arb_semaphore.h"
#include "arb_thread.h"
#include "drv_sd.h"
#include "hal_spi.h"
#include "sdInterface.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define SD_MAX_RDWR_RETRIES (5)

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct
{
   /*------------------------------------------------------------------------*
    * Resources that can be shared amongst mutliple users (either a global
    * buffer or IO device) need to be protected against race conditions. We
    * use this semaphore for just that purpose.
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_mutex;

   /*------------------------------------------------------------------------*
    * Handle to the particular spi port attached to the sd card.
    *------------------------------------------------------------------------*/
   t_SPIHNDL t_spiHndl;

   /*------------------------------------------------------------------------*
    * We are going to want to know how many 'handles' or users are attached to
    * this device.
    *------------------------------------------------------------------------*/
   uint8_t c_numUsers;

}t_sdDev;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_error sdOpen( t_DEVHANDLE t_dev);

static int16_t sdRead( t_DEVHANDLE t_dev,
                       int8_t *pc_buff,
                       uint16_t s_size);

static int16_t sdWrite( t_DEVHANDLE t_dev,
                        int8_t *pc_buff,
                        uint16_t s_size);

static int32_t sdIoctl( t_DEVHANDLE t_devHandle,
                        uint16_t s_command,
                        int32_t  i_arguments);

static t_error sdClose( t_DEVHANDLE t_dev);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
t_deviceOperations gat_sdDevOps =
{
    sdOpen,
    sdRead,
    sdWrite,
    sdIoctl,
    sdClose
};

/*---------------------------------------------------------------------------*
 * This is the device's shared memory, all actions on this variable must be
 * mutually exclusive.
 *---------------------------------------------------------------------------*/
static t_sdDev gt_sdDev;

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static int16_t sdRead( t_DEVHANDLE t_dev,
                       int8_t *pc_buff,
                       uint16_t s_size)
{
   int16_t s_bufferLevel = 0;
   t_devHandle *pt_dev = (t_devHandle *)t_dev;
   uint8_t c_retries = 0;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_sdDev.t_mutex,
             0);

   if( pt_dev->pv_privateData != NULL)
   {

      do
      {

         s_bufferLevel = (int16_t)sd_read( gt_sdDev.t_spiHndl,
                                           pt_dev->pv_privateData,
                                           (uint8_t *)pc_buff,
                                           s_size);

         if( s_bufferLevel < 0)
         {
            sd_init( gt_sdDev.t_spiHndl);
            //arb_sleep(1);
         }

         c_retries++;

      }while( (s_bufferLevel < 0) && (c_retries < SD_MAX_RDWR_RETRIES));

   }/*End if( pt_dev->pv_privateData != NULL)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_sdDev.t_mutex);

   return s_bufferLevel;

}/*End sdRead*/

static int16_t sdWrite( t_DEVHANDLE t_dev,
                        int8_t *pc_buff,
                        uint16_t s_size)
{
   int16_t s_bufferLevel = 0;
   t_devHandle *pt_dev = (t_devHandle *)t_dev;
   uint8_t c_retries = 0;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_sdDev.t_mutex,
             0);

   if( pt_dev->pv_privateData != NULL)
   {
      do
      {

         s_bufferLevel = (int16_t)sd_write( gt_sdDev.t_spiHndl,
                                            pt_dev->pv_privateData,
                                            (uint8_t *)pc_buff,
                                            s_size);

         if( s_bufferLevel < 0)
         {
            sd_init( gt_sdDev.t_spiHndl);
            //arb_sleep(1);
         }

         c_retries++;

      }while( (s_bufferLevel < 0) && (c_retries < SD_MAX_RDWR_RETRIES));

   }/*End if( pt_dev->pv_privateData != NULL)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_sdDev.t_mutex);

   return s_bufferLevel;

}/*End sdWrite*/

t_error sdOpen( t_DEVHANDLE t_dev)
{
   static t_devHandle *pt_dev;
   static char *pc_filename;
   t_error t_err = ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_sdDev.t_mutex,
             0);

   pt_dev = (t_devHandle *)t_dev;

   if( gt_sdDev.c_numUsers == 0)
   {
      if( sd_begin( gt_sdDev.t_spiHndl) == false)
      {
         t_err = ARB_OPEN_ERROR;
         goto failed;
      }

   }/*End if( gt_sdDev.c_numUsers == 1)*/

   pc_filename = (char *)pt_dev->pv_privateData;

   if( pc_filename != NULL)
   {
      uint8_t c_flags = 0;

      if( pt_dev->c_flags & ARB_O_READ)
         c_flags |= SD_FILE_READ;
      if( pt_dev->c_flags & ARB_O_WRITE)
         c_flags |= SD_FILE_WRITE;
      if( pt_dev->c_flags & ARB_O_APPEND)
         c_flags |= SD_FILE_APPEND;
      if( pt_dev->c_flags & ARB_O_SYNC)
         c_flags |= SD_FILE_SYNC;
      if( pt_dev->c_flags & ARB_O_TRUNC)
         c_flags |= SD_FILE_TRUNC;
      if( pt_dev->c_flags & ARB_O_AT_END)
         c_flags |= SD_FILE_AT_END;
      if( pt_dev->c_flags & ARB_O_CREAT)
         c_flags |= SD_FILE_CREAT;

      pt_dev->pv_privateData = sd_open( pc_filename, c_flags);
      if( pt_dev->pv_privateData == NULL)
      {
         t_err = ARB_OPEN_ERROR;
         goto failed;
      }
   }

   gt_sdDev.c_numUsers++;

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_sdDev.t_mutex);

   return ARB_PASSED;

failed:

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_sdDev.t_mutex);

   return t_err;

}/*End sdOpen*/

int32_t sdIoctl( t_DEVHANDLE t_dev,
                 uint16_t s_command,
                 int32_t  i_arguments)
{
   int32_t i_return = (int32_t)ARB_PASSED;
   t_devHandle *pt_dev = (t_devHandle *)t_dev;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_sdDev.t_mutex,
             0);

   switch( (t_sdCmd)s_command)
   {
      case SD_INIT:
         if( sd_init( gt_sdDev.t_spiHndl) == false)
            i_return = (int32_t)SD_CARD_INIT_FAILED;
      break;/*End case SD_INIT:*/

      case SD_RMDASHR:
         if( sd_rmDashR( ) == false)
            i_return = (int32_t)SD_OPERATION_FAILED;
      break;/*End case SD_RMDASHR:*/

      case SD_CARD_INFO:

      break;/*End case SD_CARD_INFO:*/

      case SD_LS:
         sd_ls();
      break;/*End case SD_LS:*/

      case SD_RM:
      {
         char *pc_filename = (char *)((uint16_t)i_arguments);
         if( sd_remove( pc_filename) == false)
            i_return = (int32_t)SD_OPERATION_FAILED;
      }
      break;/*End case SD_RM:*/

      case SD_CD:
      {
         char *pc_filename = (char *)((uint16_t)i_arguments);
         if( sd_chdir( pc_filename) == false)
            i_return = (int32_t)SD_OPERATION_FAILED;
      }
      break;/*End case SD_CD:*/

      case SD_MKDIR:
      {
         char *pc_filename = (char *)((uint16_t)i_arguments);
         if( sd_mkdir( pc_filename) == false)
            i_return = (int32_t)SD_OPERATION_FAILED;
      }
      break;/*End  case SD_MKDIR:*/

      case SD_RMDIR:
      {
         char *pc_filename = (char *)((uint16_t)i_arguments);
         if( sd_rmdir( pc_filename) == false)
            i_return = (int32_t)SD_OPERATION_FAILED;
      }
      break;/*End case SD_RMDIR:*/

      case SD_REWIND:
         /*------------------------------------------------------------------*
          * Set the position of the file back to the beginning.
          *------------------------------------------------------------------*/
         sd_rewind( pt_dev->pv_privateData);
      break;/*End case SD_REWIND:*/

      case SD_ERASE:
         /*------------------------------------------------------------------*
          * ERASE the contents of the file.
          *------------------------------------------------------------------*/
         i_return = sd_truncate( pt_dev->pv_privateData, 0);
      break;

      case SD_GET_SIZE:
         i_return = sd_fileSize( pt_dev->pv_privateData);
      break;

      default:
         i_return = (int32_t)ARB_INVALID_CMD;
      break;

   }/*End switch( (t_sdCmd)s_command)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_sdDev.t_mutex);

   return i_return;

}/*End sdIoctl*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
t_error sdClose( t_DEVHANDLE t_dev)
{
   t_devHandle *pt_dev = (t_devHandle *)t_dev;
   bool b_status;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_sdDev.t_mutex,
             0);

   gt_sdDev.c_numUsers--;

   b_status = sd_close( pt_dev->pv_privateData);
   if( b_status == true)
   {
      pt_dev->pv_privateData = NULL;
   }

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_sdDev.t_mutex);

   return ARB_PASSED;

}/*End sdClose*/

t_error drv_sdInit( t_sdSetup t_setup)
{
   t_error t_err = ARB_PASSED;
   t_spiConfig t_sConf;
   static t_spiError t_sErr;

   memset( (void *)&gt_sdDev, 0, sizeof(t_sdDev));

   /*------------------------------------------------------------------------*
    * Make sure the kernel is aware that a new device has been loaded.
    *------------------------------------------------------------------------*/
   t_err = arb_registerDevice( "sdDevice0",
                               arb_createDevId( t_setup.c_majorNum,
                               0),
                               &gat_sdDevOps);

   if( t_err < 0)
   {
      goto failed1;
   }

   /*------------------------------------------------------------------------*
    * Request a semaphore from the kernel. Since the sd port is a shared
    * resource we need to have all actions on it be mutually exclusive.
    *------------------------------------------------------------------------*/
   gt_sdDev.t_mutex = arb_semaphoreCreate( MUTEX);

   if( gt_sdDev.t_mutex < 0)
   {
      t_err = (t_error)gt_sdDev.t_mutex;
      goto failed2;

   }/*End if( gt_sdDev.t_mutex < 0)*/

   /*------------------------------------------------------------------------*
    * Request access to the spi port required for controlling this particular
    * sd card.
    *------------------------------------------------------------------------*/
   t_sConf.b_enDma    = false;
   t_sConf.i_baudRate = 800000;     /*Set the baud rate low until initialized*/
   t_sConf.t_spiMd    = SPI_MODE_0;
   t_sConf.t_spiOp    = SPI_MASTER;
   t_sConf.t_spiOrder = SPI_MSB_FIRST;

   t_sErr = hal_configureSpiChannel( t_setup.t_spiChan,
                                     t_sConf);
   if( t_sErr < 0)
   {
      t_err = ARB_HAL_ERROR;
      goto failed3;

   }/*End if( t_sErr < 0)*/

   gt_sdDev.t_spiHndl = hal_requestSpiChannel( t_setup.t_spiChan,
                                               NULL,
                                               t_setup.t_csPort,
                                               t_setup.t_csPin);
   if( gt_sdDev.t_spiHndl < 0)
   {
      t_err = ARB_HAL_ERROR;
      goto failed3;

   }/*End if( t_sErr < 0)*/

   /*------------------------------------------------------------------------*
    * We don't have any users attached to this device
    *------------------------------------------------------------------------*/
   gt_sdDev.c_numUsers = 0;

   return ARB_PASSED;

failed3:

   arb_semaphoreDestroy( gt_sdDev.t_mutex);

failed2:

   arb_destroyDevice( "sdDevice0");

failed1:

   return t_err;

}/*End drv_sdInit*/

/*---------------------------------------------------------------------------*
 * Remove the device from the system
 *---------------------------------------------------------------------------*/
void drv_sdExit( void)
{

   if( gt_sdDev.t_mutex != 0) /*If created... destroy*/
   {
      arb_semaphoreDestroy( gt_sdDev.t_mutex);
      arb_destroyDevice( "sdDevice0");
      hal_releaseSpiChannel( gt_sdDev.t_spiHndl);

      /*------------------------------------------------------------------*
       * Remove any user-space specific generated memory here...
       *------------------------------------------------------------------*/
   }/*End if( gat_templateDev[c_index].t_mutex != 0)*/

}/*End drv_sdExit*/
