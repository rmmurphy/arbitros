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
 * File Name   : arb_printf.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for opening handles and providing a
 *               standard interface to all the available debug ports on the
 *               system.
 *
 * Last Update : Feb 25, 2012
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "arb_error.h"
#include "arb_printf.h"
#include "arb_device.h"
#include "arb_sysTimer.h"
#include "drv_console.h"
#include "drv_sd.h"
#include "hal_pmic.h"
#include "utl_buffer.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct
{

   /*------------------------------------------------------------------------*
    * Handle to the console driver.
    *------------------------------------------------------------------------*/
   t_DEVHANDLE t_printfHndl;

   /*------------------------------------------------------------------------*
    * Handle to the file used for logging debug information.
    *------------------------------------------------------------------------*/
   t_DEVHANDLE t_logFileHndl;

   /*------------------------------------------------------------------------*
    * Handle to the particular circular buffer used for storing debug messages.
    *------------------------------------------------------------------------*/
   t_BUFFHANDLE t_logBuffer;

   /*------------------------------------------------------------------------*
    * This variable controls which messages are allowed to print to the
    * terminal. For instance, if 'c_termDbgLevel = PRINTF_DBG_MED' only
    * messages with priority equal to or higher than 'PRINTF_DBG_MED' will be
    * written to the terminal.
    *------------------------------------------------------------------------*/
   uint8_t c_termDbgLevel;

   /*------------------------------------------------------------------------*
    * If true, then all debug messages are written to the log file given by
    * 't_logFileHndl'.
    *------------------------------------------------------------------------*/
   bool b_wrtLogFile;

}t_printObject;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
static t_printObject gt_pObject;

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
t_error arb_setPrintfDbgLevel( uint8_t c_dbg)
{
   gt_pObject.c_termDbgLevel = c_dbg;
   switch( c_dbg)
   {

      case PRINTF_DBG_LOW:
      case PRINTF_DBG_MED:
      case PRINTF_DBG_HIGH:
      case PRINTF_DBG_OFF:
      break;
      default:
         gt_pObject.c_termDbgLevel = PRINTF_DBG_HIGH;
      break;
   }/*End if( (t_dbg >= 0) && (t_dbg <= PRINTF_DBG_OFF))*/

   return ARB_PASSED;

}/*End arb_setPrintfDbgLevel*/

uint8_t arb_getPrintfDbgLevel( void)
{
   return gt_pObject.c_termDbgLevel;
}/*End arb_getPrintfDbgLevel*/

void arb_sysPrintChar( const char c_buff)
{
   /*------------------------------------------------------------------------*
    * Write directly to the console, do not buffer the data...
    *------------------------------------------------------------------------*/
   arb_write( gt_pObject.t_printfHndl,
              (int8_t *)&c_buff,
              (uint16_t)1);

}/*End arb_sysPrintChar*/

void arb_printf( uint8_t c_flags,
                 const char *pc_buff)
{
   t_sysTime t_time;
   uint16_t s_size = 0;
   uint8_t ac_buff[20];
   int16_t s_msec;

   HAL_BEGIN_CRITICAL();

   if( (c_flags &  PRINTF_DBG_SHOW_TIME) &&
       ((c_flags & PRINTF_DBG_PRIORITY_MASK) >= gt_pObject.c_termDbgLevel))
   {
      t_time = arb_sysTimeNow();
      if( t_time.c_hours < 10)
         s_size = sprintf_P( (char *)ac_buff,
                              PSTR("[%02d:"),
                              t_time.c_hours);
      else
         s_size = sprintf_P( (char *)ac_buff,
                             PSTR("[%2d:"),
                             t_time.c_hours);

      if( t_time.c_min < 10)
         s_size += sprintf_P( (char *)&ac_buff[s_size],
                              PSTR("%02d:"),
                              t_time.c_min);
      else
         s_size += sprintf_P( (char *)&ac_buff[s_size],
                              PSTR("%2d:"),
                              t_time.c_min);

      if( t_time.c_sec < 10)
         s_size += sprintf_P( (char *)&ac_buff[s_size],
                              PSTR("%02d:"),
                              t_time.c_sec);
      else
         s_size += sprintf_P( (char *)&ac_buff[s_size],
                              PSTR("%2d:"),
                              t_time.c_sec);

      s_msec = t_time.i_usec/1000;
      if( s_msec < 10)
         s_size += sprintf_P( (char *)&ac_buff[s_size],
                               PSTR("%003d]  "),
                               s_msec);
      else if( s_msec < 100)
         s_size += sprintf_P( (char *)&ac_buff[s_size],
                              PSTR("%03d]  "),
                              s_msec);
      else if( s_msec < 1000)
         s_size += sprintf_P( (char *)&ac_buff[s_size],
                              PSTR("%3d]  "),
                              s_msec);

      /*---------------------------------------------------------------------*
       * Store the current time in the circular buffer.
       *---------------------------------------------------------------------*/
      utl_writeBlock( gt_pObject.t_logBuffer,
                      (int8_t *)ac_buff,
                      s_size);

   }

   /*------------------------------------------------------------------------*
    * Store the debug message in the circular buffer based on priority.
    *------------------------------------------------------------------------*/
   if( (c_flags & PRINTF_DBG_PRIORITY_MASK) >= gt_pObject.c_termDbgLevel)
   {

      utl_writeBlock( gt_pObject.t_logBuffer,
                      (int8_t *)pc_buff,
                      (uint16_t)strlen( (char *)pc_buff));

      /*------------------------------------------------------------------------*
       * Append a new line to the end of the buffer.
       *------------------------------------------------------------------------*/
      utl_writeByte( gt_pObject.t_logBuffer,
                     '\n');

   }/*End if( (c_flags & PRINTF_DBG_PRIORITY_MASK) >= gt_pObject.c_termDbgLevel)*/

   HAL_END_CRITICAL();

}/*End arb_printf*/

void arb_printfFlushBuf( void)
{
   uint8_t ac_buff[20];
   int8_t *pc_headPtr;
   int16_t s_level;
   int16_t s_size;
   int16_t s_rdPtr;
   int16_t s_room;
   int16_t s_bytes;

   /*------------------------------------------------------------------------*
    * Get the head location of the log buffer...
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();
   pc_headPtr = utl_getBufferPtr( gt_pObject.t_logBuffer);
   s_level    = utl_getBufferFullLevel( gt_pObject.t_logBuffer);
   s_size     = utl_getBufferSize( gt_pObject.t_logBuffer);
   s_rdPtr    = utl_getBufRdPtr(gt_pObject.t_logBuffer);
   HAL_END_CRITICAL();

   /*------------------------------------------------------------------------*
    * Write the entire contents of the buffer...
    *------------------------------------------------------------------------*/
   if( s_level >= s_size)
   {
      /*---------------------------------------------------------------------*
       * Print only s_size bytes...
       *---------------------------------------------------------------------*/
      arb_write( gt_pObject.t_printfHndl,
                 (int8_t *)pc_headPtr,
                 (uint16_t)s_size);

      if( gt_pObject.b_wrtLogFile == true)
      {

         s_bytes = arb_write( gt_pObject.t_logFileHndl,
                              (int8_t *)pc_headPtr,
                              s_size);

         if( s_bytes != s_size)
         {
            sprintf( (char *)ac_buff, "sd write failed\r");
            arb_write( gt_pObject.t_printfHndl,
                       (int8_t *)ac_buff,
                       (uint16_t)strlen( (char *)ac_buff));
         }/*End if( s_bytes != s_size)*/

      }/*End if( gt_pObject.b_wrtLogFile == true)*/

      /*---------------------------------------------------------------------*
       * Buffer overflowed so reset...
       *---------------------------------------------------------------------*/
      HAL_BEGIN_CRITICAL();
      ult_resetBuffer(gt_pObject.t_logBuffer);
      HAL_END_CRITICAL();

   }/*End if( s_level > s_size)*/
   else if( s_level > 0)
   {
      /*---------------------------------------------------------------------*
       * If wrapping the end of the buffer, print to console twice...
       *---------------------------------------------------------------------*/
      s_room = s_size - s_rdPtr;
      if( s_room < s_level)
      {
         arb_write( gt_pObject.t_printfHndl,
                    (int8_t *)&pc_headPtr[s_rdPtr],
                    (uint16_t)s_room);

         arb_write( gt_pObject.t_printfHndl,
                    (int8_t *)&pc_headPtr[0],
                    (uint16_t)(s_level - s_room));

         if( gt_pObject.b_wrtLogFile == true)
         {
            s_bytes = arb_write( gt_pObject.t_logFileHndl,
                                 (int8_t *)&pc_headPtr[s_rdPtr],
                                 (uint16_t)s_room);

            if( s_bytes != s_room)
            {
               sprintf( (char *)ac_buff, "sd write failed\r");
               arb_write( gt_pObject.t_printfHndl,
                          (int8_t *)ac_buff,
                          (uint16_t)strlen( (char *)ac_buff));
            }/*End if( s_bytes != s_room)*/

            s_bytes = arb_write( gt_pObject.t_logFileHndl,
                                 (int8_t *)&pc_headPtr[0],
                                 (uint16_t)(s_level - s_room));

            if( s_bytes != (uint16_t)(s_level - s_room))
            {
               sprintf( (char *)ac_buff, "sd write failed\r");
               arb_write( gt_pObject.t_printfHndl,
                          (int8_t *)ac_buff,
                          (uint16_t)strlen( (char *)ac_buff));
            }/*End if( s_bytes != (uint16_t)(s_level - s_room))*/

         }/*End if( gt_pObject.b_wrtLogFile == true)*/

      }/*End if( s_room < s_level)*/
      else
      {
         arb_write( gt_pObject.t_printfHndl,
                   (int8_t *)&pc_headPtr[s_rdPtr],
                   (uint16_t)s_level);

         if( gt_pObject.b_wrtLogFile == true)
         {

            s_bytes = arb_write( gt_pObject.t_logFileHndl,
                                 (int8_t *)&pc_headPtr[s_rdPtr],
                                 (uint16_t)s_level);

            if( s_bytes != s_level)
            {
               sprintf( (char *)ac_buff, "sd write failed\r");
               arb_write( gt_pObject.t_printfHndl,
                          (int8_t *)ac_buff,
                          (uint16_t)strlen( (char *)ac_buff));
            }/*End if( s_bytes != s_level)*/

         }/*End if( gt_pObject.b_wrtLogFile == true)*/
      }
      
      HAL_BEGIN_CRITICAL();
      utl_incrBufRdPtr( gt_pObject.t_logBuffer,
                        (uint16_t)s_level);
      HAL_END_CRITICAL();
      
   }/*End else if( s_level > 0)*/

}/*End arb_printfFlushBuf*/

t_error arb_printfInit( char *pc_driver,
                        int16_t s_bufSize,
                        char *pc_logFile)
{

   gt_pObject.c_termDbgLevel = PRINTF_DBG_OFF;
   gt_pObject.b_wrtLogFile   = false;

   /*------------------------------------------------------------------------*
    * Open a handle to the printf driver.
    *------------------------------------------------------------------------*/
   gt_pObject.t_printfHndl = arb_open( pc_driver,
                                       ARB_O_READ |
                                       ARB_O_WRITE);

   if( gt_pObject.t_printfHndl < 0)
   {
      return (t_error)gt_pObject.t_printfHndl;
   }

   /*------------------------------------------------------------------------*
    * If a valid file extension has been entered log printf messages to a 
    * file.
    *------------------------------------------------------------------------*/ 
   if( pc_logFile != NULL)
   {

      /*---------------------------------------------------------------------*
       * Open a file for logging debug information.
       *---------------------------------------------------------------------*/
      gt_pObject.t_logFileHndl = arb_open( pc_logFile,
                                           ARB_O_WRITE |
                                           ARB_O_CREAT |
                                           ARB_O_SYNC);

      /*---------------------------------------------------------------------*
       * Don't return a failure if the file couldn't be opened.
       *---------------------------------------------------------------------*/
      if( gt_pObject.t_logFileHndl < 0)
      {
         return (t_error)gt_pObject.t_logFileHndl;
      }

      if( arb_ioctl( gt_pObject.t_logFileHndl,
                     SD_ERASE,
                     0) < 0)
      {
         return ARB_OPEN_ERROR;
      }

      gt_pObject.b_wrtLogFile = true;

   }/*End if( pc_logFile != NULL)*/

   /*------------------------------------------------------------------------*
    * Allocate a circular buffer for holding the debug messages.
    *------------------------------------------------------------------------*/
   gt_pObject.t_logBuffer = utl_createBuffer( s_bufSize);
   if( gt_pObject.t_logBuffer < 0)
   {
      return ARB_OUT_OF_HEAP;
   }/*End if( gt_pObject.t_logBuffer < 0)*/

   return ARB_PASSED;

}/*End arb_printfInit*/
