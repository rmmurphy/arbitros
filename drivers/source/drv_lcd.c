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
 * File Name   : drv_lcd.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for controlling a 8-bit LCD with a
 *               hitachi HD44780 control module. The contents contained herein
 *               were inspired by the library 'LiquidCrystal' from the
 *               Arduino project. See www.arduino.cc/ for more information.
 *
 * Last Update : Nov, 7, 2012
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "arb_device.h"
#include "arb_semaphore.h"
#include "drv_lcd.h"
#include "arb_thread.h"
#include "hal_pmic.h"
#include "hal_uart.h"
#include "hal_clocks.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define LCD_PREFIX (0xFE)
/*---------------------------------------------------------------------------*
 * LCD commands...
 *---------------------------------------------------------------------------*/
#define _LCD_DISPLAY_ON        (0x41)
#define _LCD_DISPLAY_OFF       (0x42)
#define _LCD_SET_CURSOR        (0x45)
#define _LCD_CURSOR_HOME       (0x46)
#define _LCD_MOVE_CURSOR_LEFT  (0x49)
#define _LCD_MOVE_CURSOR_RIGHT (0x4A)
#define _LCD_BLINK_CURSOR_ON   (0x4B)
#define _LCD_BLINK_CURSOR_OFF  (0x4C)
#define _LCD_CLEAR_SCREEN      (0x51)
#define _LCD_SET_CONSTRAST     (0x52)
#define _LCD_SET_BACKLIGHT     (0x53)
#define _LCD_CUSTOM_CHAR       (0x54)
#define _LCD_MOVE_DISP_LEFT    (0x55)
#define _LCD_MOVE_DISP_RIGHT   (0x56)
#define _LCD_FIRMWARE_VER      (0x70)

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
    * Handle to the particular UART this LCD driver is using.
    *------------------------------------------------------------------------*/
   t_UARTHNDL t_uHandle;

   /*------------------------------------------------------------------------*
    * The number of rows on the LCD display.
    *------------------------------------------------------------------------*/
   uint8_t c_numRows;

   /*------------------------------------------------------------------------*
    * The number of columns on the LCD display.
    *------------------------------------------------------------------------*/
   uint8_t c_numColumns;

   /*------------------------------------------------------------------------*
    * The current cursor x,y position.
    *------------------------------------------------------------------------*/
   t_cursPos t_cPos;

   /*------------------------------------------------------------------------*
    * We are going to want to know how many 'handles' or users are attached to
    * this device.
    *------------------------------------------------------------------------*/
   uint8_t c_numUsers;

}t_lcdDev;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_error lcdOpen( t_DEVHANDLE t_devHandle);

static int32_t lcdIoctl( t_DEVHANDLE t_devHandle,
                         uint16_t s_command,
                         int32_t  i_arguments);

static t_error lcdClose( t_DEVHANDLE t_devHandle);

static int16_t lcdWrite( t_DEVHANDLE t_handle,
                         int8_t *pc_buff,
                         uint16_t s_size);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
t_deviceOperations gat_lcdDevOps =
{
    lcdOpen,
    NULL,
    lcdWrite,
    lcdIoctl,
    lcdClose
};

/*---------------------------------------------------------------------------*
 * This is the device's shared memory, all actions on this variable must be
 * mutually exclusive.
 *---------------------------------------------------------------------------*/
static t_lcdDev gt_lcdDev;

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
int16_t lcdWrite( t_DEVHANDLE t_handle,
                  int8_t *pc_buff,
                  uint16_t s_size)
{
   int16_t s_wrote;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_lcdDev.t_mutex,
             0);

   s_wrote = hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                                 pc_buff,
                                 s_size);

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_lcdDev.t_mutex);

   return s_wrote;

}/*End lcdWrite*/

t_error lcdOpen( t_DEVHANDLE t_devHandle)
{

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_lcdDev.t_mutex,
             0);

   gt_lcdDev.c_numUsers++;

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_lcdDev.t_mutex);

   return ARB_PASSED;

}/*End lcdOpen*/

int32_t lcdIoctl( t_DEVHANDLE t_devHandle,
                  uint16_t s_command,
                  int32_t i_arguments)
{
   int32_t i_return = (int32_t)LCD_PASSED;
   int8_t ac_data[2 + sizeof(t_custChar)];

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_lcdDev.t_mutex,
             0);
   
   ac_data[0] = LCD_PREFIX;
   switch( (t_lcdCmd)s_command)
   {
      case LCD_CLEAR:
         ac_data[1] = _LCD_CLEAR_SCREEN;
         hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                             ac_data, 
                             2);
         arb_sleep(1);
      break;

      case LCD_CURSOR_HOME:
         ac_data[1] = _LCD_CURSOR_HOME;
         hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                             ac_data, 
                             2);
         gt_lcdDev.t_cPos.c_row = 0;
         gt_lcdDev.t_cPos.c_col = 0;
         arb_sleep(1);
      break;

      case LCD_DISPLAY_ON:
         ac_data[1] = _LCD_DISPLAY_ON;
         hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                             ac_data, 
                             2);
      break;/*End case LCD_DISPLAY_ON:*/

      case LCD_DISPLAY_OFF:
         ac_data[1] = _LCD_DISPLAY_OFF;
         hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                             ac_data, 
                             2);
      break;/*End case LCD_DISPLAY_OFF:*/

      case LCD_CURSOR_BLINK_ON:
         ac_data[1] = _LCD_BLINK_CURSOR_ON;
         hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                             ac_data, 
                             2);
      break;/*End case LCD_CURSOR_BLINK_ON:*/

      case LCD_CURSOR_BLINK_OFF:
         ac_data[1] = _LCD_BLINK_CURSOR_OFF;
         hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                             ac_data, 
                             2);
      break;/*End case LCD_CURSOR_BLINK_OFF:*/

      case LCD_CURSOR_SET_POS:
      {
         uint8_t ac_rowOffsets[4] = { 0x00,0x40,0x14,0x54};

         t_cursPos *pt_cursPos = (t_cursPos *)((uint16_t)i_arguments);

         if( (pt_cursPos->c_row >= gt_lcdDev.c_numRows) ||
             (pt_cursPos->c_col >= gt_lcdDev.c_numColumns))
         {
            i_return = (int32_t)LCD_INVALID_POS;
         }
         else
         {
            ac_data[1] = _LCD_SET_CURSOR;
            ac_data[2] = pt_cursPos->c_col + ac_rowOffsets[pt_cursPos->c_row];
            hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                                ac_data, 
                                3);
            gt_lcdDev.t_cPos.c_row = pt_cursPos->c_row;
            gt_lcdDev.t_cPos.c_col = pt_cursPos->c_col;
         }
      }
      break;/*End case LCD_CURSOR_SET_POS:*/

      case LCD_SCROLL_DISPLAY_LEFT:
         ac_data[1] = _LCD_MOVE_DISP_LEFT;
         hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                             ac_data, 
                             2);
      break;

      case LCD_SCROLL_DISPLAY_RIGHT:
         ac_data[1] = _LCD_MOVE_DISP_RIGHT;
         hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                             ac_data, 
                             2);
      break;

      case LCD_WRITE_BUILT_IN_FONT:
         hal_uartWriteByte( gt_lcdDev.t_uHandle,
                            (uint8_t)i_arguments);
      break;/*End LCD_WRITE_BUILT_IN_FONT*/

      case LCD_BACKLIGHT_ON:
         ac_data[1] = _LCD_SET_BACKLIGHT;
         ac_data[2] = 8;
         hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                             ac_data, 
                             3);
      break;/*End case LCD_BACKLIGHT_ON:*/

      case LCD_BACKLIGHT_OFF:
         ac_data[1] = _LCD_SET_BACKLIGHT;
         ac_data[2] = 1;
         hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                             ac_data, 
                             3);
      break;/*End case LCD_BACKLIGHT_OFF:*/

      case LCD_GET_FIRMWARE:
         ac_data[1] = _LCD_FIRMWARE_VER;
         hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                             ac_data, 
                             2);
      break;/*End case LCD_GET_FIRMWARE:*/

      case LCD_STORE_CUST_CHAR:
      {
         t_custChar *pt_cust = (t_custChar *)((uint16_t)i_arguments);

         ac_data[1] = _LCD_CUSTOM_CHAR;
         memcpy( &ac_data[2], pt_cust, sizeof(t_custChar));

         hal_uartWriteBlock( gt_lcdDev.t_uHandle,
                             ac_data,
                             2 + sizeof(t_custChar));
      }
      break;/*End case LCD_STORE_CUST_CHAR:*/

      default:

         i_return = (int32_t)LCD_INVALID_CMD;

      break;

   }/*End switch( (t_lcdCmd)s_command)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_lcdDev.t_mutex);

   return i_return;

}/*End lcdIoctl*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
t_error lcdClose( t_DEVHANDLE t_devHandle)
{

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_lcdDev.t_mutex,
             0);

   gt_lcdDev.c_numUsers--;

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_lcdDev.t_mutex);

   return ARB_PASSED;

}/*End lcdClose*/

t_error drv_lcdInit( t_lcdSetup t_setup)
{
   t_error t_err = ARB_PASSED;
   t_uartConfig t_uConf;

   /*------------------------------------------------------------------------*
    * Clear the LCD object..
    *------------------------------------------------------------------------*/
   memset( (void *)&gt_lcdDev, 0, sizeof( gt_lcdDev));

   /*------------------------------------------------------------------------*
    * Make sure the kernel is aware that a new device has been loaded.
    *------------------------------------------------------------------------*/
   t_err = arb_registerDevice( "lcdDevice0",
                               arb_createDevId( t_setup.c_majorNum,
                               0),
                               &gat_lcdDevOps);

   if( t_err < 0)
   {
      goto failed1;
   }

   /*------------------------------------------------------------------------*
    * Request a semaphore from the kernel. Since the lcd port is a shared
    * resource we need to have all actions on it be mutually exclusive.
    *------------------------------------------------------------------------*/
   gt_lcdDev.t_mutex = arb_semaphoreCreate( MUTEX);

   if( gt_lcdDev.t_mutex < 0)
   {
      t_err = (t_error)gt_lcdDev.t_mutex;
      goto failed2;

   }/*End if( gt_lcdDev.t_mutex < 0)*/

   /*------------------------------------------------------------------------*
    * Grab handle to console UART
    *------------------------------------------------------------------------*/
   gt_lcdDev.t_uHandle = hal_requestUartChannel( t_setup.c_uartId);
   if( gt_lcdDev.t_uHandle < 0)
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
   t_uConf.b_enTxDma = false;
   t_uConf.pf_rxCallBack = NULL;
   /*------------------------------------------------------------------------*
    * By setting the tx call-back to NULL, all data transfers over the uart
    * are performed "in-place".
    *------------------------------------------------------------------------*/
   t_uConf.pf_txCallBack = NULL;

   /*------------------------------------------------------------------------*
    * Configure console UART
    *------------------------------------------------------------------------*/
   if( hal_configureUartChannel( gt_lcdDev.t_uHandle,
                                 t_uConf) < 0)
   {
      t_err = ARB_HAL_ERROR;
      goto failed4;
   }
   /*------------------------------------------------------------------------*
    * We don't have any users attached to this device
    *------------------------------------------------------------------------*/
   gt_lcdDev.c_numUsers   = 0;
   gt_lcdDev.c_numRows    = t_setup.c_numRows;
   gt_lcdDev.c_numColumns = t_setup.c_numColumns;

   return ARB_PASSED;

failed4:

   hal_releaseUartChannel( gt_lcdDev.t_uHandle);

failed3:

   arb_semaphoreDestroy( gt_lcdDev.t_mutex);

failed2:

   arb_destroyDevice( "lcdDevice0");

failed1:

   return t_err;

}/*End drv_lcdInit*/

/*---------------------------------------------------------------------------*
 * Remove the device from the system
 *---------------------------------------------------------------------------*/
void drv_lcdExit( void)
{

   if( gt_lcdDev.t_mutex != 0) /*If created... destroy*/
   {
      hal_releaseUartChannel( gt_lcdDev.t_uHandle);
      arb_semaphoreDestroy( gt_lcdDev.t_mutex);
      arb_destroyDevice( "lcdDevice0");
   }/*End if( gat_templateDev[c_index].t_mutex != 0)*/

}/*End drv_lcdExit*/
