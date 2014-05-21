/*---------------------------------------------------------------------------*
 * Copyright (C) 2011-2012 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * File Name   : drv_lcd.h
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
#ifndef drv_lcd_h

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif
   
   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define drv_lcd_h

   /*------------------------------------------------------------------------*
    * Inlude Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "utl_gpio.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      LCD_INVALID_POS = -3, /*Invalid x,y cursor position.*/
      LCD_INVALID_ARG = -2, /*Invalid ioctl argument*/
      LCD_INVALID_CMD = -1, /*Invalid ioctl command.*/
      LCD_PASSED      = 0   /*Configuration good.*/

   }t_lcdError; /*Possible error conditions returned by the device's
                  ioctl routine*/

   typedef enum
   {
      LCD_CLEAR,                /*Clear the LCD screen.*/
      LCD_CURSOR_HOME,          /*Returns the cursor to home.*/
      LCD_DISPLAY_ON,           /*Turn the LCD display on.*/
      LCD_DISPLAY_OFF,          /*Turn the LCD dsiplay off.*/
      LCD_CURSOR_ON,            /*Turn the cursor on.*/
      LCD_CURSOR_OFF,           /*Turn the cursor off.*/
      LCD_CURSOR_BLINK_ON,      /*Turns on blinking cursor.*/
      LCD_CURSOR_BLINK_OFF,     /*Turns off blinking cursor.*/
      LCD_CURSOR_SET_POS,       /*Sets the x,y position of the cursor.*/
      LCD_SCROLL_DISPLAY_LEFT,  /*Moves the display to the left*/
      LCD_SCROLL_DISPLAY_RIGHT, /*Moves the display to the right*/
      LCD_WRITE_BUILT_IN_FONT,  /*Write a built-in char to the screen.*/
      LCD_BACKLIGHT_ON,         /*Turn the backlight on.*/
      LCD_BACKLIGHT_OFF,        /*Turn the backlight off.*/
      LCD_GET_FIRMWARE,         /*Display the firmware address.*/
      LCD_STORE_CUST_CHAR       /*Stores a custom character.*/
   }t_lcdCmd;

   /*---------------------------------------------------------------------*
    * The x, y cursor position for the command LCD_CURSOR_SET_POS.
    *---------------------------------------------------------------------*/
   typedef struct
   {
      uint8_t c_row;
      uint8_t c_col;

   }t_cursPos;

   typedef struct
   {
      uint8_t c_address;
      uint8_t ac_bitMap[8];

   }t_custChar;

   typedef struct
   {
      /*---------------------------------------------------------------------*
       * The UART Id for the device being configured.
       *---------------------------------------------------------------------*/
      uint8_t c_uartId;

      /*---------------------------------------------------------------------*
       * Default baud rate for the device being configured.
       *---------------------------------------------------------------------*/
      uint32_t i_baudRate;

      /*---------------------------------------------------------------------*
       * The number of rows on the LCD display.
       *---------------------------------------------------------------------*/
      uint8_t c_numRows;

      /*---------------------------------------------------------------------*
       * The number of columns on the LCD display.
       *---------------------------------------------------------------------*/
      uint8_t c_numColumns;

      /*---------------------------------------------------------------------*
       * This value represents a unique number assigned to this driver and has 
       * to be different from all others registered with the kernel.
       *---------------------------------------------------------------------*/
      uint8_t c_majorNum;

   }t_lcdSetup;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_error drv_lcdInit( t_lcdSetup t_setup);

   void drv_lcdExit( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif
   
#endif/*End #ifndef drv_lcd_h*/
