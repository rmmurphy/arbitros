/*---------------------------------------------------------------------------*
 * Copyright (C) 2011-2013 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
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
 * File Name   : usr_console.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for command and control over the
 *               console module from a user-space perspective.
 *
 * Programmer  : Ryan M Murphy
 *
 * Last Update : Jan, 14, 2013
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arb_error.h"
#include "arb_semaphore.h"
#include "arb_device.h"
#include "arb_sysTimer.h"
#include "arb_mailbox.h"
#include "arb_console.h"
#include "drv_console.h"
#include "usr_platformTest.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
   UART_LOOPBACK_TEST,
   TWI_LOOPBACK_TEST,
   SPI_LOOPBACK_TEST
}t_testType;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void usr_loopbackTest( t_DEVHANDLE t_consoleHndl,
                              int8_t *pc_buff,
                              t_testType t_type)
{
   int16_t s_numOfTrials;
   int16_t s_count = 0;
   uint16_t s_size;
   int16_t s_numTxBytes;
   int16_t s_numRxBytes;
   int8_t ac_txMessage[32];
   int8_t ac_rxMessage[32];
   int8_t c_index;
   int32_t i_byteErrors = 0;
   t_MAILBOXHNDL t_platTestInMbx  = usr_getPlatTestInMailbox();
   t_MAILBOXHNDL t_platTestOutMbx = usr_getPlatTestOutMailbox();

   s_size = sprintf_P( (char *)pc_buff, PSTR(".------------------------------------------------------------------------.\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|        Test        |          Description          |       Setup       |\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|--------------------|-------------------------------|-------------------|\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);

   if( t_type == UART_LOOPBACK_TEST)
   {
      s_size = sprintf_P( (char *)pc_buff, PSTR("| UART loopback test | Loops back a message between  | Connect pins PE6  |\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
      s_size = sprintf_P( (char *)pc_buff, PSTR("|                    | the console and usr_platform- | and PE7.          |\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
      s_size = sprintf_P( (char *)pc_buff, PSTR("|                    | Test threads using UART6, two |                   |\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
      s_size = sprintf_P( (char *)pc_buff, PSTR("|                    | DMA's, and two mailbox modules|                   |\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
      s_size = sprintf_P( (char *)pc_buff, PSTR("'--------------------'-------------------------------'-------------------'\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
   }/*End if( t_type == UART_LOOPBACK_TEST)*/
   else if( t_type == TWI_LOOPBACK_TEST)
   {
      s_size = sprintf_P( (char *)pc_buff, PSTR("| TWI loopback test  | Loops back a message between  | Connect pins PD0  |\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
      s_size = sprintf_P( (char *)pc_buff, PSTR("|                    | the console and usr_platform- | and PE0 as well as|\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
      s_size = sprintf_P( (char *)pc_buff, PSTR("|                    | Test threads using TWI2, TWI3 | pins PD1 and PE1. |\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
      s_size = sprintf_P( (char *)pc_buff, PSTR("|                    | and two mailbox modules.      |                   |\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
      s_size = sprintf_P( (char *)pc_buff, PSTR("'--------------------'-------------------------------'-------------------'\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
   }/*End else if( t_type == TWI_LOOPBACK_TEST)*/
   else if( t_type == SPI_LOOPBACK_TEST)
   {
      s_size = sprintf_P( (char *)pc_buff, PSTR("| SPI loopback test  | Loops back a message between  | Connect pins PF5  |\n\r"));
      arb_write( t_consoleHndl,
      pc_buff,
      s_size);
      s_size = sprintf_P( (char *)pc_buff, PSTR("|                    | the console and usr_platform- | and PF6.          |\n\r"));
      arb_write( t_consoleHndl,
      pc_buff,
      s_size);
      s_size = sprintf_P( (char *)pc_buff, PSTR("|                    | Test threads using SPI2       |                   |\n\r"));
      arb_write( t_consoleHndl,
      pc_buff,
      s_size);
      s_size = sprintf_P( (char *)pc_buff, PSTR("|                    | and two mailbox modules.      |                   |\n\r"));
      arb_write( t_consoleHndl,
      pc_buff,
      s_size);
      s_size = sprintf_P( (char *)pc_buff, PSTR("'--------------------'-------------------------------'-------------------'\n\r"));
      arb_write( t_consoleHndl,
      pc_buff,
      s_size);
   }/*End else if( t_type == SPI_LOOPBACK_TEST)*/

   s_size = sprintf_P( (char *)pc_buff, PSTR("Select number of trials (0<->10000), press enter-> "));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);

   /*------------------------------------------------------------------------*
    * Wait for response. The 'arb_read' method is used instead of 'arb_ioctl'
    * because we are reading the entire contents of the command line without
    * parsing the message into arguments.
    *------------------------------------------------------------------------*/
   s_numTxBytes = arb_read( t_consoleHndl,
                            ac_txMessage,
                            (uint16_t)sizeof( ac_txMessage));

   s_numOfTrials = (int16_t)atoi( (char *)ac_txMessage);
   if( (s_numOfTrials <= 0) || (s_numOfTrials > 10000))
   {
      s_size = sprintf_P( (char *)pc_buff, PSTR("Invalid CMD\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
   }/*End if( (s_numOfTrials <= 0) || (s_numOfTrials > 10000))*/
   else
   {
      s_size = sprintf_P( (char *)pc_buff, PSTR("Enter the test message-> "));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);

      /*---------------------------------------------------------------------*
       * Wait for response. The 'arb_read' method is used instead of
       * 'arb_ioctl' because we are reading the entire contents of the
       * command line without parsing the message into arguments.
       *---------------------------------------------------------------------*/
      s_numTxBytes = arb_read( t_consoleHndl,
                               ac_txMessage,
                               (uint16_t)sizeof( ac_txMessage));

      /*---------------------------------------------------------------------*
       * Is the message too big for the queue?
       *---------------------------------------------------------------------*/
      if( (s_numTxBytes < 0) || (s_numTxBytes > (arb_mailboxGetQueueMaxSize(
      t_platTestInMbx) - 1)))
      {
         s_size = sprintf_P( (char *)pc_buff, PSTR("Message too big for mailbox.\n\r"));
         arb_write( t_consoleHndl,
                    pc_buff,
                    s_size);
      }
      else
      {

         /*------------------------------------------------------------------*
          * Prefix the message with a header that lets the platformTest
          * interface know that a uart loop test is being performed.
          *------------------------------------------------------------------*/
         memcpy( (void *)&ac_rxMessage, (void *)ac_txMessage, s_numTxBytes);

         if( t_type == UART_LOOPBACK_TEST)
            ac_txMessage[0] = PLAT_UART_TEST;
         else if( t_type == TWI_LOOPBACK_TEST)
            ac_txMessage[0] = PLAT_TWI_TEST;
         else if( t_type == SPI_LOOPBACK_TEST)
            ac_txMessage[0] = PLAT_SPI_TEST;

         memcpy( (void *)&ac_txMessage[1], (void *)ac_rxMessage,
         s_numTxBytes);

         /*------------------------------------------------------------------*
          * Increase the message size by the size of the header.
          *------------------------------------------------------------------*/
         s_numTxBytes++;

         while( s_count < s_numOfTrials)
         {

            /*---------------------------------------------------------------*
             * Send the message to the platformTest interface where it will
             * be looped between two peripherals and returned to this calling
             * thread with a subsequent mailbox message.
             *---------------------------------------------------------------*/
            s_numTxBytes = arb_mailboxWrite( t_platTestInMbx,
                                             ac_txMessage,
                                             s_numTxBytes);

            /*---------------------------------------------------------------*
             * Was the message successfully sent?
             *---------------------------------------------------------------*/
            if( s_numTxBytes > 0) /*Yes*/
            {

               /*------------------------------------------------------------*
                * Send the message to the platform test thread
                *------------------------------------------------------------*/
               s_numRxBytes = arb_mailboxRead( t_platTestOutMbx,
                                               ac_rxMessage,
                                               (uint16_t)sizeof( ac_rxMessage));

               if( s_numRxBytes > 0)
               {
                  for( c_index = 0; c_index < s_numTxBytes; c_index++)
                  {
                     if( ac_txMessage[c_index] != ac_rxMessage[c_index])
                        i_byteErrors++;
                  }/*End for( c_index = 0; c_index < s_numTxBytes; c_index++)*/

                  /*---------------------------------------------------------*
                   * Echo the received message, ignoring the header...
                   *---------------------------------------------------------*/
                  arb_write( t_consoleHndl,
                             &ac_rxMessage[1],
                             (s_numRxBytes - 1));

                  s_size = sprintf_P( (char *)pc_buff, PSTR("\n\r"));
                  arb_write( t_consoleHndl,
                             pc_buff,
                             s_size);

               }/*End if( s_numRxBytes > 0)*/

            }/*End if( s_numTxBytes > 0)*/

            s_count++;

            arb_sleep(1);

         }/*End while( s_count < s_numOfTrials)*/

         s_size = sprintf_P( (char *)pc_buff, PSTR("Test finished. Received %d out of %d messages with %d errors.\n\r"),
         s_count, s_numOfTrials, i_byteErrors);
         arb_write( t_consoleHndl,
                    pc_buff,
                    s_size);

      }
   }

}/*End usr_loopbackTest*/

static void usr_timerGpioTest( t_DEVHANDLE t_consoleHndl,
                               int8_t *pc_buff)
{
   uint16_t s_size;
   int16_t s_numOfTrials;
   int16_t s_numTxBytes;
   int16_t s_numRxBytes;
   int8_t ac_buff[32];
   t_MAILBOXHNDL t_platTestInMbx  = usr_getPlatTestInMailbox();
   t_MAILBOXHNDL t_platTestOutMbx = usr_getPlatTestOutMailbox();

   s_size = sprintf_P( (char *)pc_buff, PSTR(".------------------------------------------------------------------------.\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|        Test        |          Description          |       Setup       |\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|--------------------|-------------------------------|-------------------|\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| Timer/GPIO test    | Enables TIMER_1 which goes off| Connect pins PH3  |\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|                    | at a one second rate. The     | and PH4.          |\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|                    | timer toggles GPIO_2 which    |                   |\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|                    | causes an interrupt on GPIO_1.|                   |\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("'--------------------'-------------------------------'-------------------'\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);

   s_size = sprintf_P( (char *)pc_buff, PSTR("Select number of trials (0<->10000), press enter-> "));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);

   /*------------------------------------------------------------------------*
    * Wait for response. The 'arb_read' method is used instead of 'arb_ioctl'
    * because we are reading the entire contents of the command line without
    * parsing the message into arguments.
    *------------------------------------------------------------------------*/
   s_numTxBytes = arb_read( t_consoleHndl,
                            ac_buff,
                            (uint16_t)sizeof( ac_buff));

   s_numOfTrials = (int16_t)atoi( (char *)ac_buff);
   if( (s_numOfTrials <= 0) || (s_numOfTrials > 10000))
   {
      s_size = sprintf_P( (char *)pc_buff, PSTR("Invalid CMD\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
   }/*End if( (s_numOfTrials <= 0) || (s_numOfTrials > 10000))*/
   else
   {
      int8_t s_count = 0;

      ac_buff[0] = PLAT_TIMER_GPIO_TEST;
      ac_buff[1] = 1; /*Start timer*/

      while( s_count < s_numOfTrials)
      {

         /*------------------------------------------------------------------*
          * Send the message to the platformTest interface where it will
          * be looped between two peripherals and returned to this calling
          * thread with a subsequent mailbox message.
          *------------------------------------------------------------------*/
         s_numTxBytes = arb_mailboxWrite( t_platTestInMbx,
                                          ac_buff,
                                          2);

         /*---------------------------------------------------------------*
          * Was the message successfully sent?
          *---------------------------------------------------------------*/
         if( s_numTxBytes > 0) /*Yes*/
         {

            /*------------------------------------------------------------*
             * Send the message to the platform test thread
             *------------------------------------------------------------*/
            s_numRxBytes = arb_mailboxRead( t_platTestOutMbx,
                                            ac_buff,
                                            (uint16_t)sizeof( ac_buff));

            if( s_numRxBytes > 0)
            {

               s_size = sprintf_P( (char *)pc_buff, PSTR("Timer interrupt...\n\r"));
               arb_write( t_consoleHndl,
                          pc_buff,
                          s_size);

            }/*End if( s_numRxBytes > 0)*/

         }/*End if( s_numTxBytes > 0)*/

         s_count++;

         arb_sleep(1);

      }/*End while( s_count < s_numOfTrials)*/

      ac_buff[0] = PLAT_TIMER_GPIO_TEST;
      ac_buff[1] = 0; /*Stop timer*/

      s_numTxBytes = arb_mailboxWrite( t_platTestInMbx,
                                       ac_buff,
                                       2);

      s_size = sprintf_P( (char *)pc_buff, PSTR("Test finished.\n\r"));
      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);

   }

}/*End usr_timerGpioTest*/

static void usr_displayUserHelp( t_DEVHANDLE t_consoleHndl,
                                 int8_t *pc_buff)
{
   uint16_t s_size;

   /*------------------------------------------------------------------------*
    * Display a list of all the possible console commands specific
    * to a user-space application.
    *------------------------------------------------------------------------*/
   s_size = sprintf_P( (char *)pc_buff, PSTR(".------------------------------------------------------------------------.\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| Command |     Arguments     |              Description                 |\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|---------|-------------------|------------------------------------------|\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| ult     |                   | Performs a UART loopback test.           |\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| tlt     |                   | Performs a TWI loopback test.            |\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| tgt     |                   | Performs a test using a timer and 2 gpios|\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| slt     |                   | Performs a SPI loopback test.            |\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("'---------'-------------------'------------------------------------------'\n\r"));
   arb_write( t_consoleHndl,
              pc_buff,
              s_size);
}/*End usr_displayUserHelp*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
bool usr_console( t_DEVHANDLE t_consoleHndl,
                  int8_t *pc_buff,
                  t_consoleTokHndl *pt_tokHndl)
{

   bool b_success = true; /*A valid command was found...*/

   if( strcmp( (char *)pt_tokHndl->ac_tok[0], "hlpu") == 0)
   {
      /*---------------------------------------------------------------------*
       * Display a list of all the possible user-space specific commands.
       *---------------------------------------------------------------------*/
      usr_displayUserHelp( t_consoleHndl,
                           pc_buff);

   }/*End else if( strcmp( (char *)pt_tokHndl->ac_tok[0], "hlpu") == 0)*/
   else if( (strcmp( (char *)pt_tokHndl->ac_tok[0], "ult") == 0)
   && (pt_tokHndl->c_numTokens == 1))
   {
      /*---------------------------------------------------------------------*
       * Perform a loopback test using UART6, a DMA, and two mailbox modules.
       *---------------------------------------------------------------------*/
      usr_loopbackTest( t_consoleHndl,
                        pc_buff,
                        UART_LOOPBACK_TEST);
   }
   else if( (strcmp( (char *)pt_tokHndl->ac_tok[0], "tlt") == 0)
   && (pt_tokHndl->c_numTokens == 1))
   {
      /*---------------------------------------------------------------------*
       * Perform a loopback test using TWI2, TWI3, and two mailbox modules.
       *---------------------------------------------------------------------*/
      usr_loopbackTest( t_consoleHndl,
                        pc_buff,
                        TWI_LOOPBACK_TEST);
   }
   else if( (strcmp( (char *)pt_tokHndl->ac_tok[0], "slt") == 0)
   && (pt_tokHndl->c_numTokens == 1))
   {
      /*---------------------------------------------------------------------*
       * Perform a loopback test using SPI2 and two mailbox modules.
       *---------------------------------------------------------------------*/
      usr_loopbackTest( t_consoleHndl,
                        pc_buff,
                        SPI_LOOPBACK_TEST);
   }
   else if( (strcmp( (char *)pt_tokHndl->ac_tok[0], "tgt") == 0)
   && (pt_tokHndl->c_numTokens == 1))
   {
      /*---------------------------------------------------------------------*
       * Perform a loopback test using TIMER_1, GPIO_1, GPIO_2, and one
       * mailbox module.
       *---------------------------------------------------------------------*/
      usr_timerGpioTest( t_consoleHndl,
                         pc_buff);
   }
   else /*Unrecognized message*/
   {
      /*---------------------------------------------------------------------*
       * Let 'arb_console' know the command wasn't found.
       *---------------------------------------------------------------------*/
      b_success = false;
   }

   /*------------------------------------------------------------------------*
    * Return control over the console to the kernel...
    *------------------------------------------------------------------------*/
   return b_success;

}/*End usr_console*/
