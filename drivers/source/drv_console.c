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
 * File Name   : drv_console.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This driver is responsible for reading and writing to a
 *               specific uart designated for use as a console interface. The
 *               function defined herein are callable from 'thread' space.
 *
 * Last Update : Nov 11, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "arb_device.h"
#include "arb_semaphore.h"
#include "drv_console.h"
#include "hal_uart.h"
#include "utl_buffer.h"
#include "hal_pmic.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define CONSOLE_BACKSPACE_CHAR (127)
#define CONSOLE_RETURN_CHAR    (13)

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
    * This semaphore is used for waking up the user-space thread once a
    * carriage return has been detected.
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_rxBlockingSem;

   /*------------------------------------------------------------------------*
    * Handle to the particular RX buffer this console driver is using.
    *------------------------------------------------------------------------*/
   t_BUFFHANDLE t_rxBuffer;

   /*------------------------------------------------------------------------*
    * We are going to want to know how many 'handles' or users are attached to
    * this device driver.
    *------------------------------------------------------------------------*/
   uint8_t c_numUsers;

   /*------------------------------------------------------------------------*
    * Handle to the particular UART this console driver is using.
    *------------------------------------------------------------------------*/
   t_UARTHNDL t_uHandle;

   /*------------------------------------------------------------------------*
    * If true, the driver is in the process of reading bytes in from the
    * interface.
    *------------------------------------------------------------------------*/
   bool b_rxActive;

   /*------------------------------------------------------------------------*
    * The command prompt color.
    *------------------------------------------------------------------------*/
   int8_t c_cmdPromptColor;

   /*------------------------------------------------------------------------*
    * The foreground color.
    *------------------------------------------------------------------------*/
   int8_t c_fgColor;

   /*------------------------------------------------------------------------*
    * The name displayed for the current working directory
    *------------------------------------------------------------------------*/
   char ac_dirName[CONSOLE_MAX_TOKEN_SIZE];

}t_consoleDev;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_error consoleOpen( t_DEVHANDLE t_handle);

static int16_t consoleRead( t_DEVHANDLE t_handle,
                            int8_t *pc_buff,
                            uint16_t s_size);

static int16_t consoleWrite( t_DEVHANDLE t_handle,
                             int8_t *pc_buff,
                             uint16_t s_size);

static int32_t consoleIoctl( t_DEVHANDLE t_handle,
                             uint16_t s_command,
                             int32_t  i_arguments);

static t_error consoleClose( t_DEVHANDLE t_handle);

static void rxComplete( uint16_t s_byte);

static int8_t *drv_strTok( int8_t *pc_in,
                           int8_t *pc_out,
                           int8_t c_delim,
                           int8_t *pc_bytesRemaining,
                           int8_t *pc_tokenSize);

static t_consoleError drv_parseMessage( int8_t *pc_cbuff,
                                        int8_t ac_tok[][CONSOLE_MAX_TOKEN_SIZE],
                                        uint8_t *pc_tokenCount);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
t_deviceOperations gt_consoleDevOps =
{
    consoleOpen,
    consoleRead,
    consoleWrite,
    consoleIoctl,
    consoleClose

};

/*---------------------------------------------------------------------------*
 * This is the device's shared memory, all actions on this variable must be
 * mutually exclusive.
 *---------------------------------------------------------------------------*/
static t_consoleDev gt_consoleDev;

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static int8_t *drv_strTok( int8_t *pc_in,
                           int8_t *pc_out,
                           int8_t c_delim,
                           int8_t *pc_bytesRemaining,
                           int8_t *pc_tokenSize)
{
   int8_t c_tokenSizeCount = 0;

   /*------------------------------------------------------------------------*
    * Search for delimiting character, if not found stop at max string size
    * carriage return, or NULL char.
    *------------------------------------------------------------------------*/
   while( (*pc_in != CONSOLE_RETURN_CHAR)
       && (*pc_in != '\0')
       && (*pc_in != c_delim)
       && (c_tokenSizeCount < *pc_bytesRemaining))
   {
      pc_out[c_tokenSizeCount] = *pc_in;
      pc_in++;
      c_tokenSizeCount++;

      /*---------------------------------------------------------------------*
       * Do we have room to insert the NULL character at the end of the
       * sequence?
       *---------------------------------------------------------------------*/
      if( c_tokenSizeCount == CONSOLE_MAX_TOKEN_SIZE)
      {
         /*------------------------------------------------------------------*
          * Add one to the count in order to trigger the error condition.
          *------------------------------------------------------------------*/
         c_tokenSizeCount++;
         *pc_tokenSize = c_tokenSizeCount;
         return NULL;
      }
   }

   /*------------------------------------------------------------------------*
    * If the last char in the string is the delimiter then we may have more
    * tokens, index the pointer to the next character so that we are starting
    * at the first character of the next token on the next call into this
    * function.
    *------------------------------------------------------------------------*/
   if( *pc_in == c_delim)
      pc_in++;
   /*Else let the ptr point to the terminating character*/

   /*------------------------------------------------------------------------*
    * Replace 'ending' character (either c_delim, '\n', '\0' or the character
    * at (c_maxSize) with null character).
    *------------------------------------------------------------------------*/
   pc_out[c_tokenSizeCount] = '\0';

   c_tokenSizeCount++;

   /*------------------------------------------------------------------------*
    * Since we are extracting a token decrement *pc_maxSize by the number of
    * characters stored in pc_out. An addition 1 is subtracted for the
    * 'exit' character as well since its not included in the original s_count
    * value.
    *------------------------------------------------------------------------*/
   (*pc_bytesRemaining) = (*pc_bytesRemaining) - c_tokenSizeCount;
   (*pc_tokenSize)      = c_tokenSizeCount;

   return pc_in;

}/*End usr_strTok*/

static t_consoleError drv_parseMessage( int8_t *pc_cbuff,
                                        int8_t ac_tok[][CONSOLE_MAX_TOKEN_SIZE],
                                        uint8_t *pc_tokenCount)
{
   int8_t *pc_nextLoc      = NULL;
   int8_t c_bytesRemaining = utl_getBufferSize( gt_consoleDev.t_rxBuffer);
   int8_t c_tokenSize      = 0;
   t_consoleError t_err    = CONSOLE_PASSED;

   /*-----------------------------------------------------------------------*
    * Loop through the input buffer pc_cbuff and extract up to
    * CONSOLE_MAX_TOKENS tokens. The buffer should be formatted in the
    * following order
    *                      <toko> <tok1> <tok2> <tok3>
    * The tokens are separated by the delimiting character ' '. Each time
    * through the loop the function strTok searches cbuff for the first
    * delimiting character. But it will also stop if it finds a NULL ptr,
    * NULL character '\0', return character '\n', reaches the max token size,
    * or has run our of bytes to process. Upon finding the delimiting character
    * it will return three things; the current token contained in cbuff, a ptr
    * 'pc_nextLoc' to the next location in cbuff after finding the delimiting
    * char, and the new allowable max search size (which is the old
    * max search size minus the length of the extracted token and delimiting
    * char. If it exits for another reason, the returned ptr will contain the
    * location of the last character in the buffer i.e. a NULL ptr, '\0', '\n',
    * or whatever is there when reaching the max size.
    *------------------------------------------------------------------------*/
   pc_nextLoc = pc_cbuff;
   (*pc_tokenCount) = 0;
   do
   {
      pc_nextLoc = drv_strTok( pc_nextLoc,
                               &ac_tok[(*pc_tokenCount)][0],
                               ' ',
                               &c_bytesRemaining,
                               &c_tokenSize);
      (*pc_tokenCount)++;
      if( (*pc_tokenCount) > CONSOLE_MAX_TOKENS)
      {
         /*------------------------------------------------------------------*
          * Error too many tokens....
          *------------------------------------------------------------------*/
         return CONSOLE_TOO_MANY_TOKENS;
      }/*End if( (*pc_tokenCount) > CONSOLE_MAX_TOKENS)*/
      else if( c_tokenSize > CONSOLE_MAX_TOKEN_SIZE)
      {
         /*------------------------------------------------------------------*
          * Error token too large....
          *------------------------------------------------------------------*/
         return CONSOLE_TOKEN_TOO_LARGE;
      }

      if( t_err < 0)
         return t_err;

   }while( (*pc_nextLoc != '\n') && (*pc_nextLoc != '\0') && (pc_nextLoc != NULL)
   && (c_bytesRemaining > 0));

   return CONSOLE_PASSED;

}/*End usr_parseMessage*/

static void rxComplete( uint16_t s_byte)
{
   uint16_t s_bufferLevel = utl_getBufferFullLevel( gt_consoleDev.t_rxBuffer);
   uint16_t s_bufferSize  = utl_getBufferSize( gt_consoleDev.t_rxBuffer);

   gt_consoleDev.b_rxActive = true;

   /*------------------------------------------------------------------------*
    * If a backspace has been entered, erase the bytes in the buffer until
    * there is no longer any more data available.
    *------------------------------------------------------------------------*/
   if( (s_byte == CONSOLE_BACKSPACE_CHAR) && (s_bufferLevel > 0))
   {
      /*---------------------------------------------------------------------*
       * Echo back received byte- in this case the backspace character.
       *---------------------------------------------------------------------*/
      hal_uartWriteByte( gt_consoleDev.t_uHandle,
                         s_byte);

      /*---------------------------------------------------------------------*
       * Remove the last byte in the buffer...
       *---------------------------------------------------------------------*/
      utl_buffEraseTailByte( gt_consoleDev.t_rxBuffer);

   }/*End if( (s_byte == CONSOLE_BACKSPACE_CHAR) && (s_bufferLevel > 0))*/
   else if( s_byte == CONSOLE_RETURN_CHAR)
   {

      /*---------------------------------------------------------------------*
       * Echo back received byte
       *---------------------------------------------------------------------*/
      hal_uartWriteByte( gt_consoleDev.t_uHandle,
                         s_byte);

      /*------------------------------------------------------------------*
       * Write new line character out uart interface
       *------------------------------------------------------------------*/
      hal_uartWriteByte( gt_consoleDev.t_uHandle,
                         '\n');

      gt_consoleDev.b_rxActive = false;

      /*------------------------------------------------------------------*
       * Disable the receive interrupt until the data has been handled.
       *------------------------------------------------------------------*/
      hal_disableUartRxInt( gt_consoleDev.t_uHandle);

      /*------------------------------------------------------------------*
       * Add a the NULL character to the end of the buffer - this is for
       * data parsing purposes only.
       *------------------------------------------------------------------*/
      utl_writeByte( gt_consoleDev.t_rxBuffer,
                     '\0');

      /*------------------------------------------------------------------*
       * Signal any waiting threads that a carriage return has been
       * received.
       *------------------------------------------------------------------*/
      arb_signal( gt_consoleDev.t_rxBlockingSem);


   }/*End else if( c_byte == CONSOLE_BACKSPACE_CHAR)*/
   else if( (s_bufferLevel < (s_bufferSize - 1)) && (s_byte !=
   CONSOLE_BACKSPACE_CHAR) && (s_byte != 27)) /*Leave room for NULL char*/
   {

      /*---------------------------------------------------------------------*
       * Echo back received byte....
       *---------------------------------------------------------------------*/
      hal_uartWriteByte( gt_consoleDev.t_uHandle,
                         s_byte);

      /*---------------------------------------------------------------------*
       * Fill RX buffer with received bytes until the buffer is full
       * leaving room for the insertion of the NULL character.
       *---------------------------------------------------------------------*/
      utl_writeByte( gt_consoleDev.t_rxBuffer,
                     s_byte);

   }

}/*End rxComplete*/

static t_error consoleOpen( t_DEVHANDLE t_handle)
{

   t_uartError t_uErr;
   t_error t_err = ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_consoleDev.t_txMutex,
             0);

   /*------------------------------------------------------------------------*
    * Keep track of the number of user-space applications using the driver.
    *------------------------------------------------------------------------*/
   gt_consoleDev.c_numUsers++;

   /*------------------------------------------------------------------------*
    * If there is at least one user-space handle attached to this driver
    * than enable the receive interrupt.
    *------------------------------------------------------------------------*/
   if( gt_consoleDev.c_numUsers == 1)
   {

      t_uErr = hal_enableUartRxInt( gt_consoleDev.t_uHandle);

      if( t_uErr < 0)
         t_err = ARB_HAL_ERROR;

   }/*End if( gt_consoleDev.c_numUsers == 1)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_consoleDev.t_txMutex);

   return t_err;

}/*End consoleOpen*/

static int16_t consoleRead( t_DEVHANDLE t_handle,
                            int8_t *pc_buff,
                            uint16_t s_size)
{
   int16_t s_bufferLevel = 0;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_consoleDev.t_rxMutex,
             0);

   /*------------------------------------------------------------------------*
    * Wait for data to be available in the RX buffer.
    *------------------------------------------------------------------------*/
   arb_wait( gt_consoleDev.t_rxBlockingSem,
             0);

   /*------------------------------------------------------------------------*
    * Level plus an extra NULL character.
    *------------------------------------------------------------------------*/
   s_bufferLevel = utl_getBufferFullLevel( gt_consoleDev.t_rxBuffer);

   /*------------------------------------------------------------------------*
    * Can the user-space buffer hold all the data plus the NULL character?
    *------------------------------------------------------------------------*/
   if( s_bufferLevel > s_size)
   {
      ult_resetBuffer( gt_consoleDev.t_rxBuffer);

      hal_enableUartRxInt( gt_consoleDev.t_uHandle);

      /*---------------------------------------------------------------------*
       * Release the lock
       *---------------------------------------------------------------------*/
      arb_signal( gt_consoleDev.t_rxMutex);
      return (int16_t)ARB_READ_ERROR;

   }/*End if( s_bufferLevel > s_size)*/

   utl_readBlock( gt_consoleDev.t_rxBuffer,
                  pc_buff,
                  s_bufferLevel);

   /*------------------------------------------------------------------------*
    * In order to treat the buffer as if its linear (when using the ioctl
    * command 'CONSOLE_PARSE_CMD_LINE') we need to reset the pointers each
    * time we read its contents.
    *------------------------------------------------------------------------*/
   ult_resetBuffer( gt_consoleDev.t_rxBuffer);

   hal_enableUartRxInt( gt_consoleDev.t_uHandle);

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_consoleDev.t_rxMutex);

   return s_bufferLevel;

}/*End consoleRead*/

static int16_t consoleWrite( t_DEVHANDLE t_handle,
                             int8_t *pc_buff,
                             uint16_t s_size)
{
   uint16_t s_numBytes;

   /*------------------------------------------------------------------------*
    * A call to this function can occur within an interrupt, therefore 
    * mutual exclusion is performed by disabling interrupts for a brief
    * period of time.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   s_numBytes = hal_uartWriteBlock( gt_consoleDev.t_uHandle,
                                    pc_buff,
                                    s_size);


   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   HAL_END_CRITICAL();

   return s_numBytes;

}/*End consoleWrite*/

static int32_t consoleIoctl( t_DEVHANDLE t_handle,
                             uint16_t s_command,
                             int32_t i_arguments)
{
   int32_t i_return = (int32_t)CONSOLE_PASSED;
   t_consoleTokHndl *pt_tokens = NULL;
   int8_t *pc_rxBuffPtr = NULL;
   uint16_t s_size = 0;
   int8_t ac_buff[40];

   switch( (t_consoleCmd)s_command)
   {
      case CONSOLE_GET_RX_BUFFER_SIZE:

         i_return = (int32_t)utl_getBufferSize( gt_consoleDev.t_rxBuffer);

      break;/*End case CONSOLE_GET_RX_BUFFER_SIZE:*/

      case CONSOLE_GET_BAUD_RATE:

         i_return = (int32_t)hal_uartGetBaudRate( gt_consoleDev.t_uHandle);

      break;/*End case CONSOLE_GET_BAUD_RATE:*/

      case CONSOLE_SET_BAUD_RATE:

      break;/*End case CONSOLE_SET_BAUD_RATE:*/

      case CONSOLE_PARSE_CMD_LINE:

         /*------------------------------------------------------------------*
          * We are going to access global memory or a register, so perform lock
          *------------------------------------------------------------------*/
         arb_wait( gt_consoleDev.t_rxMutex,
                   0);

         /*------------------------------------------------------------------*
          * Grab pointer to the user-space token variable...
          *------------------------------------------------------------------*/
         pt_tokens = (t_consoleTokHndl *)((uint16_t)i_arguments);

         /*------------------------------------------------------------------*
          * Wait for data to be available in the RX buffer.
          *------------------------------------------------------------------*/
         arb_wait( gt_consoleDev.t_rxBlockingSem,
                   0);

         /*------------------------------------------------------------------*
          * Get the head location of the RX buffer...
          *------------------------------------------------------------------*/
         pc_rxBuffPtr = utl_getBufferPtr( gt_consoleDev.t_rxBuffer);

         /*------------------------------------------------------------------*
          * Parse the string into tokens represented by...
          * <cmd> <arg1> <arg2> <arg3>, where ac_tok[0] = <cmd>, ac_tok[1] =
          * <arg1>, ac_tok[2] = <arg2>, ac_tok[3] = <arg3>
          *------------------------------------------------------------------*/
         i_return = (int32_t)drv_parseMessage( pc_rxBuffPtr,
                                               pt_tokens->ac_tok,
                                               &pt_tokens->c_numTokens);

         /*------------------------------------------------------------------*
          * In order to treat the buffer as if its linear (when using the ioctl
          * command 'CONSOLE_PARSE_CMD_LINE') we need to reset the pointers
          * each time we read its contents.
          *------------------------------------------------------------------*/
         ult_resetBuffer( gt_consoleDev.t_rxBuffer);

         hal_enableUartRxInt( gt_consoleDev.t_uHandle);

         /*------------------------------------------------------------------*
          * Release the lock
          *------------------------------------------------------------------*/
         arb_signal( gt_consoleDev.t_rxMutex);

      break;/*End case CONSOLE_PARSE_CMD_LINE:*/

      case CONSOLE_DISPLAY_PROMPT:

         /*------------------------------------------------------------------*
          * We are going to access global memory or a register, so perform lock
          *------------------------------------------------------------------*/
         arb_wait( gt_consoleDev.t_txMutex,
                   0);

         /*------------------------------------------------------------------*
          * Set the background color and repaint the entire screen
          *------------------------------------------------------------------*/
         s_size = sprintf( (char *)ac_buff, 
                           "\e[1;3%cm[%s]#\e[1;3%cm ",
                           gt_consoleDev.c_cmdPromptColor,
                           gt_consoleDev.ac_dirName,
                           gt_consoleDev.c_fgColor);

         hal_uartWriteBlock( gt_consoleDev.t_uHandle,
                             ac_buff,
                             s_size);

         /*------------------------------------------------------------------*
          * Release the lock
          *------------------------------------------------------------------*/
         arb_signal( gt_consoleDev.t_txMutex);

      break;/*End case CONSOLE_DISPLAY_PROMPT:*/

      case CONSOLE_SET_PROMPT:
      {
         /*------------------------------------------------------------------*
          * We are going to access global memory or a register, so perform lock
          *------------------------------------------------------------------*/
         arb_wait( gt_consoleDev.t_txMutex,
                   0);

         char *pc_name = (char *)((uint16_t)i_arguments);
         sprintf( gt_consoleDev.ac_dirName, "%s", pc_name);

         /*------------------------------------------------------------------*
          * Release the lock
          *------------------------------------------------------------------*/
         arb_signal( gt_consoleDev.t_txMutex);
      }
      break;/*End case CONSOLE_SET_PROMPT:*/

      case CONSOLE_RESET_TERMINAL:

         /*------------------------------------------------------------------*
          * We are going to access global memory or a register, so perform lock
          *------------------------------------------------------------------*/
         arb_wait( gt_consoleDev.t_txMutex,
                   0);

         /*------------------------------------------------------------------*
          * Reset the terminal to its default settings...
          *------------------------------------------------------------------*/
         s_size = sprintf_P((char *)ac_buff, PSTR("\ec"));

         hal_uartWriteBlock( gt_consoleDev.t_uHandle,
                             ac_buff,
                             s_size);

         /*------------------------------------------------------------------*
          * Release the lock
          *------------------------------------------------------------------*/
         arb_signal( gt_consoleDev.t_txMutex);

      break;/*End case CONSOLE_RESET_TERMINAL:*/

      case CONSOLE_SET_BG_COLOR:

         /*------------------------------------------------------------------*
          * We are going to access global memory or a register, so perform lock
          *------------------------------------------------------------------*/
         arb_wait( gt_consoleDev.t_txMutex,
                   0);

         if( (i_arguments < CONSOLE_BLACK) || (i_arguments > CONSOLE_WHITE))
         {
            i_return = (int32_t)CONSOLE_INVALID_COLOR;
         }
         else
         {
            /*---------------------------------------------------------------*
             * Set the background color and repaint the entire screen
             *---------------------------------------------------------------*/
            s_size = sprintf( (char *)ac_buff, 
                              "\e[4%cm\e[2J",
                              (char)i_arguments);

            hal_uartWriteBlock( gt_consoleDev.t_uHandle,
                                ac_buff,
                                s_size);

         }

         /*------------------------------------------------------------------*
          * Release the lock
          *------------------------------------------------------------------*/
         arb_signal( gt_consoleDev.t_txMutex);

      break;/*End case CONSOLE_SET_BG_COLOR:*/

      case CONSOLE_SET_PROMPT_COLOR:

         if( (i_arguments < CONSOLE_BLACK) || (i_arguments > CONSOLE_WHITE))
         {
            i_return = (int32_t)CONSOLE_INVALID_COLOR;
         }
         else
         {
            gt_consoleDev.c_cmdPromptColor = (int8_t)i_arguments;
         }

      break;/*End case CONSOLE_SET_PROMPT_COLOR*/

      case CONSOLE_SET_FG_COLOR:

         /*------------------------------------------------------------------*
          * We are going to access global memory or a register, so perform lock
          *------------------------------------------------------------------*/
         arb_wait( gt_consoleDev.t_txMutex,
                   0);

         if( (i_arguments < CONSOLE_BLACK) || (i_arguments > CONSOLE_WHITE))
         {
            i_return = (int32_t)CONSOLE_INVALID_COLOR;
         }
         else
         {
            gt_consoleDev.c_fgColor = (int8_t)i_arguments;
            /*---------------------------------------------------------------*
             * Set the foreground color...
             *---------------------------------------------------------------*/
            s_size = sprintf( (char *)ac_buff, 
                              "\e[1;3%cm", 
                              (char)i_arguments);

            hal_uartWriteBlock( gt_consoleDev.t_uHandle,
                                ac_buff,
                                s_size);

         }

         /*------------------------------------------------------------------*
          * Release the lock
          *------------------------------------------------------------------*/
         arb_signal( gt_consoleDev.t_txMutex);

      break;/*End case CONSOLE_SET_FG_COLOR:*/

      default:

         i_return = (int32_t)CONSOLE_INVALID_CMD;

      break;

   }/*End switch( (t_consoleCmd)s_command)*/

   return i_return;

}/*End consoleIoctl*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
static t_error consoleClose( t_DEVHANDLE t_handle)
{
   t_uartError t_uErr;
   t_error t_err = ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_consoleDev.t_rxMutex,
             0);

   /*------------------------------------------------------------------------*
    * Keep track of the number of user-space applications using the driver.
    *------------------------------------------------------------------------*/
   gt_consoleDev.c_numUsers--;

   /*------------------------------------------------------------------------*
    * If there are no more handles attached to this driver than disable the
    * receive interrupt.
    *------------------------------------------------------------------------*/
   if( gt_consoleDev.c_numUsers == 0)
   {

      t_uErr = hal_disableUartRxInt( gt_consoleDev.t_uHandle);

      if( t_uErr < 0)
         t_err = ARB_HAL_ERROR;

   }/*End if( gt_consoleDev.c_numUsers == 0)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_consoleDev.t_rxMutex);

   return t_err;

}/*End consoleClose*/

t_error drv_consoleInit( t_consoleSetup t_setup)
{
   t_error t_err = ARB_PASSED;
   t_uartConfig t_uConf;

   /*------------------------------------------------------------------------*
    * Make sure the kernel is aware that a new device has been loaded.
    *------------------------------------------------------------------------*/
   t_err = arb_registerDevice( "consoleDevice0",
                               arb_createDevId( t_setup.c_majorNum, 0),
                               &gt_consoleDevOps);

   if( t_err < 0)
   {
      goto failed1;
   }

   /*------------------------------------------------------------------------*
    * Request a semaphore from the kernel. Since the signal port is a shared
    * resource we need to have all actions on it be mutually exclusive.
    *------------------------------------------------------------------------*/
   gt_consoleDev.t_rxMutex = arb_semaphoreCreate( MUTEX);

   if( gt_consoleDev.t_rxMutex < 0)
   {
      t_err = (t_error)gt_consoleDev.t_rxMutex;
      goto failed2;

   }/*End if( gt_consoleDev.t_rxMutex < 0)*/

   /*------------------------------------------------------------------------*
    * Request a semaphore from the kernel. We will use this semaphore for
    * signaling the user-space program when the RX buffer has data.
    *------------------------------------------------------------------------*/
   gt_consoleDev.t_rxBlockingSem = arb_semaphoreCreate( COUNTING);

   if( gt_consoleDev.t_rxBlockingSem < 0)
   {
      t_err = (t_error)gt_consoleDev.t_rxBlockingSem;
      goto failed3;

   }/*End if( gt_consoleDev.t_rxBlockingSem < 0)*/

   /*------------------------------------------------------------------------*
    * Grab handle to console UART
    *------------------------------------------------------------------------*/
   gt_consoleDev.t_uHandle = hal_requestUartChannel( t_setup.c_uartId);
   if( gt_consoleDev.t_uHandle < 0)
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
    * By setting the tx call-back to NULL, all data transfers over the uart
    * are performed "in-place".
    *------------------------------------------------------------------------*/
   t_uConf.pf_txCallBack = NULL;

   /*------------------------------------------------------------------------*
    * Configure console UART
    *------------------------------------------------------------------------*/
   if( hal_configureUartChannel( gt_consoleDev.t_uHandle,
                                 t_uConf) < 0)
   {
      t_err = ARB_HAL_ERROR;
      goto failed5;
   }

   /*------------------------------------------------------------------------*
    * Allocate RX buffer of size = A*B1, where A = the max number of
    * possible tokens, B = the size of each token including a terminating
    * character, and 1 byte for the character that ends the string.
    *------------------------------------------------------------------------*/
   gt_consoleDev.t_rxBuffer = utl_createBuffer( CONSOLE_MAX_TOKENS*
   CONSOLE_MAX_TOKEN_SIZE + 1);
   if( gt_consoleDev.t_rxBuffer < 0)
   {
      t_err = ARB_OUT_OF_HEAP;
      goto failed5;
   }/*End if( gt_consoleDev.t_txBuffer < 0)*/

   /*------------------------------------------------------------------------*
    * Request a semaphore from the kernel. Since the signal port is a shared
    * resource we need to have all actions on it be mutually exclusive.
    *------------------------------------------------------------------------*/
   gt_consoleDev.t_txMutex = arb_semaphoreCreate( MUTEX);

   if( gt_consoleDev.t_txMutex < 0)
   {
      t_err = (t_error)gt_consoleDev.t_txMutex;
      goto failed6;

   }/*End if( gt_consoleDev.t_txMutex < 0)*/

   /*------------------------------------------------------------------------*
    * We don't have any users attached to this device
    *------------------------------------------------------------------------*/
   gt_consoleDev.c_numUsers = 0;

   /*------------------------------------------------------------------------*
    * This variable is used as a means of blocking the TX register empty
    * interrupt from sending a byte of data out of the uart interface while
    * the driver is currently receiving data over the user interface.
    *------------------------------------------------------------------------*/
   gt_consoleDev.b_rxActive = false;

   /*------------------------------------------------------------------------*
    * Set the default colors...
    *------------------------------------------------------------------------*/
   gt_consoleDev.c_cmdPromptColor = CONSOLE_GREEN;
   gt_consoleDev.c_fgColor = CONSOLE_WHITE;

   /*------------------------------------------------------------------------*
    * Set the default prompt working directory.
    *------------------------------------------------------------------------*/
   sprintf( gt_consoleDev.ac_dirName, "/");

   return ARB_PASSED;

failed6:

   utl_destroyBuffer( gt_consoleDev.t_rxBuffer);

failed5:

   hal_releaseUartChannel( gt_consoleDev.t_uHandle);

failed4:

   arb_semaphoreDestroy( gt_consoleDev.t_rxBlockingSem);

failed3:

   arb_semaphoreDestroy( gt_consoleDev.t_rxMutex);

failed2:

   arb_destroyDevice( "consoleDevice0");

failed1:

   return t_err;

}/*End drv_consoleInit*/

/*---------------------------------------------------------------------------*
 * Remove the device from the system
 *---------------------------------------------------------------------------*/
void drv_consoleExit( void)
{

   if( gt_consoleDev.t_rxMutex != 0) /*If created... destroy*/
   {

      utl_destroyBuffer( gt_consoleDev.t_rxBuffer);
      hal_releaseUartChannel( gt_consoleDev.t_uHandle);
      arb_semaphoreDestroy( gt_consoleDev.t_rxBlockingSem);
      arb_semaphoreDestroy( gt_consoleDev.t_rxMutex);
      arb_semaphoreDestroy( gt_consoleDev.t_txMutex);
      arb_destroyDevice( "consoleDevice0");

      memset( (void *)&gt_consoleDev, 0, sizeof( gt_consoleDev));

   }/*End if( gt_consoleDev.t_rxMutex != 0)*/

}/*End drv_consoleExit*/
