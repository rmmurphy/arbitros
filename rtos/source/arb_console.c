/*************************************************************************//**
 * @file arb_console.c
 *
 * @ingroup RTOS
 *
 * @brief Main handler for the system console interface.
 *
 * @details This file defines a set of functions that provide the tools
 *          for interfacing with the operating system or user-space application
 *          via a terminal window. The primary handler is a blocking
 *          thread named #arb_console, which wakes any time a message is
 *          is received over the terminal via the device driver drv_console.c.
 *          The following figure demonstrates how messages are exchanged and
 *          validated between the driver, console thread, and user-space 
 *          handler.
 * @image html consoleMsgFlow.jpg Example: Console message handling
 *        After waking, the thread validates the tokenized message by checking 
 *        the first entry against a list of basic kernel commands (<b>ls, cd, 
 *        help, dev, top, sdl, rm, sct, and head</b>). If the token doesn't 
 *        match, the entire message is routed to the user-space layer for 
 *        further evaluation. After processing the message the user-space 
 *        layer returns control back to #arb_console with an indicator letting
 *        the thread know if the message was handled properly.
 *
 * @paragpraph @ref <term> Standard Output (Terminal)
 * @image html cmdPrompt1.jpg Example: Using commands ls, cd, help, and dev
 *        As seen in the figure, Arbitros offers a rich set of terminal 
 *        commands. The first command <b>ls</b>, outputs a list of all the 
 *        directories and files currently housed on the system. In this 
 *        example, only the 'LOGS' directory with 'DMSG.TXT' is present. These 
 *        particular files are created during system initialization and used by
 *        the function #arb_printf for logging user-space or kernel debug 
 *        messages. The <b>help</b> command lists all the commands--with 
 *        descriptions--recognized by the terminal handler #arb_console. 
 *        Finally, the command <b>dev</b> displays a list of all the device 
 *        drivers and number of open device driver handles.
 * @image html cmdPrompt2.jpg Example: Using commands top, sdl, and rm
 *        The second figure highlights the commands <b>top, sdl, and rm</b>. The
 *        command <b>top</b> displays metrics such as the size of various memory
 *        sections (.data, .bss, and .heap), as well as the amount RAM left on
 *        the system. Additionally, <b>top</b> displays an estimate of processor
 *        thread loading comprised of metrics averaged over a one minute and
 *        five minute interval. This feature allows an Arbitros developer to
 *        tune the performance of a particular thread or collection of threads
 *        in order to minimize the burden on the system. The command <b>sdl</b>
 *        with argument 0,1, or 2 turns on one of #arb_printf's three debug 
 *        levels #PRINTF_DBG_LOW, #PRINTF_DBG_MED, and #PRINTF_DBG_HIGH, 
 *        respectively. As seen in the figure, <b>sdl 0</b> enables the lowest 
 *        debug level #PRINTF_DBG_LOW, which displays a message every time the
 *        thread #arb_idle resets the system watchdog timer. This particular
 *        debug configuration will tell the kernel to pass any priority 
 *        #arb_printf debug message onto the terminal. Increasing the debug
 *        level, such as configuring <b>sdl 1 (or 2)</b>, will tell Arbitros to
 *        pass #arb_printf messages with the same or higher priority onto the 
 *        terminal. The ability to display messages based on priority eases
 *        software integration, allowing a developer to "de-clutter" the 
 *        terminal output in order to highlight the most important information.
 *        The final command <b>rm -r</b>, recursively removes all files and
 *        folders stored on the system flash drive.
 *
 * @copyright Copyright (C) 2011-2013 Ryan M. Murphy <ryan.m.murphy.77@gmail.
 *            com> This program is free software: you can redistribute it and/
 *            or modify it under the terms of the GNU General Public License as
 *            published by the Free Software Foundation, either version 3 of
 *            the License, or (at your option) any later version. This program
 *            is distributed in the hope that it will be useful, but WITHOUT
 *            ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 *            or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 *            License for more details. You should have received a copy of the
 *            GNU General Public License along with this program.  If not, see
 *            <http://www.gnu.org/licenses/>.
 *
 * @author Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 * @date Jan, 14, 2013
 *
 * @version 1.0
 *****************************************************************************/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arb_error.h"
#include "arb_thread.h"
#include "arb_device.h"
#include "arb_sysTimer.h"
#include "arb_console.h"
#include "arb_printf.h"
#include "arb_scheduler.h"
#include "drv_console.h"
#include "drv_sd.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*************************************************************************//**
 * @brief This macro controls the number of lines displayed to the terminal
 *        when printing the contents of a file using the command 'head'.
 *****************************************************************************/
#define PRINTF_NUM_LINES_TO_PRINT (20)

/*---------------------------------------------------------------------------*
 * Private Typedefs
 *---------------------------------------------------------------------------*/

/*************************************************************************//**
 * @struct t_consoleObject
 *
 * @brief Grouping of objects commonly used across all functions within this 
 *        file.
 *****************************************************************************/
typedef struct
{
   /**********************************************************************//**
    * Handle to the console thread created during module initialization
    * (#arb_consoleInit).
    **************************************************************************/
   t_THRDHANDLE t_consoleThread;

   /**********************************************************************//**
    * Handle to the console thread created during module initialization
    * (#arb_consoleInit). Handles are the primary mechanism for linking a
    * thread (i.e. #arb_console)--through the kernel--to the particular device
    * driver its trying to access.
    **************************************************************************/
   t_DEVHANDLE t_consoleHndl;

   /**********************************************************************//**
    * Handle to the fat32 SD card driver created during module initialization
    * (#arb_consoleInit). This handle provides a way of exposing file system 
    * functionality--such as directory searching, file deletion, and reading
    * the contents of a file--to an external user via the terminal command 
    * window by way of the #arb_console thread.
    **************************************************************************/
   t_DEVHANDLE t_sdHndl;

   /**********************************************************************//**
    * Pointer to the user-space console handler configured during module
    * initialization (#arb_consoleInit). The console thread (#arb_console)
    * passes control to this function after determining that a command (entered
    * over the terminal) is not part of the Arbitros basic kernel command set.
    **************************************************************************/
   bool (*pf_funPtr)( t_DEVHANDLE t_consoleHndl,
                      int8_t *pc_buff,
                      t_consoleTokHndl *pt_tokHndl);

}t_consoleObject;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static void arb_console( t_parameters t_param,
                         t_arguments  t_args);

static bool arb_head( int8_t *pc_buff,
                      t_consoleTokHndl *pt_tokHndl);

static void arb_displayKernelHelp( int8_t *pc_buff);

static void arb_setDebugLevel( t_consoleTokHndl *pt_tokHndl,
                               int8_t *pc_buff);

static void arb_displayDeviceList( int8_t *pc_buff);

static void arb_displaySystemStatistics( int8_t *pc_buff);

/*************************************************************************//**
 * @var gt_conObject
 *
 * @brief Global variable containing objects common to all the functions within 
 *        the file arb_console.c.
 *****************************************************************************/
static t_consoleObject gt_conObject;

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/

/*************************************************************************//**
 * @fn static bool arb_head( int8_t *pc_buff,
 *                           t_consoleTokHndl *pt_tokHndl)
 *
 * @brief Reads the contents of a file and writes them to the command window.
 *
 * @details This function prints the contents of the file (pointed to by <b>
 *          pt_tokHndl</b>) entered via the terminal window using the command 
 *          <b>head <filename></b>. The file is iteratively printed 
 *          #PRINTF_NUM_LINES_TO_PRINT lines at a time. At the end of each 
 *          iteration, the user is asked if another #PRINTF_NUM_LINES_TO_PRINT
 *          lines should be displayed or if termination is required.
 *
 * @param[in] pc_buff Scratch buffer used for reading/writing messages between
 *            the command window and system console thread #arb_console.
 *
 * @param[in] pt_tokHndl pointer to the current list of command line tokens.
 *
 * @return 'true', if the file (<filename>) was opened, otherwise 'false'.
 *****************************************************************************/
static bool arb_head( int8_t *pc_buff,
                      t_consoleTokHndl *pt_tokHndl)
{

   t_DEVHANDLE t_fileHndl;
   int16_t s_numBytesToNewLine;
   int16_t s_numBytes;
   int8_t ac_data[21];
   int8_t *pc_devName;
   size_t t_size1;
   size_t t_size2;
   int16_t s_size;
   uint8_t c_lineCount;
   int32_t i_fileSize;
   char *pc_newLinePos;

   ac_data[20] = '\0';

   pc_devName = arb_getDevName( gt_conObject.t_sdHndl);

   t_size1 = sizeof( pc_devName);
   t_size2 = sizeof( &pt_tokHndl->ac_tok[1]);

   if( (t_size1 + t_size2) > MAX_CONSOLE_BUFF_SIZE)
   {
      return false;
   }

   /*------------------------------------------------------------------------*
    * Concatenate the hard drive device name with the file name so that
    * 'arb_device.c' knows where to look.
    *------------------------------------------------------------------------*/
   s_numBytes = sprintf( (char *)pc_buff,
                         "%s/%s",
                          pc_devName,
                          (char *)&pt_tokHndl->ac_tok[1]);

   /*------------------------------------------------------------------------*
    * Open the file for reading
    *------------------------------------------------------------------------*/
   t_fileHndl = arb_open( (char *)pc_buff,
                          ARB_O_READ);

   if( t_fileHndl > 0)
   {

      i_fileSize = arb_ioctl( t_fileHndl,
                              SD_GET_SIZE,
                              0);

      c_lineCount = 0;
      do
      {

         /*------------------------------------------------------------------*
          * Read 'PRINTF_NUM_LINES_TO_PRINT' bytes from the file...
          *------------------------------------------------------------------*/
         s_numBytes = arb_read( t_fileHndl,
                                ac_data,
                                20);

         i_fileSize = i_fileSize - s_numBytes;

         /*------------------------------------------------------------------*
          * Find first occurrence of new line character.
          *------------------------------------------------------------------*/
         pc_newLinePos = strchr( (char *)ac_data, '\n');

         /*------------------------------------------------------------------*
          * Does the buffer contain a new line?
          *------------------------------------------------------------------*/
         if( pc_newLinePos != NULL) /*Yes*/
         {

            /*---------------------------------------------------------------*
             * Have 'PRINTF_NUM_LINES_TO_PRINT' lines been displayed?
             *---------------------------------------------------------------*/
            c_lineCount++;
            if( c_lineCount == PRINTF_NUM_LINES_TO_PRINT) /*Yes*/
            {

               /*------------------------------------------------------------*
                * Print all the characters up to and including the new line.
                *------------------------------------------------------------*/
               s_numBytesToNewLine = (int16_t)pc_newLinePos -
               (int16_t)&ac_data[0];

               if( s_numBytesToNewLine > 0)
               {
                  arb_write( gt_conObject.t_consoleHndl,
                             ac_data,
                             s_numBytesToNewLine+1);

               }/*End if( s_numBytes > 0)*/

               s_size = sprintf_P( (char *)pc_buff, PSTR("\r\nPress 'Enter' to resume, 'q' to stop.\n\r"));
               arb_write( gt_conObject.t_consoleHndl,
                          pc_buff,
                          s_size);

               /*------------------------------------------------------------*
                * Block until user responds to message.
                *------------------------------------------------------------*/
               arb_ioctl( gt_conObject.t_consoleHndl,
                          CONSOLE_PARSE_CMD_LINE,
                          (uint32_t)((uint16_t)pt_tokHndl));

               /*------------------------------------------------------------*
                * Quit reading the file?
                *------------------------------------------------------------*/
               if( strcasecmp( (char *)&pt_tokHndl[0], "q") == 0) /*Yes*/
                  break;

               /*------------------------------------------------------------*
                * Print the rest of the data in the buffer that occurred after
                * the position of the new line character.
                *------------------------------------------------------------*/
               s_numBytes = s_numBytes - (s_numBytesToNewLine + 1);
               if( s_numBytes > 0)
               {
                  arb_write( gt_conObject.t_consoleHndl,
                             (int8_t *)(pc_newLinePos + 1),
                             s_numBytes);
               }/*End if( s_numBytes > 0)*/

            }/*End if( c_lineCount == PRINTF_NUM_LINES_TO_PRINT)*/

         }/*End if( pc_newLinePos != NULL)*/

         /*------------------------------------------------------------------*
          * If c_lineCount = 'PRINTF_NUM_LINES_TO_PRINT' then the data
          * contained in the buffer 'ac_data' has already been displayed.
          *------------------------------------------------------------------*/
         if( c_lineCount < PRINTF_NUM_LINES_TO_PRINT)
         {
            if( s_numBytes > 0)
            {
               arb_write( gt_conObject.t_consoleHndl,
                          ac_data,
                          s_numBytes);

            }/*End if( s_numBytes > 0)*/
         }
         else
            c_lineCount = 0;

      }while( i_fileSize > 0);

   }/*End if( t_fileHndl > 0)*/
   else
      return false;

   /*------------------------------------------------------------------------*
    * Close the file.
    *------------------------------------------------------------------------*/
   arb_close( t_fileHndl);

   return true;

}/*End arb_head*/

/*************************************************************************//**
 * @fn static void arb_displayKernelHelp( int8_t *pc_buff)
 *
 * @brief Displays a list of kernel commands to the terminal window.
 *
 * @details After the thread #arb_console receives the <b>help</b> command via
 *          the terminal window it calls this function in order to display a
 *          list of all possible kernel commands.
 *
 * @param[in] pc_buff scratch buffer used for writing the kernel commands and
 *            their description to the terminal window.
 *
 * @return None.
 *****************************************************************************/
static void arb_displayKernelHelp( int8_t *pc_buff)
{
   uint16_t s_size;

   s_size = sprintf_P( (char *)pc_buff, PSTR(".------------------------------------------------------------------------.\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| Command |     Arguments     |              Description                 |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|---------|-------------------|------------------------------------------|\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| sdl     | <arg1>            | Set the debug level, where <arg1> =      |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|         |                   | 0,1,2 (low, med, high)                   |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| dev     |                   | Returns a list of active drivers.        |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| top     |                   | Displays system statistics.              |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| ls      | <arg1>            | Displays the contents of dir <arg1>.     |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| cd      | <arg1>            | Change to dir <arg1>.                    |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| rm      | <arg1>            | Remove a file <arg1>.                    |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| rmdir   | <arg1>            | Remove a directory.                      |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| mkdir   | <arg1>            | Create a directory <arg1>.               |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| touch   | <arg1>            | Create file <arg1>.                      |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| head    | <arg1>            | Prints a file <arg1> to the terminal.    |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("'---------'-------------------'------------------------------------------'\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
}/*End arb_displayKernelHelp*/

/*************************************************************************//**
 * @fn static void arb_setDebugLevel( t_consoleTokHndl *pt_tokHndl,
 *                                    int8_t *pc_buff)
 *
 * @brief Puts the system into debug mode where #arb_printf information is
 *        written to the terminal.
 *
 * @details After the thread arb_console() receives a <b>sdl <arg></b> command,
 *          it calls this function in order to configure the system--both
 *          kernel and console interface--for writing debug information to
 *          terminal window. If there are software debug messages (i.e. 
 *          #arb_printf) with priority equal to or higher than <arg> (e.g. 0,1,
 *          2) they will subsequently be displayed at the terminal output; 
 *          otherwise, the terminal will remain blank. After entering this 
 *          mode, the user can press the return key--at any time--in order to 
 *          return to nominal console operation.
 * @see term for further information
 *
 * @param[in] pc_buff Scratch buffer used for writing messages to the terminal.
 *
 * @param[in] pt_tokHndl pointer to the tokenized list of terminal commands and
 *            arguments.
 *
 * @return None.
 *****************************************************************************/
static void arb_setDebugLevel( t_consoleTokHndl *pt_tokHndl,
                               int8_t *pc_buff)
{
   uint8_t c_dbg = PRINTF_DBG_OFF;
   t_consoleError t_conError;
   uint16_t s_size;

   if( pt_tokHndl->ac_tok[1][0] == '0')
      c_dbg = PRINTF_DBG_LOW;
   else if( pt_tokHndl->ac_tok[1][0] == '1')
      c_dbg = PRINTF_DBG_MED;
   else if( pt_tokHndl->ac_tok[1][0] == '2')
      c_dbg = PRINTF_DBG_HIGH;
   else
   {
      s_size = sprintf_P( (char *)pc_buff, PSTR("Invalid Level\n\r"));
      arb_write( gt_conObject.t_consoleHndl,
                 pc_buff,
                 s_size);
   }

   if( c_dbg <= PRINTF_DBG_HIGH)
   {
      s_size = sprintf_P( (char *)pc_buff, PSTR("The 'Enter' button will enable/disable a debugging session. Press 'Enter' to begin...\n\r"));
      arb_write( gt_conObject.t_consoleHndl,
                 pc_buff,
                 s_size);

      /*---------------------------------------------------------------*
       * Block until a carriage return has been entered.
       *---------------------------------------------------------------*/
      t_conError = arb_ioctl( gt_conObject.t_consoleHndl,
                              CONSOLE_PARSE_CMD_LINE,
                              (uint32_t)((uint16_t)pt_tokHndl));

      /*---------------------------------------------------------------*
       * Enable debug...
       *---------------------------------------------------------------*/
      arb_setPrintfDbgLevel( c_dbg);

      /*---------------------------------------------------------------*
       * Block until a carriage return has been entered.
       *---------------------------------------------------------------*/
      t_conError = arb_ioctl( gt_conObject.t_consoleHndl,
                              CONSOLE_PARSE_CMD_LINE,
                              (uint32_t)((uint16_t)pt_tokHndl));

      /*---------------------------------------------------------------*
       * Disable debug...
       *---------------------------------------------------------------*/
      arb_setPrintfDbgLevel( PRINTF_DBG_OFF);
   }/*End if( c_dbg <= PRINTF_DBG_HIGH)*/

}/*End arb_setDebugLevel*/

/*************************************************************************//**
 * @fn static void arb_displayDeviceList( int8_t *pc_buff)
 *
 * @brief 
 *
 * @details 
 *
 * @param[in] pc_buff Scratch buffer used for writing messages to the terminal.
 *
 * @return None.
 *****************************************************************************/
static void arb_displayDeviceList( int8_t *pc_buff)
{
   t_CONTHNDL t_deviceList = arb_getDeviceList();
   t_LINKHNDL t_curr;
   t_device *pt_dev;
   uint16_t s_count;
   uint16_t s_size;

   s_size = sprintf_P( (char *)pc_buff, PSTR(".-------------------------------------------------------------.\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|   Driver Name   | Major Number | Minor Number| # of Handles |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|-----------------|--------------|-------------|--------------|\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);

   UTL_TRAVERSE_CONTAINER_HEAD( t_curr, t_deviceList, s_count)
   {
      pt_dev = (t_device *)UTL_GET_LINK_ELEMENT_PTR( t_curr);
      s_size = sprintf_P( (char *)pc_buff, PSTR("| %16s| %2d           | %2d          | %2d           |\n\r"),
      pt_dev->ac_deviceName,
      ARB_GET_DEV_MAJOR(pt_dev->t_devId),
      ARB_GET_DEV_MINOR(pt_dev->t_devId),
      pt_dev->c_numDevHandles);

      arb_write( gt_conObject.t_consoleHndl,
                 pc_buff,
                 s_size);
   }

   s_size = sprintf_P( (char *)pc_buff, PSTR("'-----------------'--------------'-------------'--------------'\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);

}/*End arb_displayDeviceList*/

static void arb_displaySystemStatistics( int8_t *pc_buff)
{
   uint16_t s_size;
   extern char *__data_start;
   extern char *__data_end;
   extern char *__bss_end;
   extern char *__brkval;
   extern size_t __malloc_margin;
   int32_t i_heapSize;
   int32_t i_dataSize;
   int32_t i_bssSize;
   int32_t i_freeSize;
   int32_t i_ramUsed;
   int32_t i_percUsed;

   i_dataSize = (int32_t)(uint16_t)&__data_end - (int32_t)(uint16_t)&__data_start;
   i_bssSize  = (int32_t)(uint16_t)&__bss_end - (int32_t)(uint16_t)&__data_end;
   i_heapSize = (int32_t)(uint16_t)__brkval - (int32_t)(uint16_t)&__bss_end;
   i_freeSize = (int32_t)RAMEND - (int32_t)(uint16_t)__brkval - (int32_t)__malloc_margin;
   i_ramUsed  = i_dataSize + i_bssSize + i_heapSize;
   i_percUsed = (i_ramUsed*(int32_t)100)/(RAMEND - (int32_t)(uint16_t)&__data_start);

   s_size = sprintf_P( (char *)pc_buff, PSTR(".---------.---------------.---------------.\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("| Section |    Address    |   Size Bytes  |\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|---------|---------------|---------------|\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|   .data |        0x%x | %13d |\n\r"), (uint16_t)&__data_start, i_dataSize);
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|    .bss |        0x%x | %13d |\n\r"), (uint16_t)&__data_end, i_bssSize);
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|   .heap |        0x%x | %13d |\n\r"), (uint16_t)&__bss_end, i_heapSize);
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("'---------'---------------'---------------'\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|Used RAM |               | %13d |\n\r"), i_ramUsed);
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|         |               | %12d%% |\n\r"), i_percUsed);
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("'---------'---------------'---------------'\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|Free RAM |               | %13d |\n\r"), i_freeSize);
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|         |               | %12d%% |\n\r"), (100 - i_percUsed));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("'---------'---------------'---------------'\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|Sys Load after 1 minute  | %12.1f%% |\n\r"), ((float)arb_getOneMinLoadingEst()*100.0f) / ARB_LOAD_EST_ONE);
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("'-------------------------'---------------'\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("|Sys Load after 5 minutes | %12.1f%% |\n\r"), ((float)arb_getFiveMinLoadingEst()*100.0f) / ARB_LOAD_EST_ONE);
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);
   s_size = sprintf_P( (char *)pc_buff, PSTR("'-------------------------'---------------'\n\r"));
   arb_write( gt_conObject.t_consoleHndl,
              pc_buff,
              s_size);

}/*End arb_displaySystemStatistics*/

/*************************************************************************//**
 * @fn static void arb_console( t_parameters t_param,
 *                              t_arguments t_args)
 *
 * @brief Arbitros system console thread which controls reading and writing
          to the standard output (command window).
 *
 * @details This console thread provides a mechanism for exchanging text
 *          messages between an external user--via a command line interface
 *          --and Arbitros kernel or user-space application. The thread blocks
 *          until detecting a carriage return, from which it wakes and reads
 *          the contents of the device driver's (drv_console.c) buffer. The
 *          new message is checked against a set of 'Linux like' Arbitros
 *          kernel commands such as; <b>sct, help, sdl, dev, top, ls, rm, cd,
 *          and head</b>. If the message is recognized as a kernel command it
 *          is subsequently processed; otherwise, control of the console is 
 *          passed onto the user-space application via a function pointer 
 *          passed in as a parameter to #arb_consoleInit during system 
 *          initialization.
 *
 * @param[in] t_param User-definable parameter passed in at time of thread
 *            initialization.
 *
 * @param[in] t_args User-definable argument passed in at time of thread
 *            initialization.
 *
 * @return None.
 *****************************************************************************/
static void arb_console( t_parameters t_param,
                         t_arguments  t_args)
{
   int8_t ac_buff[MAX_CONSOLE_BUFF_SIZE];

   /*------------------------------------------------------------------------*
    * Banner designed using a text to ascii art generator provided by
    * http://www.patorjk.com/software/taag
    *------------------------------------------------------------------------*/
   int8_t ac_banner[] = "\
    __\n\r\
  / _ \\\n\r\
 | ( ) |\n\r\
  \\_\\|_|\n\r\
 arbitros\n\r";

   uint16_t s_size;
   /*------------------------------------------------------------------------*
    * Tokenized list of the last know set of strings entered via the terminal 
    * command window.
    *------------------------------------------------------------------------*/
   t_consoleTokHndl t_tokHndl;
   t_consoleError t_conError;

   t_conError = arb_ioctl( gt_conObject.t_consoleHndl,
                           CONSOLE_RESET_TERMINAL,
                           0);

   t_conError = arb_ioctl( gt_conObject.t_consoleHndl,
                           CONSOLE_SET_BG_COLOR,
                           CONSOLE_BLACK);

   t_conError = arb_ioctl( gt_conObject.t_consoleHndl,
                           CONSOLE_SET_PROMPT_COLOR,
                           CONSOLE_GREEN);

   t_conError = arb_ioctl( gt_conObject.t_consoleHndl,
                           CONSOLE_SET_FG_COLOR,
                           CONSOLE_RED);

   arb_write( gt_conObject.t_consoleHndl,
              ac_banner,
              (uint16_t)sizeof(ac_banner));

   t_conError = arb_ioctl( gt_conObject.t_consoleHndl,
                           CONSOLE_SET_FG_COLOR,
                           CONSOLE_WHITE);

   /*-------------------------------------------------------------------------*
    * Display prompt...
    *-------------------------------------------------------------------------*/
   t_conError = arb_ioctl( gt_conObject.t_consoleHndl,
                           CONSOLE_DISPLAY_PROMPT,
                           0);

   while( RUN_FOREVER)
   {

      /*---------------------------------------------------------------------*
       * Block until a carriage return has been entered.
       *---------------------------------------------------------------------*/
      t_conError = arb_ioctl( gt_conObject.t_consoleHndl,
                              CONSOLE_PARSE_CMD_LINE,
                              (uint32_t)((uint16_t)&t_tokHndl));

      if( t_conError == CONSOLE_TOKEN_TOO_LARGE)
      {
         s_size = sprintf_P( (char *)ac_buff, PSTR("Invalid token size\n\r"));
         arb_write( gt_conObject.t_consoleHndl, 
                    ac_buff, 
                    s_size);
      }/*End if( t_conError == CONSOLE_TOKEN_TOO_LARGE)*/
      else if( t_conError == CONSOLE_TOO_MANY_TOKENS)
      {
         s_size = sprintf_P( (char *)ac_buff, PSTR("Invalid number of tokens\n\r"));
         arb_write( gt_conObject.t_consoleHndl, 
                    ac_buff, 
                    s_size);
      }/*End else if( t_conError == CONSOLE_TOO_MANY_TOKENS)*/
      else if( (strcasecmp_P( (char *)t_tokHndl.ac_tok[0], PSTR("sct")) == 0)
      && (t_tokHndl.c_numTokens == 4)) /*Set current time*/
      {
         uint8_t c_hours;
         uint8_t c_min;
         uint8_t c_sec;

         c_hours = (uint8_t)atoi( (const char *)t_tokHndl.ac_tok[1]);
         c_min = (uint8_t)atoi( (const char *)t_tokHndl.ac_tok[2]);
         c_sec = (uint8_t)atoi( (const char *)t_tokHndl.ac_tok[3]);

         arb_setSysTime( c_hours,
                         c_min,
                         c_sec);
      }/*End else if( strcasecmp( (char *)t_tokHndl.ac_tok[0], "sct") == 0)*/
      else if( strcasecmp_P( (char *)t_tokHndl.ac_tok[0], PSTR("help")) == 0)
      {
         /*------------------------------------------------------------------*
          * Display a list of all the possible RTOS specific commands.
          *------------------------------------------------------------------*/
         arb_displayKernelHelp( ac_buff);
      }/*End else if( strcasecmp( (char *)t_tokHndl.ac_tok[0], "help") == 0)*/
      else if( (strcasecmp_P( (char *)t_tokHndl.ac_tok[0], PSTR("sdl")) == 0)
      && (t_tokHndl.c_numTokens == 2)) /*Set the debug level*/
      {
         /*------------------------------------------------------------------*
          * Turn on/off a given level of system debug.
          *------------------------------------------------------------------*/
         arb_setDebugLevel( &t_tokHndl,
                            ac_buff);
      }
      else if( (strcasecmp_P( (char *)t_tokHndl.ac_tok[0], PSTR("dev")) == 0)
      && (t_tokHndl.c_numTokens == 1))
      {
         /*------------------------------------------------------------------*
          * Display a list of all the device drivers registered with the
          * kernel.
          *------------------------------------------------------------------*/
         arb_displayDeviceList( ac_buff);
      }
      else if( (strcasecmp_P( (char *)t_tokHndl.ac_tok[0], PSTR("top")) == 0)
      && (t_tokHndl.c_numTokens == 1))
      {
         /*------------------------------------------------------------------*
          * Display the memory footprint of the system.
          *------------------------------------------------------------------*/
         arb_displaySystemStatistics( ac_buff);
      }
      else if( (strcasecmp_P( (char *)t_tokHndl.ac_tok[0], PSTR("ls")) == 0)
      && (t_tokHndl.c_numTokens == 1))
      {
         arb_ioctl( gt_conObject.t_sdHndl,
                    SD_LS,
                    0);
      }
      else if( (strcasecmp_P( (char *)t_tokHndl.ac_tok[0], PSTR("rm")) == 0)
      && (t_tokHndl.c_numTokens == 2))
      {
         if( strcasecmp_P( (char *)t_tokHndl.ac_tok[1], PSTR("-r")) == 0)
         {
            /*---------------------------------------------------------------*
             * Remove all the files within, and the current working directory
             * itself.
             *---------------------------------------------------------------*/
            arb_ioctl( gt_conObject.t_sdHndl,
                       SD_RMDASHR,
                       0);
         }/*End if( (strcasecmp_P( (char *)t_tokHndl.ac_tok[1], PSTR("-r")) == 0)*/
         else
         {
            /*---------------------------------------------------------------*
             * Remove the file specified by 't_tokHndl.ac_tok[1]'.
             *---------------------------------------------------------------*/
            arb_ioctl( gt_conObject.t_sdHndl,
                       SD_RM,
                       (int32_t)((uint16_t)&t_tokHndl.ac_tok[1]));
         }

      }
      else if( (strcasecmp_P( (char *)t_tokHndl.ac_tok[0], PSTR("rmdir")) == 0)
      && (t_tokHndl.c_numTokens == 2))
      {
         arb_ioctl( gt_conObject.t_sdHndl,
                    SD_RMDIR,
                    (int32_t)((uint16_t)&t_tokHndl.ac_tok[1]));
      }
      else if( (strcasecmp_P( (char *)t_tokHndl.ac_tok[0], PSTR("mkdir")) == 0)
      && (t_tokHndl.c_numTokens == 2))
      {
         arb_ioctl( gt_conObject.t_sdHndl,
                    SD_MKDIR,
                    (int32_t)((uint16_t)&t_tokHndl.ac_tok[1]));
      }
      else if( (strcasecmp_P( (char *)t_tokHndl.ac_tok[0], PSTR("cd")) == 0)
      && (t_tokHndl.c_numTokens == 2))
      {
         t_sdError t_err;

         t_err = (t_sdError)arb_ioctl( gt_conObject.t_sdHndl,
                                       SD_CD,
                                       (int32_t)((uint16_t)&t_tokHndl.ac_tok[1]));

         /*------------------------------------------------------------------*
          * If directory change accepted, update the prompt.
          *------------------------------------------------------------------*/
         if( t_err == SD_PASSED)
         {
            arb_ioctl( gt_conObject.t_consoleHndl,
                       CONSOLE_SET_PROMPT,
                       (int32_t)((uint16_t)&t_tokHndl.ac_tok[1]));

            if( strcasecmp_P( (char *)t_tokHndl.ac_tok[1], PSTR("/")) == 0)
            {
               arb_ioctl( gt_conObject.t_consoleHndl,
                          CONSOLE_SET_PROMPT_COLOR,
                          CONSOLE_GREEN);
            }
            else
            {
               arb_ioctl( gt_conObject.t_consoleHndl,
                          CONSOLE_SET_PROMPT_COLOR,
                          CONSOLE_RED);
            }
         }/*End if( t_err == SD_PASSED)*/
      }
      else if( (strcasecmp_P( (char *)t_tokHndl.ac_tok[0], PSTR("head")) == 0)
      && (t_tokHndl.c_numTokens == 2))
      {
         arb_head( ac_buff,
                   &t_tokHndl);
      }
      else /*Pass control over to a user-space application...*/
      {
         /*------------------------------------------------------------------*
          * The function pointed to by 'pf_funPtr' is an extension of
          * 'arb_console' that is modifiable from a user-space perspective.
          * If the user-space application requires control over its various
          * threads via a CMD line interface then all subsequent commands
          * should be placed within the function pointed to by 'pf_funPtr'.
          *------------------------------------------------------------------*/
         if( gt_conObject.pf_funPtr != NULL)
         {
            if( gt_conObject.pf_funPtr( gt_conObject.t_consoleHndl,
                                        ac_buff,
                                        &t_tokHndl) == false)
            {
               s_size = sprintf_P( (char *)ac_buff, PSTR("Invalid CMD\n\r"));
               arb_write( gt_conObject.t_consoleHndl,
                          ac_buff,
                          s_size);
            }
         }/*End if( gt_conObject.pf_funPtr != NULL)*/
      }

      /*---------------------------------------------------------------------*
       * Display prompt...
       *---------------------------------------------------------------------*/
      t_conError = arb_ioctl( gt_conObject.t_consoleHndl,
                              CONSOLE_DISPLAY_PROMPT,
                              0);

   }/*End while( RUN_FOREVER)*/

}/*End arb_console*/

/*************************************************************************//**
 * @fn arb_consoleInit
 *
 *****************************************************************************/
t_error arb_consoleInit( char *pc_consDriver,
                         char *pc_sdDriver,
                         t_stackSize t_stack,
                         t_thrdPrio t_pri,
                         bool (*pf_funPtr)( t_DEVHANDLE t_consoleHndl,
                                            int8_t *pc_buff,
                                            t_consoleTokHndl *pt_tokHndl))
{

   /*------------------------------------------------------------------------*
    * Create a new thread.
    *------------------------------------------------------------------------*/
   gt_conObject.t_consoleThread = arb_threadCreate( arb_console,
                                                    1,
                                                    0,
                                                    t_stack,
                                                    t_pri);

   if( gt_conObject.t_consoleThread < 0)
   {
      return (t_error)gt_conObject.t_consoleThread;

   }/*End if( gt_conObject.t_consoleThread < 0)*/

   /*------------------------------------------------------------------------*
    * Open a handle to the console driver.
    *------------------------------------------------------------------------*/
   gt_conObject.t_consoleHndl = arb_open( pc_consDriver,
                                          ARB_O_READ |
                                          ARB_O_WRITE);

   if( gt_conObject.t_consoleHndl < 0)
   {
      return (t_error)gt_conObject.t_consoleHndl;
   }

   gt_conObject.pf_funPtr = pf_funPtr;

   /*------------------------------------------------------------------------*
    * Open a handle to the sd card driver.
    *------------------------------------------------------------------------*/
   gt_conObject.t_sdHndl = arb_open( pc_sdDriver,
                                     ARB_O_READ |
                                     ARB_O_WRITE);

   /*------------------------------------------------------------------------*
    * If there is no sd card present on the system then set the handle to
    * 0.
    *------------------------------------------------------------------------*/
   if( gt_conObject.t_sdHndl == ARB_DEVICE_NOT_FOUND)
   {
      gt_conObject.t_sdHndl = 0;
   }/*End if( gt_conObject.t_sdHndl == ARB_DEVICE_NOT_FOUND)*/
   else if( gt_conObject.t_sdHndl < 0)
   {
      return (t_error)gt_conObject.t_sdHndl;
   }/*End else if( gt_conObject.t_sdHndl < 0)*/

   return ARB_PASSED;

}/*End arb_consoleInit*/
