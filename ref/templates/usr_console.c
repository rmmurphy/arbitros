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

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Typedefs
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
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
   /*########################################################################
     # Describe your commands here like the following example...
     #
     # s_size = sprintf_P( (char *)pc_buff, PSTR("| pwr     |                   | Returns the measured platform voltage.   |\n\r"));
     # arb_write( t_consoleHndl,
     #            pc_buff,
     #            s_size);
     ########################################################################*/
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
