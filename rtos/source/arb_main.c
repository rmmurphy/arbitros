/*************************************************************************//**
 * @defgroup RTOS
 *
 * @brief Collection of files comprising the Arbitros real-time operating
 *        system (RTOS).
 *
 * @details This module contains the collection of files--arb_console.c,
 *          arb_device.c, arb_error.c, arb_idle.c, arb_mailbox.c, arb_main.c,
 *          arb_memory.c, arb_printf.c, arb_scheduler.c, arb_semaphore.c,
 *          arb_sysTimer.c, and arb_thread.c--comprising the heart of the
 *          the Arbitros RTOS. The first file arb_console.c, provides the
 *          functionality for interfacing the kernel and user-space
 *          application with a terminal window via a user specified console
 *          driver. The second file arb_device.c, defines a standard interface
 *          for linking a user-space application to an external device through
 *          the kernel. For those familiar with Linux, Arbitros incorporates
 *          standard function calls such as #arb_open, #arb_close, #arb_read,
 *          #arb_write, and #arb_ioctl in order to abstract--from the user--the
 *          intricate details that define how the driver communicates with an
 *          off-chip device. As seen in the figure, this framework allows a
 *          user-space application to interact with a particular entity--for
 *          instance a WIFI module--without having to know the specific
 *          details about how the device communicates (such as the particular
 *          UART and GPIO ports used). Furthermore, this driver "blue print" 
 *          defines the way in which device driver software should be written
 *          in order to promote abstraction and commonality (at the user-space
 *          application layer) across a wide variety of potential device driver 
 *          software developers. This construct will make it easier for anyone
 *          to develop new hardware and corresponding driver software 
 *          applications (like the WIFI example in the figure) that immediately
 *          inter operate (i.e. no software changes required) with existing 
 *          user-space software applications.
 * @image html arbDeviceTopLevel.jpg Example: WIFI Device Driver Mapping
 *        The third file arb_error.c, 
 *
 * @file arb_main.c
 *
 * @ingroup RTOS
 *
 * @brief Main code entry point.
 *
 * @details This file calls four main tasks after system power-on for the
 *          purpose of platform initialization. The first task #hal_setCpuFreq,
 *          configures the default CPU frequency. The second function
 *          #hal_configureIntLevel, sets the nominal interrupt level--nested
 *          interrupts are not allowed. The third function #usr_appInit, hands
 *          over control to the user-space layer for the purpose of device
 *          driver registration, scheduler initialization, and user-space
 *          thread configuration. Implementation of this function is required of
 *          all user-space applications; however, the particular operations
 *          performed within are application dependent. Meaning, the choice of
 *          a particular device driver to include or a particular kernel
 *          setting (like logging #arb_printf data to an SD card) is completely
 *          up to the user-space application developer. After completion of the
 *          call to #usr_appInit, control is passed back to #main where the 
 *          first thread gets launched after completing the call to
 *          #arb_schedulerStart.
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
 * @date September 26, 2012
 *
 * @version 1.0
 *****************************************************************************/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include "arb_scheduler.h"
#include "arb_printf.h"
#include "hal_clocks.h"
#include "hal_pmic.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 *  Global Function Prototypes
 *---------------------------------------------------------------------------*/
extern void usr_appInit( void);

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/

/*************************************************************************//**
 * @fn int main( void)
 *
 * @brief Performs power-on system initialization.
 *
 * @return Function never returns.
 *****************************************************************************/
int main( void)
{

   /*------------------------------------------------------------------------*
    * Configure CPU clock for 32Mhz operation
    *------------------------------------------------------------------------*/
   hal_setCpuFreq( 32000000);

   /*------------------------------------------------------------------------*
    * Configure the highest level of interrupts - nesting is not allowed.
    *------------------------------------------------------------------------*/
   hal_configureIntLevel( INT_LEVEL_2);

   /*------------------------------------------------------------------------*
    * Pass control over to the user-space layer which will register device
    * drivers with the kernel, initialize the scheduler, and configure any
    * user-space threads. The operations herein are completely platform
    * dependent leaving the choice of device drivers and arbitros settings
    * completely up to the user-space layer.
    *------------------------------------------------------------------------*/
   usr_appInit();

   /*------------------------------------------------------------------------*
    * Return control over to the kernel layer which will launch the highest
    * priority thread- up to this point interrupts have been disabled.
    *------------------------------------------------------------------------*/
   arb_schedulerStart();

   return 0;

}/*End main*/

