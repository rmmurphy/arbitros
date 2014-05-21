/*---------------------------------------------------------------------------*
 * Copyright (C) 2014 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 * File Name   : usr_application.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file contains all threads defined by the user-space
 *               application layer.
 *
 * Last Update : Feb, 27, 2014
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "arb_error.h"
#include "arb_thread.h"
#include "arb_semaphore.h"
#include "arb_device.h"
#include "arb_sysTimer.h"
#include "arb_printf.h"
#include "drv_signal.h"
#include "usr_application.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define APP_SLEEP_TICKS (10)

/*---------------------------------------------------------------------------*
 * Private Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
   /*------------------------------------------------------------------------*
    * Generic application thread definition
    *------------------------------------------------------------------------*/
   t_THRDHANDLE t_appThread;

   /*------------------------------------------------------------------------*
    * Handle to the LED driver.
    *------------------------------------------------------------------------*/
   t_DEVHANDLE t_signalHndl;

}t_appObjct;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
static t_appObjct gt_appObjct;

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void usr_application( t_parameters t_param,
                             t_arguments  t_args)
{

   while( RUN_FOREVER)
   {
      /*#####################################################################*
       * Place user defined operations here...
       *#####################################################################/
       
      /*---------------------------------------------------------------------*
       * Allow other threads to run...
       *---------------------------------------------------------------------*/
      arb_sleep( APP_SLEEP_TICKS);

   }/*End while( RUN_FOREVER)*/

}/*End usr_application*/

t_error usr_applicationInit( void)
{

   /*------------------------------------------------------------------------*
    * Initialize the application object...
    *------------------------------------------------------------------------*/
   memset( (void *)&gt_appObjct, 0, sizeof( t_appObjct));

   /*------------------------------------------------------------------------*
    * Create a new thread.
    *------------------------------------------------------------------------*/
   gt_appObjct.t_appThread = arb_threadCreate( usr_application,
                                               1,
                                               0,
                                               ARB_STACK_512B,
                                               0);

   if( gt_appObjct.t_appThread < 0)
   {
      return gt_appObjct.t_appThread;

   }/*End if( gt_appObjct.t_appThread < 0)*/

   /*------------------------------------------------------------------------*
    * Open a handle to the signal driver.
    *------------------------------------------------------------------------*/
   gt_appObjct.t_signalHndl = arb_open( "signalDevice0",
                                        ARB_O_READ |
                                        ARB_O_WRITE);

   if( gt_appObjct.t_signalHndl < 0)
   {
      return gt_appObjct.t_signalHndl;
   }

   return ARB_PASSED;

}/*End usr_applicationInit*/
