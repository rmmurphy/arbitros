/*---------------------------------------------------------------------------*
 * Copyright (C) 2014 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 * File Name   : usr_mathTest.c
 *
 * Project     : Watts-rock
 *               <http://code.google.com/p/watts-rock-motherboard/>
 *
 * Description : This file is responsible for testing functionality from
 *               utl_math.c.
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
#include "utl_math.h"
#include "usr_mathTest.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define MATH_SLEEP_TICKS (10)

/*---------------------------------------------------------------------------*
 * Private Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
   /*------------------------------------------------------------------------*
    * The main math test interface thread
    *------------------------------------------------------------------------*/
   t_THRDHANDLE t_mathThread;

   /*------------------------------------------------------------------------*
    * Handle to the LED driver.
    *------------------------------------------------------------------------*/
   t_DEVHANDLE t_signalHndl;

}t_mathObjct;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
static t_mathObjct gt_mathObjct;

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void usr_mathTest( t_parameters t_param,
                          t_arguments  t_args)
{
   uint32_t i_phase = 0;
   int16_t s_cos;
   int16_t s_sin;
   int16_t s_angle;
   char ac_buff[120];
   uint16_t s_size;
   float f_cos;
   float f_sin;
   float f_error;
   static t_sysTime t_time1;
   static t_sysTime t_time2;
   static int16_t s_callLoad;
   static int32_t i_loading1;
   static int32_t i_loading2;
   static float f_phase;

   /*------------------------------------------------------------------------*
    * Determine how long it takes to call the system time function.
    *------------------------------------------------------------------------*/
   t_time1 = arb_sysTimeNow();
   t_time2 = arb_sysTimeNow();
   s_callLoad = t_time2.i_usec - t_time1.i_usec;

   while( RUN_FOREVER)
   {
      i_phase = i_phase + 64;
      if( i_phase > UTL_MATH_FXDPNT_TWO_PI_WRAP)
         i_phase = i_phase - UTL_MATH_FXDPNT_TWO_PI_WRAP;

      /*---------------------------------------------------------------------*
       * Determine the loading when calling the utl_math.c cosine routine.
       *---------------------------------------------------------------------*/
      t_time1 = arb_sysTimeNow();
      s_cos = utl_cos16_16( i_phase);
      t_time2 = arb_sysTimeNow();
      i_loading1 = t_time2.i_usec-t_time1.i_usec;
      i_loading1 = i_loading1 - s_callLoad;
      if( i_loading1 < 0)
         i_loading1 = i_loading1 + 1000000;

      f_phase = (2.0f*3.14159f*(float)i_phase)/UTL_MATH_FXDPNT_TWO_PI_WRAP;
      t_time1 = arb_sysTimeNow( );
      f_cos = cosf( f_phase);
      t_time2 = arb_sysTimeNow();
      
      f_sin = sinf( f_phase);
      i_loading2 = t_time2.i_usec-t_time1.i_usec;
      if( i_loading2 < 0)
         i_loading2 = i_loading2 + 1000000;
      i_loading2 = i_loading2 - s_callLoad;

      f_error = fabs((f_cos - ((float)s_cos/32768.0f))*100.0f/f_cos);

      s_size = sprintf( ( char *)ac_buff, "fixed cos=%d ansii cos=%d err=%.2f%% speed=%dx\r",
                        s_cos,
                        (int16_t)(f_cos*32767.0f),
                        (double)f_error,
                        (int)i_loading2/i_loading1);
      arb_printf( PRINTF_DBG_MED |PRINTF_DBG_SHOW_TIME,
                  (const char *)ac_buff);

      s_sin = (int16_t)(f_sin*32767.0f);
      s_cos = (int16_t)(f_cos*32767.0f);
      t_time1 = arb_sysTimeNow();
      s_angle = utl_atan2_16(s_sin, s_cos);
      t_time2 = arb_sysTimeNow();
      i_loading1 = t_time2.i_usec-t_time1.i_usec;
      i_loading1 = i_loading1 - s_callLoad;
      if( i_loading1 < 0)
         i_loading1 = i_loading1 + 1000000;

      if( f_phase > 3.14158f)
         f_phase = f_phase - 2.0f*3.14159f;

      f_error = fabs((f_phase - ((float)s_angle*3.14159f/32768.0f))*100.0f/f_phase);

      t_time1 = arb_sysTimeNow( );
      f_phase = atan2f( f_sin, f_cos);
      t_time2 = arb_sysTimeNow( );
      i_loading2 = t_time2.i_usec-t_time1.i_usec;
      if( i_loading2 < 0)
         i_loading2 = i_loading2 + 1000000;
      i_loading2 = i_loading2 - s_callLoad;

      s_size = sprintf( ( char *)ac_buff, "fixed atan2=%d ansii atan2=%d err=%.2f%% speed=%dx\r",
                        s_cos,
                        (int16_t)(f_cos*32767.0f),
                        (double)f_error,
                        (int)i_loading2/i_loading1);
      arb_printf( PRINTF_DBG_MED |PRINTF_DBG_SHOW_TIME, 
                  (const char *)ac_buff);

      /*---------------------------------------------------------------------*
       * Allow other threads to run...
       *---------------------------------------------------------------------*/
      arb_sleep( MATH_SLEEP_TICKS);

   }/*End while( RUN_FOREVER)*/

}/*End usr_mathTest*/

t_error usr_mathTestInit( void)
{

   /*------------------------------------------------------------------------*
    * Initialize the BMS object...
    *------------------------------------------------------------------------*/
   memset( (void *)&gt_mathObjct, 0, sizeof( t_mathObjct));

   /*------------------------------------------------------------------------*
    * Create a new thread.
    *------------------------------------------------------------------------*/
   gt_mathObjct.t_mathThread = arb_threadCreate( usr_mathTest,
                                                 1,
                                                 0,
                                                 ARB_STACK_512B,
                                                 0);

   if( gt_mathObjct.t_mathThread < 0)
   {
      return gt_mathObjct.t_mathThread;

   }/*End if( gt_mathObjct.t_mathThread < 0)*/

   /*------------------------------------------------------------------------*
    * Open a handle to the signal driver.
    *------------------------------------------------------------------------*/
   gt_mathObjct.t_signalHndl = arb_open( "signalDevice0",
                                         ARB_O_READ |
                                         ARB_O_WRITE);

   if( gt_mathObjct.t_signalHndl < 0)
   {
      return gt_mathObjct.t_signalHndl;
   }

   return ARB_PASSED;

}/*End usr_mathTestInit*/
