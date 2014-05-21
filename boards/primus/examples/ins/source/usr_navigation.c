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
 * File Name   : usr_navigation.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This thread is responsible for abstracting command and
 *               control of the AHRS for an external application.
 *
 * Last Update : Oct 7, 2012
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "arb_error.h"
#include "usr_navigation.h"
#include "arb_thread.h"
#include "arb_semaphore.h"
#include "arb_device.h"
#include "arb_printf.h"
#include "arb_sysTimer.h"
#include "drv_signal.h"
#include "utl_stateMachine.h"
#include "usr_console.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*
 * The estimate processor loading for the update state of drv_ins.c. The
 * loading estimate was performed with -oS optimization.
 *---------------------------------------------------------------------------*/
#define NAV_INS_PROC_LOADING (.014f) /*seconds*/
#define NAV_DESIRED_UPDATE_RATE_DT (.05f) /*seconds*/

/*---------------------------------------------------------------------------*
 * The time the thread needs to sleep in order to acheive the desired update
 * rate 'NAV_DESIRED_UPDATE_RATE_DT' given the estimate processor loading
 * 'NAV_INS_PROC_LOADING'.
 *---------------------------------------------------------------------------*/
#define NAV_UPDATE_RATE_DT (uint8_t)((float)ARB_TICKS_PER_SECOND*(float)\
(NAV_DESIRED_UPDATE_RATE_DT - NAV_INS_PROC_LOADING))

/*---------------------------------------------------------------------------*
 * Private Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
   /*------------------------------------------------------------------------*
    * Handle to the state-machine object that defines how the navigation
    * thread transitions between states.
    *------------------------------------------------------------------------*/
   t_STMNHNDL t_stateMn;

   /*------------------------------------------------------------------------*
    * During a state transition this variable keeps track of where it last
    * was.
    *------------------------------------------------------------------------*/
   t_navState t_prevState;

   /*------------------------------------------------------------------------*
    * The current navigation state.
    *------------------------------------------------------------------------*/
   t_navState t_curState;

   /*------------------------------------------------------------------------*
    * The status of the calibration in progress.
    *------------------------------------------------------------------------*/
   t_insCalStatus t_calStatus;

   /*------------------------------------------------------------------------*
    * Handle to the INS driver.
    *------------------------------------------------------------------------*/
   t_DEVHANDLE t_insHndl;

   /*------------------------------------------------------------------------*
    * Handle to the Signal driver.
    *------------------------------------------------------------------------*/
   t_DEVHANDLE t_signalHndl;

   /*------------------------------------------------------------------------*
    * The navigation thread handle.
    *------------------------------------------------------------------------*/
   t_THRDHANDLE t_navThread;

   /*------------------------------------------------------------------------*
    * Resources that can be shared amongst multiple users (either a global
    * buffer or IO device) need to be protected against race conditions. We
    * use this semaphore for just that purpose.
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_mutex;

   /*------------------------------------------------------------------------*
    * Semaphore for synchronizing events with external threads.
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_signal;

   /*------------------------------------------------------------------------*
    * The last sample acquired during an active magnetometer or
    * accelerometer calibration cycle.
    *------------------------------------------------------------------------*/
   int16_t as_cal[3];

}t_navStruct;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static void usr_navigation( t_parameters t_param,
                            t_arguments  t_args);

static void usr_navStateMachineInit( void);

static void usr_navIdle( int32_t i_stateData,
                         uint8_t c_prevState);

static void usr_navMagCal( int32_t i_stateData,
                           uint8_t c_prevState);

static void usr_navAccelCal( int32_t i_stateData,
                             uint8_t c_prevState);

static void usr_navDcmInit( int32_t i_stateData,
                            uint8_t c_prevState);

static void usr_navActive( int32_t i_stateData,
                           uint8_t c_prevState);

static void usr_navError( int32_t i_stateData,
                          uint8_t c_prevState);

static void usr_navCalSampComp( int32_t i_stateData,
                                uint8_t c_prevState);

static void usr_navCalComplete( int32_t i_stateData,
                                uint8_t c_prevState);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
static t_navStruct gt_navStruct;

/*---------------------------------------------------------------------------*
 * Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void usr_navIdle( int32_t i_stateData,
                         uint8_t c_prevState)
{
   gt_navStruct.t_curState = NAV_IDLE;

   if( c_prevState != NAV_IDLE)
   {
      gt_navStruct.t_curState = NAV_IDLE;

      arb_ioctl( gt_navStruct.t_signalHndl,
                 YELLOW_LED_TOGGLE,
                 0);

      arb_ioctl( gt_navStruct.t_insHndl,
                 INS_RESET,
                 0);

      /*---------------------------------------------------------------------*
       * Keep track of where we came from...
       *---------------------------------------------------------------------*/
      gt_navStruct.t_prevState = (t_navState)c_prevState;

   }/*End if( c_prevState != NAV_IDLE)*/

}/*End usr_navIdle*/

static void usr_navCalSampComp( int32_t i_stateData,
                                uint8_t c_prevState)
{

   if( c_prevState != NAV_CAL_SAMP_COMP)
   {
      gt_navStruct.t_curState = NAV_CAL_SAMP_COMP;

      /*---------------------------------------------------------------------*
       * Keep track of where we came from...
       *---------------------------------------------------------------------*/
      gt_navStruct.t_prevState = (t_navState)c_prevState;

      /*---------------------------------------------------------------------*
       * Signal the console letting it know a sample has been captured.
       *---------------------------------------------------------------------*/
      arb_signal(gt_navStruct.t_signal);

   }/*End if( c_prevState != NAV_ACCEL_CAL)*/

   /*------------------------------------------------------------------------*
    * Wait to be told to acquire a new sample...
    *------------------------------------------------------------------------*/

}/*End usr_navCalSampComp*/

static void usr_navCalComplete( int32_t i_stateData,
                                uint8_t c_prevState)
{

   if( c_prevState != NAV_CAL_COMPLETE)
   {
      gt_navStruct.t_curState = NAV_CAL_COMPLETE;

      /*---------------------------------------------------------------------*
       * Keep track of where we came from...
       *---------------------------------------------------------------------*/
      gt_navStruct.t_prevState = (t_navState)c_prevState;

      /*---------------------------------------------------------------------*
       * Signal the console letting it know a sample has been captured.
       *---------------------------------------------------------------------*/
      arb_signal(gt_navStruct.t_signal);

   }/*End if( c_prevState != NAV_ACCEL_CAL)*/

}/*End usr_navCalComplete*/

static void usr_navMagCal( int32_t i_stateData,
                           uint8_t c_prevState)
{
   t_stmnError t_err;

   if( c_prevState != NAV_MAG_CAL)
   {
      gt_navStruct.t_curState = NAV_MAG_CAL;

      if( c_prevState != NAV_CAL_SAMP_COMP)
         gt_navStruct.t_calStatus = (t_insCalStatus)arb_ioctl( gt_navStruct.
                                                               t_insHndl,
                                                               INS_RESET_CAL,
                                                               0);

      /*---------------------------------------------------------------------*
       * Keep track of where we came from...
       *---------------------------------------------------------------------*/
      gt_navStruct.t_prevState = (t_navState)c_prevState;

   }/*End if( c_prevState != NAV_MAG_CAL)*/

   /*------------------------------------------------------------------------*
    * Tell the INS driver to calibrate the magnetometer. Each time ioctl gets
    * called, a new sample is acquired. When INS_MAX_CAL_SAMPLES have been
    * stored the magnetometer bias, principle axis, and axis scale factors are
    * estimated using an ellipsoid fit algorithm.
    *------------------------------------------------------------------------*/
   gt_navStruct.t_calStatus =
   (t_insCalStatus)arb_ioctl( gt_navStruct.
                              t_insHndl,
                              INS_CALIBRATE_MAG,
                              (int32_t)((int16_t)&gt_navStruct.
                              as_cal[0]));

   /*------------------------------------------------------------------------*
    * Put the state machine back into IDLE and wait for an external
    * application to tell us to acquire the next sample.
    *------------------------------------------------------------------------*/
   if( gt_navStruct.t_calStatus == INS_CAL_IN_PROGRESS)
   {
      t_err = utl_stMnChangeState( gt_navStruct.t_stateMn,
                                    (uint8_t)NAV_CAL_SAMP_COMP,
                                    0);
   }/*End if( t_calStatus == INS_CAL_COMPLETE)*/
   else
   {
      utl_stMnChangeState( gt_navStruct.t_stateMn,
                           (uint8_t)NAV_CAL_COMPLETE,
                           0);
   }

}/*End usr_navMagCal*/

static void usr_navAccelCal( int32_t i_stateData,
                             uint8_t c_prevState)
{

   if( c_prevState != NAV_ACCEL_CAL)
   {
      gt_navStruct.t_curState = NAV_ACCEL_CAL;

      if( c_prevState != NAV_CAL_SAMP_COMP)
         gt_navStruct.t_calStatus = (t_insCalStatus)arb_ioctl( gt_navStruct.
                                                               t_insHndl,
                                                               INS_RESET_CAL,
                                                               0);

      /*---------------------------------------------------------------------*
       * Keep track of where we came from...
       *---------------------------------------------------------------------*/
      gt_navStruct.t_prevState = (t_navState)c_prevState;

   }/*End if( c_prevState != NAV_ACCEL_CAL)*/

   /*------------------------------------------------------------------------*
    * Tell the INS driver to calibrate the accelerometer Each time ioctl gets
    * called, a new sample is acquired. When INS_MAX_CAL_SAMPLES have been
    * stored the accelerometer bias, principle axis, and axis scale factors
    * are estimated using an ellipsoid fit algorithm.
    *------------------------------------------------------------------------*/
   gt_navStruct.t_calStatus =
   (t_insCalStatus)arb_ioctl( gt_navStruct.
                              t_insHndl,
                              INS_CALIBRATE_ACCEL,
                              (int32_t)((int16_t)&gt_navStruct.
                              as_cal[0]));

   /*------------------------------------------------------------------------*
    * Put the state machine back into IDLE and wait for an external
    * application to tell us to acquire the next sample.
    *------------------------------------------------------------------------*/
   if( gt_navStruct.t_calStatus == INS_CAL_IN_PROGRESS)
   {
      utl_stMnChangeState( gt_navStruct.t_stateMn,
                           (uint8_t)NAV_CAL_SAMP_COMP,
                           0);
   }/*End if( t_calStatus == INS_CAL_COMPLETE)*/
   else
   {
      utl_stMnChangeState( gt_navStruct.t_stateMn,
                           (uint8_t)NAV_CAL_COMPLETE,
                           0);
   }

}/*End usr_navAccelCal*/

static void usr_navDcmInit( int32_t i_stateData,
                            uint8_t c_prevState)
{
   if( c_prevState != NAV_DCM_INIT)
   {
      gt_navStruct.t_curState = NAV_DCM_INIT;

      /*---------------------------------------------------------------------*
       * Keep track of where we came from...
       *---------------------------------------------------------------------*/
      gt_navStruct.t_prevState = (t_navState)c_prevState;

   }/*End if( c_prevState != NAV_DCM_INIT)*/

   if( arb_ioctl( gt_navStruct.t_insHndl,
                  INS_DCM_INIT,
                  0) == INS_CAL_COMPLETE)
   {
      utl_stMnChangeState( gt_navStruct.t_stateMn,
                           (uint8_t)NAV_ACTIVE,
                           0);
   }

}/*End usr_navDcmInit*/

static void usr_navActive( int32_t i_stateData,
                           uint8_t c_prevState)
{
   volatile t_sysTime time1,time2;

   if( c_prevState != NAV_ACTIVE)
   {
      gt_navStruct.t_curState = NAV_ACTIVE;

      /*---------------------------------------------------------------------*
       * Keep track of where we came from...
       *---------------------------------------------------------------------*/
      gt_navStruct.t_prevState = (t_navState)c_prevState;

   }/*End if( c_prevState != NAV_ACTIVE)*/

   time1 = arb_sysTimeNow();
   arb_ioctl( gt_navStruct.t_insHndl,
               INS_UPDATE,
               0);
   time2 = arb_sysTimeNow();

}/*End usr_navActive*/

static void usr_navError( int32_t i_stateData,
                          uint8_t c_prevState)
{
   if( c_prevState != NAV_ERROR)
   {
      gt_navStruct.t_curState = NAV_ERROR;

      /*---------------------------------------------------------------------*
       * Keep track of where we came from...
       *---------------------------------------------------------------------*/
      gt_navStruct.t_prevState = (t_navState)c_prevState;

   }/*End if( c_prevState != NAV_ERROR)*/

}/*End usr_navError*/

static void usr_navigation( t_parameters t_param,
                            t_arguments  t_args)
{
   t_stmnError t_stErr;

   /*------------------------------------------------------------------------*
    * Bring the INS sensors out of reset...
    *------------------------------------------------------------------------*/
   arb_ioctl( gt_navStruct.t_insHndl,
              INS_SENS_STARTUP,
              0);

   while( RUN_FOREVER)
   {
      /*---------------------------------------------------------------------*
       * Global data is being accessed, apply lock...
       *---------------------------------------------------------------------*/
      arb_wait( gt_navStruct.t_mutex, 0);

      /*---------------------------------------------------------------------*
       * Transition to the next state...
       *---------------------------------------------------------------------*/
      t_stErr = utl_stMnEngine( gt_navStruct.t_stateMn);

      /*---------------------------------------------------------------------*
       * unlock...
       *---------------------------------------------------------------------*/
      arb_signal( gt_navStruct.t_mutex);

      /*---------------------------------------------------------------------*
       * Allow other threads to run...
       *---------------------------------------------------------------------*/
      arb_sleep( NAV_UPDATE_RATE_DT);

   }/*End while( RUN_FOREVER)*/

}/*End usr_navigation*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
bool usr_navSetState( t_navState t_nxtState)
{
   bool b_success = true;
    t_stmnError t_err;

   /*------------------------------------------------------------------------*
    * Global data is being accessed, apply lock...
    *------------------------------------------------------------------------*/
   arb_wait( gt_navStruct.t_mutex, 0);

   t_err = utl_stMnChangeState( gt_navStruct.t_stateMn,
                                (uint8_t)t_nxtState,
                                0);
   if( t_err != STMN_PASSED)
   {
      b_success = false;
   }

   /*------------------------------------------------------------------------*
    * unlock...
    *------------------------------------------------------------------------*/
   arb_signal( gt_navStruct.t_mutex);

   return b_success;

}/*End usr_navSetState*/

t_navState usr_navGetState( void)
{
   return gt_navStruct.t_curState;
}/*End usr_navGetState*/

void usr_navGetCalSample( int16_t *ps_calSample)
{
   ps_calSample[0] = gt_navStruct.as_cal[0];
   ps_calSample[1] = gt_navStruct.as_cal[1];
   ps_calSample[2] = gt_navStruct.as_cal[2];
}/*End usr_navGetCalSample*/

void usr_navWaitCalSample( void)
{
   arb_wait(gt_navStruct.t_signal, 0);
}/*End usr_navWaitCalSample*/

t_insCalStatus usr_navGetCalStatus( void)
{
   return gt_navStruct.t_calStatus;
}/*End usr_navGetCalStatus*/

void usr_navGetMagCal( float *pf_R,
                       float *pf_scale,
					        float *pf_bias)
{
	int32_t i_result;
	t_ellipsoidCal t_magCal;

   i_result = arb_ioctl( gt_navStruct.t_insHndl,
                         INS_GET_MAG_CALIBRATION,
                         (int32_t)(int16_t)&t_magCal);

   if( t_magCal.t_status == INS_CAL_COMPLETE)
   {
   	pf_bias[0] = (float)t_magCal.ps_bias[0] / t_magCal.s_env;
   	pf_bias[1] = (float)t_magCal.ps_bias[1] / t_magCal.s_env;
   	pf_bias[2] = (float)t_magCal.ps_bias[2] / t_magCal.s_env;

   	pf_scale[0] = (float)t_magCal.ps_scale[0] / ((int16_t)1<<t_magCal.c_n);
   	pf_scale[1] = (float)t_magCal.ps_scale[1] / ((int16_t)1<<t_magCal.c_n);
   	pf_scale[2] = (float)t_magCal.ps_scale[2] / ((int16_t)1<<t_magCal.c_n);

   	pf_R[0] = (float)t_magCal.ps_R[0] / 32767.0;
   	pf_R[1] = (float)t_magCal.ps_R[1] / 32767.0;
   	pf_R[2] = (float)t_magCal.ps_R[2] / 32767.0;
   	pf_R[3] = (float)t_magCal.ps_R[3] / 32767.0;
   	pf_R[4] = (float)t_magCal.ps_R[4] / 32767.0;
   	pf_R[5] = (float)t_magCal.ps_R[5] / 32767.0;
   	pf_R[6] = (float)t_magCal.ps_R[6] / 32767.0;
   	pf_R[7] = (float)t_magCal.ps_R[7] / 32767.0;
   	pf_R[8] = (float)t_magCal.ps_R[8] / 32767.0;
	}
   else
   {
      memset( (void *)pf_bias, 0, 3*sizeof( float));
      memset( (void *)pf_scale, 0, 3*sizeof( float));
      memset( (void *)pf_R, 0, 9*sizeof( float));
   }

}/*End usr_navGetMagCal*/

void usr_navGetAccelCal( float *pf_R,
                         float *pf_scale,
					          float *pf_bias)
{
	int32_t i_result;
	t_ellipsoidCal t_accelCal;

   i_result = arb_ioctl( gt_navStruct.t_insHndl,
                         INS_GET_ACCEL_CALIBRATION,
                         (int32_t)(int16_t)&t_accelCal);

   if( t_accelCal.t_status == INS_CAL_COMPLETE)
   {

   	pf_bias[0] = (float)t_accelCal.ps_bias[0] / t_accelCal.s_env;
   	pf_bias[1] = (float)t_accelCal.ps_bias[1] / t_accelCal.s_env;
   	pf_bias[2] = (float)t_accelCal.ps_bias[2] / t_accelCal.s_env;

   	pf_scale[0] = (float)t_accelCal.ps_scale[0] / ((int16_t)1<<t_accelCal.c_n);
   	pf_scale[1] = (float)t_accelCal.ps_scale[1] / ((int16_t)1<<t_accelCal.c_n);
   	pf_scale[2] = (float)t_accelCal.ps_scale[2] / ((int16_t)1<<t_accelCal.c_n);

   	pf_R[0] = (float)t_accelCal.ps_R[0] / 32767.0;
   	pf_R[1] = (float)t_accelCal.ps_R[1] / 32767.0;
   	pf_R[2] = (float)t_accelCal.ps_R[2] / 32767.0;
   	pf_R[3] = (float)t_accelCal.ps_R[3] / 32767.0;
   	pf_R[4] = (float)t_accelCal.ps_R[4] / 32767.0;
   	pf_R[5] = (float)t_accelCal.ps_R[5] / 32767.0;
   	pf_R[6] = (float)t_accelCal.ps_R[6] / 32767.0;
   	pf_R[7] = (float)t_accelCal.ps_R[7] / 32767.0;
   	pf_R[8] = (float)t_accelCal.ps_R[8] / 32767.0;
	}
   else
   {
      memset( (void *)pf_bias, 0, 3*sizeof( float));
      memset( (void *)pf_scale, 0, 3*sizeof( float));
      memset( (void *)pf_R, 0, 9*sizeof( float));
   }

}/*End usr_navGetAccelCal*/

void usr_navGetStateEst( t_currentMeas *pt_currentMeas)
{
   int32_t i_result;

   /*------------------------------------------------------------------------*
    * Global data is being accessed, apply lock...
    *------------------------------------------------------------------------*/
   arb_wait( gt_navStruct.t_mutex, 0);

   i_result = arb_ioctl( gt_navStruct.t_insHndl,
                         INS_GET_STATE_EST,
                         (int32_t)(int16_t)pt_currentMeas);

   arb_signal( gt_navStruct.t_mutex);

}/*End usr_navGetStateEst*/

static void usr_navStateMachineInit( void)
{
   uint8_t ac_transMap[NAV_NUM_STATES][NAV_NUM_STATES];

   /*------------------------------------------------------------------------*
    * Request access to a new state-machine object.
    *------------------------------------------------------------------------*/
   gt_navStruct.t_stateMn = utl_requestStMnObject( NAV_NUM_STATES,
                                                   NAV_DCM_INIT);
   if( gt_navStruct.t_stateMn < 0)
      exit(0);

   /*------------------------------------------------------------------------*
    * Store the call-back function for each possible state transition.
    *------------------------------------------------------------------------*/
   if( utl_stMnPopFunMap( gt_navStruct.t_stateMn,
                          &usr_navIdle,
                          (uint8_t)NAV_IDLE) < 0)
   {
      exit(0);
   }

   if( utl_stMnPopFunMap( gt_navStruct.t_stateMn,
                          &usr_navMagCal,
                          (uint8_t)NAV_MAG_CAL) < 0)
   {
      exit(0);
   }

   if( utl_stMnPopFunMap( gt_navStruct.t_stateMn,
                          &usr_navAccelCal,
                          (uint8_t)NAV_ACCEL_CAL) < 0)
   {
      exit(0);
   }

   if( utl_stMnPopFunMap( gt_navStruct.t_stateMn,
                          &usr_navDcmInit,
                          (uint8_t)NAV_DCM_INIT) < 0)
   {
      exit(0);
   }

   if( utl_stMnPopFunMap( gt_navStruct.t_stateMn,
                          &usr_navActive,
                          (uint8_t)NAV_ACTIVE) < 0)
   {
      exit(0);
   }

   if( utl_stMnPopFunMap( gt_navStruct.t_stateMn,
                          &usr_navError,
                          (uint8_t)NAV_ERROR) < 0)
   {
      exit(0);
   }

   if( utl_stMnPopFunMap( gt_navStruct.t_stateMn,
                          &usr_navCalSampComp,
                          (uint8_t)NAV_CAL_SAMP_COMP) < 0)
   {
      exit(0);
   }

   if( utl_stMnPopFunMap( gt_navStruct.t_stateMn,
                          &usr_navCalComplete,
                          (uint8_t)NAV_CAL_COMPLETE) < 0)
   {
      exit(0);
   }

   memset( (void *)ac_transMap, 0, sizeof( ac_transMap));

   /*------------------------------------------------------------------------*
    * Define all possible state transitions...
    *------------------------------------------------------------------------*/
   ac_transMap[NAV_IDLE][NAV_IDLE]      = 1;
   ac_transMap[NAV_IDLE][NAV_MAG_CAL]   = 1;
   ac_transMap[NAV_IDLE][NAV_ACCEL_CAL] = 1;
   ac_transMap[NAV_IDLE][NAV_DCM_INIT]  = 1;
   ac_transMap[NAV_IDLE][NAV_ACTIVE]    = 1;

   ac_transMap[NAV_MAG_CAL][NAV_MAG_CAL]       = 1;
   ac_transMap[NAV_MAG_CAL][NAV_IDLE]          = 1;
   ac_transMap[NAV_MAG_CAL][NAV_ERROR]         = 1;
   ac_transMap[NAV_MAG_CAL][NAV_CAL_SAMP_COMP] = 1;
   ac_transMap[NAV_MAG_CAL][NAV_CAL_COMPLETE]  = 1;

   ac_transMap[NAV_ACCEL_CAL][NAV_ACCEL_CAL]     = 1;
   ac_transMap[NAV_ACCEL_CAL][NAV_IDLE]          = 1;
   ac_transMap[NAV_ACCEL_CAL][NAV_ERROR]         = 1;
   ac_transMap[NAV_ACCEL_CAL][NAV_CAL_SAMP_COMP] = 1;
   ac_transMap[NAV_ACCEL_CAL][NAV_CAL_COMPLETE]  = 1;

   ac_transMap[NAV_DCM_INIT][NAV_DCM_INIT] = 1;
   ac_transMap[NAV_DCM_INIT][NAV_ACTIVE]   = 1;
   ac_transMap[NAV_DCM_INIT][NAV_ERROR]    = 1;
   ac_transMap[NAV_DCM_INIT][NAV_IDLE]     = 1;

   ac_transMap[NAV_ERROR][NAV_ERROR]     = 1;
   ac_transMap[NAV_ERROR][NAV_MAG_CAL]   = 1;
   ac_transMap[NAV_ERROR][NAV_ACCEL_CAL] = 1;
   ac_transMap[NAV_ERROR][NAV_DCM_INIT]  = 1;
   ac_transMap[NAV_ERROR][NAV_ACTIVE]    = 1;
   ac_transMap[NAV_ERROR][NAV_IDLE]      = 1;

   ac_transMap[NAV_ACTIVE][NAV_ACTIVE] = 1;
   ac_transMap[NAV_ACTIVE][NAV_IDLE]   = 1;
   ac_transMap[NAV_ACTIVE][NAV_ERROR]  = 1;

   ac_transMap[NAV_CAL_COMPLETE][NAV_CAL_COMPLETE] = 1;
   ac_transMap[NAV_CAL_COMPLETE][NAV_DCM_INIT]     = 1;
   ac_transMap[NAV_CAL_COMPLETE][NAV_ACTIVE]       = 1;
   ac_transMap[NAV_CAL_COMPLETE][NAV_IDLE]         = 1;

   ac_transMap[NAV_CAL_SAMP_COMP][NAV_CAL_SAMP_COMP] = 1;
   ac_transMap[NAV_CAL_SAMP_COMP][NAV_MAG_CAL]       = 1;
   ac_transMap[NAV_CAL_SAMP_COMP][NAV_ACCEL_CAL]     = 1;
   ac_transMap[NAV_CAL_SAMP_COMP][NAV_IDLE]          = 1;

   /*------------------------------------------------------------------------*
    * Store the state transition map.
    *------------------------------------------------------------------------*/
   utl_stMnPopTransMap( gt_navStruct.t_stateMn,
                        &ac_transMap[0][0]);

}/*End usr_navStateMachineInit*/

void usr_navigationInit( void)
{

   /*------------------------------------------------------------------------*
    * Initialize the top-level navigation object.
    *------------------------------------------------------------------------*/
   memset( (void *)&gt_navStruct, 0, sizeof( t_navStruct));

   /*------------------------------------------------------------------------*
    * Create a new thread.
    *------------------------------------------------------------------------*/
   gt_navStruct.t_navThread = arb_threadCreate( usr_navigation,
                                                1,
                                                0,
                                                ARB_STACK_1536B,
                                                0);

   if( gt_navStruct.t_navThread < 0)
   {
      exit(0);

   }/*End if( gt_navStruct.t_navThread < 0)*/

   /*------------------------------------------------------------------------*
    * Open a handle to the INS driver.
    *------------------------------------------------------------------------*/
   gt_navStruct.t_insHndl = arb_open( "insDevice0",
                                      ARB_O_READ |
                                      ARB_O_WRITE);

   if( gt_navStruct.t_insHndl < 0)
      exit(0);

   gt_navStruct.t_signalHndl = arb_open( "signalDevice0",
                                         ARB_O_READ |
                                         ARB_O_WRITE);

   if( gt_navStruct.t_signalHndl < 0)
      exit(0);

   usr_navStateMachineInit();

   gt_navStruct.t_mutex = arb_semaphoreCreate( MUTEX);

   if( gt_navStruct.t_mutex < 0)
      exit(0);

   gt_navStruct.t_signal = arb_semaphoreCreate( SIGNAL);

   if( gt_navStruct.t_signal < 0)
      exit(0);

}/*End usr_navigationInit*/
