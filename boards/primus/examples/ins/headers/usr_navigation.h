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
#ifndef usr_navigation_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define usr_navigation_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "drv_ins.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      NAV_IDLE = 0,     /*Navigation thread is not doing anything.*/
      NAV_CAL_COMPLETE, /*A Calibration has been successfully performed*/
      NAV_CAL_SAMP_COMP,/*A Calibration sample has been acquired*/
      NAV_MAG_CAL,      /*The magnetometer is being calibrated.*/
      NAV_ACCEL_CAL,    /*The accelerometer is being calibrated.*/
      NAV_DCM_INIT,     /*Initialize the DCM and estimate gyro bias.*/
      NAV_ACTIVE,       /*The error-state Kalman filter is actively tracking
                           the current attitude.*/
      NAV_ERROR,        /*A error has been detected.*/
      NAV_NUM_STATES
   }t_navState;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   void usr_navigationInit( void);

   /*------------------------------------------------------------------------*
    * Returns the current sample acquired during a NAV_MAG_CAL or
    * NAV_ACCEL_CAL state.
    *------------------------------------------------------------------------*/
   void usr_navGetCalSample( int16_t *ps_calSample);

   /*------------------------------------------------------------------------*
    * Returns the current magnetometer calibration parameters.
    *------------------------------------------------------------------------*/
   void usr_navGetMagCal( float *pf_R,
                          float *pf_scale,
                          float *pf_bias);

   /*------------------------------------------------------------------------*
    * Returns the current accelerometer calibration parameters.
    *------------------------------------------------------------------------*/
   void usr_navGetAccelCal( float *pf_R,
                            float *pf_scale,
                            float *pf_bias);

   /*------------------------------------------------------------------------*
    * Returns the current gyro calibration parameters as determined by the
    * error-state Kalman filter.
    *------------------------------------------------------------------------*/
   void usr_navGetGyroCal( float *pf_scale,
                           float *pf_bias);

   /*------------------------------------------------------------------------*
    * Return the current Kalman filtered estimates.
    *------------------------------------------------------------------------*/
   void usr_navGetStateEst( t_currentMeas *pt_currentMeas);

   /*------------------------------------------------------------------------*
    * Transitions the thread into one of 5 possible states as defined by
    * t_navState.
    *------------------------------------------------------------------------*/
   bool usr_navSetState( t_navState t_state);

   /*------------------------------------------------------------------------*
    * Returns the current state of the thread.
    *------------------------------------------------------------------------*/
   t_navState usr_navGetState( void);

   /*------------------------------------------------------------------------*
    * Returns the current state of an active calibration.
    *------------------------------------------------------------------------*/
   t_insCalStatus usr_navGetCalStatus( void);

   /*------------------------------------------------------------------------*
    * Synchronizes events with external threadls
    *------------------------------------------------------------------------*/
   void usr_navWaitCalSample( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef usr_navigation_h*/
