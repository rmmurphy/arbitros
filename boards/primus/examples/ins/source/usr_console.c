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
 * File Name   : usr_console.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for command and control over the
 *               console module. This involves creating the threads,
 *               and semaphores, which provide direct access to the console
 *               from a user space application.
 *
 * Programmer  : Ryan M Murphy
 *
 * Last Update : Dec, 10, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "arb_error.h"
#include "arb_thread.h"
#include "arb_semaphore.h"
#include "arb_device.h"
#include "arb_sysTimer.h"
#include "drv_console.h"
#include "usr_navigation.h"
#include "arb_printf.h"
#include "drv_ins.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Typedefs
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static void usr_sensorCal( t_DEVHANDLE t_consoleHndl,
                           t_navState t_calState,
                           int8_t *pc_buff);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void usr_sensorCal( t_DEVHANDLE t_consoleHndl,
                           t_navState t_calState,
                           int8_t *pc_buff)
{
   int16_t as_xyz[3];
   uint16_t s_size;
   int8_t c_index = 0;
   float af_R[3][3];
   float af_scale[3];
   float af_bias[3];

   /*------------------------------------------------------------------------*
    * Make sure the navigation thread is in the IDLE state.
    *------------------------------------------------------------------------*/
   while( usr_navGetState() != NAV_IDLE)
   {
      usr_navSetState( NAV_IDLE);
      arb_sleep( ARB_TICKS_PER_SECOND >> 1); /*.5sec update rate*/
   }

   switch( t_calState)
   {
      case NAV_MAG_CAL:

         do
      	{

            s_size = sprintf_P( (char *)pc_buff, PSTR("#calmsg #collect Rotate device to a new location, press enter when ready\r\n"));

            arb_write( t_consoleHndl,
                       pc_buff,
                       s_size);

            /*---------------------------------------------------------------*
             * Wait here until enter has been pressed...
             *---------------------------------------------------------------*/
            s_size = arb_read( t_consoleHndl,
                               pc_buff,
                               (int16_t)sizeof(pc_buff));

            /*---------------------------------------------------------------*
             * Does the user want to cancel the calibration?
             *---------------------------------------------------------------*/
            if( pc_buff[0] == 'q')
            {
               usr_navSetState( NAV_IDLE);
               break;
            }/*End if( pc_buff[0] == 'q')*/

            /*---------------------------------------------------------------*
             * Tell the INS driver to acquire the first measurement...
             *---------------------------------------------------------------*/
            usr_navSetState( NAV_MAG_CAL);

            /*---------------------------------------------------------------*
             * Wait until a new sample is acquired...
             *---------------------------------------------------------------*/
            usr_navWaitCalSample();

            /*---------------------------------------------------------------*
             * Read back the sample that was just gathered.
             *---------------------------------------------------------------*/
            usr_navGetCalSample( as_xyz);

            s_size = sprintf_P( (char *)pc_buff, PSTR("#calmsg #cal3dpoint %f %f %f %f %f\r\n"),
                                (float)as_xyz[0]/INS_MAX_CAL_MAG_ENV,
                                (float)as_xyz[1]/INS_MAX_CAL_MAG_ENV,
                                (float)as_xyz[2]/INS_MAX_CAL_MAG_ENV,
                                (float)c_index,
                                (float)INS_MAX_CAL_SAMPLES);

            arb_write( t_consoleHndl,
                       pc_buff,
                       s_size);

            arb_sleep( 10);

            c_index++;

         }while( usr_navGetState() == NAV_CAL_SAMP_COMP);

      break;/*End case NAV_MAG_CAL:*/

      case NAV_ACCEL_CAL:

         do
      	{

            s_size = sprintf_P( (char *)pc_buff, PSTR("#calmsg #collect Rotate device to a new location, press enter when ready\r\n"));

            arb_write( t_consoleHndl,
                       pc_buff,
                       s_size);

            /*---------------------------------------------------------------*
             * Wait here until enter has been pressed...
             *---------------------------------------------------------------*/
            s_size = arb_read( t_consoleHndl,
                               pc_buff,
                               (int16_t)sizeof(pc_buff));

            /*---------------------------------------------------------------*
             * Does the user want to cancel the calibration?
             *---------------------------------------------------------------*/
            if( pc_buff[0] == 'q')
            {
               usr_navSetState( NAV_IDLE);
               break;
            }/*End if( pc_buff[0] == 'q')*/

            /*---------------------------------------------------------------*
             * Tell the INS driver to acquire the first measurement...
             *---------------------------------------------------------------*/
            usr_navSetState( NAV_ACCEL_CAL);

            /*---------------------------------------------------------------*
             * Wait until a new sample is acquired...
             *---------------------------------------------------------------*/
            usr_navWaitCalSample();

            /*---------------------------------------------------------------*
             * Read back the sample that was just gathered.
             *---------------------------------------------------------------*/
            usr_navGetCalSample( as_xyz);

            s_size = sprintf_P( (char *)pc_buff, PSTR("#calmsg #cal3dpoint %f %f %f %f %f\r\n"),
                                (float)as_xyz[0]/INS_MAX_CAL_GRAV_ENV,
                                (float)as_xyz[1]/INS_MAX_CAL_GRAV_ENV,
                                (float)as_xyz[2]/INS_MAX_CAL_GRAV_ENV,
                                (float)c_index,
                                (float)INS_MAX_CAL_SAMPLES);

            arb_write( t_consoleHndl,
                       pc_buff,
                       s_size);

            arb_sleep( 10);

            c_index++;

         }while( usr_navGetState() == NAV_CAL_SAMP_COMP);

      break;/*End case NAV_ACCEL_CAL:*/

      default:

      break;

   }/*End switch( t_calState)*/

	if( usr_navGetCalStatus() == INS_CAL_COMPLETE)
	{
      if( t_calState == NAV_MAG_CAL)
      {
         /*------------------------------------------------------------------*
          * Get the current calibration parameters...
          *------------------------------------------------------------------*/
         usr_navGetMagCal( &af_R[0][0],
                           af_scale,
					            af_bias);

		}/*End if( t_calState == NAV_MAG_CAL)*/
      else if( t_calState == NAV_ACCEL_CAL)
      {
         usr_navGetAccelCal( &af_R[0][0],
                             af_scale,
					              af_bias);

      }/*End else if( t_calState == NAV_ACCEL_CAL)*/

		s_size = sprintf_P( (char *)pc_buff, PSTR("#calmsg #calrotmatrix %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\r\n"),
		af_R[0][0], af_R[0][1], af_R[0][2], af_R[1][0], af_R[1][1], af_R[1][2], af_R[2][0], af_R[2][1], af_R[2][2]);

      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);

      arb_sleep( 10);

		s_size = sprintf_P( (char *)pc_buff, PSTR("#calmsg #axisscale %.4f %.4f %.4f\r\n"),
		af_scale[0], af_scale[1], af_scale[2]);

      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);

      arb_sleep( 10);

		s_size = sprintf_P( (char *)pc_buff, PSTR("#calmsg #axisbias %.4f %.4f %.4f\r\n"),
		af_bias[0], af_bias[1], af_bias[2]);

      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);

      arb_sleep( 10);

      s_size = sprintf_P( (char *)pc_buff, PSTR("#calmsg #result passed\r\n"));

      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);


	}/*End if( usr_navGetCalStatus() == INS_CAL_COMPLETE)*/
	else
	{
		s_size = sprintf_P( (char *)pc_buff, PSTR("#calmsg #result failed\r\n"));

      arb_write( t_consoleHndl,
                 pc_buff,
                 s_size);
	}

}/*End usr_sensorCal*/

bool usr_console( t_DEVHANDLE t_consoleHndl,
                  int8_t *pc_buff,
                  t_consoleTokHndl *pt_tokHndl)
{

   bool b_success = true; /*A valid command was found...*/

   if( strcmp( (char *)pt_tokHndl->ac_tok[0], "sas") == 0) /*Set active state*/
   {
      usr_navSetState( NAV_ACTIVE);
   }/*End if( strcmp( (char *)pt_tokHndl->ac_tok[0], "sam") == 0)*/
   else if( strcmp( (char *)pt_tokHndl->ac_tok[0], "sis") == 0) /*Set idle state*/
   {
      usr_navSetState( NAV_ACCEL_CAL);
   }/*End else if( strcmp( (char *)pt_tokHndl->ac_tok[0], "sim") == 0)*/
   else if( strcmp( (char *)pt_tokHndl->ac_tok[0], "sdi") == 0) /*Set DCM init*/
   {
      usr_navSetState( NAV_DCM_INIT);
   }/*End else if( strcmp( (char *)pt_tokHndl->ac_tok[0], "sim") == 0)*/
   else if( strcmp( (char *)pt_tokHndl->ac_tok[0], "pmc") == 0) /*Perform mag cal*/
   {
      usr_sensorCal( t_consoleHndl,
                     NAV_MAG_CAL,
                     pc_buff);
   }/*Emd if( strcmp( (char *)pt_tokHndl->ac_tok[0], "pmc") == 0)*/
   else if( strcmp( (char *)pt_tokHndl->ac_tok[0], "pac") == 0) /*Perform accel cal*/
   {
      usr_sensorCal( t_consoleHndl,
                     NAV_ACCEL_CAL,
                     pc_buff);
   }/*End else if( strcmp( (char *)pt_tokHndl->ac_tok[0], "pac") == 0)*/
   else if( strcmp( (char *)pt_tokHndl->ac_tok[0], "gsd") == 0) /*Get state data*/
   {
      t_currentMeas t_meas;
      uint16_t s_size;
      t_navState t_state = usr_navGetState();

      usr_navGetStateEst( &t_meas);

      /*---------------------------------------------------------------------*
       * Return "Current State Data"
       *---------------------------------------------------------------------*/
		s_size = sprintf_P( (char *)pc_buff,
                          PSTR("#csd %f %f %f %f %f %f %f %f %f %f %f\r\n"),
                          t_meas.f_corrRoll,
                          t_meas.f_rawRoll,
                          t_meas.f_corrPitch,
                          t_meas.f_rawPitch,
                          t_meas.f_corrYaw,
                          t_meas.f_rawYaw,
                          t_meas.f_platMovingCount,
                          t_meas.af_avrResMag[0],
                          t_meas.af_avrResMag[1],
                          t_meas.af_avrResMag[2],
                          (float)t_state);

		arb_write( t_consoleHndl,
		           pc_buff,
		           s_size);
   }
   else /*Unrecognized message*/
   {
      b_success = false;
   }

   /*------------------------------------------------------------------------*
    * Return control over the console to the kernel...
    *------------------------------------------------------------------------*/
   return b_success;

}/*End arb_console*/
