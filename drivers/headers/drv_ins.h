/*---------------------------------------------------------------------------*
 * Copyright (C) 2012 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
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
 * File Name   : drv_ins.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This driver is responsible for communicating with the
 *               ADXL345 accelerometer, HMC5843 magnetometer, and
 *               ITG-3200 gyro.
 *
 * References: 1. Principles of GNSS, INERTIAL, AND MULTISENSOR INTEGRATED
 *                NAVIGATION SYSTEMS - Paul D. Groves
 *             2. Effective Kalman Filter for MEMS-IMU/Magnetometers
 *                Integrated Attitude and Heading Reference Systems - Wei Li
 *                and Jinling Wang
 *             3. Aided Navigation - Jay A Farrell
 *             4. Direction Cosine Matrix IMU: Theory = William Premerlani
 *                and Paul Bizard
 *             5. Matlab program 'Ellipsoid fit' by Yury Petrov
 *
 * Last Update : September 26, 2012
 *---------------------------------------------------------------------------*/
#ifndef drv_ins_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define drv_ins_h

   /*------------------------------------------------------------------------*
    * The maximum number of samples gathered during calibration of the
    * accelerometer and magnetometer.
    *------------------------------------------------------------------------*/
   #define INS_MAX_CAL_SAMPLES (16)

   /*------------------------------------------------------------------------*
    * The number of coefficients needed by the ellipsoid-fit calibration
    * routine.
    *------------------------------------------------------------------------*/
   #define INS_EF_NUM_COEF (9)

   /*------------------------------------------------------------------------*
    * The calibrated value of the accelerometer envelope (represents 1G).
    *------------------------------------------------------------------------*/
   #define INS_MAX_CAL_GRAV_ENV (256)

   /*------------------------------------------------------------------------*
    * The calibrated value of the magnetometer envelope.
    *------------------------------------------------------------------------*/
   #define INS_MAX_CAL_MAG_ENV (512)

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "arb_error.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      INS_INVALID_BAUD     = -6, /*Invalid baud rate*/
      INS_INVALID_ARG      = -5, /*Invalid ioctl argument*/
      INS_INVALID_CMD      = -4, /*Invalid ioctl command.*/
      INS_NULL_PTR         = -3, /*Pointer is not mapped to a valid address.*/
      INS_OUT_OF_HEAP      = -1, /*No more memory.*/
      INS_PASSED           = 0   /*Configuration good.*/

   }t_insError; /*Possible error conditions returned by the device's ioctl
                  routine*/

   typedef enum
   {

      INS_SENS_STARTUP = 0,      /*Bring the sensors out of reset...*/
      INS_CALIBRATE_MAG,         /*Perform least-squares ellipsoid fit*/
      INS_CALIBRATE_ACCEL,       /*Perform least-squares ellipsoid fit*/
      INS_GET_MAG_CALIBRATION,   /*Returns the current mag calibration*/
      INS_GET_ACCEL_CALIBRATION, /*Returns the current accel calibration*/
      INS_GET_GYRO_CALIBRATION,  /*Returns the current gyro calibration*/
      INS_GET_STATE_EST,         /*Returns the Euler angles and metrics*/
      INS_GET_DCM,               /*Returns the DCM*/
      INS_DCM_INIT,              /*Initialize the DCM matrix*/
      INS_UPDATE,                /*Perform Kalman filter tracking*/
      INS_RESET_CAL,             /*Reset the calibration state*/
      INS_RESET                  /*Sets the DCM to a attitude of 0*/

   }t_insCmd;

   typedef enum
   {

      INS_CAL_FAILED = 0,  /*No valid calibration in memory.*/
      INS_CAL_IN_PROGRESS, /*A calibration is currently active.*/
      INS_CAL_COMPLETE,    /*Calibration finished.*/
      INS_CAL_INV_CMD      /*Invalid calibration command*/
   }t_insCalStatus;

   typedef struct
   {
      t_insCalStatus t_status;
      int16_t *ps_R;     /*Principle axis rotation matrix*/
      int16_t *ps_bias;  /*DC bias*/
      int16_t *ps_scale; /*Axis misalignment factor*/
      int8_t c_n;        /*Axis misalignment Q factor*/
      int16_t s_env;     /*Max envelope.*/
   }t_ellipsoidCal;

   typedef struct
   {
      /*---------------------------------------------------------------------*
       * Corrected roll, pitch, and yaw
       *---------------------------------------------------------------------*/
      float f_corrRoll;
      float f_corrPitch;
      float f_corrYaw;

      /*---------------------------------------------------------------------*
       * Uncorrected roll, pitch, and yaw
       *---------------------------------------------------------------------*/
      float f_rawRoll;
      float f_rawPitch;
      float f_rawYaw;

      /*---------------------------------------------------------------------*
       * If greater than 0, the platform is considered moving and the error-
       * state Kalman filter correction phase is disabled.
       *---------------------------------------------------------------------*/
      float f_platMovingCount;

      /*---------------------------------------------------------------------*
       * The average Kalman filter residual abs value--which is an indicator
       * of tracking performance.
       *---------------------------------------------------------------------*/
      float af_avrResMag[3];

   }t_currentMeas;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_error drv_insInit( void);

   void drv_insExit( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef drv_ins_h*/
