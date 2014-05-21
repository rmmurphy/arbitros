/*---------------------------------------------------------------------------*
 * Copyright (C) 2012 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
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
 * File Name   : drv_ins.c
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
 *             2. Effective Adaptive Kalman Filter for MEMS-IMU/Magnetometers
 *                Integrated Attitude and Heading Reference Systems - Wei Li
 *                and Jinling Wang
 *             3. Aided Navigation - Jay A Farrell
 *             4. Direction Cosine Matrix IMU: Theory = William Premerlani
 *                and Paul Bizard
 *             5. Matlab program 'Ellipsoid fit' by Yury Petrov
 *
 * Last Update : September 26, 2012
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "arb_device.h"
#include "arb_semaphore.h"
#include "drv_ins.h"
#include "hal_uart.h"
#include "hal_twi.h"
#include "arb_printf.h"
#include "arb_sysTimer.h"
#include "utl_math.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * This value represents a unique number assigned to this driver and has to be
 * different from all others registered with the kernel.
 *---------------------------------------------------------------------------*/
#define INS_MAJOR_NUMBER (6)

#define INS_TIMER          (TIMER_2)
#define INS_TWI            (TWI_2)
#define INS_TWI_BAUD_RATE  (400000)
#define INS_ACCEL_ADDRESS  (0x53)
#define INS_MAGN_ADDRESS   (0x1E)
#define INS_GYRO_ADDRESS   (0x68)

/*---------------------------------------------------------------------------*
 * The gyro calibration update rate
 *---------------------------------------------------------------------------*/
#define INS_GYRO_CAL_DT                 (ARB_TICKS_PER_SECOND >> 2)
#define INS_GYRO_CAL_NUM_MEAS           (1024)
#define INS_GYRO_CAL_FORGETING_FACT     (1.0/((float)INS_GYRO_CAL_NUM_MEAS))
#define INS_MAX_DELTA_PHASE_DEG         (4.0f) /*Degrees*/
#define INS_MAX_DELTA_PHASE_RAD_FXDPT   ((int16_t)((INS_MAX_DELTA_PHASE_DEG*\
                                        3.14159f*(float)16384.0f)/180.0f))
#define INS_MAX_RES_MAG_ERROR_DEG       (45.0f) /*Degrees*/
#define INS_MAX_RES_MAG_ERROR_RAD_FXDPT ((int16_t)(INS_MAX_RES_MAG_ERROR_DEG*\
                                        ((float)UTL_MATH_FXDPNT_PI)/180.0f))
#define INS_MAX_TWI_RETRIES             (10)

/*---------------------------------------------------------------------------*
 * The update rate of the INS
 *---------------------------------------------------------------------------*/
#define INS_dt (.05) /*INS DCM update rate in seconds*/
#define INS_DT_SLEEP (uint16_t)(INS_dt*(float)ARB_TICKS_PER_SECOND)

/*---------------------------------------------------------------------------*
 * The number of measurements to average during INS DCM initialization
 *---------------------------------------------------------------------------*/
#define INS_DCM_INIT_MAX_SAMP_TO_AVRG (int32_t)(.5 / INS_dt)
#define INS_GYRO_MAX_ROTATION         (2000.0) /*+/- 2000 deg/sec*/
#define INS_GYRO_CONV_FACTOR          ((INS_GYRO_MAX_ROTATION*3.1415926f\
/180.0f)*512.0f)
#define INS_AXIS_SCALE_Q_FACTOR       (13)

/*---------------------------------------------------------------------------*
 * The averaging factor for attitude residuals (alpha = 1 - dt/T),
 * where T = 5sec and dt = .05sec
 *---------------------------------------------------------------------------*/
#define INS_ATT_RES_AVR_Q (12)
#define INS_ATT_RES_FORGETTING_FACTOR \
((int16_t)(.95f*(float)(((int16_t)1<<INS_ATT_RES_AVR_Q) - 1))) /*Q0.12*/

/*---------------------------------------------------------------------------*
 * The amount of extra resolution given to the attitude residual variance
 * estimate.
 *---------------------------------------------------------------------------*/
#define INS_ATT_RES_VAR_EXTRA_RES (4)

/*---------------------------------------------------------------------------*
 * The amount of time to wait for the Kalman filter to lock after coming out
 * of reset.
 *---------------------------------------------------------------------------*/
#define INS_KALMAN_LOCK_WAIT (100)

/*---------------------------------------------------------------------------*
 * The accelerometer roll/pitch measurement noise. Normally these values are
 * set to the measured variance of the given sensor. However, in order to
 * maintain stability of the fixed-point Kalman filter this value was tuned
 * by hand.
 *---------------------------------------------------------------------------*/
#define INS_ROLL_PITCH_MEAS_NOISE (1024)

/*---------------------------------------------------------------------------*
 * The magnetometer yaw measurement noise. Normally these values are set to
 * the measured variance of the given sensor. However, in order to maintain
 * stability of the fixed-point Kalman filter this value was tuned by hand.
 *---------------------------------------------------------------------------*/
#define INS_YAW_MEAS_NOISE (1024)

/*---------------------------------------------------------------------------*
 * The amount above the average residual the residual needs to cross in order
 * to declare a bad measurement.
 *---------------------------------------------------------------------------*/
#define INS_RES_MEAS_ERROR_SCALER (5)

/*---------------------------------------------------------------------------*
 * The number of consecutive measurement errors needed in order to declare
 * loss of lock.
 *---------------------------------------------------------------------------*/
#define INS_KALMAN_LOSS_LOCK_COUNT (64)

/*---------------------------------------------------------------------------*
 * Private Data types
 *---------------------------------------------------------------------------*/
typedef struct
{
   /*------------------------------------------------------------------------*
    * The rotation matrix which defines the principle axis angles for the
    * the best fit ellipsoid in the least-squares sense.
	 * Q0.15
    *------------------------------------------------------------------------*/
   int16_t as_R[3][3];

   /*------------------------------------------------------------------------*
    * The scaling factor applied along the principle axis in order to
    * transform the ellipsoid back into a sphere. This variable compensates
    * for axis gain differences and soft-iron bias.
    * Q13
    *------------------------------------------------------------------------*/
   int16_t as_scale[3];

   /*------------------------------------------------------------------------*
    * The hard iron bias factors.
	 * Q0.15
    *------------------------------------------------------------------------*/
   int16_t as_bias[3];

   /*------------------------------------------------------------------------*
    * The strength of the magnetic field vector.
    *------------------------------------------------------------------------*/
   int16_t s_calMagFieldStr;

   /*------------------------------------------------------------------------*
    * The status of the sensor calibration.
    *------------------------------------------------------------------------*/
   t_insCalStatus t_cal;

}t_magDev;

typedef struct
{

   /*------------------------------------------------------------------------*
    * The estimated/calibrated gyro bias. This value is updated by the Kalman
    * filter Q0.15
    *------------------------------------------------------------------------*/
   int16_t as_bias[3];

   /*------------------------------------------------------------------------*
    * The estimated/calibrated gyro bias. This value is determined during
    * INS_DCM_INIT.
    * Q0.15
    *------------------------------------------------------------------------*/
   int16_t as_calBias[3];

   /*------------------------------------------------------------------------*
    * The scaling factor as determined by the error-state Kalman filter in
    * order to compensate for differences along the gyro axis of rotation.
    * Q13
    *------------------------------------------------------------------------*/
   int16_t as_scale[3];

   /*------------------------------------------------------------------------*
    * The running average of the amount of phase change per time step dt.
    * Q1.14 number where 2^14 - 1 = 1 radian
    *------------------------------------------------------------------------*/
   int16_t s_avrDPhase;

}t_gyroDev;

typedef struct
{

   /*------------------------------------------------------------------------*
    * The rotation matrix which defines the principle axis angles for the
    * the best fit ellipsoid in the least-squares sense.
    * Q0.15
    *------------------------------------------------------------------------*/
   int16_t as_R[3][3];

   /*------------------------------------------------------------------------*
    * The scaling factor applied along the principle axis in order to
    * transform the ellipsoid back into a sphere.
    * Q2.13
    *------------------------------------------------------------------------*/
   int16_t as_scale[3];

   /*------------------------------------------------------------------------*
    * The accelerometer bias
    * Q0.15
    *------------------------------------------------------------------------*/
   int16_t as_bias[3];

   /*------------------------------------------------------------------------*
    * The calibrated strength of gravity when the platform is stationary.
    * This value represents 1g of force.
    *------------------------------------------------------------------------*/
   int16_t s_calGravity;

   /*------------------------------------------------------------------------*
    * The status of the sensor calibration.
    *------------------------------------------------------------------------*/
   t_insCalStatus t_cal;

   /*------------------------------------------------------------------------*
    * The running average of the accelerometer specific force.
    *------------------------------------------------------------------------*/
   int16_t s_avrSpecForce;

}t_accelDev;

typedef struct
{

   /*------------------------------------------------------------------------*
    * Resources that can be shared amongst multiple users (either a global
    * buffer or IO device) need to be protected against race conditions. We
    * use this semaphore for just that purpose.
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_mutex;

   /*------------------------------------------------------------------------*
    * This semaphore is used for waking up the user-space thread once a
    * measurement has finished.
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_rxBlockingSem;

   /*------------------------------------------------------------------------*
    * We are going to want to know how many 'handles' or users are attached to
    * this device driver.
    *------------------------------------------------------------------------*/
   uint8_t c_numUsers;

   /*------------------------------------------------------------------------*
    * Handle to the particular UART this ins driver is using.
    *------------------------------------------------------------------------*/
   t_UARTHNDL t_uHandle;

   /*------------------------------------------------------------------------*
    * Handle to the particular TWI this ins driver is using.
    *------------------------------------------------------------------------*/
   t_TWIHNDL t_tHandle;

   /*------------------------------------------------------------------------*
    * The discrete cosine matrix
    * Q15
    *------------------------------------------------------------------------*/
   int16_t as_dcm[3][3];

   /*------------------------------------------------------------------------*
    * The current magnetometer calibration values
    *------------------------------------------------------------------------*/
   t_magDev t_mag;

   /*------------------------------------------------------------------------*
    * The current gyro calibration values.
    *------------------------------------------------------------------------*/
   t_gyroDev t_gyro;

   /*------------------------------------------------------------------------*
    * The current accelerometer calibration values.
    *------------------------------------------------------------------------*/
   t_accelDev t_accel;

   /*------------------------------------------------------------------------*
    * The corrected Euler attitude as determined from the DCM.
    * Q15 where pi = 32767
    *------------------------------------------------------------------------*/
   int16_t as_dcmAttitude[3];

   /*------------------------------------------------------------------------*
    * The uncorrected Euler attitude
    *------------------------------------------------------------------------*/
   int16_t as_rawAtt[3];

   /*------------------------------------------------------------------------*
    * The time of the last INS update...
    *------------------------------------------------------------------------*/
   uint32_t i_lastTime;

   /*------------------------------------------------------------------------*
    * Area of memory needed in order to perform the ellipsoid fitting routine
    * during calibration.
    *------------------------------------------------------------------------*/
   float gaf_scratchBuf[(INS_MAX_CAL_SAMPLES*INS_EF_NUM_COEF*2) + \
   (INS_EF_NUM_COEF*INS_EF_NUM_COEF)];

   /*------------------------------------------------------------------------*
    * Variable that keeps track of the current location in gas_H.
    *------------------------------------------------------------------------*/
   int8_t c_HWrtPtr;

   /*------------------------------------------------------------------------*
    * The error-state Kalman Filter error covariance matrix.
    *     | P00 P01 P02 |
    * P = | P10 P11 P12 |
    *     | P20 P21 P22 |
    *------------------------------------------------------------------------*/
   int16_t as_P[9][9];

   /*------------------------------------------------------------------------*
    * The estimated variance of the difference between the raw attitude and
    * the DCM corrected attitude.
    *------------------------------------------------------------------------*/
   int32_t ai_avrRes[3];

   /*------------------------------------------------------------------------*
    * The difference between the DCM attitude/heading and raw sensor
    * attitude/heading.
    *------------------------------------------------------------------------*/
   int16_t as_res[3];

   /*------------------------------------------------------------------------*
    * If greater than 0, the platform is considered moving and the error-state
    * Kalman filter correction phase is disabled.
    *------------------------------------------------------------------------*/
   int8_t c_platMovingCount;

   /*------------------------------------------------------------------------*
    * If true, then a roll measurement error has been detected.
    *------------------------------------------------------------------------*/
   bool b_rollMeasError;

   /*------------------------------------------------------------------------*
    * If true, then a pitch measurement error has been detected.
    *------------------------------------------------------------------------*/
   bool b_pitchMeasError;

   /*------------------------------------------------------------------------*
    * If true, then a yaw measurement error has been detected.
    *------------------------------------------------------------------------*/
   bool b_yawMeasError;

   /*------------------------------------------------------------------------*
    * If true, then a roll, pitch, or yaw measurement error has been valid
    * for 's_measErrorCount' consecutive frames.
    *------------------------------------------------------------------------*/
   bool b_lossOfLock;

   /*------------------------------------------------------------------------*
    * Keeps track of the number of consecutive frames a measurement error has
    * been detected.
    *------------------------------------------------------------------------*/
   int16_t s_measErrorCount;

   /*------------------------------------------------------------------------*
    * Keeps track of the number of kalman filter updates after first entering
    * the tracking state.
    *------------------------------------------------------------------------*/
   uint8_t c_initialLockCount;

}t_insDev;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_error insOpen( t_DEVHANDLE t_handle);

static int16_t insRead( t_DEVHANDLE t_handle,
                        int8_t *pc_buff,
                        uint16_t s_size);

static int16_t insWrite( t_DEVHANDLE t_handle,
                         int8_t *pc_buff,
                         uint16_t s_size);

static int32_t insIoctl( t_DEVHANDLE t_handle,
                         uint16_t s_command,
                         int32_t  i_arguments);

static t_error insClose( t_DEVHANDLE t_handle);

static bool ellipsoidFit( int16_t *ps_R,
                          int16_t *ps_bias,
                          int16_t *ps_scale,
                          int16_t s_envelope);

static void dcmInit( int16_t *ps_att);

static void getAttitudeInDeg( int16_t *ps_att,
                              int16_t *ps_attDeg);

static void applyCalibration( int16_t *ps_smp,
                              int16_t *ps_R,
                              int16_t *ps_scale,
                              int16_t *ps_bias);

static void dcmUpdate( void);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
t_deviceOperations gt_insDevOps =
{
    insOpen,
    insRead,
    insWrite,
    insIoctl,
    insClose

};

static uint8_t gc_debugUpdateCount = 0;

/*---------------------------------------------------------------------------*
 * This is the device's shared memory, all actions on this variable must be
 * mutually exclusive.
 *---------------------------------------------------------------------------*/
static t_insDev gt_insDev;

/*---------------------------------------------------------------------------*
 * Storage location for the samples gathered during calibration.
 *---------------------------------------------------------------------------*/
static int16_t gas_H[INS_MAX_CAL_SAMPLES][3];

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static inline void getMagMeas( int16_t *ps_xyz)
{
   t_twiError t_err;
   uint8_t ac_data[6];

   /*------------------------------------------------------------------------*
    * Read the current magnetometer measurements from registers 0x03,
    * 0x04, and 0x05, representing the x,y, and z axis'. There is no need for
    * the master to update the device register when requesting a new axis
    * measurement - the device automatically increments the address after the
    * completion of a two bytes read (MS byte first) on a particular axis.
    *------------------------------------------------------------------------*/
   ac_data[0] = 0x03;
   t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                               ac_data,
                               1,
                               (uint8_t)INS_MAGN_ADDRESS,
                               INS_MAX_TWI_RETRIES);

   t_err = hal_twiMasterRead( gt_insDev.t_tHandle,
                              ac_data,
                              6,
                              (uint8_t)INS_MAGN_ADDRESS,
                              INS_MAX_TWI_RETRIES);

   /*------------------------------------------------------------------------*
    * Align axis so that they are representative of the right hand rule. For
    * the particular case of the Sparkfun SEN-10183 this means the x axis
    * points toward the TWI connectors, the y axis points to the right, and
    * the z axis points down. Since this is different then whats on the
    * label the y and z axis need to be negated.
    *------------------------------------------------------------------------*/
   ps_xyz[0] = ((int16_t)ac_data[0] << 8) | (int16_t)ac_data[1];
   ps_xyz[1] = -(((int16_t)ac_data[2] << 8) | (int16_t)ac_data[3]);
   ps_xyz[2] = -(((int16_t)ac_data[4] << 8) | (int16_t)ac_data[5]);

}/*End getMagMeas*/

static inline void getGyroMeas( int16_t *ps_xyz)
{
   t_twiError t_err;
   uint8_t ac_data[6];

   /*------------------------------------------------------------------------*
    * Read the current gyro measurements from registers 0x1D,
    * 0x1E, and 0x1F, representing the x,y, and z axis'. There is no need for
    * the master to update the device register when requesting a new axis
    * measurement - the device automatically increments the address after the
    * completion of a two bytes read (MS byte first) on a particular axis.
    *------------------------------------------------------------------------*/
   ac_data[0] = 0x1D;
   t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                               ac_data,
                               1,
                               (uint8_t)INS_GYRO_ADDRESS,
                               INS_MAX_TWI_RETRIES);

   t_err = hal_twiMasterRead( gt_insDev.t_tHandle,
                              ac_data,
                              6,
                              (uint8_t)INS_GYRO_ADDRESS,
                              INS_MAX_TWI_RETRIES);

   /*------------------------------------------------------------------------*
    * Align axis so that they are representative of the right hand rule. For
    * the particular case of the Sparkfun SEN-10183 this means that gyro axis
    * are different then those printed on the breakout board- 'x' is actually
    * the sensor negative 'y' axis, 'y' is the sensor negative 'x' axis, and
    * 'z' is the sensor negative 'z' axis.
    *------------------------------------------------------------------------*/
   ps_xyz[0] = -(((int16_t)ac_data[2] << 8) | (int16_t)ac_data[3]);
   ps_xyz[1] = -(((int16_t)ac_data[0] << 8) | (int16_t)ac_data[1]);
   ps_xyz[2] = -(((int16_t)ac_data[4] << 8) | (int16_t)ac_data[5]);

}/*End getGyroMeas*/

static inline void getAccelMeas( int16_t *ps_xyz)
{
   t_twiError t_err;
   uint8_t ac_data[6];

   /*------------------------------------------------------------------------*
    * Read the current accelerometer measurements from registers 0x32, through
    * 0x37, representing the x,y, and z axis'. There is no need for
    * the master to update the device register when requesting a new axis
    * measurement - the device automatically increments the address after the
    * completion each bytes read in LS to MS byte order.
    *------------------------------------------------------------------------*/
   ac_data[0] = 0x32;
   t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                               ac_data,
                               1,
                               (uint8_t)INS_ACCEL_ADDRESS,
                               INS_MAX_TWI_RETRIES);

   t_err = hal_twiMasterRead( gt_insDev.t_tHandle,
                              ac_data,
                              6,
                              (uint8_t)INS_ACCEL_ADDRESS,
                              INS_MAX_TWI_RETRIES);

   /*------------------------------------------------------------------------*
    * Align axis so that they are representative of the right hand rule. For
    * the particular case of the Sparkfun SEN-10183 this means that acceler
    * ometer axis are different then those printed on the breakout board- 'x'
    * is actually the sensor 'y' axis, 'y' is the sensor 'x' axis, and
    * 'z' is unchanged.
    *------------------------------------------------------------------------*/
   ps_xyz[0] = (((int16_t)ac_data[3] << 8) | (int16_t)ac_data[2]);
   ps_xyz[1] = (((int16_t)ac_data[1] << 8) | (int16_t)ac_data[0]);
   ps_xyz[2] = (((int16_t)ac_data[5] << 8) | (int16_t)ac_data[4]);

}/*End getAccelMeas*/

static bool ellipsoidFit( int16_t *ps_R,
                          int16_t *ps_bias,
                          int16_t *ps_scale,
                          int16_t s_envelope)
{
   int32_t i_r = 0;
   int32_t i_c = 0;
   bool b_passed = false;
   float f_sum      = 0.0;
   float *pf_A      = NULL;
   float *pf_v      = NULL;
   float *pf_D      = NULL;
   float *pf_DTrans = NULL;
   float *pf_temp1  = NULL;
   float *pf_temp2  = NULL;
   float *pf_temp3  = NULL;
   float *pf_temp4  = NULL;
   float *pf_temp5  = NULL;
   float *pf_temp6  = NULL;
   float *pf_temp7  = NULL;
   float *pf_temp8  = NULL;

   float af_bias[3];  /*The hard iron bias*/
   float af_scale[3]; /*Scale factor and soft-iron bias compensation*/
   float af_R[3][3];  /*The principle axis rotation matrix*/
   float f_temp1 = 0.0;
   float f_temp2 = 0.0;
   float f_temp3 = 0.0;

   /*------------------------------------------------------------------------*
    * Perform an ellipsoid fit to the samples af_H using the least squares
    * method. This code is based on the work described in "Magnetometer
    * Calibration" by Seungmin Lee, Gihun Bae, and Katie Kortum and the
    * ellipsoid fit matlab algorithm by Yury Petrov see
    * http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
    * and hsl.dynalias.com/hsl/uploads/papers/UGV_F09_Magnetometer.docx for
    * further details...
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Form the design matrix D according to the general form Ax^2 + By^2 +
    * Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = Y. See line 61 in the
    * matlab function ellipsoid_fit for further details. Where the
    * D = [x^2 y^2 z^2 2xy 2xz 2yz 2x 2y 2z] and the 9 coefficients V = [A B C
    * D E F G H I].
    *------------------------------------------------------------------------*/
   pf_D = &gt_insDev.gaf_scratchBuf[0];
   for( i_r = 0; i_r < INS_MAX_CAL_SAMPLES; i_r++)
   {
      (*(pf_D + i_r*9 + 0)) = (float)gas_H[i_r][0]*
      (float)gas_H[i_r][0]; /*x*x*/
      (*(pf_D + i_r*9 + 1)) = (float)gas_H[i_r][1]*
      (float)gas_H[i_r][1]; /*y*y*/
      (*(pf_D + i_r*9 + 2)) = (float)gas_H[i_r][2]*
      (float)gas_H[i_r][2]; /*z*z*/
      (*(pf_D + i_r*9 + 3)) = 2*(float)gas_H[i_r][0]*
      (float)gas_H[i_r][1]; /*2*x*y*/
      (*(pf_D + i_r*9 + 4)) = 2*(float)gas_H[i_r][0]*
      (float)gas_H[i_r][2]; /*2*x*z*/
      (*(pf_D + i_r*9 + 5)) = 2*(float)gas_H[i_r][1]*
      (float)gas_H[i_r][2]; /*2*y*z*/
      (*(pf_D + i_r*9 + 6)) = 2*(float)gas_H[i_r][0]; /*2*x*/
      (*(pf_D + i_r*9 + 7)) = 2*(float)gas_H[i_r][1]; /*2*y*/
      (*(pf_D + i_r*9 + 8)) = 2*(float)gas_H[i_r][2]; /*2*z*/

   }/*End for( i_r = 0; i_r < INS_MAX_CAL_SAMPLES; i_r++)*/

   /*------------------------------------------------------------------------*
    * The ellipse can be represented by the equation DV = Y where Y represents
    * the magnitude we want the ellipse to have. Solving for v =
    * (( D' * D )^-1)*( D' * ones( INS_MAX_CAL_SAMPLES, 1 ) )...line 109 in the
    * matlab function ellipsoid_fit.
    *------------------------------------------------------------------------*/
   pf_DTrans = &gt_insDev.gaf_scratchBuf[INS_MAX_CAL_SAMPLES*9]; /*D' - 9 x INS_MAX_CAL_SAMPLES matrix*/
   utl_matTransF( pf_D,
                  pf_DTrans,
                  INS_MAX_CAL_SAMPLES,
                  9);

   pf_temp1  = &gt_insDev.gaf_scratchBuf[(INS_MAX_CAL_SAMPLES*9)*2]; /*(D'*D) - 9 x 9 matrix*/
   utl_matMultF( pf_DTrans,
                 9,
                 INS_MAX_CAL_SAMPLES,
                 pf_D,
                 INS_MAX_CAL_SAMPLES,
                 9,
                 pf_temp1);

   pf_temp2 = &gt_insDev.gaf_scratchBuf[0]; /*D'*Y - 9 x 1 matrix*/
   for( i_r = 0; i_r < 9; i_r++)
   {
      f_sum = 0;
      for( i_c = 0; i_c < INS_MAX_CAL_SAMPLES; i_c++)
      {
         f_sum += (*(pf_DTrans + i_r*INS_MAX_CAL_SAMPLES + i_c));

      }/*End for( i_c = 0; i_c < INS_MAX_CAL_SAMPLES; i_c++)*/

     pf_temp2[i_r] = f_sum;

   }/*End for( i_r = 0; i_r < 9; i_r++)*/

   pf_temp3 = &gt_insDev.gaf_scratchBuf[9]; /*(D'*D)^-1 - 9 x 9 matrix*/
   b_passed = utl_matInvF( pf_temp1,
                           pf_temp3,
                           9);

   if( b_passed == true)/*Is the matrix invertible?*/
   {
      pf_v = &gt_insDev.gaf_scratchBuf[9 + 9*9]; /* (D'*D)^-1*D'*Y - 9 x 1 matrix*/
      utl_matMultF( pf_temp3,
                    9,
                    9,
                    pf_temp2,
                    9,
                    1,
                    pf_v);

      /*---------------------------------------------------------------------*
       * Form the algebraic form of the ellipsoid... line 114 in the matlab
       * function ellipsoid_fit.
       *---------------------------------------------------------------------*/
      pf_A = &gt_insDev.gaf_scratchBuf[0]; /* A = [ v(1) v(4) v(5) v(7); ...
                                                    v(4) v(2) v(6) v(8); ...
                                                    v(5) v(6) v(3) v(9); ...
                                                    v(7) v(8) v(9) -1 ]; - size 4 x 4*/
      *(pf_A + 4*0 + 0) = pf_v[0];
      *(pf_A + 4*0 + 1) = pf_v[3];
      *(pf_A + 4*0 + 2) = pf_v[4];
      *(pf_A + 4*0 + 3) = pf_v[6];
      *(pf_A + 4*1 + 0) = pf_v[3];
      *(pf_A + 4*1 + 1) = pf_v[1];
      *(pf_A + 4*1 + 2) = pf_v[5];
      *(pf_A + 4*1 + 3) = pf_v[7];
      *(pf_A + 4*2 + 0) = pf_v[4];
      *(pf_A + 4*2 + 1) = pf_v[5];
      *(pf_A + 4*2 + 2) = pf_v[2];
      *(pf_A + 4*2 + 3) = pf_v[8];
      *(pf_A + 4*3 + 0) = pf_v[6];
      *(pf_A + 4*3 + 1) = pf_v[7];
      *(pf_A + 4*3 + 2) = pf_v[8];
      *(pf_A + 4*3 + 3) = -1;

      /*---------------------------------------------------------------------*
       * Start...find the center coordinates... line 121 in the matlab function
       * ellipsoid_fit.
       *---------------------------------------------------------------------*/
      pf_temp1 = &gt_insDev.gaf_scratchBuf[16];

      pf_temp1[0] = pf_v[6];
      pf_temp1[1] = pf_v[7];
      pf_temp1[2] = pf_v[8];

      pf_temp2 = &gt_insDev.gaf_scratchBuf[16+3]; /* -A( 1:3, 1:3 ) - 3 x 3 matrix*/

      for( i_r = 0; i_r < 3; i_r++)
         for( i_c = 0; i_c < 3; i_c++)
            (*(pf_temp2 + i_r*3 + i_c)) = -(*(pf_A + i_r*4 + i_c));

      pf_temp3 = &gt_insDev.gaf_scratchBuf[16+3+9]; /* -A( 1:3, 1:3 )' - 3 x 3 matrix*/
      utl_matTransF( pf_temp2,
                     pf_temp3,
                     3,
                     3);

      pf_temp4 = &gt_insDev.gaf_scratchBuf[16+3+9+9]; /* -A( 1:3, 1:3 )'*-A( 1:3, 1:3 ) -
                                            3 x 3 matrix*/
      utl_matMultF( pf_temp3,
                    3,
                    3,
                    pf_temp2,
                    3,
                    3,
                    pf_temp4);

      b_passed = utl_matInvF( pf_temp4,
                              pf_temp2,
                              3);

      if( b_passed == true)
      {
         utl_matMultF( pf_temp3,
                       3,
                       3,
                       pf_temp1,
                       3,
                       1,
                       pf_temp4);

         utl_matMultF( pf_temp2,
                       3,
                       3,
                       pf_temp4,
                       3,
                       1,
                       af_bias);

         /*------------------------------------------------------------------*
          * End find ellipsoid center.
          *------------------------------------------------------------------*/

         /*------------------------------------------------------------------*
          * Form the translation matrix.
          *------------------------------------------------------------------*/
         pf_temp5 = &gt_insDev.gaf_scratchBuf[16+3+9+9+9]; /* T = eye( 4 ) - 4 x 4
                                                              matrix*/
         utl_matEyeF( pf_temp5,
                      4,
                      1.0);

         (*(pf_temp5 + 3*4 + 0)) = af_bias[0];
         (*(pf_temp5 + 3*4 + 1)) = af_bias[1];
         (*(pf_temp5 + 3*4 + 2)) = af_bias[2];

         /*------------------------------------------------------------------*
          * Translate the ellipse to the center using R = T * A * T'... line
          * 126 in the matlab function ellipsoid_fit.
          *------------------------------------------------------------------*/
         pf_temp6 = &gt_insDev.gaf_scratchBuf[16+3+9+9+9+16]; /*T' - 4 x 4 matrix*/
         utl_matTransF( pf_temp5,
                        pf_temp6,
                        4,
                        4);

         pf_temp7 = &gt_insDev.gaf_scratchBuf[16+3+9+9+9+16+16]; /*A * T' - 4 x 4 matrix*/
         utl_matMultF( pf_A,
                       4,
                       4,
                       pf_temp6,
                       4,
                       4,
                       pf_temp7);

         pf_temp8 = &gt_insDev.gaf_scratchBuf[16+3+9+9+9+16+16+16]; /*T * A * T' - 4 x 4
                                                                      matrix*/
         utl_matMultF( pf_temp5,
                       4,
                       4,
                       pf_temp7,
                       4,
                       4,
                       pf_temp8);

         /*------------------------------------------------------------------*
          * R( 1:3, 1:3 ) / -R( 4, 4 ) - 3 x 3 matrix
          *------------------------------------------------------------------*/
         for( i_r = 0; i_r < 3; i_r++)
         {
            for( i_c = 0; i_c < 3; i_c++)
            {
               (*(pf_temp2 + i_r*3 + i_c)) = (*(pf_temp8 + i_r*4 + i_c)) /
               (-*(pf_temp8 + 3*4 + 3));
            }
         }

         /*------------------------------------------------------------------*
          * Find the eigenvalues and eigenvectors which represent the
          * magnitude and directions of the principle axis' of the ellipse
          * given by [ evecs evals ] = eig( R( 1:3, 1:3 ) / -R( 4, 4 ) )...
          * line 128 in the matlab function ellipsoid_fit.
          *------------------------------------------------------------------*/
         pf_temp3 = &gt_insDev.gaf_scratchBuf[16+3+9];/* evals - 1 x 3 matrix*/
         b_passed = utl_matEigsF( pf_temp2,
                                  3,
                                  &af_scale[0],
                                  &af_R[0][0]);

         /*------------------------------------------------------------------*
          * Calculate the calibration factors needed to shape the ellipsoid
          * axis' back onto a sphere or radius gf_calibratedEnvelope. Where the
          * best fit ellipsoid radius is given by radii = sqrt( 1 ./ diag(
          * evals ) ) and the resulting calibration factor is a diagonal matrix
          * with values gf_calibratedEnvelope/ radii...line 129 of the matlab
          * function ellipsoid_fit.
          *------------------------------------------------------------------*/
         f_temp1 = 1.0 / af_scale[0];
         f_temp2 = 1.0 / af_scale[1];
         f_temp3 = 1.0 / af_scale[2];

         /*------------------------------------------------------------------*
          * If values out of range then exit.
          *------------------------------------------------------------------*/
         if( (f_temp1 <= 0) ||
             (f_temp2 <= 0) ||
             (f_temp3 <= 0))
         {
            b_passed = false;
         }
         else
         {

            af_scale[0] = ((float)s_envelope) / sqrtf( f_temp1);
            af_scale[1] = ((float)s_envelope) / sqrtf( f_temp2);
            af_scale[2] = ((float)s_envelope) / sqrtf( f_temp3);

            /*---------------------------------------------------------------*
             * Convert the floating point ellipsoid fit parameters into s:m:f
             * fixed-point format where s = 1 is the sign bit, m = 16 is the
             * integer bits, and f = 15 are the fractional bits.
             *---------------------------------------------------------------*/
            ps_bias[0] = (int16_t)af_bias[0];
            ps_bias[1] = (int16_t)af_bias[1];
            ps_bias[2] = (int16_t)af_bias[2];

            ps_R[0] = (int16_t)(af_R[0][0]*32767.0);
            ps_R[1] = (int16_t)(af_R[0][1]*32767.0);
            ps_R[2] = (int16_t)(af_R[0][2]*32767.0);
            ps_R[3] = (int16_t)(af_R[1][0]*32767.0);
            ps_R[4] = (int16_t)(af_R[1][1]*32767.0);
            ps_R[5] = (int16_t)(af_R[1][2]*32767.0);
            ps_R[6] = (int16_t)(af_R[2][0]*32767.0);
            ps_R[7] = (int16_t)(af_R[2][1]*32767.0);
            ps_R[8] = (int16_t)(af_R[2][2]*32767.0);

            ps_scale[0] = (int16_t)(af_scale[0]*(float)(1 << INS_AXIS_SCALE_Q_FACTOR));
            ps_scale[1] = (int16_t)(af_scale[1]*(float)(1 << INS_AXIS_SCALE_Q_FACTOR));
            ps_scale[2] = (int16_t)(af_scale[2]*(float)(1 << INS_AXIS_SCALE_Q_FACTOR));
         }

      }/*End if( b_passed == true)*/

   }/*End if( b_passed == true)*/

   return b_passed;

}/*End ellipsoidFit*/

static inline int16_t phaseError( int16_t s_phase1,
                                 int16_t s_phase2)

{
   int32_t i_error = (int32_t)s_phase1 - (int32_t)s_phase2;

   if( i_error <= UTL_MATH_FXDPNT_NEGATIVE_PI)
      i_error = i_error + UTL_MATH_FXDPNT_TWO_PI_WRAP;
   else if( i_error > UTL_MATH_FXDPNT_PI)
      i_error = i_error - UTL_MATH_FXDPNT_TWO_PI_WRAP;

   return (int16_t)i_error;

}/*End phaseError*/

static inline void eSKalmanResVarEst( int16_t *ps_rawAtt)
{
   int32_t *pi_resVar = gt_insDev.ai_avrRes;
   int32_t i_temp;
   int16_t *ps_res = &gt_insDev.as_res[0];

   ps_res[0] = (int16_t)utl_abs32_32( phaseError( gt_insDev.as_dcmAttitude[0],
   ps_rawAtt[0]));
   ps_res[1] = (int16_t)utl_abs32_32( phaseError( gt_insDev.as_dcmAttitude[1],
   ps_rawAtt[1]));
   ps_res[2] = (int16_t)utl_abs32_32( phaseError( gt_insDev.as_dcmAttitude[2],
   ps_rawAtt[2]));

   i_temp = pi_resVar[0]*(int32_t)INS_ATT_RES_FORGETTING_FACTOR;
   i_temp = i_temp + ((int32_t)ps_res[0] << (INS_ATT_RES_VAR_EXTRA_RES+1))*(4095 -
   INS_ATT_RES_FORGETTING_FACTOR);
   pi_resVar[0] = (int32_t)((i_temp + ((int32_t)1 << (INS_ATT_RES_AVR_Q - 1)))
   >> INS_ATT_RES_AVR_Q);

   i_temp = pi_resVar[1]*(int32_t)INS_ATT_RES_FORGETTING_FACTOR;
   i_temp = i_temp + ((int32_t)ps_res[1] << (INS_ATT_RES_VAR_EXTRA_RES+1))*(4095 -
   INS_ATT_RES_FORGETTING_FACTOR);
   pi_resVar[1] = (int32_t)((i_temp + ((int32_t)1 << (INS_ATT_RES_AVR_Q - 1)))
   >> INS_ATT_RES_AVR_Q);

   i_temp = pi_resVar[2]*(int32_t)INS_ATT_RES_FORGETTING_FACTOR;
   i_temp = i_temp + ((int32_t)ps_res[2] << (INS_ATT_RES_VAR_EXTRA_RES+1))*(4095 -
   INS_ATT_RES_FORGETTING_FACTOR);
   pi_resVar[2] = (int32_t)((i_temp + ((int32_t)1 << (INS_ATT_RES_AVR_Q - 1)))
   >> INS_ATT_RES_AVR_Q);

}/*End eSKalmanResVarEst*/

static void eSKalmanFilterInit( void)
{
   uint8_t c_index;

   /*-------------------------------------------------------------------------*
    * Initialize the error covariance 9x9 matrix to all zeros except for the
    * diagonals--this will help the Kalman lock faster by letting it know that
    * we are uncertain about the state of the system.
    *-------------------------------------------------------------------------*/
   memset( (void *)gt_insDev.as_P, 0, sizeof( gt_insDev.as_P));

}/*End eSKalmanFilterInit*/

static inline void eSKalmanUpdateSystemNoise( int16_t *ps_Q,
                                              int16_t s_dt)
{
   int16_t *ps_res = &gt_insDev.as_res[0];

   /*-------------------------------------------------------------------------*
    * This value represents the random walk error on the gyro bias. The
    * nominal value according to the ITG-3200 states that there is .38 Hz/s of
    * error on average. In order to account for quantization errors, double
    * this amount
    *-------------------------------------------------------------------------*/
   int16_t s_nomQ = utl_mult16x16_16( 213, s_dt, 15); /*(.38*pi/180)*dt rad*/

   /*-------------------------------------------------------------------------*
    * Update attitude system (process) noise. This step corrects for Kalman
    * tracking errors by increasing the roll, pitch and yaw process noise
    * whenever a large error in DCM estimated attitude (relative to the raw
    * attitude) has been detected.
    *-------------------------------------------------------------------------*/
   if( gt_insDev.b_rollMeasError == true)
      ps_Q[0] = 1629;
   else
      ps_Q[0] = s_nomQ; /*(.38*pi/180)*dt rad*/

   if( gt_insDev.b_pitchMeasError == true)
      ps_Q[1] = 1629;
   else
      ps_Q[1] = s_nomQ; /*(.38*pi/180)*dt rad*/

   if( gt_insDev.b_yawMeasError == true)
      ps_Q[2] = 1629;
   else
      ps_Q[2] = s_nomQ; /*(.38*pi/180)*dt rad*/

   /*----------------------------------------------------------------------*
    * Update gyro bias system (process) noise. The value is set to 2.5% of the
    * current bias estimate. See section 4.4.1 of [1]
    *----------------------------------------------------------------------*/
   ps_Q[3] = (int16_t)utl_abs32_32( utl_mult16x16_16( utl_mult16x16_16(
   gt_insDev.t_gyro.as_bias[0], 819, 15), s_dt, 15));
   if( ps_Q[3] < 5)
      ps_Q[3] = 5;
   ps_Q[4] = (int16_t)utl_abs32_32( utl_mult16x16_16( utl_mult16x16_16(
   gt_insDev.t_gyro.as_bias[1], 819, 15), s_dt, 15));
   if( ps_Q[4] < 5)
      ps_Q[4] = 5;
   ps_Q[5] = (int16_t)utl_abs32_32( utl_mult16x16_16( utl_mult16x16_16(
   gt_insDev.t_gyro.as_bias[2], 819, 15), s_dt, 15));
   if( ps_Q[5] < 5)
      ps_Q[5] = 5;

   /*------------------------------------------------------------------------*
    * gt_insDev.t_gyro.as_scale has a different Q factor there for we need
    * to make sure that as_QgScale is scaled so that it is Q15.
    *------------------------------------------------------------------------*/
   ps_Q[6] = 5;
   ps_Q[7] = 5;
   ps_Q[8] = 5;

}/*End eSKalmanUpdateSystemNoise*/

static inline void eSKalmanFilterPredict( int32_t *pi_gyr,
                                          int16_t s_dt)
{
   int16_t as_Q[9]; /*attitude, gyro bias, gyro scale*/
   int16_t as_phi[9][9];
   int16_t as_tmp[9][9];
   int16_t as_tmp2[9][9];
   int8_t c_index;
   int32_t i_temp;

   eSKalmanUpdateSystemNoise( as_Q,
                              s_dt);

   /*------------------------------------------------------------------------*
    * Propagate the error covariance matrix forward in time...
    * P = phi*P*phi' + Q
    * eq 3.11 of [1]
    * where,
    *       |I3 DCM*dt -DCM*G*dt |
    * phi = |03  I3     03       |
    *       |03  03     I3       |
    * [1] eq 12.39 with v, r, and ba removed and gyro scale factor
    * added
    * [2] eq 10 with the rotation rate removed and attitude propagation
    * term negated
    * [3] Section 10.5.3 with xa removed Fg = 0, gyro scale rate added
    * and attitude propagation term negated.
    *------------------------------------------------------------------------*/
   memset( (void *)as_phi, 0, sizeof( as_phi));

   as_phi[0][0] = 32767;
   as_phi[1][1] = 32767;
   as_phi[2][2] = 32767;
   as_phi[3][3] = 32767;
   as_phi[4][4] = 32767;
   as_phi[5][5] = 32767;
   as_phi[6][6] = 32767;
   as_phi[7][7] = 32767;
   as_phi[8][8] = 32767;

   as_phi[0][3] = utl_mult16x16_16( gt_insDev.as_dcm[0][0], s_dt, 15);
   as_phi[0][4] = utl_mult16x16_16( gt_insDev.as_dcm[0][1], s_dt, 15);
   as_phi[0][5] = utl_mult16x16_16( gt_insDev.as_dcm[0][2], s_dt, 15);
   as_phi[1][3] = utl_mult16x16_16( gt_insDev.as_dcm[1][0], s_dt, 15);
   as_phi[1][4] = utl_mult16x16_16( gt_insDev.as_dcm[1][1], s_dt, 15);
   as_phi[1][5] = utl_mult16x16_16( gt_insDev.as_dcm[1][2], s_dt, 15);
   as_phi[2][3] = utl_mult16x16_16( gt_insDev.as_dcm[2][0], s_dt, 15);
   as_phi[2][4] = utl_mult16x16_16( gt_insDev.as_dcm[2][1], s_dt, 15);
   as_phi[2][5] = utl_mult16x16_16( gt_insDev.as_dcm[2][2], s_dt, 15);

   as_phi[0][6] = -(int16_t)(((int32_t)as_phi[0][3]*(int32_t)pi_gyr[0]) >> 15);
   as_phi[0][7] = -(int16_t)(((int32_t)as_phi[0][4]*(int32_t)pi_gyr[1]) >> 15);
   as_phi[0][8] = -(int16_t)(((int32_t)as_phi[0][5]*(int32_t)pi_gyr[2]) >> 15);
   as_phi[1][6] = -(int16_t)(((int32_t)as_phi[1][3]*(int32_t)pi_gyr[0]) >> 15);
   as_phi[1][7] = -(int16_t)(((int32_t)as_phi[1][4]*(int32_t)pi_gyr[1]) >> 15);
   as_phi[1][8] = -(int16_t)(((int32_t)as_phi[1][5]*(int32_t)pi_gyr[2]) >> 15);
   as_phi[2][6] = -(int16_t)(((int32_t)as_phi[2][3]*(int32_t)pi_gyr[0]) >> 15);
   as_phi[2][7] = -(int16_t)(((int32_t)as_phi[2][4]*(int32_t)pi_gyr[1]) >> 15);
   as_phi[2][8] = -(int16_t)(((int32_t)as_phi[2][5]*(int32_t)pi_gyr[2]) >> 15);

   /*------------------------------------------------------------------------*
    * phi*P
    *------------------------------------------------------------------------*/
   utl_matMult16x16_16( &as_phi[0][0],
                        9,
                        9,
                        &gt_insDev.as_P[0][0],
                        9,
                        9,
                        &as_tmp[0][0],
                        1,
                        15);

   utl_matTrans16( &as_phi[0][0],
                   &as_tmp2[0][0],
                   9,
                   9);

   /*------------------------------------------------------------------------*
    * phi*P*phi'
    *------------------------------------------------------------------------*/
   utl_matMult16x16_16( &as_tmp[0][0],
                        9,
                        9,
                        &as_tmp2[0][0],
                        9,
                        9,
                        &gt_insDev.as_P[0][0],
                        1,
                        15);

   /*------------------------------------------------------------------------*
    * P = phi*P*phi' + Q
    *------------------------------------------------------------------------*/
   for( c_index = 0; c_index < 9; c_index++)
   {

      /*---------------------------------------------------------------------*
       * Limit the growth of the error covariance matrix in order to
       * maintain 16-bit math.
       *---------------------------------------------------------------------*/
      i_temp = (int32_t)gt_insDev.as_P[c_index][c_index] + (int32_t)as_Q[c_index];
      if( i_temp > 8192)
         i_temp = 8192;
      gt_insDev.as_P[c_index][c_index] = (int16_t)i_temp;

   }

}/*End eSKalmanFilterPredict*/

static inline void eSKalmanFilterCorrect( int16_t *ps_deltaZ,
                                          int16_t *ps_deltaAtt,
                                          int16_t *ps_deltaGBias,
                                          int16_t *ps_deltaGScale,
                                          int16_t s_dt)
{
   int32_t ai_R[3]; /*Measurement noise where as_R[0] = gravity x noise,
                      R[1] = gravity y noise, and R[2] = mag y noise*/
   int16_t as_H00[3][3];
   int16_t as_H00T[3][3];
   int16_t as_tmp[3][3];
   int16_t as_tmp1[3][3];
   int16_t as_tmp2[3][3];
   int16_t as_tmp3[3][3];
   int16_t as_K00[3][3];
   int16_t as_K10[3][3];
   int16_t as_K20[3][3];
   int16_t as_eyeMinusKH[9][9];
   int16_t as_tmpP[9][9];
   int32_t ai_T[3];
   int32_t i_tmp;
   int8_t c_count;
   uint8_t c_index;
   int16_t *ps_res = &gt_insDev.as_res[0];

   /*------------------------------------------------------------------------*
    * Update the measurement noise estimate. Whenever there is a large
    * amount of acceleration we want to ignore correcting the DCM since the
    * accelerometer is measuring specific force instead of gravity.
    * Similar to [3] eq 10.61
    *------------------------------------------------------------------------*/
   if( gt_insDev.c_platMovingCount > 0)
   {
      ai_R[0] = 3276700;
      ai_R[1] = 3276700;
      ai_R[2] = 3276700;
   }
   else /*Not moving*/
   {

      if( gt_insDev.b_rollMeasError == true)
         ai_R[0] = 3276700;
      else
         ai_R[0] = INS_ROLL_PITCH_MEAS_NOISE;

      if( gt_insDev.b_pitchMeasError == true)
         ai_R[1] = 3276700;
      else
         ai_R[1] = INS_ROLL_PITCH_MEAS_NOISE;

      if( gt_insDev.b_yawMeasError == true)
         ai_R[2] = 3276700;
      else
         ai_R[2] = INS_YAW_MEAS_NOISE;
   }

   /*------------------------------------------------------------------------*
    * Calculate the measurement matrix
    *
    * H = | H00 03 03 | - the second and third term are ignored in the
    * following calculations in order to reduce processor loading.
    *
    * [2] eq 17 with the magnetometer yaw term added
    * [3] eq's 10.42 and 10.45 are combined with the Rbn term removed
    *  and ge/me terms negated.
    *------------------------------------------------------------------------*/
   as_H00[0][0] = 0;
   as_H00[0][1] = -32768;
   as_H00[0][2] = 0;
   as_H00[1][0] = 32767;
   as_H00[1][1] = 0;
   as_H00[1][2] = 0;
   as_H00[2][0] = 0;
   as_H00[2][1] = 0;
   as_H00[2][2] = -32768;

   /*------------------------------------------------------------------------*
    * Compute H'
    *------------------------------------------------------------------------*/
   as_H00T[0][0] = 0;
   as_H00T[0][1] = 32767;
   as_H00T[0][2] = 0;
   as_H00T[1][0] = -32768;
   as_H00T[1][1] = 0;
   as_H00T[1][2] = 0;
   as_H00T[2][0] = 0;
   as_H00T[2][1] = 0;
   as_H00T[2][2] = -32768;

   utl_matTrans16( &as_H00[0][0],
                   &as_H00T[0][0],
                   3,
                   3);

   as_tmp[0][0] = gt_insDev.as_P[0][0];
   as_tmp[0][1] = gt_insDev.as_P[0][1];
   as_tmp[0][2] = gt_insDev.as_P[0][2];
   as_tmp[1][0] = gt_insDev.as_P[1][0];
   as_tmp[1][1] = gt_insDev.as_P[1][1];
   as_tmp[1][2] = gt_insDev.as_P[1][2];
   as_tmp[2][0] = gt_insDev.as_P[2][0];
   as_tmp[2][1] = gt_insDev.as_P[2][1];
   as_tmp[2][2] = gt_insDev.as_P[2][2];

   /*------------------------------------------------------------------------*
    *                | PHT00 |
    * Perform P*H' = | PHT10 |  where PHT00 through PHT20 are 3x3 matrix'
    *                | PHT20 |  this operation is simplified due to the fact
    * that the first 3x3 matrix of H' is the only non-zero matrix.
    *------------------------------------------------------------------------*/
   utl_matMult16x16_16( &as_tmp[0][0],
                        3,
                        3,
                        &as_H00T[0][0],
                        3,
                        3,
                        &as_tmp1[0][0],
                        1,
                        15);

   as_tmp[0][0] = gt_insDev.as_P[3][0];
   as_tmp[0][1] = gt_insDev.as_P[3][1];
   as_tmp[0][2] = gt_insDev.as_P[3][2];
   as_tmp[1][0] = gt_insDev.as_P[4][0];
   as_tmp[1][1] = gt_insDev.as_P[4][1];
   as_tmp[1][2] = gt_insDev.as_P[4][2];
   as_tmp[2][0] = gt_insDev.as_P[5][0];
   as_tmp[2][1] = gt_insDev.as_P[5][1];
   as_tmp[2][2] = gt_insDev.as_P[5][2];

   utl_matMult16x16_16( &as_tmp[0][0],
                        3,
                        3,
                        &as_H00T[0][0],
                        3,
                        3,
                        &as_tmp2[0][0],
                        1,
                        15);

   as_tmp[0][0] = gt_insDev.as_P[6][0];
   as_tmp[0][1] = gt_insDev.as_P[6][1];
   as_tmp[0][2] = gt_insDev.as_P[6][2];
   as_tmp[1][0] = gt_insDev.as_P[7][0];
   as_tmp[1][1] = gt_insDev.as_P[7][1];
   as_tmp[1][2] = gt_insDev.as_P[7][2];
   as_tmp[2][0] = gt_insDev.as_P[8][0];
   as_tmp[2][1] = gt_insDev.as_P[8][1];
   as_tmp[2][2] = gt_insDev.as_P[8][2];

   utl_matMult16x16_16( &as_tmp[0][0],
                        3,
                        3,
                        &as_H00T[0][0],
                        3,
                        3,
                        &as_tmp3[0][0],
                        1,
                        15);

   /*------------------------------------------------------------------------*
    * Compute H*P*H'
    *------------------------------------------------------------------------*/
   utl_matMult16x16_16( &as_H00[0][0],
                        3,
                        3,
                        &as_tmp1[0][0],
                        3,
                        3,
                        &as_tmp[0][0],
                        1,
                        15);

   /*------------------------------------------------------------------------*
    * Compute H*P*H' + R
    *------------------------------------------------------------------------*/
   ai_T[0] = (int32_t)as_tmp[0][0] + (int32_t)ai_R[0];
   ai_T[1] = (int32_t)as_tmp[1][1] + (int32_t)ai_R[1];
   ai_T[2] = (int32_t)as_tmp[2][2] + (int32_t)ai_R[2];

   /*------------------------------------------------------------------------*
    * Since the off diagonal elements of H*P*H' are small, they are
    * ignored in order to avoid a matrix inverse operation.
    *------------------------------------------------------------------------*/
   i_tmp = (((int32_t)1<<15)*(int32_t)1024) / ai_T[0]; /*Q10*/

   if( i_tmp > 32767)
      i_tmp = 32767;

   as_tmp[0][0] = i_tmp;

   i_tmp = (((int32_t)1<<15)*(int32_t)1024) / ai_T[1]; /*Q10*/

   if( i_tmp > 32767)
      i_tmp = 32767;

   as_tmp[1][1] = i_tmp;

   i_tmp = (((int32_t)1<<15)*(int32_t)1024) / ai_T[2]; /*Q10*/

   if( i_tmp > 32767)
      i_tmp = 32767;

   as_tmp[2][2] = i_tmp;

   /*------------------------------------------------------------------------*
    * Update the Kalman gain
    * K = (P*H')/(H*P*H'+ R)
    * [1] eq 3.15
    *------------------------------------------------------------------------*/
   utl_matMult16x16_16( &as_tmp1[0][0], /*Q15*/
                        3,
                        3,
                        &as_tmp[0][0],  /*Q10*/
                        3,
                        3,
                        &as_K00[0][0],  /*Q15*/
                        0,
                        10);

   utl_matMult16x16_16( &as_tmp2[0][0], /*Q15*/
                        3,
                        3,
                        &as_tmp[0][0],  /*Q10*/
                        3,
                        3,
                        &as_K10[0][0],  /*Q15*/
                        0,
                        10);

   utl_matMult16x16_16( &as_tmp3[0][0], /*Q15*/
                        3,
                        3,
                        &as_tmp[0][0],  /*Q10*/
                        3,
                        3,
                        &as_K20[0][0],  /*Q15*/
                        0,
                        10);

   /*------------------------------------------------------------------------*
    * Update the state vector estimate ignoring the previous xk term
    * since this is an error-state Kalman filter
    * [1] eq 3.16 and section 3.2.6
    *------------------------------------------------------------------------*/
   i_tmp = 0;
   i_tmp = utl_mac16x16_32( as_K00[0][0], ps_deltaZ[0], i_tmp, 1);
   i_tmp = utl_mac16x16_32( as_K00[0][1], ps_deltaZ[1], i_tmp, 1);
   i_tmp = utl_mac16x16_32( as_K00[0][2], ps_deltaZ[2], i_tmp, 1);
   ps_deltaAtt[0] = utl_rShft32_16( i_tmp, 14); /*Q15*/
   i_tmp = 0;
   i_tmp = utl_mac16x16_32( as_K00[1][0], ps_deltaZ[0], i_tmp, 1);
   i_tmp = utl_mac16x16_32( as_K00[1][1], ps_deltaZ[1], i_tmp, 1);
   i_tmp = utl_mac16x16_32( as_K00[1][2], ps_deltaZ[2], i_tmp, 1);
   ps_deltaAtt[1] = utl_rShft32_16( i_tmp, 14); /*Q15*/
   i_tmp = 0;
   i_tmp = utl_mac16x16_32( as_K00[2][0], ps_deltaZ[0], i_tmp, 1);
   i_tmp = utl_mac16x16_32( as_K00[2][1], ps_deltaZ[1], i_tmp, 1);
   i_tmp = utl_mac16x16_32( as_K00[2][2], ps_deltaZ[2], i_tmp, 1);
   ps_deltaAtt[2] = utl_rShft32_16( i_tmp, 14); /*Q15*/

   i_tmp = 0;
   i_tmp = utl_mac16x16_32( as_K10[0][0], ps_deltaZ[0], i_tmp, 0);
   i_tmp = utl_mac16x16_32( as_K10[0][1], ps_deltaZ[1], i_tmp, 0);
   i_tmp = utl_mac16x16_32( as_K10[0][2], ps_deltaZ[2], i_tmp, 0);
   ps_deltaGBias[0] = utl_rShft32_16( i_tmp, 15); /*Q15*/
   i_tmp = 0;
   i_tmp = utl_mac16x16_32( as_K10[1][0], ps_deltaZ[0], i_tmp, 0);
   i_tmp = utl_mac16x16_32( as_K10[1][1], ps_deltaZ[1], i_tmp, 0);
   i_tmp = utl_mac16x16_32( as_K10[1][2], ps_deltaZ[2], i_tmp, 0);
   ps_deltaGBias[1] = utl_rShft32_16( i_tmp, 15); /*Q15*/
   i_tmp = 0;
   i_tmp = utl_mac16x16_32( as_K10[2][0], ps_deltaZ[0], i_tmp, 0);
   i_tmp = utl_mac16x16_32( as_K10[2][1], ps_deltaZ[1], i_tmp, 0);
   i_tmp = utl_mac16x16_32( as_K10[2][2], ps_deltaZ[2], i_tmp, 0);
   ps_deltaGBias[2] = utl_rShft32_16( i_tmp, 15); /*Q15*/

   i_tmp = 0;
   i_tmp = utl_mac16x16_32( as_K20[0][0], ps_deltaZ[0], i_tmp, 0);
   i_tmp = utl_mac16x16_32( as_K20[0][1], ps_deltaZ[1], i_tmp, 0);
   i_tmp = utl_mac16x16_32( as_K20[0][2], ps_deltaZ[2], i_tmp, 0);
   ps_deltaGScale[0] = utl_rShft32_16( i_tmp, 15); /*Q15*/
   i_tmp = 0;
   i_tmp = utl_mac16x16_32( as_K20[1][0], ps_deltaZ[0], i_tmp, 0);
   i_tmp = utl_mac16x16_32( as_K20[1][1], ps_deltaZ[1], i_tmp, 0);
   i_tmp = utl_mac16x16_32( as_K20[1][2], ps_deltaZ[2], i_tmp, 0);
   ps_deltaGScale[1] = utl_rShft32_16( i_tmp, 15); /*Q15*/
   i_tmp = 0;
   i_tmp = utl_mac16x16_32( as_K20[2][0], ps_deltaZ[0], i_tmp, 0);
   i_tmp = utl_mac16x16_32( as_K20[2][1], ps_deltaZ[1], i_tmp, 0);
   i_tmp = utl_mac16x16_32( as_K20[2][2], ps_deltaZ[2], i_tmp, 0);
   ps_deltaGScale[2] = utl_rShft32_16( i_tmp, 15); /*Q15*/

   /*------------------------------------------------------------------------*
    * Correct the error covariance matrix
    * P = (eye(9)-K*H)*P;
    * [1] eq 3.17
    *------------------------------------------------------------------------*/
   utl_matMult16x16_16( &as_K00[0][0], /*Q15*/
                        3,
                        3,
                        &as_H00[0][0], /*Q15*/
                        3,
                        3,
                        &as_tmp1[0][0],/*Q15*/
                        0,
                        15);

   utl_matMult16x16_16( &as_K10[0][0], /*Q15*/
                        3,
                        3,
                        &as_H00[0][0], /*Q15*/
                        3,
                        3,
                        &as_tmp2[0][0],/*Q15*/
                        0,
                        15);

   utl_matMult16x16_16( &as_K20[0][0], /*Q15*/
                        3,
                        3,
                        &as_H00[0][0], /*Q15*/
                        3,
                        3,
                        &as_tmp3[0][0],/*Q15*/
                        0,
                        15);

   memset( (void *)as_eyeMinusKH, 0, sizeof( as_eyeMinusKH));
   for( c_count = 0; c_count < 9; c_count++)
      as_eyeMinusKH[c_count][c_count] = 32767;

   as_eyeMinusKH[0][0] -= as_tmp1[0][0];
   as_eyeMinusKH[0][1] -= as_tmp1[0][1];
   as_eyeMinusKH[0][2] -= as_tmp1[0][2];
   as_eyeMinusKH[1][0] -= as_tmp1[1][0];
   as_eyeMinusKH[1][1] -= as_tmp1[1][1];
   as_eyeMinusKH[1][2] -= as_tmp1[1][2];
   as_eyeMinusKH[2][0] -= as_tmp1[2][0];
   as_eyeMinusKH[2][1] -= as_tmp1[2][1];
   as_eyeMinusKH[2][2] -= as_tmp1[2][2];

   as_eyeMinusKH[3][0] -= as_tmp2[0][0];
   as_eyeMinusKH[3][1] -= as_tmp2[0][1];
   as_eyeMinusKH[3][2] -= as_tmp2[0][2];
   as_eyeMinusKH[4][0] -= as_tmp2[1][0];
   as_eyeMinusKH[4][1] -= as_tmp2[1][1];
   as_eyeMinusKH[4][2] -= as_tmp2[1][2];
   as_eyeMinusKH[5][0] -= as_tmp2[2][0];
   as_eyeMinusKH[5][1] -= as_tmp2[2][1];
   as_eyeMinusKH[5][2] -= as_tmp2[2][2];

   as_eyeMinusKH[6][0] -= as_tmp3[0][0];
   as_eyeMinusKH[6][1] -= as_tmp3[0][1];
   as_eyeMinusKH[6][2] -= as_tmp3[0][2];
   as_eyeMinusKH[7][0] -= as_tmp3[1][0];
   as_eyeMinusKH[7][1] -= as_tmp3[1][1];
   as_eyeMinusKH[7][2] -= as_tmp3[1][2];
   as_eyeMinusKH[8][0] -= as_tmp3[2][0];
   as_eyeMinusKH[8][1] -= as_tmp3[2][1];
   as_eyeMinusKH[8][2] -= as_tmp3[2][2];

   memcpy( (void *)as_tmpP, (void *)gt_insDev.as_P, sizeof( as_tmpP));
   utl_matMult16x16_16( &as_eyeMinusKH[0][0],  /*Q15*/
                        9,
                        9,
                        &as_tmpP[0][0],        /*Q15*/
                        9,
                        9,
                        &gt_insDev.as_P[0][0], /*Q15*/
                        2,
                        15);

   for( c_index = 0; c_index < 9; c_index++)
   {
      /*Ignore rounding errors, anything greater is a loss of lock*/
      if( (gt_insDev.as_P[c_index][c_index] < 0) && (gt_insDev.as_P[c_index][c_index] > -5))
      {
         gt_insDev.as_P[c_index][c_index] = 0;
      }

   }/*End for( c_index = 0; c_index < 9; c_index++)*/

}/*End eSKalmanFilterCorrect*/

/*---------------------------------------------------------------------------*
 * Form the error-state Kalman filter measurement innovation based on the
 * gravity vector and magnetic field vector, where deltaZ is the attitude error
 * measurement based on the set of 'unlabeled' eq's in section 10.5.1.2 and
 * 10.5.1.3 of [3].
 *---------------------------------------------------------------------------*/
static inline void formMeasInnov( int16_t *ps_acc,
                                  int16_t *ps_mag,
                                  int16_t *ps_deltaZ)
{
   int32_t i_temp;
   int16_t s_gravX; /*x gravity vector in the navigation frame*/
   int16_t s_gravY; /*y gravity vector in the navigation frame*/
   int16_t s_magY;  /*y magnetic field vector in the navigation frame*/
   int16_t as_acc2[3];
   int16_t as_mag2[3];

   /*------------------------------------------------------------------------*
    * In order for the error-state Kalman filter to work properly, the attitude
    * error innovation vector must be scaled to the same reference as that of
    * the gyro. Therefore, since 1 rad = 32767 then the x and y accelerometer
    * vectors must be scaled so that 1g = 32767. The same holds true for the
    * magnetic field vector, 1 tesla should = 32767. For the time being
    * scale these values so that they are Q0.12 - if gt_insDev.t_accel.
    * s_calGravity = gt_insDev.t_mag.s_calMagFieldStr then this step can
    * be ignored.
    *------------------------------------------------------------------------*/
   as_acc2[0] = utl_div16x16_16( ps_acc[0], gt_insDev.t_accel.s_calGravity,
   12); /*Q12*/
   as_acc2[1] = utl_div16x16_16( ps_acc[1], gt_insDev.t_accel.s_calGravity,
   12); /*Q12*/
   as_acc2[2] = utl_div16x16_16( ps_acc[2], gt_insDev.t_accel.s_calGravity,
   12); /*Q12*/

   as_mag2[0] = utl_div16x16_16( ps_mag[0], gt_insDev.t_mag.s_calMagFieldStr,
   12); /*Q12*/
   as_mag2[1] = utl_div16x16_16( ps_mag[1], gt_insDev.t_mag.s_calMagFieldStr,
   12); /*Q12*/
   as_mag2[2] = utl_div16x16_16( ps_mag[2], gt_insDev.t_mag.s_calMagFieldStr,
   12); /*Q12*/

   /*------------------------------------------------------------------------*
    * Calculate the x and y gravity vector components in the navigation frame.
    *------------------------------------------------------------------------*/
   i_temp = 0;
   i_temp = utl_mac16x16_32( gt_insDev.as_dcm[0][0], as_acc2[0], i_temp, 1);
   i_temp = utl_mac16x16_32( gt_insDev.as_dcm[0][1], as_acc2[1], i_temp, 1);
   s_gravX = utl_rShft32_16( utl_mac16x16_32( gt_insDev.as_dcm[0][2],
   as_acc2[2], i_temp, 1),  11); /*Q12 to Q15*/

   i_temp = 0;
   i_temp = utl_mac16x16_32( gt_insDev.as_dcm[1][0], as_acc2[0], i_temp, 1);
   i_temp = utl_mac16x16_32( gt_insDev.as_dcm[1][1], as_acc2[1], i_temp, 1);
   s_gravY = utl_rShft32_16( utl_mac16x16_32( gt_insDev.as_dcm[1][2],
   as_acc2[2], i_temp, 1), 11); /*Q12 to Q15*/

   /*------------------------------------------------------------------------*
    * Calculate the y magnetic field vector component in the navigation frame.
    *------------------------------------------------------------------------*/
   i_temp = 0;
   i_temp = utl_mac16x16_32( gt_insDev.as_dcm[1][0], as_mag2[0], i_temp, 1);
   i_temp = utl_mac16x16_32( gt_insDev.as_dcm[1][1], as_mag2[1], i_temp, 1);
   s_magY = utl_rShft32_16( utl_mac16x16_32( gt_insDev.as_dcm[1][2],
   as_mag2[2], i_temp, 1), 11); /*Q12 to Q15*/

   /*------------------------------------------------------------------------*
    * Form the error innovation vector which is the x and y components of
    * g_calibrated - g_nav and the y component of m_calibrated - m_nav.
    *------------------------------------------------------------------------*/
   ps_deltaZ[0] = -s_gravX;
   ps_deltaZ[1] = -s_gravY;
   ps_deltaZ[2] = -s_magY;

}/*End formMeasInnov*/

/*---------------------------------------------------------------------------*
 * Due to the fact that the code is not setup to handle phase changes > pi,
 * Limit the amount of phase rotation detected by the gyro in a given 'dt'.
 *---------------------------------------------------------------------------*/
static inline int16_t limitGyroRot( int32_t i_rot)
{
   if( i_rot > UTL_MATH_FXDPNT_PI)
      return UTL_MATH_FXDPNT_PI;
   else if( i_rot < UTL_MATH_FXDPNT_NEGATIVE_PI)
      return UTL_MATH_FXDPNT_NEGATIVE_PI;

   return (int16_t)i_rot;

}/*End limitGyroRot*/

/*---------------------------------------------------------------------------*
 * Normalize the DCM according to eq's 18, 19, 20, and 21 of [4]
 *---------------------------------------------------------------------------*/
static inline void dcmNormalize( void)
{
   int16_t s_errOvrTwo = 0;
   int16_t as_xOrth[3];
   int16_t as_yOrth[3];
   int16_t as_zOrth[3];
   int32_t i_scalerX;
   int32_t i_scalerY;
   int32_t i_scalerZ;
   int32_t i_temp = 0;

   i_temp = 0;
   i_temp = utl_mac16x16_32( gt_insDev.as_dcm[0][0], gt_insDev.as_dcm[1][0],
   i_temp, 2);
   i_temp = utl_mac16x16_32( gt_insDev.as_dcm[0][1], gt_insDev.as_dcm[1][1],
   i_temp, 2);
   i_temp = utl_mac16x16_32( gt_insDev.as_dcm[0][2], gt_insDev.as_dcm[1][2],
   i_temp, 2);
   s_errOvrTwo = utl_rShft32_16( i_temp, 14); /*Q15*/

   i_temp = (int32_t)gt_insDev.as_dcm[0][0] << 15;
   i_temp = utl_mac16x16_32( -s_errOvrTwo, gt_insDev.as_dcm[1][0], i_temp, 0);
   as_xOrth[0] = utl_rShft32_16( i_temp, 16); /*Q14*/
   i_temp = (int32_t)gt_insDev.as_dcm[0][1] << 15;
   i_temp = utl_mac16x16_32( -s_errOvrTwo, gt_insDev.as_dcm[1][1], i_temp, 0);
   as_xOrth[1] = utl_rShft32_16( i_temp, 16); /*Q14*/
   i_temp = (int32_t)gt_insDev.as_dcm[0][2] << 15;
   i_temp = utl_mac16x16_32( -s_errOvrTwo, gt_insDev.as_dcm[1][2], i_temp, 0);
   as_xOrth[2] = utl_rShft32_16( i_temp, 16); /*Q14*/

   i_temp = (int32_t)gt_insDev.as_dcm[1][0] << 15;
   i_temp = utl_mac16x16_32( -s_errOvrTwo, gt_insDev.as_dcm[0][0], i_temp, 0);
   as_yOrth[0] = utl_rShft32_16( i_temp, 16); /*Q14*/
   i_temp = (int32_t)gt_insDev.as_dcm[1][1] << 15;
   i_temp = utl_mac16x16_32( -s_errOvrTwo, gt_insDev.as_dcm[0][1], i_temp, 0);
   as_yOrth[1] = utl_rShft32_16( i_temp, 16); /*Q14*/
   i_temp = (int32_t)gt_insDev.as_dcm[1][2] << 15;
   i_temp = utl_mac16x16_32( -s_errOvrTwo, gt_insDev.as_dcm[0][2], i_temp, 0);
   as_yOrth[2] = utl_rShft32_16( i_temp, 16); /*Q14*/

   i_temp = 0;
   i_temp = utl_mac16x16_32( as_xOrth[1], as_yOrth[2], i_temp, 0);
   i_temp = utl_mac16x16_32( -as_xOrth[2], as_yOrth[1], i_temp, 0);
   as_zOrth[0] = utl_rShft32_16( i_temp, 14); /*Q14*/
   i_temp = 0;
   i_temp = utl_mac16x16_32( as_xOrth[2], as_yOrth[0], i_temp, 0);
   i_temp = utl_mac16x16_32( -as_xOrth[0], as_yOrth[2], i_temp, 0);
   as_zOrth[1] = utl_rShft32_16( i_temp, 14); /*Q14*/
   i_temp = 0;
   i_temp = utl_mac16x16_32( as_xOrth[0], as_yOrth[1], i_temp, 0);
   i_temp = utl_mac16x16_32( -as_xOrth[1], as_yOrth[0], i_temp, 0);
   as_zOrth[2] = utl_rShft32_16( i_temp, 14); /*Q14*/

   i_scalerX = utl_vMult16x16_32( as_xOrth, as_xOrth, 0, 3, 1, 13);   /*Q14*/
   i_scalerX = utl_hardLimit32_32(((int32_t)16383*(int32_t)3 -
   i_scalerX) >> 1, 15); /*Q14 number*/

   i_scalerY = utl_vMult16x16_32( as_yOrth, as_yOrth, 0, 3, 1, 13);   /*Q14*/
   i_scalerY = utl_hardLimit32_32(((int32_t)16383*(int32_t)3 -
   i_scalerY) >> 1, 15); /*Q14 number*/

   i_scalerZ = utl_vMult16x16_32( as_zOrth, as_zOrth, 0, 3, 1, 13);   /*Q14*/
   i_scalerZ = utl_hardLimit32_32(((int32_t)16383*(int32_t)3 -
   i_scalerZ) >> 1, 15); /*Q14 number*/

   gt_insDev.as_dcm[0][0] = utl_mult16x16_32( i_scalerX, as_xOrth[0], \
   13); /*Q14 to Q15*/
   gt_insDev.as_dcm[0][1] = utl_mult16x16_32( i_scalerX, as_xOrth[1], \
   13); /*Q14 to Q15*/
   gt_insDev.as_dcm[0][2] = utl_mult16x16_32( i_scalerX, as_xOrth[2], \
   13); /*Q14 to Q15*/

   gt_insDev.as_dcm[1][0] = utl_mult16x16_32( i_scalerY, as_yOrth[0], \
   13); /*Q14 to Q15*/
   gt_insDev.as_dcm[1][1] = utl_mult16x16_32( i_scalerY, as_yOrth[1], \
   13); /*Q14 to Q15*/
   gt_insDev.as_dcm[1][2] = utl_mult16x16_32( i_scalerY, as_yOrth[2], \
   13); /*Q14 to Q15*/

   gt_insDev.as_dcm[2][0] = utl_mult16x16_32( i_scalerZ, as_zOrth[0], \
   13); /*Q14 to Q15*/
   gt_insDev.as_dcm[2][1] = utl_mult16x16_32( i_scalerZ, as_zOrth[1], \
   13); /*Q14 to Q15*/
   gt_insDev.as_dcm[2][2] = utl_mult16x16_32( i_scalerZ, as_zOrth[2], \
   13); /*Q14 to Q15*/

}/*End dcmNormalize*/

/*---------------------------------------------------------------------------*
 * Perform DCM integration according to eq 5.6 and 5.7 of [1]
 *---------------------------------------------------------------------------*/
static inline void dcmIntegration( int32_t *pi_gyr,
                                   int16_t *ps_dPhase,
                                   int16_t s_dt)
{
   int16_t as_skew[3][3];
   int16_t as_tempDcm[3][3];
   int8_t c_row;
   int8_t c_col;

   ps_dPhase[0] = limitGyroRot( utl_mult32x32_32( pi_gyr[0], \
   (int32_t)s_dt, 16)); /*Q14*/
   ps_dPhase[1] = limitGyroRot( utl_mult32x32_32( pi_gyr[1], \
   (int32_t)s_dt, 16)); /*Q14*/
   ps_dPhase[2] = limitGyroRot( utl_mult32x32_32( pi_gyr[2], \
   (int32_t)s_dt, 16)); /*Q14*/

   as_skew[0][0] = 16383;//32767;
   as_skew[0][1] = -ps_dPhase[2];
   as_skew[0][2] = ps_dPhase[1];

   as_skew[1][0] = ps_dPhase[2];
   as_skew[1][1] = 16383;//32767;
   as_skew[1][2] = -ps_dPhase[0];

   as_skew[2][0] = -ps_dPhase[1];
   as_skew[2][1] = ps_dPhase[0];
   as_skew[2][2] = 16383;//32767;

   /*------------------------------------------------------------------------*
    * Scale the DCM so that its a Q1.14 value, in order to prevent an
    * overflow in the matrix multiply routine.
    *------------------------------------------------------------------------*/
   as_tempDcm[0][0] = gt_insDev.as_dcm[0][0] >> 1;
   as_tempDcm[0][1] = gt_insDev.as_dcm[0][1] >> 1;
   as_tempDcm[0][2] = gt_insDev.as_dcm[0][2] >> 1;
   as_tempDcm[1][0] = gt_insDev.as_dcm[1][0] >> 1;
   as_tempDcm[1][1] = gt_insDev.as_dcm[1][1] >> 1;
   as_tempDcm[1][2] = gt_insDev.as_dcm[1][2] >> 1;
   as_tempDcm[2][0] = gt_insDev.as_dcm[2][0] >> 1;
   as_tempDcm[2][1] = gt_insDev.as_dcm[2][1] >> 1;
   as_tempDcm[2][2] = gt_insDev.as_dcm[2][2] >> 1;

   utl_matMult16x16_16( &as_tempDcm[0][0],
                        3,
                        3,
                        &as_skew[0][0],
                        3,
                        3,
                        &gt_insDev.as_dcm[0][0],
                        0,
                        14);

   /*------------------------------------------------------------------------*
    * Hard limit and scale the DCM back to a Q15 number...
    *------------------------------------------------------------------------*/
   for( c_row = 0; c_row < 3; c_row++)
   {
      for( c_col = 0; c_col < 3; c_col++)
      {
         if( gt_insDev.as_dcm[c_row][c_col] > 16383)
            gt_insDev.as_dcm[c_row][c_col] = 16383;
         else if( gt_insDev.as_dcm[c_row][c_col] < -16384)
            gt_insDev.as_dcm[c_row][c_col] = -16384;

         gt_insDev.as_dcm[c_row][c_col] = gt_insDev.as_dcm[c_row][c_col] << 1;

      }
   }

}/*End dcmIntegration*/

/*---------------------------------------------------------------------------*
 * Perform DCM correction according to eq eq 12.7 of [1]
 *---------------------------------------------------------------------------*/
static inline void dcmCorrect( int16_t *ps_err)
{
   int16_t as_skew[3][3];
   int16_t as_tempDcm[3][3];
   int8_t c_row;
   int8_t c_col;

   as_skew[0][0] = 16383;
   as_skew[0][1] = ps_err[2]>>1;
   as_skew[0][2] = -ps_err[1]>>1;
   as_skew[1][0] = -ps_err[2]>>1;
   as_skew[1][1] = 16383;
   as_skew[1][2] = ps_err[0]>>1;
   as_skew[2][0] = ps_err[1]>>1;
   as_skew[2][1] = -ps_err[0]>>1;
   as_skew[2][2] = 16383;

   /*------------------------------------------------------------------------*
    * Scale the DCM so that its a Q1.14 value, in order to prevent an
    * overflow in the matrix multiply routine.
    *------------------------------------------------------------------------*/
   as_tempDcm[0][0] = gt_insDev.as_dcm[0][0] >> 1;
   as_tempDcm[0][1] = gt_insDev.as_dcm[0][1] >> 1;
   as_tempDcm[0][2] = gt_insDev.as_dcm[0][2] >> 1;
   as_tempDcm[1][0] = gt_insDev.as_dcm[1][0] >> 1;
   as_tempDcm[1][1] = gt_insDev.as_dcm[1][1] >> 1;
   as_tempDcm[1][2] = gt_insDev.as_dcm[1][2] >> 1;
   as_tempDcm[2][0] = gt_insDev.as_dcm[2][0] >> 1;
   as_tempDcm[2][1] = gt_insDev.as_dcm[2][1] >> 1;
   as_tempDcm[2][2] = gt_insDev.as_dcm[2][2] >> 1;

   utl_matMult16x16_16( &as_skew[0][0],
                        3,
                        3,
                        &as_tempDcm[0][0],
                        3,
                        3,
                        &gt_insDev.as_dcm[0][0],
                        0,
                        14);

   /*------------------------------------------------------------------------*
    * Hard limit and scale the DCM back to a Q15 number...
    *------------------------------------------------------------------------*/
   for( c_row = 0; c_row < 3; c_row++)
   {
      for( c_col = 0; c_col < 3; c_col++)
      {
         if( gt_insDev.as_dcm[c_row][c_col] > 16383)
            gt_insDev.as_dcm[c_row][c_col] = 16383;
         else if( gt_insDev.as_dcm[c_row][c_col] < -16384)
            gt_insDev.as_dcm[c_row][c_col] = -16384;

         gt_insDev.as_dcm[c_row][c_col] = gt_insDev.as_dcm[c_row][c_col] << 1;
      }
   }

}/*End dcmIntegration*/

/*---------------------------------------------------------------------------*
 * Perform DCM initialization according to eq 2.15 of [1]
 *---------------------------------------------------------------------------*/
static void dcmInit( int16_t *ps_att)
{
   int16_t s_roll  = ps_att[0];
   int16_t s_pitch = ps_att[1];
   int16_t s_yaw   = ps_att[2];
   int16_t s_temp;
   int32_t i_temp2;

   gt_insDev.as_dcm[0][0] = utl_mult16x16_16( utl_cos16_16(s_pitch),
   utl_cos16_16(s_yaw), 15);

   s_temp = utl_mult16x16_16( utl_sin16_16(s_roll), utl_sin16_16(s_pitch), 15);
   i_temp2 = 0;
   i_temp2 = -utl_mac16x16_32( utl_cos16_16(s_roll), utl_sin16_16(s_yaw), i_temp2, 1);
   i_temp2 = utl_mac16x16_32( s_temp, utl_cos16_16(s_yaw), i_temp2, 1);
   gt_insDev.as_dcm[0][1] = utl_rShft32_16( i_temp2, 14);

   s_temp = utl_mult16x16_16( utl_cos16_16(s_roll), utl_sin16_16(s_pitch), 15);
   i_temp2 = 0;
   i_temp2 = utl_mac16x16_32( utl_sin16_16(s_roll), utl_sin16_16(s_yaw), i_temp2, 1);
   i_temp2 = utl_mac16x16_32( s_temp, utl_cos16_16(s_yaw), i_temp2, 1);
   gt_insDev.as_dcm[0][2] = utl_rShft32_16( i_temp2, 14);

   gt_insDev.as_dcm[1][0] = utl_mult16x16_16( utl_cos16_16(s_pitch),
   utl_sin16_16(s_yaw), 15);

   s_temp = utl_mult16x16_16( utl_sin16_16(s_roll), utl_sin16_16(s_pitch), 15);
   i_temp2 = 0;
   i_temp2 = utl_mac16x16_32( utl_cos16_16(s_roll), utl_cos16_16(s_yaw), i_temp2, 1);
   i_temp2 = utl_mac16x16_32( s_temp, utl_sin16_16(s_yaw), i_temp2, 1);
   gt_insDev.as_dcm[1][1] = utl_rShft32_16( i_temp2, 14);

   s_temp = utl_mult16x16_16( utl_cos16_16(s_roll), utl_sin16_16(s_pitch), 15);
   i_temp2 = 0;
   i_temp2 = -utl_mac16x16_32( utl_sin16_16(s_roll), utl_cos16_16(s_yaw), i_temp2, 1);
   i_temp2 = utl_mac16x16_32( s_temp, utl_sin16_16(s_yaw), i_temp2, 1);
   gt_insDev.as_dcm[1][2] = utl_rShft32_16( i_temp2, 14);

   gt_insDev.as_dcm[2][0] = -utl_sin16_16(s_pitch);
   gt_insDev.as_dcm[2][1] = utl_mult16x16_16( utl_sin16_16(s_roll),
   utl_cos16_16(s_pitch), 15);
   gt_insDev.as_dcm[2][2] = utl_mult16x16_16( utl_cos16_16(s_roll),
   utl_cos16_16(s_pitch), 15);

}/*End dcmInit*/

/*---------------------------------------------------------------------------*
 * Convert from DCM representation to Euler using eq 2.17 of [1]
 *---------------------------------------------------------------------------*/
static inline void dcmToEuler( int16_t *ps_att)
{
   int32_t i_temp;
   int32_t i_temp2;

   ps_att[0] = utl_atan2_16( gt_insDev.as_dcm[2][1], gt_insDev.as_dcm[2][2]);

   /*------------------------------------------------------------------------*
    * In order to get accurate results when the 'x' is small in the sqrt x,
    * the value of x is shifted up by 10.
    * y = sqrt( 1 - dcm[2][0]*dcm[2][0])
    *------------------------------------------------------------------------*/
   i_temp2 = utl_mult16x16_32( gt_insDev.as_dcm[2][0], gt_insDev.as_dcm[2][0],
   5);

   i_temp = (((int32_t)1 << 25) - 1) - i_temp2;

   i_temp = utl_sqrt32_32( i_temp, 15) >> 5;

   if( i_temp > 32767)
      i_temp = 32767;

   ps_att[1] = -utl_atan2_16( gt_insDev.as_dcm[2][0], i_temp);
   ps_att[2] = utl_atan2_16( gt_insDev.as_dcm[1][0], gt_insDev.as_dcm[0][0]);

}/*End dcmToEuler*/

static inline void gyroDegToRad( int16_t *ps_gyDeg,
                                 int32_t *pi_gyRad)
{

   /*------------------------------------------------------------------------*
    * The raw gyro readings for all three axis's, where the x, y, and z
    * readings are stored in index 0, 1, and 2 respectively. The ITG-3200
    * specifies that the gyro has a max rating of +/- 2000 deg/sec given by
    * 32767/-32768 in gyro fixed-point (gfp) notation. This value need to be
    * converted to rad/sec where 1 rad = 32767. This conversion can be
    * accomplished by the following:
    *    y = (x gfp)*(2000 deg/sec)/(32767 gfp)*(3.14159*32767 rad)/(180 deg)
    *      = (x gfp)*((2000*3.14159)/180) (rad/sec)
    *      = ((x gfp)*INS_GYRO_CONV_FACTOR) >> 9, where
    *    INS_GYRO_CONV_FACTOR = ((2000*3.14159)/180)*512
    * Naturally, there is a loss of precision with this conversion due to the
    * scaling, this error will be corrected by the Kalman filter's estimate of
    * the axis scale factors. Additionally, the truncation is treated as
    * additional system (process) noise within the Kalman filter algorithm.
    * Q16.15 number
    *------------------------------------------------------------------------*/
   pi_gyRad[0] = utl_mult32x32_32( (int32_t)ps_gyDeg[0],
   (int32_t)INS_GYRO_CONV_FACTOR, 9);
   pi_gyRad[1] = utl_mult32x32_32( (int32_t)ps_gyDeg[1],
   (int32_t)INS_GYRO_CONV_FACTOR, 9);
   pi_gyRad[2] = utl_mult32x32_32( (int32_t)ps_gyDeg[2],
   (int32_t)INS_GYRO_CONV_FACTOR, 9);

}/*End gyroDegToRad*/

static inline void readSensors( int16_t *ps_mag,
                                int16_t *ps_gyr,
                                int16_t *ps_acc)
{
   getMagMeas( ps_mag);
	getGyroMeas( ps_gyr);
	getAccelMeas( ps_acc);

}/*End readSensors*/

/*---------------------------------------------------------------------------*
 * Returns the uncorrected attitude as determined by the accelerometer and
 * magnetometer using eq's 10.14 and 10.15 of [3] and eq 10.6 of [1].
 *---------------------------------------------------------------------------*/
static inline void getRawAttitude( int16_t *ps_acc,
                                   int16_t *ps_mag,
                                   int16_t *ps_att)
{
   int16_t s_cosRoll;
   int16_t s_sinRoll;
   int16_t s_cosPitch;
   int16_t s_sinPitch;
   int16_t s_magX;
   int16_t s_magY;
   int16_t s_temp1;
   int32_t i_temp2 = 0;

   /*------------------------------------------------------------------------*
    * Calculate the roll...
    *------------------------------------------------------------------------*/
   ps_att[0] = utl_atan2_16( ps_acc[1], ps_acc[2]);

   i_temp2 = utl_mac16x16_32( ps_acc[1], ps_acc[1], i_temp2, 0);
   i_temp2 = utl_mac16x16_32( ps_acc[2], ps_acc[2], i_temp2, 0);

   i_temp2 = (int32_t)(i_temp2 >> 5);

   i_temp2 = utl_sqrt32_32( i_temp2, 15) >> 5;

   /*------------------------------------------------------------------------*
    * Calculate the pitch...
    *------------------------------------------------------------------------*/
   ps_att[1] = utl_atan2_16( -ps_acc[0], (int16_t)i_temp2);

   s_cosRoll = utl_cos16_16( ps_att[0]);
   s_sinRoll = utl_sin16_16( ps_att[0]);

   s_cosPitch = utl_cos16_16( ps_att[1]);
   s_sinPitch = utl_sin16_16( ps_att[1]);

   i_temp2 = 0;
   i_temp2 = utl_mac16x16_32( ps_mag[1], s_sinRoll, i_temp2, 0);
   i_temp2 = utl_mac16x16_32( ps_mag[2], s_cosRoll, i_temp2, 0);
   s_temp1 = utl_rShft32_16( i_temp2, 15); /*Q18*/

   i_temp2 = 0;
   i_temp2 = utl_mac16x16_32( ps_mag[0], s_cosPitch, i_temp2, 0);
   i_temp2 = utl_mac16x16_32( s_sinPitch, s_temp1, i_temp2, 0);
   s_magX = utl_rShft32_16( i_temp2, 9); /*Q21*/

   i_temp2 = 0;
   i_temp2 = utl_mac16x16_32( -ps_mag[1], s_cosRoll, i_temp2, 0);
   i_temp2 = utl_mac16x16_32( ps_mag[2], s_sinRoll, i_temp2, 0);
   s_magY = utl_rShft32_16(i_temp2, 9);    /*Q21*/

   /*------------------------------------------------------------------------*
    * Calculate the yaw...
    *------------------------------------------------------------------------*/
   ps_att[2] = utl_atan2_16( s_magY, s_magX);

}/*End getRawAttitude*/

static inline void calcAvrDynamics( int16_t *ps_acc,
                                    int16_t *ps_dPhase)
{
   int32_t i_sum = 0;
   int32_t i_deltaPhase = 0;
   int32_t i_tmp = 0;
   int32_t i_spForce = 0;

   /*-------------------------------------------------------------------------*
    * specific force = abs(sqrt( x^2+y^2+z^2) - gravity)
    *-------------------------------------------------------------------------*/
   i_sum = 0;
   i_sum = utl_mac16x16_32( ps_acc[0], ps_acc[0], i_sum, 0);
   i_sum = utl_mac16x16_32( ps_acc[1], ps_acc[1], i_sum, 0);
   i_sum = utl_mac16x16_32( ps_acc[2], ps_acc[2], i_sum, 0);
   i_spForce = utl_abs32_32( utl_sqrt32_32( i_sum, 0) - gt_insDev.t_accel.
   s_calGravity);

   /*-------------------------------------------------------------------------*
    * Hard limit to maintain 16-bit math...
    *-------------------------------------------------------------------------*/
   if( i_spForce > 32767)
      i_spForce = 32767;

   /*-------------------------------------------------------------------------*
    * Average the specific force over a period of f = 1/(1 - alpha) frames
    * where alpha = .8, or 5 frames.
    * Q11
    *-------------------------------------------------------------------------*/
   i_tmp = 0;
   i_tmp = utl_mac16x16_32( gt_insDev.t_accel.s_avrSpecForce, 1637, i_tmp, 0);
   i_tmp = utl_mac16x16_32( i_spForce, 410, i_tmp, 0);

   gt_insDev.t_accel.s_avrSpecForce = utl_rShft32_16( i_tmp, 11);

   /*-------------------------------------------------------------------------*
    * magnitude of phase change = sqrt( x^2+y^2+z^2)
    *-------------------------------------------------------------------------*/
   i_deltaPhase = 0;
   i_deltaPhase = utl_mac16x16_32( ps_dPhase[0], ps_dPhase[0], i_deltaPhase, 0);
   i_deltaPhase = utl_mac16x16_32( ps_dPhase[1], ps_dPhase[1], i_deltaPhase, 0);
   i_deltaPhase = utl_mac16x16_32( ps_dPhase[2], ps_dPhase[2], i_deltaPhase, 0);
   i_deltaPhase = utl_rShft32_16( i_deltaPhase, 14);
   i_deltaPhase = utl_sqrt32_32( i_deltaPhase, 14);

   /*-------------------------------------------------------------------------*
    * Average the phase change over a period of f = 1/(1 - alpha) frames
    * where alpha = .8, or 5 frames.
    * Q11
    *-------------------------------------------------------------------------*/
   i_tmp = 0;
   i_tmp = utl_mac16x16_32( gt_insDev.t_gyro.s_avrDPhase, 1637, i_tmp, 0);
   i_tmp = utl_mac16x16_32( i_deltaPhase, 410, i_tmp, 0);

   gt_insDev.t_gyro.s_avrDPhase = utl_rShft32_16( i_tmp, 11);

   if( gt_insDev.c_platMovingCount > 0)
      gt_insDev.c_platMovingCount -=1;

   if( (((gt_insDev.t_accel.s_avrSpecForce >
       (gt_insDev.t_accel.s_calGravity >> 2)) ||
       (i_spForce > (gt_insDev.t_accel.s_calGravity >> 2)) ||
       (i_deltaPhase > INS_MAX_DELTA_PHASE_RAD_FXDPT))
       && (gt_insDev.c_initialLockCount == INS_KALMAN_LOCK_WAIT)))
   {
      gt_insDev.c_platMovingCount = 2;
   }

}/*End calcAvrDynamics*/

static void getAttitudeInDeg( int16_t *ps_att,
                              int16_t *ps_attDeg)
{
   ps_attDeg[0] = utl_rShft32_16((int32_t)ps_att[0]*(int32_t)180, 15);
   ps_attDeg[1] = utl_rShft32_16((int32_t)ps_att[1]*(int32_t)180, 15);
   ps_attDeg[2] = utl_rShft32_16((int32_t)ps_att[2]*(int32_t)180, 15);

}/*End getAttitudeInDeg*/

static void applyCalibration( int16_t *ps_smp,
                              int16_t *ps_R,
                              int16_t *ps_scale,
                              int16_t *ps_bias)
{
   int16_t as_temp[3];
   int32_t i_sum;

   /*---------------------------------------------------------------------------*
    * Remove bias.
    *---------------------------------------------------------------------------*/
   ps_smp[0] -= ps_bias[0];
   ps_smp[1] -= ps_bias[1];
   ps_smp[2] -= ps_bias[2];

   /*---------------------------------------------------------------------------*
    * Rotate the sample to align with the calculated ellipsoid principle axis.
    *---------------------------------------------------------------------------*/
   i_sum = 0;
   i_sum = utl_mac16x16_32( ps_smp[0], ps_R[0], i_sum, 1);
   i_sum = utl_mac16x16_32( ps_smp[1], ps_R[1], i_sum, 1);
   i_sum = utl_mac16x16_32( ps_smp[2], ps_R[2], i_sum, 1);
   as_temp[0] = utl_rShft32_16( i_sum, 14);

   i_sum = 0;
   i_sum = utl_mac16x16_32( ps_smp[0], ps_R[1*3 + 0], i_sum, 1);
   i_sum = utl_mac16x16_32( ps_smp[1], ps_R[1*3 + 1], i_sum, 1);
   i_sum = utl_mac16x16_32( ps_smp[2], ps_R[1*3 + 2], i_sum, 1);
   as_temp[1] = utl_rShft32_16( i_sum, 14);

   i_sum = 0;
   i_sum = utl_mac16x16_32( ps_smp[0], ps_R[2*3 + 0], i_sum, 1);
   i_sum = utl_mac16x16_32( ps_smp[1], ps_R[2*3 + 1], i_sum, 1);
   i_sum = utl_mac16x16_32( ps_smp[2], ps_R[2*3 + 2], i_sum, 1);
   as_temp[2] = utl_rShft32_16( i_sum, 14);

   /*---------------------------------------------------------------------------*
    * Scale the sample so that its axis are aligned.
    *---------------------------------------------------------------------------*/
   as_temp[0] = utl_mult16x16_16( as_temp[0], ps_scale[0],
   INS_AXIS_SCALE_Q_FACTOR);
   as_temp[1] = utl_mult16x16_16( as_temp[1], ps_scale[1],
   INS_AXIS_SCALE_Q_FACTOR);
   as_temp[2] = utl_mult16x16_16( as_temp[2], ps_scale[2],
   INS_AXIS_SCALE_Q_FACTOR);

   /*---------------------------------------------------------------------------*
    * Rotate the sample back to its original position.
    *---------------------------------------------------------------------------*/
   i_sum = 0;
   i_sum = utl_mac16x16_32( as_temp[0], ps_R[0*3 + 0], i_sum, 1);
   i_sum = utl_mac16x16_32( as_temp[1], ps_R[1*3 + 0], i_sum, 1);
   i_sum = utl_mac16x16_32( as_temp[2], ps_R[2*3 + 0], i_sum, 1);
   ps_smp[0] = utl_rShft32_16( i_sum, 14);

   i_sum = 0;
   i_sum = utl_mac16x16_32( as_temp[0], ps_R[0*3 + 1], i_sum, 1);
   i_sum = utl_mac16x16_32( as_temp[1], ps_R[1*3 + 1], i_sum, 1);
   i_sum = utl_mac16x16_32( as_temp[2], ps_R[2*3 + 1], i_sum, 1);
   ps_smp[1] = utl_rShft32_16( i_sum, 14);

   i_sum = 0;
   i_sum = utl_mac16x16_32( as_temp[0], ps_R[0*3 + 2], i_sum, 1);
   i_sum = utl_mac16x16_32( as_temp[1], ps_R[1*3 + 2], i_sum, 1);
   i_sum = utl_mac16x16_32( as_temp[2], ps_R[2*3 + 2], i_sum, 1);
   ps_smp[2] = utl_rShft32_16( i_sum, 14);

}/*End applyCalibration*/

static inline void checkForTrackingErrors( void)
{
   uint8_t c_index;

   gt_insDev.b_lossOfLock = false;

   if( gt_insDev.c_initialLockCount == INS_KALMAN_LOCK_WAIT)
   {

      if( gt_insDev.as_res[0] > (((int32_t)(gt_insDev.ai_avrRes[0] >>
          INS_ATT_RES_VAR_EXTRA_RES))*(int32_t)INS_RES_MEAS_ERROR_SCALER))
         gt_insDev.b_rollMeasError = true;
      else
         gt_insDev.b_rollMeasError = false;

      if( gt_insDev.as_res[1] > (((int32_t)(gt_insDev.ai_avrRes[1] >>
          INS_ATT_RES_VAR_EXTRA_RES))*(int32_t)INS_RES_MEAS_ERROR_SCALER))
         gt_insDev.b_pitchMeasError = true;
      else
         gt_insDev.b_pitchMeasError = false;

      if( gt_insDev.as_res[2] > (((int32_t)(gt_insDev.ai_avrRes[2] >>
          INS_ATT_RES_VAR_EXTRA_RES))*(int32_t)INS_RES_MEAS_ERROR_SCALER))
         gt_insDev.b_yawMeasError = true;
      else
         gt_insDev.b_yawMeasError = false;

      /*-------------------------------------------------------------------------*
       * Has there been a measurement error for an extended period of time, if so
       * then declare loss-of-lock.
       *-------------------------------------------------------------------------*/
      if( gt_insDev.b_rollMeasError ||
          gt_insDev.b_pitchMeasError ||
          gt_insDev.b_yawMeasError)
      {
         gt_insDev.s_measErrorCount++;
         if( gt_insDev.s_measErrorCount == INS_KALMAN_LOSS_LOCK_COUNT)
            gt_insDev.b_lossOfLock = true;
      }
      else
         gt_insDev.s_measErrorCount = 0;

      for( c_index = 0; c_index < 9; c_index++)
      {
         /*---------------------------------------------------------------------*
          * The off-diagonal entities are the "covariances" and can be
          * negative, however the diagonal values are the variances of the
          * attitude, gyro bias, and gyro scale factor - do not allow these
          * to go negative.
          *---------------------------------------------------------------------*/
         if( gt_insDev.as_P[c_index][c_index] < 0)
         {
            gt_insDev.b_lossOfLock = true;
         }

      }

   }/*End if( gt_insDev.c_initialLockCount == INS_KALMAN_LOCK_WAIT)*/

}/*End checkForTrackingErrors*/

static void insReset( void)
{
   gt_insDev.t_gyro.as_bias[0] = gt_insDev.t_gyro.as_calBias[0];
   gt_insDev.t_gyro.as_bias[1] = gt_insDev.t_gyro.as_calBias[1];
   gt_insDev.t_gyro.as_bias[2] = gt_insDev.t_gyro.as_calBias[2];

   gt_insDev.t_gyro.as_scale[0] = ((int16_t)1 << INS_AXIS_SCALE_Q_FACTOR)
   - 1;
   gt_insDev.t_gyro.as_scale[1] = ((int16_t)1 << INS_AXIS_SCALE_Q_FACTOR)
   - 1;
   gt_insDev.t_gyro.as_scale[2] = ((int16_t)1 << INS_AXIS_SCALE_Q_FACTOR)
   - 1;

   gt_insDev.t_accel.s_avrSpecForce = 0;
   gt_insDev.t_gyro.s_avrDPhase = 0;
   gt_insDev.c_platMovingCount = 0;
   gt_insDev.c_initialLockCount = 0;
   gt_insDev.b_rollMeasError = false;
   gt_insDev.b_pitchMeasError = false;
   gt_insDev.b_yawMeasError = false;
   gt_insDev.b_lossOfLock = false;
   gt_insDev.s_measErrorCount = 0;

   gt_insDev.ai_avrRes[0] = 0;
   gt_insDev.ai_avrRes[1] = 0;
   gt_insDev.ai_avrRes[2] = 0;

   dcmInit( gt_insDev.as_rawAtt);
   eSKalmanFilterInit();

}/*End insReset*/

static void dcmUpdate( void)
{
   static int16_t as_mag[3] = {0,0,0};
   static int16_t as_gyr[3] = {0,0,0}; /*Raw gyro sensor readings in degrees*/
   static int32_t ai_gyr[3] = {0,0,0}; /*Gyro readings converted to rad/sec where pi rad =
                                         32767*/
   int16_t as_acc[3];
   int16_t as_deltaZ[3]; /*Measurement innovation*/
   int16_t as_deltaAtt[3]; /*Kalman estimated attitude error*/
   int16_t as_deltaGScale[3]; /*Kalman estimated gyro scale factor error*/
   int16_t as_deltaGBias[3]; /*Kalman estimated gyro bias error*/
   int16_t as_res[3]; /*Difference between the raw and DCM attitude*/
   int16_t as_dPhase[3]; /*The amount of gyro movement over the last dt.
                           Q1.14*/
   int16_t s_dt; /*AHRS update rate*/
   t_sysTime t_time;
   int32_t i_dt;
   int32_t i_sum;

   /*------------------------------------------------------------------------*
    * Acquire new sensor data.
    *------------------------------------------------------------------------*/
   readSensors( as_mag,
                as_gyr,
                as_acc);

   /*------------------------------------------------------------------------*
    * Convert the gyro measurement into rad/sec where 1 rad = 32767.
    *------------------------------------------------------------------------*/
   gyroDegToRad( as_gyr,
                 ai_gyr);

   /*------------------------------------------------------------------------*
    * If there has been a valid magnetometer calibration compensate for axis
    * misalignment and bias...
    *------------------------------------------------------------------------*/
   if( gt_insDev.t_mag.t_cal == INS_CAL_COMPLETE)
   {
      applyCalibration( as_mag,
                        &gt_insDev.t_mag.as_R[0][0],
                        gt_insDev.t_mag.as_scale,
                        gt_insDev.t_mag.as_bias);

   }/*End if( gt_insDev.t_mag.t_cal == INS_CAL_COMPLETE)*/

   /*------------------------------------------------------------------------*
    * If there has been a valid accelerometer calibration compensate for axis
    * misalignment and bias...
    *------------------------------------------------------------------------*/
   if( gt_insDev.t_accel.t_cal == INS_CAL_COMPLETE)
   {
      applyCalibration( as_acc,
                        &gt_insDev.t_accel.as_R[0][0],
                        gt_insDev.t_accel.as_scale,
                        gt_insDev.t_accel.as_bias);

   }/*End if( gt_insDev.t_accel.t_cal == INS_CAL_COMPLETE)*/

   /*------------------------------------------------------------------------*
    * Correct for gyro bias and misalignment using the parameters estimated by
    * the error-state Kalman filter.
    *------------------------------------------------------------------------*/
   ai_gyr[0] = ai_gyr[0] - (int32_t)gt_insDev.t_gyro.as_bias[0];
   ai_gyr[0] = utl_mult32x32_32( ai_gyr[0], (int32_t)gt_insDev.t_gyro.
   as_scale[0], INS_AXIS_SCALE_Q_FACTOR);

   ai_gyr[1] = ai_gyr[1] - (int32_t)gt_insDev.t_gyro.as_bias[1];
   ai_gyr[1] = utl_mult32x32_32( ai_gyr[1], (int32_t)gt_insDev.t_gyro.
   as_scale[1], INS_AXIS_SCALE_Q_FACTOR);

   ai_gyr[2] = ai_gyr[2] - (int32_t)gt_insDev.t_gyro.as_bias[2];
   ai_gyr[2] = utl_mult32x32_32( ai_gyr[2], (int32_t)gt_insDev.t_gyro.
   as_scale[2], INS_AXIS_SCALE_Q_FACTOR);

   t_time = arb_sysTimeNow();
   i_dt = t_time.i_usec - gt_insDev.i_lastTime;
   if( i_dt < 0)
      i_dt += 1000000; /*Modulo the number of usec in 1 second*/

   /*------------------------------------------------------------------------*
    * Store the last time the algorithm updated.
    *------------------------------------------------------------------------*/
   gt_insDev.i_lastTime = t_time.i_usec;

   /*------------------------------------------------------------------------*
    * Update the DCM and error-state Kalman filter time step.
    *------------------------------------------------------------------------*/
   s_dt = (int16_t)(((float)i_dt/1000000)*32768.0f);

   /*------------------------------------------------------------------------*
    * Perform DCM integration according to eq 5.6 and 5.7 of [1].
    *------------------------------------------------------------------------*/
   dcmIntegration( ai_gyr,
                   as_dPhase, /*The delta movement over the last dt. Q1.14*/
                   s_dt);     /*dt in Q0.15 format*/

   /*------------------------------------------------------------------------*
    * Normalize the DCM according to eq's 18, 19, 20, and 21 of [4].
    *------------------------------------------------------------------------*/
   dcmNormalize();

   /*------------------------------------------------------------------------*
    * Convert from DCM representation to Euler attitude using eq 2.17 of [1].
    *------------------------------------------------------------------------*/
   dcmToEuler( gt_insDev.as_dcmAttitude);

   /*------------------------------------------------------------------------*
    * Get the "raw" attitude as determined by the accelerometer and
    * magnetometer using eq's 10.14 and 10.15 of [3] and eq 10.6 of [1].
    *------------------------------------------------------------------------*/
   getRawAttitude( as_acc,
                   as_mag,
                   gt_insDev.as_rawAtt);

   /*------------------------------------------------------------------------*
    * Calculate the specific force of the acceleration vector and the
    * average amount of rotation per time step dt.
    *------------------------------------------------------------------------*/
   calcAvrDynamics( as_acc,
                    as_dPhase);

   /*------------------------------------------------------------------------*
    * Update the Kalman filter measurement error during times of low system
    * dynamics.
    *------------------------------------------------------------------------*/
   eSKalmanResVarEst( gt_insDev.as_rawAtt);

   /*------------------------------------------------------------------------*
    * Form the error-state Kalman filter measurement innovation based on the
    * gravity vector and magnetic field vector, where deltaZ is the attitude
    * error measurement based on the set of 'unlabeled' eq's in section
    * 10.5.1.2 and 10.5.1.3 of [3].
    *------------------------------------------------------------------------*/
   formMeasInnov( as_acc,
                  as_mag,
                  as_deltaZ);

   checkForTrackingErrors();

   /*------------------------------------------------------------------------*
    * Increase the uncertainty in the error covariance matrix.
    *------------------------------------------------------------------------*/
   eSKalmanFilterPredict( ai_gyr,
                          s_dt);

   /*------------------------------------------------------------------------*
    * Apply Kalman filter correction to the error covariance matrix and
    * estimate the error in the attitude, gyro bias, and gyro axis scale
    * factors.
    *------------------------------------------------------------------------*/
   eSKalmanFilterCorrect( as_deltaZ,
                          as_deltaAtt,
                          as_deltaGBias,
                          as_deltaGScale,
                          s_dt);

   if( (as_deltaGScale[0] < 0) && (as_deltaGScale[0] > -5))
      as_deltaGScale[0] = 0;

   if( (as_deltaGScale[1] < 0) && (as_deltaGScale[1] > -5))
      as_deltaGScale[1] = 0;

   if( (as_deltaGScale[2] < 0) && (as_deltaGScale[2] > -5))
      as_deltaGScale[2] = 0;

   /*------------------------------------------------------------------------*
    * Update the gyro bias and scale factor parameters based on the error
    * calculated by the Kalman filter
    *------------------------------------------------------------------------*/
   gt_insDev.t_gyro.as_bias[0] += as_deltaGBias[0];
   gt_insDev.t_gyro.as_bias[1] += as_deltaGBias[1];
   gt_insDev.t_gyro.as_bias[2] += as_deltaGBias[2];

   /*------------------------------------------------------------------------*
    * Since, gt_insDev.t_gyro.as_scale is a Q13, it needs to be scaled up to
    * a Q15 before accumulating the error.
    *------------------------------------------------------------------------*/
   i_sum = ((int32_t)gt_insDev.t_gyro.as_scale[0] << 2) +
   ((int32_t)as_deltaGScale[0]); /*Q15 + Q15*/
   gt_insDev.t_gyro.as_scale[0] = utl_rShft32_16( i_sum, 2); /*Q15 to Q13*/

   i_sum = ((int32_t)gt_insDev.t_gyro.as_scale[1] << 2) +
   ((int32_t)as_deltaGScale[1]); /*Q15 + Q15*/
   gt_insDev.t_gyro.as_scale[1] = utl_rShft32_16( i_sum, 2); /*Q15 to Q13*/

   i_sum = ((int32_t)gt_insDev.t_gyro.as_scale[2] << 2) +
   ((int32_t)as_deltaGScale[2]); /*Q15 + Q15*/
   gt_insDev.t_gyro.as_scale[2] = utl_rShft32_16( i_sum, 2); /*Q15 to Q13*/

   /*------------------------------------------------------------------------*
    * Perform DCM correction according to eq eq 12.7 of [1]
    *------------------------------------------------------------------------*/
   dcmCorrect( as_deltaAtt);

   if( gt_insDev.c_initialLockCount < INS_KALMAN_LOCK_WAIT)
      gt_insDev.c_initialLockCount++;

   /*------------------------------------------------------------------------*
    * Has the Kalman filter lost lock? If so, then reinitialize the DCM
    * based on raw attitude and heading.
    *------------------------------------------------------------------------*/
   if( gt_insDev.b_lossOfLock == true)
   {
      insReset();
   }/*End if( gt_insDev.b_lossOfLock == true)*/

{
 char ac_buff[100];
 int16_t as_att1[3];
 int16_t as_att2[3];

 gc_debugUpdateCount++;

 if( gc_debugUpdateCount == 5)
 {
    gc_debugUpdateCount = 0;
#if 0

    sprintf_P( (char *)ac_buff, PSTR("%d, %d, %d\r"),
    as_mag[0], as_mag[1],  as_mag[2]);
    arb_printf( PRINTF_DBG_HIGH,
                ac_buff);

    sprintf_P( (char *)ac_buff, PSTR("%d, %d, %d\r"),
    as_gyr[0], as_gyr[1],  as_gyr[2]);
    arb_printf( PRINTF_DBG_HIGH,
                ac_buff);

    sprintf_P( (char *)ac_buff, PSTR("%d, %d, %d\r"),
    as_acc[0], as_acc[1],  as_acc[2]);
    arb_printf( PRINTF_DBG_HIGH,
                ac_buff);
#endif

    getAttitudeInDeg( gt_insDev.as_dcmAttitude,
                      as_att1);

    getAttitudeInDeg( gt_insDev.as_rawAtt,
                      as_att2);

    sprintf_P( (char *)ac_buff, PSTR("roll = [%4d,%4d], pitch = [%4d,%4d], yaw = [%4d,%4d], dynamics = %d\r"),
    as_att2[0], as_att1[0],  as_att2[1], as_att1[1], as_att2[2], as_att1[2], gt_insDev.c_platMovingCount);
    arb_printf( PRINTF_DBG_HIGH | PRINTF_DBG_SHOW_TIME,
                ac_buff);

    sprintf_P( (char *)ac_buff, PSTR("gyro scale x = %4d, gyro scale y = %4d, gyro scale z = %4d\r"),
    gt_insDev.t_gyro.as_scale[0], gt_insDev.t_gyro.as_scale[1],  gt_insDev.t_gyro.as_scale[2]);
    arb_printf( PRINTF_DBG_HIGH | PRINTF_DBG_SHOW_TIME,
                ac_buff);

    sprintf_P( (char *)ac_buff, PSTR("gyro bias x = %4d, gyro bias y = %4d, gyro bias z = %4d\r"),
    gt_insDev.t_gyro.as_bias[0], gt_insDev.t_gyro.as_bias[1],  gt_insDev.t_gyro.as_bias[2]);
    arb_printf( PRINTF_DBG_HIGH | PRINTF_DBG_SHOW_TIME,
                ac_buff);

 }

}

}

static t_error insOpen( t_DEVHANDLE t_handle)
{

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_insDev.t_mutex,
             0);

   /*------------------------------------------------------------------------*
    * Keep track of the number of user-space applications using the driver.
    *------------------------------------------------------------------------*/
   gt_insDev.c_numUsers++;

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_insDev.t_mutex);

   return ARB_PASSED;

}/*End insOpen*/

static int16_t insRead( t_DEVHANDLE t_handle,
                        int8_t *pc_buff,
                        uint16_t s_size)
{
   int16_t s_return = (int16_t)ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_insDev.t_mutex,
             0);

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_insDev.t_mutex);

   return s_return;

}/*End insRead*/

static int16_t insWrite( t_DEVHANDLE t_handle,
                         int8_t *pc_buff,
                         uint16_t s_size)
{
   int16_t s_return = (int16_t)ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_insDev.t_mutex,
             0);

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_insDev.t_mutex);

   return s_return;

}/*End insWrite*/

static int32_t insIoctl( t_DEVHANDLE t_handle,
                         uint16_t s_command,
                         int32_t  i_arguments)
{
   int32_t i_return = (int32_t)INS_PASSED;
   static t_twiError t_err;
   uint8_t ac_data[10];
   int16_t as_mag[3];
   int16_t as_gyr[3]; /*Raw gyro sensor readings in degrees*/
   int32_t ai_gyr[3]; /*Gyro readings converted to rad/sec where pi rad = 32767*/
   int16_t as_acc[3];

   /*-----------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *-----------------------------------------------------------------------*/
   arb_wait( gt_insDev.t_mutex,
             0);

   switch( (t_insCmd)s_command)
   {
      case INS_RESET:
         gt_insDev.as_rawAtt[0] = 0;
         gt_insDev.as_rawAtt[1] = 0;
         gt_insDev.as_rawAtt[2] = 0;
         insReset();
      break;

      case INS_GET_STATE_EST:
      {
         t_currentMeas *pt_currentMeas = (t_currentMeas *)((uint16_t)i_arguments);

         pt_currentMeas->f_corrRoll  = ((float)gt_insDev.as_dcmAttitude[0]*180.0f)/32768.0f;
         pt_currentMeas->f_corrPitch = ((float)gt_insDev.as_dcmAttitude[1]*180.0f)/32768.0f;
         pt_currentMeas->f_corrYaw   = ((float)gt_insDev.as_dcmAttitude[2]*180.0f)/32768.0f;

         pt_currentMeas->f_rawRoll   = ((float)gt_insDev.as_rawAtt[0]*180.0f)/32768.0f;
         pt_currentMeas->f_rawPitch  = ((float)gt_insDev.as_rawAtt[1]*180.0f)/32768.0f;
         pt_currentMeas->f_rawYaw    = ((float)gt_insDev.as_rawAtt[2]*180.0f)/32768.0f;
         pt_currentMeas->f_platMovingCount = (float)gt_insDev.c_platMovingCount;
         pt_currentMeas->af_avrResMag[0] = ((float)gt_insDev.ai_avrRes[0]*180.0f)/
         ((float)((int32_t)1<<(15+INS_ATT_RES_VAR_EXTRA_RES)));
         pt_currentMeas->af_avrResMag[1] = ((float)gt_insDev.ai_avrRes[1]*180.0f)/
         ((float)((int32_t)1<<(15+INS_ATT_RES_VAR_EXTRA_RES)));
         pt_currentMeas->af_avrResMag[2] = ((float)gt_insDev.ai_avrRes[2]*180.0f)/
         ((float)((int32_t)1<<(15+INS_ATT_RES_VAR_EXTRA_RES)));
      }
      break;/*End case INS_GET_STATE_EST:*/

      /*---------------------------------------------------------------------*
       * Bring the Sensors out of reset...
       *---------------------------------------------------------------------*/
      case INS_SENS_STARTUP:

         /*------------------------------------------------------------------*
          * Configure the magnetometer for continuous measurements
          *------------------------------------------------------------------*/
         ac_data[0] = 0x02;
         ac_data[1] = 0x00;
         t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                                     ac_data,
                                     2,
                                     (uint8_t)INS_MAGN_ADDRESS,
                                     INS_MAX_TWI_RETRIES);

         arb_sleep( 10);

         /*------------------------------------------------------------------*
          * Configure the magnetometer for a 50Hz update rate
          *------------------------------------------------------------------*/
         ac_data[0] = 0x00;
         ac_data[1] = 0x18;
         t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                                     ac_data,
                                     2,
                                     (uint8_t)INS_MAGN_ADDRESS,
                                     INS_MAX_TWI_RETRIES);

         arb_sleep( 10);

         /*------------------------------------------------------------------*
          * Configure the accelerometer for stand-by mode while its config
          * is being changed.
          *------------------------------------------------------------------*/
         ac_data[0] = 0x2D;
         ac_data[1] = 0x00;
         t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                                     ac_data,
                                     2,
                                     (uint8_t)INS_ACCEL_ADDRESS,
                                     INS_MAX_TWI_RETRIES);

         arb_sleep( 10);

         /*------------------------------------------------------------------*
          * Configure the accelerometer for full measurement resolution.
          *------------------------------------------------------------------*/
         ac_data[0] = 0x2D;
         ac_data[1] = 0x08;
         t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                                     ac_data,
                                     2,
                                     (uint8_t)INS_ACCEL_ADDRESS,
                                     INS_MAX_TWI_RETRIES);

         arb_sleep( 10);

         /*------------------------------------------------------------------*
          * Configure the accelerometer for a 50Hz output rate.
          *------------------------------------------------------------------*/
         ac_data[0] = 0x2C;
         ac_data[1] = 0x09;
         t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                                     ac_data,
                                     2,
                                     (uint8_t)INS_ACCEL_ADDRESS,
                                     INS_MAX_TWI_RETRIES);

         arb_sleep( 10);

         /*------------------------------------------------------------------*
          * Put the accelerometer into measurement mode.
          *------------------------------------------------------------------*/
         ac_data[0] = 0x2D;
         ac_data[1] = 0x08;
         t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                                     ac_data,
                                     2,
                                     (uint8_t)INS_ACCEL_ADDRESS,
                                     INS_MAX_TWI_RETRIES);

         arb_sleep( 10);

         /*------------------------------------------------------------------*
          * Reset the gyro to defaults...
          *------------------------------------------------------------------*/
         ac_data[0] = 0x3E;
         ac_data[1] = 0x80;
         t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                                     ac_data,
                                     2,
                                     (uint8_t)INS_GYRO_ADDRESS,
                                     INS_MAX_TWI_RETRIES);

         arb_sleep( 10);

         /*------------------------------------------------------------------*
          * Set the resolution to full scale and low-pass filter BW to 42Hz.
          *------------------------------------------------------------------*/
         ac_data[0] = 0x16;
         ac_data[1] = 0x1B;
         t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                                     ac_data,
                                     2,
                                     (uint8_t)INS_GYRO_ADDRESS,
                                     INS_MAX_TWI_RETRIES);

         arb_sleep( 10);

         /*------------------------------------------------------------------*
          * Set gyro update rate to 50Hz
          *------------------------------------------------------------------*/
         ac_data[0] = 0x15;
         ac_data[1] = 0x0A;
         t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                                     ac_data,
                                     2,
                                     (uint8_t)INS_GYRO_ADDRESS,
                                     INS_MAX_TWI_RETRIES);

         arb_sleep( 10);

         /*------------------------------------------------------------------*
          * Tell the gyro to use the z axis as its PLL reference.
          *------------------------------------------------------------------*/
         ac_data[0] = 0x3E;
         ac_data[1] = 0x00;
         t_err = hal_twiMasterWrite( gt_insDev.t_tHandle,
                                     ac_data,
                                     2,
                                     (uint8_t)INS_GYRO_ADDRESS,
                                     INS_MAX_TWI_RETRIES);

         arb_sleep( 10);

      break;/*End case INS_SENS_STARTUP:*/

      case INS_UPDATE:

         dcmUpdate();

      break;

      /*---------------------------------------------------------------------*
       * Determine the bias on each gyro axis, and get the initial roll,
       * pitch, and yaw values by averaging the accelerometer and
       * magnetometer x,y, and z values respectively. Once the data has been
       * gathered the DCM is initialized according to eq 2.15 from [1].
       * During this time the platform SHOULD be stationary! There isn't any
       * real-time constraints on initialization, therefore operations are
       * performed using floating point arithmetic.
       *---------------------------------------------------------------------*/
      case INS_DCM_INIT:
      {
         int32_t i_index;
         float af_avrMag[3] = {0.0f,0.0f,0.0f};
         float af_avrGyr[3] = {0.0f,0.0f,0.0f};
         float af_avrAcc[3] = {0.0f,0.0f,0.0f};
         float f_mag;

         for( i_index = 0; i_index < INS_DCM_INIT_MAX_SAMP_TO_AVRG; i_index++)
         {

            /*---------------------------------------------------------------*
             * Acquire new sensor data.
             *---------------------------------------------------------------*/
            readSensors( as_mag,
                         as_gyr,
                         as_acc);

            /*---------------------------------------------------------------*
             * If there has been a valid calibration compensate for axis
             * misalignment and bias...
             *---------------------------------------------------------------*/
            if( gt_insDev.t_mag.t_cal == INS_CAL_COMPLETE)
            {
               applyCalibration( as_mag,
                                 &gt_insDev.t_mag.as_R[0][0],
                                 gt_insDev.t_mag.as_scale,
                                 gt_insDev.t_mag.as_bias);

            }/*End if( gt_insDev.t_mag.t_cal == INS_CAL_COMPLETE)*/

            /*---------------------------------------------------------------*
             * If there has been a valid accelerometer calibration compensate
             * for axis misalignment and bias...
             *---------------------------------------------------------------*/
            if( gt_insDev.t_accel.t_cal == INS_CAL_COMPLETE)
            {
               applyCalibration( as_acc,
                                 &gt_insDev.t_accel.as_R[0][0],
                                 gt_insDev.t_accel.as_scale,
                                 gt_insDev.t_accel.as_bias);

            }/*End if( gt_insDev.t_accel.t_cal == INS_CAL_COMPLETE)*/

            /*---------------------------------------------------------------*
             * Accumulate sensor readings...
             *---------------------------------------------------------------*/
            af_avrMag[0] += (float)as_mag[0];
            af_avrMag[1] += (float)as_mag[1];
            af_avrMag[2] += (float)as_mag[2];

            /*---------------------------------------------------------------*
             * Convert the gyro measurement into rad/sec where 1 rad = 32767
             *---------------------------------------------------------------*/
            gyroDegToRad( as_gyr,
                          ai_gyr);

            af_avrGyr[0] += (float)ai_gyr[0];
            af_avrGyr[1] += (float)ai_gyr[1];
            af_avrGyr[2] += (float)ai_gyr[2];

            af_avrAcc[0] += (float)as_acc[0];
            af_avrAcc[1] += (float)as_acc[1];
            af_avrAcc[2] += (float)as_acc[2];

				arb_sleep( INS_DT_SLEEP);

         }/*End for( i_index = 0; i_index < INS_DCM_INIT_MAX_SAMP_TO_AVRG;
         i_index++)*/

         /*------------------------------------------------------------------*
          * Calculate the magnitude of the average magnetic field vector.
          * eq 10.17 of [3]
          *------------------------------------------------------------------*/
         af_avrMag[0] = af_avrMag[0] / INS_DCM_INIT_MAX_SAMP_TO_AVRG;
         af_avrMag[1] = af_avrMag[1] / INS_DCM_INIT_MAX_SAMP_TO_AVRG;
         af_avrMag[2] = af_avrMag[2] / INS_DCM_INIT_MAX_SAMP_TO_AVRG;

         f_mag = sqrtf( af_avrMag[0]*af_avrMag[0] + af_avrMag[1]*af_avrMag[1] +
         af_avrMag[2]*af_avrMag[2]);

         gt_insDev.t_mag.s_calMagFieldStr = (int16_t)f_mag;

         /*------------------------------------------------------------------*
          * If the remainder than round up...
          *------------------------------------------------------------------*/
         if( (f_mag - (float)gt_insDev.t_mag.s_calMagFieldStr) > .5f)
            gt_insDev.t_mag.s_calMagFieldStr += 1;

         /*------------------------------------------------------------------*
          * Calculate the average gyro bias vector.
          *------------------------------------------------------------------*/
         gt_insDev.t_gyro.as_bias[0] = (int16_t)(af_avrGyr[0] /
         INS_DCM_INIT_MAX_SAMP_TO_AVRG);
         gt_insDev.t_gyro.as_bias[1] = (int16_t)(af_avrGyr[1] /
         INS_DCM_INIT_MAX_SAMP_TO_AVRG);
         gt_insDev.t_gyro.as_bias[2] = (int16_t)(af_avrGyr[2] /
         INS_DCM_INIT_MAX_SAMP_TO_AVRG);

         /*------------------------------------------------------------------*
          * Save a copy of the bias to be used in case the Kalman filter is
          * reset after encountering a runtime error.
          *------------------------------------------------------------------*/
         gt_insDev.t_gyro.as_calBias[0] = gt_insDev.t_gyro.as_bias[0];
         gt_insDev.t_gyro.as_calBias[1] = gt_insDev.t_gyro.as_bias[1];
         gt_insDev.t_gyro.as_calBias[2] = gt_insDev.t_gyro.as_bias[2];

         /*------------------------------------------------------------------*
          * Calculate the magnitude of the average gravity vector.
          * eq 10.13 of [3]
          *------------------------------------------------------------------*/
         af_avrAcc[0] = af_avrAcc[0] / INS_DCM_INIT_MAX_SAMP_TO_AVRG;
         af_avrAcc[1] = af_avrAcc[1] / INS_DCM_INIT_MAX_SAMP_TO_AVRG;
         af_avrAcc[2] = af_avrAcc[2] / INS_DCM_INIT_MAX_SAMP_TO_AVRG;

         f_mag = sqrtf( af_avrAcc[0]*af_avrAcc[0] + af_avrAcc[1]*af_avrAcc[1] +
         af_avrAcc[2]*af_avrAcc[2]);

         gt_insDev.t_accel.s_calGravity = (int16_t)f_mag;

         /*------------------------------------------------------------------*
          * If remainder, than round up...
          *------------------------------------------------------------------*/
         if( (f_mag - (float)gt_insDev.t_accel.s_calGravity) > .5f)
            gt_insDev.t_accel.s_calGravity += 1;

         /*------------------------------------------------------------------*
          * Get the raw attitude using the average accelerometer and
          * magnetometer readings.
          * eq's 10.14, 10.15 and 10.18 of [3]
          *------------------------------------------------------------------*/
         as_acc[0] = (int16_t)af_avrAcc[0];
         as_acc[1] = (int16_t)af_avrAcc[1];
         as_acc[2] = (int16_t)af_avrAcc[2];

         as_mag[0] = (int16_t)af_avrMag[0];
         as_mag[1] = (int16_t)af_avrMag[1];
         as_mag[2] = (int16_t)af_avrMag[2];

         getRawAttitude( as_acc,
                         as_mag,
                         gt_insDev.as_rawAtt);

         /*------------------------------------------------------------------*
          * Initialize the DCM using eq 2.15 of [1].
          *------------------------------------------------------------------*/
         dcmInit( gt_insDev.as_rawAtt);
         eSKalmanFilterInit();

         i_return = (int32_t)INS_CAL_COMPLETE;

      }
      break;/*End case INS_DCM_INIT:*/

      case INS_RESET_CAL:
         gt_insDev.t_mag.t_cal = INS_CAL_FAILED;
         gt_insDev.t_accel.t_cal = INS_CAL_FAILED;
         i_return = (int32_t)gt_insDev.t_mag.t_cal;
      break;/*End case INS_RESET_CAL:*/

      case INS_CALIBRATE_MAG:
      {
         int16_t *ps_results;
         int32_t i_index;
         float af_avrMag[3] = {0.0f,0.0f,0.0f};

         if( gt_insDev.t_mag.t_cal != INS_CAL_IN_PROGRESS)
         {
            gt_insDev.t_mag.t_cal = INS_CAL_IN_PROGRESS;
            gt_insDev.c_HWrtPtr = 0;
         }

         i_return = (int32_t)gt_insDev.t_mag.t_cal;

         if( gt_insDev.c_HWrtPtr < INS_MAX_CAL_SAMPLES)
         {
            for( i_index = 0; i_index < 10; i_index++)
            {
               /*------------------------------------------------------------*
                * Acquire new sensor data.
                *------------------------------------------------------------*/
               readSensors( as_mag,
                            as_gyr,
                            as_acc);

               af_avrMag[0] += (float)as_mag[0];
               af_avrMag[1] += (float)as_mag[1];
               af_avrMag[2] += (float)as_mag[2];

   				arb_sleep( INS_DT_SLEEP);
            }

            gas_H[gt_insDev.c_HWrtPtr][0] = (int16_t)(af_avrMag[0] / i_index);
            gas_H[gt_insDev.c_HWrtPtr][1] = (int16_t)(af_avrMag[1] / i_index);
            gas_H[gt_insDev.c_HWrtPtr][2] = (int16_t)(af_avrMag[2] / i_index);

            /*---------------------------------------------------------------*
             * Return the current reading...
             *---------------------------------------------------------------*/
            ps_results = (int16_t *)((int16_t)i_arguments);
            ps_results[0] = gas_H[gt_insDev.c_HWrtPtr][0];
            ps_results[1] = gas_H[gt_insDev.c_HWrtPtr][1];
            ps_results[2] = gas_H[gt_insDev.c_HWrtPtr][2];

            gt_insDev.c_HWrtPtr++;

         }/*End if( (*pc_HWrtPtr) < INS_MAX_CAL_SAMPLES)*/

	      if( gt_insDev.c_HWrtPtr == INS_MAX_CAL_SAMPLES)
         {

            if( ellipsoidFit( &gt_insDev.t_mag.as_R[0][0],
                              &gt_insDev.t_mag.as_bias[0],
							         &gt_insDev.t_mag.as_scale[0],
                              INS_MAX_CAL_MAG_ENV))
            {
               gt_insDev.t_mag.s_calMagFieldStr = INS_MAX_CAL_MAG_ENV;
               gt_insDev.t_mag.t_cal = INS_CAL_COMPLETE;
               i_return = (int32_t)gt_insDev.t_mag.t_cal;
            }
            else
            {
               gt_insDev.t_mag.t_cal = INS_CAL_FAILED;
               i_return = (int32_t)gt_insDev.t_mag.t_cal;
            }

         }/*End if( (*pc_HWrtPtr) == INS_MAX_CAL_SAMPLES)*/
      }
      break;/*End INS_CALIBRATE_MAG*/

      case INS_CALIBRATE_ACCEL:
      {
         int32_t i_index;
         float af_avrAcc[3] = {0.0f,0.0f,0.0f};
         int16_t *ps_results;

         if( gt_insDev.t_accel.t_cal != INS_CAL_IN_PROGRESS)
         {
            gt_insDev.t_accel.t_cal = INS_CAL_IN_PROGRESS;
            gt_insDev.c_HWrtPtr = 0;
         }

         i_return = (int32_t)gt_insDev.t_accel.t_cal;

         if( gt_insDev.c_HWrtPtr < INS_MAX_CAL_SAMPLES)
         {
            for( i_index = 0; i_index < 10; i_index++)
            {
               /*------------------------------------------------------------*
                * Acquire new sensor data.
                *------------------------------------------------------------*/
               readSensors( as_mag,
                            as_gyr,
                            as_acc);

               af_avrAcc[0] += (float)as_acc[0];
               af_avrAcc[1] += (float)as_acc[1];
               af_avrAcc[2] += (float)as_acc[2];

   				arb_sleep( INS_DT_SLEEP);
            }

            gas_H[gt_insDev.c_HWrtPtr][0] = (int16_t)(af_avrAcc[0] / i_index);
            gas_H[gt_insDev.c_HWrtPtr][1] = (int16_t)(af_avrAcc[1] / i_index);
            gas_H[gt_insDev.c_HWrtPtr][2] = (int16_t)(af_avrAcc[2] / i_index);

            /*---------------------------------------------------------------*
             * Return the current reading...
             *---------------------------------------------------------------*/
            ps_results = (int16_t *)((int16_t)i_arguments);
            ps_results[0] = gas_H[gt_insDev.c_HWrtPtr][0];
            ps_results[1] = gas_H[gt_insDev.c_HWrtPtr][1];
            ps_results[2] = gas_H[gt_insDev.c_HWrtPtr][2];

            gt_insDev.c_HWrtPtr++;

         }/*End if( (*pc_HWrtPtr) < INS_MAX_CAL_SAMPLES)*/

	      if( gt_insDev.c_HWrtPtr == INS_MAX_CAL_SAMPLES)
         {
            if( ellipsoidFit( &gt_insDev.t_accel.as_R[0][0],
                              &gt_insDev.t_accel.as_bias[0],
							         &gt_insDev.t_accel.as_scale[0],
                              INS_MAX_CAL_GRAV_ENV))
            {
               gt_insDev.t_accel.s_calGravity = INS_MAX_CAL_GRAV_ENV;
               gt_insDev.t_accel.t_cal = INS_CAL_COMPLETE;
               i_return = (int32_t)gt_insDev.t_accel.t_cal;
            }
            else
            {
               gt_insDev.t_accel.t_cal = INS_CAL_FAILED;
               i_return = (int32_t)gt_insDev.t_accel.t_cal;
            }

         }/*End if( (*pc_HWrtPtr) == INS_MAX_CAL_SAMPLES)*/
      }
      break;/*End case INS_CALIBRATE_ACCEL:*/

      case INS_GET_MAG_CALIBRATION:
      {
		   t_ellipsoidCal *pt_magCal;

         pt_magCal = (t_ellipsoidCal *)((int16_t)i_arguments);
         pt_magCal->t_status = gt_insDev.t_mag.t_cal;
         pt_magCal->ps_bias  = &gt_insDev.t_mag.as_bias[0];
		   pt_magCal->ps_scale = &gt_insDev.t_mag.as_scale[0];
         pt_magCal->ps_R     = &gt_insDev.t_mag.as_R[0][0];
		   pt_magCal->c_n      = INS_AXIS_SCALE_Q_FACTOR;
		   pt_magCal->c_n      = INS_AXIS_SCALE_Q_FACTOR;
         pt_magCal->s_env    = INS_MAX_CAL_MAG_ENV;

		   i_return = (int32_t)ARB_PASSED;
	   }
      break;/*End case INS_GET_MAG_CALIBRATION:*/

      case INS_GET_ACCEL_CALIBRATION:
      {
		   t_ellipsoidCal *pt_accelCal;

         pt_accelCal = (t_ellipsoidCal *)((int16_t)i_arguments);
         pt_accelCal->t_status = gt_insDev.t_accel.t_cal;
         pt_accelCal->ps_bias  = &gt_insDev.t_accel.as_bias[0];
		   pt_accelCal->ps_scale = &gt_insDev.t_accel.as_scale[0];
         pt_accelCal->ps_R     = &gt_insDev.t_accel.as_R[0][0];
		   pt_accelCal->c_n      = INS_AXIS_SCALE_Q_FACTOR;
         pt_accelCal->s_env    = INS_MAX_CAL_GRAV_ENV;

		   i_return = (int32_t)ARB_PASSED;
	   }
      break;/*End case INS_GET_ACCEL_CALIBRATION:*/

      case INS_GET_GYRO_CALIBRATION:
      {
		   t_ellipsoidCal *pt_gyroCal;

         pt_gyroCal = (t_ellipsoidCal *)((int16_t)i_arguments);
         pt_gyroCal->ps_scale = &gt_insDev.t_gyro.as_scale[0];
         pt_gyroCal->ps_bias  = &gt_insDev.t_gyro.as_bias[0];

		   i_return = (int32_t)ARB_PASSED;
	   }
      break;/*End case INS_GET_GYRO_CALIBRATION:*/

      default:

         i_return = (int32_t)ARB_INVALID_CMD;

      break;

   }/*End switch( (t_insCmd)s_command)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_insDev.t_mutex);

   return i_return;

}/*End insIoctl*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
static t_error insClose( t_DEVHANDLE t_handle)
{

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_insDev.t_mutex,
             0);

   /*------------------------------------------------------------------------*
    * Keep track of the number of user-space applications using the driver.
    *------------------------------------------------------------------------*/
   gt_insDev.c_numUsers--;

   /*------------------------------------------------------------------------*
    * Remove any user-space specific generated memory here...
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_insDev.t_mutex);

   return ARB_PASSED;

}/*End insClose*/

t_error drv_insInit( void)
{
   t_error t_err = ARB_PASSED;
   t_twiConfig  t_tConf;

   /*------------------------------------------------------------------------*
    * Make sure the kernel is aware that a new device has been loaded.
    *------------------------------------------------------------------------*/
   t_err = arb_registerDevice( "insDevice0",
                               arb_createDevId( INS_MAJOR_NUMBER, 0),
                               &gt_insDevOps);

   if( t_err < 0)
   {
      goto failed1;
   }

   /*------------------------------------------------------------------------*
    * Make sure the top level data structure is reset...
    *------------------------------------------------------------------------*/
   memset( (void *)&gt_insDev, 0, sizeof( gt_insDev));

   gt_insDev.t_mutex = arb_semaphoreCreate( MUTEX);

   if( gt_insDev.t_mutex < 0)
   {
      t_err = (t_error)gt_insDev.t_mutex;
      goto failed2;

   }/*End if( gt_insDev.t_mutex < 0)*/

   /*------------------------------------------------------------------------*
    * Request a semaphore from the kernel. We will use this semaphore for
    * signaling the user-space program when a measurement has finished.
    *------------------------------------------------------------------------*/
   gt_insDev.t_rxBlockingSem = arb_semaphoreCreate( COUNTING);

   if( gt_insDev.t_rxBlockingSem < 0)
   {
      t_err = (t_error)gt_insDev.t_rxBlockingSem;
      goto failed3;

   }/*End if( gt_insDev.t_rxBlockingSem < 0)*/

   /*------------------------------------------------------------------------*
    * Request access to the INS TWI channel...
    *------------------------------------------------------------------------*/
   gt_insDev.t_tHandle = hal_requestTwiChannel( INS_TWI);

   if( gt_insDev.t_tHandle < 0)
   {
      t_err = ARB_HAL_ERROR;
      goto failed4;
   }

   /*------------------------------------------------------------------------*
    * Configure the INS TWI channel as master...
    *------------------------------------------------------------------------*/
   t_tConf.t_mode = TWI_MASTER;
   t_tConf.i_baud = INS_TWI_BAUD_RATE;

   t_err = hal_configureTwiChannel( gt_insDev.t_tHandle, t_tConf);

   if( t_err < 0)
   {
      t_err = ARB_HAL_ERROR;
      goto failed5;
   }

   /*------------------------------------------------------------------------*
    * We dont have any users attached to this device.
    *------------------------------------------------------------------------*/
   gt_insDev.c_numUsers = 0;

   /*------------------------------------------------------------------------*
    * Initialize the gyro axis scale factor to 1.
    *------------------------------------------------------------------------*/
   gt_insDev.t_gyro.as_scale[0] = ((int16_t)1 << INS_AXIS_SCALE_Q_FACTOR) - 1;
   gt_insDev.t_gyro.as_scale[1] = ((int16_t)1 << INS_AXIS_SCALE_Q_FACTOR) - 1;
   gt_insDev.t_gyro.as_scale[2] = ((int16_t)1 << INS_AXIS_SCALE_Q_FACTOR) - 1;

   /*------------------------------------------------------------------------*
    * Recall calibration parameters from eeprom.
    *------------------------------------------------------------------------*/
   gt_insDev.t_mag.as_R[0][0] = 18815;
   gt_insDev.t_mag.as_R[0][1] = 26566;
   gt_insDev.t_mag.as_R[0][2] = 3719;
   gt_insDev.t_mag.as_R[1][0] = 26608;
   gt_insDev.t_mag.as_R[1][1] = -19060;
   gt_insDev.t_mag.as_R[1][2] = 1538;
   gt_insDev.t_mag.as_R[2][0] = -3410;
   gt_insDev.t_mag.as_R[2][1] = -2136;
   gt_insDev.t_mag.as_R[2][2] = 32518;

   gt_insDev.t_mag.as_scale[0] = 6260;
   gt_insDev.t_mag.as_scale[1] = 6386;
   gt_insDev.t_mag.as_scale[2] = 7104;

   gt_insDev.t_mag.as_bias[0] = -112;
   gt_insDev.t_mag.as_bias[1] = 32;
   gt_insDev.t_mag.as_bias[2] = -6;

   gt_insDev.t_accel.as_R[0][0] = 23610;
   gt_insDev.t_accel.as_R[0][1] = 2907;
   gt_insDev.t_accel.as_R[0][2] = 22533;
   gt_insDev.t_accel.as_R[1][0] = 22716;
   gt_insDev.t_accel.as_R[1][1] = -2416;
   gt_insDev.t_accel.as_R[1][2] = -23490;
   gt_insDev.t_accel.as_R[2][0] = 422;
   gt_insDev.t_accel.as_R[2][1] = -32548;
   gt_insDev.t_accel.as_R[2][2] = 3753;

   gt_insDev.t_accel.as_scale[0] = 7486;
   gt_insDev.t_accel.as_scale[1] = 8247;
   gt_insDev.t_accel.as_scale[2] = 8551;

   gt_insDev.t_accel.as_bias[0] = -7;
   gt_insDev.t_accel.as_bias[1] = 20;
   gt_insDev.t_accel.as_bias[2] = -27;

   gt_insDev.t_mag.s_calMagFieldStr = INS_MAX_CAL_MAG_ENV;
   gt_insDev.t_accel.s_calGravity   = INS_MAX_CAL_GRAV_ENV;

   gt_insDev.t_mag.t_cal = INS_CAL_COMPLETE;
   gt_insDev.t_accel.t_cal = INS_CAL_COMPLETE;

   eSKalmanFilterInit();

   return ARB_PASSED;

failed5:

   hal_releaseTwiChannel( gt_insDev.t_tHandle);

failed4:

   arb_semaphoreDestroy( gt_insDev.t_rxBlockingSem);

failed3:

   arb_semaphoreDestroy( gt_insDev.t_mutex);

failed2:

   arb_destroyDevice( "insDevice0");

failed1:

   return t_err;

}/*End drv_insInit*/

/*---------------------------------------------------------------------------*
 * Remove the device from the system
 *---------------------------------------------------------------------------*/
void drv_insExit( void)
{

   if( gt_insDev.t_mutex != 0) /*If created... destroy*/
   {
      arb_semaphoreDestroy( gt_insDev.t_mutex);
      arb_semaphoreDestroy( gt_insDev.t_rxBlockingSem);
      hal_releaseTwiChannel( gt_insDev.t_tHandle);
      arb_destroyDevice( "insDevice0");

      memset( (void *)&gt_insDev, 0, sizeof( gt_insDev));

   }/*End if( gt_insDev[c_index].t_mutex != 0)*/

}/*End drv_insExit*/
