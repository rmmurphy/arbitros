/*---------------------------------------------------------------------------*
 * File Name   : drv_sonar.c
 *
 * Description : This file is responsible for controlling the LV-MaxSonar-EZ1
 *               range finder. The LV-MaxSonar-EZ1 outputs an analog voltage 
 *               (Vin) that is proportional to the distance of the object being 
 *               measured. This means that calculating the distance is as 
 *               simple as scaling Vin by the slope of the converter 
 *               (SONAR_SLOPE = SONAR_VREF/SONAR_MAX). In order to perform this
 *               calculation Vin is derived from every new ADC conversion (RES)
 *               using the following equation;
 *
 *                           RES - ADC_OFFSET     
 *                   Vin = ------------------ * ADC_VREF
 *                               ADC_TOP
 *                     
 *                   Where the ADC conversion result RES is given by,
 *    
 *                           (Vin + DELTA_V)                 Vin*ADC_TOP      
 *                   RES =  ----------------- * ADC_TOP =  --------------  
 *                              ADC_VREF                      ADC_VREF          
 *    
 *                      DELTA_V*ADC_TOP
 *                   + -----------------
 *                          ADC_VREF
 *    
 *                                Vin*ADC_TOP
 *                   or, RES = ----------------- + ADC_OFFSET
 *                                  ADC_VREF
 *    
 *                   Where ADC_OFFSET = ADC positive offset, ADC_VREF = ADC 
 *                   reference voltage, and ADC_TOP = top value of the ADC
 *
 *               Then the distance (d) is then found by simply scaling Vin by 
 *               the slope,
 *
 *                                             
 *                   d =  Vin*SONAR_SLOPE 
 *                                                 
 *
 *                          RES - ADC_OFFSET 
 *                     =  ------------------- * (ADC_VREF*SONAR_SLOPE)
 *                              ADC_TOP
 *
 *                This step can be further simplified by pluggin in numbers for
 *                the constants,
 *
 *                         (RES - 205)      3.3       512        (RES - 205)  
 *                   d = -------------- * ------ * -------- = ---------------
 *                            4096         1.6       3.3           12.8
 *
 *                Where 12.8 is the divisor needed for converting the offset 
 *                compensated ADC conversion from volts-to-inches. Since 
 *                floating point arithmatic and divide operations are unwanted, 
 *                this value can be converted into a Q0.8 fixed-point number 
 *                using the following operation;
 *
 *                   ADC_V_TO_I = 256 / 12
 *
 *                Finally, using this value we can rewrite the distance 
 *                calculation according to,
 *         
 *                   d = ((RES - 205)*ADC_V_TO_I) >> 8
 * 
 *
 * Programmer  : Ryan M Murphy
 *
 * Date        : June, 23, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Inlude Files
 *---------------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "avr_compiler.h"
#include "arb_device.h"
#include "arb_semaphore.h"
#include "drv_sonar.h"
#include "hal_gpio.h"
#include "hal_adc.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define SONAR_MAJOR_NUMBER  (2)
#define SONAR_MAX_MINORS    (1)

#define SONAR_ADC_MODULE        (ADC1_MODULE)
#define SONAR_ADC_PORT          (GPIO_PORTA)
#define SONAR_ADC_SAMPLE_PIN    (PIN_1)
#define SONAR_MEAS_START_PIN    (PIN_0) 
#define SONAR_MEAS_FINISHED_PIN (PIN_2)
#define MAX_AVERAGES            (32)

#define SONAR_MAX_DISTANCE (512.0f) /*inches*/
#define SONAR_REF_VOLTAGE  (3.3f)   /*Volts*/
#define ADC_VREF           (3.3f/1.6f)
#define ADC_DELTA_V        (.05f*ADC_VREF)
#define ADC_TOP            (4096.0f) /*Top ADC value 2^12*/
#define ADC_OFFSET         (uint8_t)(ADC_DELTA_V*ADC_TOP/ADC_VREF)
#define ADC_V_TO_I         (20)

//#define SONAR_DEBUG

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct
{
   /*------------------------------------------------------------------------*
    * Resources that can be shared amongst mutliple users (either a global
    * buffer or IO device) need to be protected against race conditions. We 
    * use this semaphore for just that purpose.
    *------------------------------------------------------------------------*/
   t_semHandle t_mutex;

   /*------------------------------------------------------------------------*
    * This semaphore is used for waking up the user-space thread once a 
    * measurement has finished.
    *------------------------------------------------------------------------*/
   t_semHandle t_blockingSem;

   /*------------------------------------------------------------------------*
    * Handle to the gpio interrupt we use for knowing when a measurement has
    * finished.
    *------------------------------------------------------------------------*/
   t_GPIOHNDL t_gpioHandle;

   /*------------------------------------------------------------------------*
    * Handle to the ADC virtual channel this driver is using.
    *------------------------------------------------------------------------*/
   t_ADCCHANHNDL t_adcHandle;

   /*------------------------------------------------------------------------*
    * We are going to want to know how many 'handles' or users are attached to
    * this device.
    *------------------------------------------------------------------------*/
   uint8_t c_numUsers;

   /*------------------------------------------------------------------------*
    * The current sonar measurement in inches
    *------------------------------------------------------------------------*/
   uint16_t s_currMeas;

   /*------------------------------------------------------------------------*
    * The running total of the last 'c_numAvr' measurments.
    *------------------------------------------------------------------------*/
   uint32_t i_measSum;

   /*------------------------------------------------------------------------*
    * The number of consecutive reading to average.
    *------------------------------------------------------------------------*/
   uint8_t c_numAvr;

   /*------------------------------------------------------------------------*
    * If true, we are actively taking measurements
    *------------------------------------------------------------------------*/
   bool b_measEnabled;

   /*------------------------------------------------------------------------*
    * Keeps track of the number of measurements to average.
    *------------------------------------------------------------------------*/
   uint8_t c_avrCount;

}t_sonarDev;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_error sonarOpen( t_DEVHANDLE t_devHandle);

static int32_t sonarIoctl( t_DEVHANDLE t_devHandle,
                           uint16_t s_command,
                           int32_t  i_arguments);

static t_error sonarClose( t_DEVHANDLE t_devHandle);

static void measurementFinished( void)  __attribute__ ( ( noinline ) );

static void adcConversionComplete( void)  __attribute__ ( ( noinline ) );

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
t_deviceOperations gat_sonarDevOps = 
{
    sonarOpen,
    NULL,
    NULL,
    sonarIoctl,
    sonarClose
};

#ifdef SONAR_DEBUG
   #define DEBUG_SIZE 8
   uint16_t gas_readings[DEBUG_SIZE];
   uint8_t gc_index = 0;
#endif

/*---------------------------------------------------------------------------*
 * This is the device's shared memory, all actions on this variable must be
 * mutually exclusive.
 *---------------------------------------------------------------------------*/
static t_sonarDev gat_sonarDev[SONAR_MAX_MINORS];

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
t_error sonarOpen( t_DEVHANDLE t_devHandle)
{
   
   t_error t_err = ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gat_sonarDev[0].t_mutex,
             0);

   gat_sonarDev[0].c_numUsers++;

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gat_sonarDev[0].t_mutex);

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gat_sonarDev[0].t_mutex);

   return t_err;

}/*End sonarOpen*/

int32_t sonarIoctl( t_DEVHANDLE t_devHandle,
                    uint16_t s_command,
                    int32_t  i_arguments)
{
   int32_t i_return = (int32_t)ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory, so perform lock...all other
    * threads will have to wait until the current thread is finished
    * accessing the device's shared memory.
    *------------------------------------------------------------------------*/
   arb_wait( gat_sonarDev[0].t_mutex,
             0);

   switch( (t_sonarCmd)s_command)
   {
      case TAKE_MEASUREMENT:

#ifdef SONAR_DEBUG
   memset( gas_readings, 0, DEBUG_SIZE*2);
   gc_index = 0;
#endif

         if( (i_arguments < 1) || ( i_arguments > MAX_AVERAGES))
         {

            i_return = (int32_t)ARB_INVALID_ARG;

         }/*End if( (i_arguments < 1) || ( i_arguments > MAX_AVERAGES))*/
         else
         {

            gat_sonarDev[0].c_numAvr = (uint8_t)i_arguments;
            
            gat_sonarDev[0].i_measSum = 0;

            gat_sonarDev[0].c_avrCount = 0;

            gat_sonarDev[0].b_measEnabled = true;

            /*---------------------------------------------------------------*
             * Tell the device to take a measurement.
             *---------------------------------------------------------------*/            
            hal_gpioOn( SONAR_ADC_PORT,
                        SONAR_MEAS_START_PIN);

            /*---------------------------------------------------------------*
             * Block the calling user-space thread until the measurement has
             * finished.
             *---------------------------------------------------------------*/
            arb_wait( gat_sonarDev[0].t_blockingSem,
                      0);
         }

      case READ_LAST_MEASUREMENT:

         /*------------------------------------------------------------------*
          * Report the last known measurement.
          *------------------------------------------------------------------*/
         i_return = (int32_t)gat_sonarDev[0].s_currMeas;

      break;/*End case READ_LAST_MEASUREMENT:*/

      default:

         i_return = (int32_t)ARB_INVALID_CMD;

      break;

   }/*End switch( (t_sonarCmd)s_command)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gat_sonarDev[0].t_mutex);

   return i_return;

}/*End sonarIoctl*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
t_error sonarClose( t_DEVHANDLE t_devHandle)
{

   /*------------------------------------------------------------------------*
    * We are going to access global memory, so perform lock...all other
    * threads will have to wait until the current thread is finished
    * accessing the device's shared memory.
    *------------------------------------------------------------------------*/
   arb_wait( gat_sonarDev[0].t_mutex,
             0);

   gat_sonarDev[0].c_numUsers--;

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gat_sonarDev[0].t_mutex);

   return ARB_PASSED;

}/*End sonarClose*/

t_error drv_sonarInit( void)
{

   t_error t_err       = ARB_PASSED;
   t_gpioConf t_conf;
   t_intConf t_int;
   t_adcModConf t_adcMod;
   t_adcChanConf t_adcChan;
   uint8_t c_index     = 0;
   char ac_devName[MAX_DEVICE_NAME_BYTES];

   /*------------------------------------------------------------------------*
    * We are going to be removing items from memory that may be being used
    * by another process.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   for( c_index = 0; c_index < SONAR_MAX_MINORS; c_index++)
   {
      /*---------------------------------------------------------------------*
       * Make sure the kernel is aware that a new device has been loaded.
       *---------------------------------------------------------------------*/
      sprintf(ac_devName, "sonarDevice%d", c_index);
      t_err = arb_registerDevice( ac_devName,
                                  arb_createDevId( SONAR_MAJOR_NUMBER, 
                                  c_index),
                                  &gat_sonarDevOps);

      if( t_err < 0)
      {
         goto failed1;
      }

      /*---------------------------------------------------------------------*
       * Request a semaphore from the kernel. Since the signal port is a shared
       * resource we need to have all actions on it be mutually exclusive.
       *---------------------------------------------------------------------*/
      gat_sonarDev[c_index].t_mutex = arb_semaphoreCreate( MUTEX);

      if( gat_sonarDev[c_index].t_mutex < 0)
      {
         t_err = (t_error)gat_sonarDev[c_index].t_mutex;
         goto failed2;

      }/*End if( gat_sonarDev[c_index].t_mutex < 0)*/

      /*---------------------------------------------------------------------*
       * Request a semaphore from the kernel. We will use this semaphore for
       * signaling the user-space program when a measurement has finished.
       *---------------------------------------------------------------------*/
      gat_sonarDev[c_index].t_blockingSem = arb_semaphoreCreate( COUNTING);

      if( gat_sonarDev[c_index].t_blockingSem < 0)
      {
         t_err = (t_error)gat_sonarDev[c_index].t_blockingSem;
         goto failed3;

      }/*End if( gat_sonarDev[c_index].t_blockingSem < 0)*/

      /*---------------------------------------------------------------------*
       * Configure pin 1 as the ADC input and pin 2 ( begin measurement)
       * as an output
       *---------------------------------------------------------------------*/
      t_conf.c_inputMask  = SONAR_ADC_SAMPLE_PIN;
      t_conf.c_outputMask = SONAR_MEAS_START_PIN;
      t_conf.t_inConf     = PULLDOWN;
      t_conf.t_outConf    = TOTEM;

      if( hal_configureGpioPort( SONAR_ADC_PORT, t_conf) < 0)
      {
         t_err = ARB_HAL_ERROR;
         goto failed4; 
      }

      /*---------------------------------------------------------------------*
       * Pin 3 has a different pull configuration so its setup individually.
       *---------------------------------------------------------------------*/
      t_conf.c_inputMask  = SONAR_MEAS_FINISHED_PIN;
      t_conf.c_outputMask = 0;
      t_conf.t_inConf     = PULLUP;

      if( hal_configureGpioPort( SONAR_ADC_PORT, t_conf) < 0)
      {
         t_err = ARB_HAL_ERROR;
         goto failed4;
      }

      t_int.c_pin     = SONAR_MEAS_FINISHED_PIN;
      t_int.t_inSense = FALLING;
      t_int.pf_funPtr = &measurementFinished;

      gat_sonarDev[c_index].t_gpioHandle = hal_requestGpioInt( SONAR_ADC_PORT, t_int);
      if( gat_sonarDev[c_index].t_gpioHandle < 0)
      {
         t_err = ARB_HAL_ERROR;
         goto failed4;
      }

      /*------------------------------------------------------------------------*
       * Configure the ADCA module
       *------------------------------------------------------------------------*/
      t_adcMod.t_mode  = UNSIGNED;
      t_adcMod.t_res   = RES_12BIT;
      t_adcMod.t_ref   = INTERNAL_VCC_OVER_1Pnt6;
      t_adcMod.t_clock = PERIPH_CLOCK_OVER_16;

      if( hal_configureAdcModule( SONAR_ADC_MODULE, t_adcMod) < 0)
      {
         t_err = ARB_HAL_ERROR;
         goto failed5;
      }

      t_adcChan.c_posPin    = SONAR_ADC_SAMPLE_PIN;
      t_adcChan.t_inMode    = SINGLE_ENDED_EXT;
      t_adcChan.b_enableInt = true;
      t_adcChan.pf_funPtr   = &adcConversionComplete;

      gat_sonarDev[c_index].t_adcHandle = hal_requestAdcChannel( SONAR_ADC_MODULE, 
      t_adcChan);

      if( gat_sonarDev[c_index].t_adcHandle < 0)
      {
         t_err = ARB_HAL_ERROR;
         goto failed5;
      }

      /*---------------------------------------------------------------------*
       * We dont have any users attached to this device.... yet
       *---------------------------------------------------------------------*/
      gat_sonarDev[c_index].c_numUsers = 0;

      /*---------------------------------------------------------------------*
       * Initialize the last measurement to 0 inches.
       *---------------------------------------------------------------------*/
      gat_sonarDev[c_index].s_currMeas = 0;

      /*---------------------------------------------------------------------*
       * The aggregate of a measurement currently taking place.
       *---------------------------------------------------------------------*/
      gat_sonarDev[c_index].i_measSum = 0;

      /*---------------------------------------------------------------------*
       * The number of consecutive reading to average.
       *---------------------------------------------------------------------*/
      gat_sonarDev[c_index].c_numAvr = 0;

      gat_sonarDev[c_index].b_measEnabled = false;

      /*---------------------------------------------------------------------*
       * Running count of the current measurement. When reaching c_numAvr the
       * current measurement is over.
       *---------------------------------------------------------------------*/
      gat_sonarDev[c_index].c_avrCount = 0;

   }/*End for( c_index = 0; c_index < SONAR_MAX_MINORS; c_index++)*/

   HAL_END_CRITICAL(); //Enable interrupts

   return ARB_PASSED;

failed5:

   hal_releaseGpioInt( gat_sonarDev[c_index].t_gpioHandle);

failed4:

   arb_semaphoreDestroy( &gat_sonarDev[c_index].t_blockingSem);

failed3:

   arb_semaphoreDestroy( &gat_sonarDev[c_index].t_mutex);

failed2:

   arb_destroyDevice( ac_devName);

failed1:

   HAL_END_CRITICAL(); //Enable interrupts

   return t_err;

}/*End drv_sonarInit*/

/*---------------------------------------------------------------------------*
 * Remove the device from the system
 *---------------------------------------------------------------------------*/
void drv_sonarExit( void)
{

   uint8_t c_index    = 0;
   char ac_devName[MAX_DEVICE_NAME_BYTES];

   /*------------------------------------------------------------------------*
    * We are going to be removing items from memory that may be being used
    * by another process.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   for( c_index = 0; c_index < SONAR_MAX_MINORS; c_index++)
   {
      if( gat_sonarDev[c_index].t_mutex != 0) /*If created... destroy*/
      {
         sprintf(ac_devName, "sonarDevice%d",c_index);
         arb_semaphoreDestroy( &gat_sonarDev[c_index].t_mutex);
         arb_semaphoreDestroy( &gat_sonarDev[c_index].t_blockingSem);
         arb_destroyDevice( ac_devName);

         hal_releaseGpioInt( gat_sonarDev[c_index].t_gpioHandle);
         hal_releaseAdcChannel( gat_sonarDev[c_index].t_adcHandle);

         memset( (char *)&gat_sonarDev[c_index], 0, sizeof( t_sonarDev));

      }/*End if( gat_sonarDev[c_index].t_mutex != 0)*/

   }/*End for( c_index = 0; c_index < SONAR_MAX_MINORS; c_index++)*/

   HAL_END_CRITICAL(); //Enable interrupts

}/*End drv_sonarExit*/

static void adcConversionComplete()
{
   int16_t s_temp = 0;
   
   s_temp = hal_getAdcSample( gat_sonarDev[0].t_adcHandle);

   gat_sonarDev[0].i_measSum += s_temp;
   gat_sonarDev[0].c_avrCount++;

#ifdef SONAR_DEBUG
   gas_readings[gc_index] = s_temp;
   gc_index++;
   if( gc_index == DEBUG_SIZE)
      gc_index = 0;
#endif

   if( gat_sonarDev[0].c_avrCount == gat_sonarDev[0].c_numAvr)
   {
      gat_sonarDev[0].b_measEnabled = false;

      /*---------------------------------------------------------------------*
       * Calculate the average distance in inches.
       *---------------------------------------------------------------------*/
      s_temp = (uint16_t)((gat_sonarDev[0].i_measSum + 
      (uint32_t)(gat_sonarDev[0].c_avrCount >> 1)) / 
      gat_sonarDev[0].c_avrCount);
      s_temp = (s_temp - ADC_OFFSET);

      if( s_temp < 0)
         s_temp = 0;

      gat_sonarDev[0].s_currMeas = (uint16_t)(((uint32_t)s_temp*(uint32_t)
      ADC_V_TO_I) >> 8);

      /*---------------------------------------------------------------------*
       * Wake any threads waiting for a reading from the sensor.
       *---------------------------------------------------------------------*/   
      arb_signal( gat_sonarDev[0].t_blockingSem);

   }/*End if( gat_sonarDev.c_avrCount == gat_sonarDev.c_numAvr)*/

}/*End adcConversionComplete*/

static void measurementFinished( void)
{
   if( gat_sonarDev[0].b_measEnabled == true)
   {
      /*---------------------------------------------------------------------*
       * Tell the sonar to stop taking measurements.
       *---------------------------------------------------------------------*/
      hal_startAdcConversion( gat_sonarDev[0].t_adcHandle);
   }

}/*End measurementFinished*/

