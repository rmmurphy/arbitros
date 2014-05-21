/*---------------------------------------------------------------------------*
 * File Name   : drv_panTilt.c
 *
 * Description : This file is responsible for controlling a pan/tilt deivce
 *
 * Programmer  : Ryan M Murphy
 *
 * Date        : June, 15, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Inlude Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "avr_compiler.h"
#include "arb_device.h"
#include "arb_semaphore.h"
#include "drv_panTilt.h"
#include "hal_timer.h"
#include "hal_gpio.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define PANTILT_MAJOR_NUMBER (1)
#define PANTILT_MAX_MINORS   (1)

#define PANTILT_TIMER        (TIMER_4) /*Timer/counter D1*/
#define PANTILT_PORT         (GPIO_PORTD)
#define PANTILT_PAN_PIN      (PIN_4)
#define PANTILT_TILT_PIN     (PIN_5)

/*---------------------------------------------------------------------------*
 * Here we define the period of our pan and tilt timers. The timers are
 * initialized to run at a 500khz rate, which is acheived by dividing the
 * XMEGA system clock of 32Mhz down by 64.
 *---------------------------------------------------------------------------*/
#define PANTILT_TIMER_TICK_RATE       (500000) /*ticks/sec*/
#define PANTILT_TIMER_PERIOD          (.02f)  /*seconds*/         
#define PANTILT_TIMER_PERIOD_IN_USEC  (PANTILT_TIMER_PERIOD*1000000)
#define PANTILT_TIMER_PERIOD_IN_TICKS (uint16_t)(PANTILT_TIMER_PERIOD*\
PANTILT_TIMER_TICK_RATE  - 1)

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
    * Pan/tilt timer handle.
    *------------------------------------------------------------------------*/
   t_TIMERHNDL t_timer;

   /*------------------------------------------------------------------------*
    * We are going to want to know how many 'handles' or users are attached to
    * this device.
    *------------------------------------------------------------------------*/
   uint8_t c_numUsers;

   /*------------------------------------------------------------------------*
    * The current position (in degrees -45 to 45) of the pan servo
    *------------------------------------------------------------------------*/
   int8_t c_posPan;

   /*------------------------------------------------------------------------*
    * The current position (in degrees -45 to 45) of the tilt servo
    *------------------------------------------------------------------------*/
   int8_t c_posTilt;

}t_panTiltDev;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_error panTiltOpen( t_DEVHANDLE t_devHandle);

static int32_t panTiltIoctl( t_DEVHANDLE t_devHandle,
                             uint16_t s_command,
                             int32_t  i_arguments);

static t_error panTiltClose( t_DEVHANDLE t_devHandle);

static void panTiltTimerStart( void);

static void panTiltTimerStop( void);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
t_deviceOperations gat_panTiltDevOps = 
{
    panTiltOpen,
    NULL,
    NULL,
    panTiltIoctl,
    panTiltClose
};

/*---------------------------------------------------------------------------*
 * This is the device's shared memory, all actions on this variable must be
 * mutually exclusive.
 *---------------------------------------------------------------------------*/
static t_panTiltDev gat_panTiltDev[PANTILT_MAX_MINORS];

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void panTiltTimerStart( void)
{

   hal_startTimer( gat_panTiltDev[0].t_timer);

}/*End panTiltTimerStart*/

static void panTiltTimerStop( void)
{

   hal_stopTimer( gat_panTiltDev[0].t_timer);

}/*End panTiltTimerStop*/

t_error panTiltOpen( t_DEVHANDLE t_devHandle)
{
   
   gat_panTiltDev[0].c_numUsers++;

   return ARB_PASSED;

}/*End panTiltOpen*/

int32_t panTiltIoctl( t_DEVHANDLE t_devHandle,
                      uint16_t s_command,
                      int32_t  i_arguments)
{
   int16_t s_pos = 0;
   int32_t i_return = (int32_t)ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gat_panTiltDev[0].t_mutex,
             0);

   switch( (t_panTiltCmd)s_command)
   {
      case PAN_ABSOLUTE: /*Pan the device somewhere between -45 and 45 degs*/

         /*------------------------------------------------------------------*
          * Translate (i_arguments) from degrees to pulse width (in ticks)
          *------------------------------------------------------------------*/

         /*------------------------------------------------------------------*
          * Limit the range of movement in order to protect from over stress-
          * ing the servo.
          *------------------------------------------------------------------*/
	      if( i_arguments < PAN_NEG_45_DEGREES)
            i_arguments = PAN_NEG_45_DEGREES;
         else if( i_arguments > PAN_POS_45_DEGREES)
            i_arguments = PAN_POS_45_DEGREES;

         hal_setCompareValue( gat_panTiltDev[0].t_timer,
                              COMPAREA,
                              (uint16_t)i_arguments);
                                       
         gat_panTiltDev[0].c_posPan = (int8_t)i_arguments;

      break;/*End case PAN_ABSOLUTE:*/

      case PAN_RELATIVE: /*Pan the device relative to our last known position*/

         s_pos = (int16_t)gat_panTiltDev[0].c_posPan;

         s_pos += (int16_t)i_arguments;

         /*------------------------------------------------------------------*
          * Limit the range of movement in order to protect from over stress-
          * ing the servo.
          *------------------------------------------------------------------*/
	      if( s_pos < PAN_NEG_45_DEGREES)
            s_pos = PAN_NEG_45_DEGREES;
         else if( s_pos > PAN_POS_45_DEGREES)
            s_pos = PAN_POS_45_DEGREES;

         hal_setCompareValue( gat_panTiltDev[0].t_timer,
                              COMPAREA,
                              s_pos);

         gat_panTiltDev[0].c_posPan = s_pos;

      break;/*End case PAN_RELATIVE:*/

      case TILT_ABSOLUTE: /*Tilt the device*/

         /*------------------------------------------------------------------*
          * Translate (i_arguments) from degrees to pulse width (in ticks)
          *------------------------------------------------------------------*/

         /*------------------------------------------------------------------*
          * Limit the range of movement in order to protect from over stress-
          * ing the servo.
          *------------------------------------------------------------------*/
	      if( i_arguments < TILT_NEG_45_DEGREES)
            i_arguments = TILT_NEG_45_DEGREES;
         else if( i_arguments > TILT_POS_45_DEGREES)
            i_arguments = TILT_POS_45_DEGREES;

         hal_setCompareValue( gat_panTiltDev[0].t_timer,
                              COMPAREB,
                              (uint16_t)i_arguments);

         gat_panTiltDev[0].c_posTilt = (int8_t)i_arguments;

      break;/*End  case TILT_ABSOLUTE:*/

      case TILT_RELATIVE: /*Tilt the device relative to our last known position*/

         s_pos = (int16_t)gat_panTiltDev[0].c_posTilt;

         s_pos += (int16_t)i_arguments;

         /*------------------------------------------------------------------*
          * Limit the range of movement in order to protect from over stress-
          * ing the servo.
          *------------------------------------------------------------------*/
	      if( s_pos < TILT_NEG_45_DEGREES)
            s_pos = TILT_NEG_45_DEGREES;
         else if( s_pos > TILT_POS_45_DEGREES)
            s_pos = TILT_POS_45_DEGREES;

         hal_setCompareValue( gat_panTiltDev[0].t_timer,
                              COMPAREB,
                              s_pos);

         gat_panTiltDev[0].c_posTilt = s_pos;

      break;/*End case TILT_RELATIVE:*/

      case START_PWM:

         panTiltTimerStart();

      break;/*End case START_PWM:*/

      case STOP_PWM:

         panTiltTimerStop();

      break;/*End case STOP_PWM:*/

      default:

         i_return = (int32_t)ARB_INVALID_CMD;

      break;

   }/*End switch( (t_panTiltCmd)s_command)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gat_panTiltDev[0].t_mutex);

   return i_return;

}/*End panTiltIoctl*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
t_error panTiltClose( t_DEVHANDLE t_devHandle)
{
   gat_panTiltDev[0].c_numUsers--;

   return ARB_PASSED;

}/*End panTiltClose*/

t_error drv_panTiltInit( void)
{
   t_error t_err = ARB_PASSED;
   t_timerConfig t_config;
   t_gpioConf t_conf;
   uint8_t c_index = 0;
   char ac_devName[MAX_DEVICE_NAME_BYTES];

   /*------------------------------------------------------------------------*
    * We are going to be removing items from memory that may be being used
    * by another process.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   for( c_index = 0; c_index < PANTILT_MAX_MINORS; c_index++)
   {
      /*---------------------------------------------------------------------*
       * Make sure the kernel is aware that a new device has been loaded.
       *---------------------------------------------------------------------*/
      sprintf(ac_devName, "panTiltDevice%d", c_index);
      t_err = arb_registerDevice( ac_devName,
                                  arb_createDevId( PANTILT_MAJOR_NUMBER, 
                                  c_index),
                                  &gat_panTiltDevOps);

      if( t_err < 0)
      {
         HAL_END_CRITICAL(); //Enable interrupts
         return t_err;
      }

      /*---------------------------------------------------------------------*
       * Request a semaphore from the kernel. Since the signal port is a shared
       * resource we need to have all actions on it be mutually exclusive.
       *---------------------------------------------------------------------*/
      gat_panTiltDev[c_index].t_mutex = arb_semaphoreCreate( MUTEX);

      if( gat_panTiltDev[c_index].t_mutex < 0)
      {
         t_err = (t_error)gat_panTiltDev[c_index].t_mutex;
         goto exit1;

      }/*End if( gat_templateDev[c_index].t_mutex < 0)*/

      /*---------------------------------------------------------------------*
       * We dont have any users attached to this device
       *---------------------------------------------------------------------*/
      gat_panTiltDev[c_index].c_numUsers = 0;

      /*------------------------------------------------------------------------*
       * Grab a handle to the timer we are going to use.
       *------------------------------------------------------------------------*/
      gat_panTiltDev[c_index].t_timer = hal_requestTimer( PANTILT_TIMER);

      if( gat_panTiltDev[c_index].t_timer < 0)
      {
         t_err = ARB_HAL_ERROR;
         goto exit2;

      }/*End if( gat_panTiltDev[c_index].t_timer < 0)*/

      /*------------------------------------------------------------------------*
       * Configure the timer for PWM mode
       *------------------------------------------------------------------------*/
      t_config.t_mode   = SINGLE_SLOPE;
      t_config.t_dir    = DIRECTION_UP;
      t_config.f_period = PANTILT_TIMER_PERIOD;
   
      if( hal_configureTimer( gat_panTiltDev[c_index].t_timer, 
                              t_config) < 0)
      {
         t_err = ARB_HAL_ERROR;
         goto exit3;
      }

      /*------------------------------------------------------------------------*
       * Enable the Pan compare channel
       *------------------------------------------------------------------------*/
      if( hal_enableCompareChannel( gat_panTiltDev[c_index].t_timer, 
                                    COMPAREA) < 0)
      {
         t_err = ARB_HAL_ERROR;
         goto exit3;
      }

      /*------------------------------------------------------------------------*
       * Enable the Tilt compare channel
       *------------------------------------------------------------------------*/
      if( hal_enableCompareChannel( gat_panTiltDev[c_index].t_timer, 
                                    COMPAREB) < 0)
      {
         t_err = ARB_HAL_ERROR;
         goto exit3;
      }

      /*------------------------------------------------------------------------*
       * Set the Pan and Tilt pins as outputs.
       *------------------------------------------------------------------------*/
      t_conf.c_inputMask  = 0;
      t_conf.c_outputMask = PANTILT_PAN_PIN | PANTILT_TILT_PIN;
      t_conf.t_outConf    = TOTEM;

      if( hal_configureGpioPort( PANTILT_PORT, 
                                 t_conf) < 0)
      {
         t_err = ARB_HAL_ERROR;
         goto exit3;
      }
          
   	/*------------------------------------------------------------------------*
       * Give the PWM port an initial pulse width to drive the PAN servo to our
       * known reference position which we will call '0 degrees'.
       *------------------------------------------------------------------------*/
      if( hal_setCompareValue( gat_panTiltDev[c_index].t_timer, 
                               COMPAREA,
                               PAN_ZERO_DEGREES) < 0)
      {
         t_err = ARB_HAL_ERROR;
         goto exit3;
      }
                                                	
      gat_panTiltDev[c_index].c_posPan = 0;

   	/*------------------------------------------------------------------------*
       * Give the PWM port an initial pulse width to drive the TILT servo to our
       * known reference position which we will call '0 degrees'.
       *------------------------------------------------------------------------*/
      if( hal_setCompareValue( gat_panTiltDev[c_index].t_timer,
                               COMPAREB,
                               TILT_ZERO_DEGREES) < 0)
      {
         t_err = ARB_HAL_ERROR;
         goto exit3;
      }

      gat_panTiltDev[c_index].c_posTilt = 0;

      /*------------------------------------------------------------------------*
       * Start the timer.
       *------------------------------------------------------------------------*/
      panTiltTimerStart();

   }/*End for( c_index = 0; c_index < PANTILT_MAX_MINORS; c_index++)*/

   HAL_END_CRITICAL(); //Enable interrupts

   return t_err;

exit3:

   hal_releaseTimer( gat_panTiltDev[c_index].t_timer);

exit2:

   arb_semaphoreDestroy( &gat_panTiltDev[c_index].t_mutex);

exit1:

   arb_destroyDevice( ac_devName);
   HAL_END_CRITICAL(); //Enable interrupts

   return t_err;

}/*End drv_panTiltInit*/

/*---------------------------------------------------------------------------*
 * Remove the device from the system
 *---------------------------------------------------------------------------*/
void drv_panTiltExit( void)
{

   uint8_t c_index    = 0;
   char ac_devName[MAX_DEVICE_NAME_BYTES];

   /*------------------------------------------------------------------------*
    * We are going to be removing items from memory that may be being used
    * by another process.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   for( c_index = 0; c_index < PANTILT_MAX_MINORS; c_index++)
   {
      if( gat_panTiltDev[c_index].t_mutex != 0) /*If created... destroy*/
      {
         sprintf(ac_devName, "panTiltDevice%d",c_index);
         arb_semaphoreDestroy( &gat_panTiltDev[c_index].t_mutex);
         hal_releaseTimer( gat_panTiltDev[c_index].t_timer);
         arb_destroyDevice( ac_devName);

         /*------------------------------------------------------------------*
          * Remove any user-space specific generated memory here...
          *------------------------------------------------------------------*/
      }/*End if( gat_panTiltDev[c_index].t_mutex != 0)*/

   }/*End for( c_index = 0; c_index < PANTILT_MAX_MINORS; c_index++)*/

   HAL_END_CRITICAL(); //Enable interrupts

}/*End drv_panTiltExit*/

