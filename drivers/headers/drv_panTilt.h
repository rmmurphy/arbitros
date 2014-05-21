/*---------------------------------------------------------------------------*
 * File Name   : drv_panTilt.c
 *
 * Description : This file is responsible for controlling a pan/tilt deivce
 *
 * Programmer  : Ryan M Murphy
 *
 * Date        : June, 15, 2011
 *---------------------------------------------------------------------------*/
#ifndef drv_panTilt_h

   #ifdef __cplusplus
   extern "C" {
   #endif
   
   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define drv_panTilt_h

   /*------------------------------------------------------------------------*
    * Here we define the minimum and maximum range of our PAN/TILT servos.
    * !!TODO move these inside the .c file
    *------------------------------------------------------------------------*/
   #define PAN_NEG_45_DEGREES  (750)  /*Pulse width in ticks*/
   #define PAN_ZERO_DEGREES    (1050)
   #define PAN_POS_45_DEGREES  (1500)

   #define TILT_NEG_45_DEGREES  (400)
   #define TILT_ZERO_DEGREES    (600)
   #define TILT_POS_45_DEGREES  (800) 
   
   /*------------------------------------------------------------------------*
    * Inlude Files
    *------------------------------------------------------------------------*/
    #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      PAN_ABSOLUTE = 0,
      PAN_RELATIVE,
      TILT_ABSOLUTE,
      TILT_RELATIVE,
      START_PWM,
      STOP_PWM

   }t_panTiltCmd;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_error drv_panTiltInit( void);

   void drv_panTiltExit( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif
   
#endif/*End #ifndef drv_panTilt_h*/
