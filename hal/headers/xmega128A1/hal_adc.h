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
 * File Name   : hal_adc.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for a device
 *               specific ADC module.
 *
 * Last Update : Sept 22, 2011
 *---------------------------------------------------------------------------*/
#ifndef hal_adc_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define hal_adc_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "hal_gpio.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      ADC_MODULE_NOT_INIT   = -17,/*An ADC module has not been initialized*/
      ADC_INVALID_MODULE    = -16,/*Module does not exist*/
      ADC_INVALID_INT       = -15,/*Incorrect enabling of interrupts*/
      ADC_INVALID_HNDL      = -14,/*Handle not found*/
      ADC_NO_CHAN_AVAILABLE = -13,/*There are no channels left on the module*/
      ADC_POSNEG_PINS_EQUAL = -12,/*In diff mode pos pin = neg pin*/
      ADC_INVALID_INT_INPUT = -11,/*Incorrect ADC internal input setting*/
      ADC_INVALID_GAIN      = -10,/*Incorrect ADC gain*/
      ADC_INVALID_INMODE    = -9, /*Incorrect ADC input mode*/
      ADC_INVALID_CLOCK     = -8, /*Incorrect ADC reference clock*/
      ADC_INVALID_REF       = -7, /*Incorrect ADC reference voltage*/
      ADC_INVALID_RES       = -6, /*Incorrect ADC data resolution*/
      ADC_INVALID_CONMODE   = -5, /*Incorrect ADC conversion mode*/
      ADC_PIN_IS_OUTPUT     = -4, /*Trying to access a pin as an input that is
                                    configured as an output.*/
      ADC_NULL_PTR          = -3, /*Pointer is not mapped to a valid address.*/
      ADC_OUT_OF_HEAP       = -2, /*No more memory.*/
      ADC_INVALID_PIN       = -1, /*Trying to access an invalid pin.*/
      ADC_PASSED            = 0   /*Configuration good.*/

   }t_adcError;

   typedef enum
   {
      SIGNED = 0,
      UNSIGNED
   }t_convMode; /*The type of ADC measurement*/

   typedef enum
   {
      RES_12BIT = 0,
      RES_8BIT
   }t_mesResolution; /*The resolution of the ADC measurement*/

   typedef enum
   {
      INTERNAL_1VOLT = 0,
      INTERNAL_VCC_OVER_1Pnt6,
      EXTERNAL_PORTA,
      EXTERNAL_PORTB

   }t_refVoltage; /*The reference voltage for the ADC measurement*/

   typedef enum
   {
      PERIPH_CLOCK_OVER_4 = 0,
      PERIPH_CLOCK_OVER_8,
      PERIPH_CLOCK_OVER_16,
      PERIPH_CLOCK_OVER_32,
      PERIPH_CLOCK_OVER_64,
      PERIPH_CLOCK_OVER_128,
      PERIPH_CLOCK_OVER_256,
      PERIPH_CLOCK_OVER_512

   }t_refClock; /*The ADC reference clock*/

   typedef enum
   {
      SINGLE_ENDED_EXT = 0,       /*We are sampling a signal on a single pin*/
      DIFFERENTIAL_NO_GAIN_EXT,   /*Differentiating a signal across two pins without
                                   internal amplification*/
      DIFFERENTIAL_WITH_GAIN_EXT, /*Differentiating a signal across two pins
                                    with internal amplification*/
      INTERNAL_TEMP,              /*Sampling the internal temperature sensor*/
      INTERNAL_BANDGAP,           /*Sampling the internal bandgap volatage*/
      INTERNAL_ONE_TENTH_VCC,     /*Sampling a scaled version of VCC*/
      INTERNAL_DAC,               /*Sampling the DAC output*/
   }t_inputMode; /*The type of signal we are measuring*/

   typedef enum
   {
      GAIN_1X = 0,
      GAIN_2X,
      GAIN_4X,
      GAIN_8X,
      GAIN_16X,
      GAIN_32X,
      GAIN_64X

   }t_diffGain; /*Amplification factor for a differential input*/

   typedef enum
   {
      ADC1_MODULE = 0,
      ADC2_MODULE

   }t_adcModuleId;

   typedef struct
   {
      /*---------------------------------------------------------------------*
       * Configure all four channels for a particular ADC module for unsigned
       * or signed signal measurement.
       *---------------------------------------------------------------------*/
      t_convMode t_mode;

      /*---------------------------------------------------------------------*
       * Configure all four channels for a particular ADC module for a
       * specific measurement resolution - either 8-bit or 12-bit.
       *---------------------------------------------------------------------*/
      t_mesResolution t_res;

      /*---------------------------------------------------------------------*
       * Configure all four channels for a particular ADC module for a
       * specific measurement voltage reference.
       *---------------------------------------------------------------------*/
      t_refVoltage t_ref;

      /*---------------------------------------------------------------------*
       * Configure a particular ADC module for a specific reference clock.
       *---------------------------------------------------------------------*/
      t_refClock t_clock;

   }t_adcModConf; /*The parameters pertain to configuration of the entire ADC
                    module - which includes all 4 virtual channels.*/

   typedef struct
   {

      /*---------------------------------------------------------------------*
       * The positive input pin this ADC channel is mapped to. Where PIN0 = 1,
       * PIN1 = 2, PIN3 = 4, ...PIN7 = 128. See drv_gpio.h for pin
       * definitions.
       *---------------------------------------------------------------------*/
      uint8_t c_posPin;

      /*---------------------------------------------------------------------*
       * The negative input pin this ADC channel is mapped to. Where PIN0 = 1,
       * PIN1 = 2, PIN3 = 4, ...PIN7 = 128. See drv_gpio.h for pin
       * definitions. This pin is only used in differential mode.
       *---------------------------------------------------------------------*/
      uint8_t c_negPin;

      /*---------------------------------------------------------------------*
       * The type of signal the ADC is measuring, single ended, differential,
       * or internal.
       *---------------------------------------------------------------------*/
      t_inputMode t_inMode;

      /*---------------------------------------------------------------------*
       * If t_mode is set for DIFFERENTIAL_WITH_GAIN, then this is the ammount
       * of amplification applied to the input before performing an ADC
       * conversion on a particular channel.
       *---------------------------------------------------------------------*/
      t_diffGain t_gain;

      /*---------------------------------------------------------------------*
       * If true then an interrupt is generated on an ADC conversion complete.
       *---------------------------------------------------------------------*/
      bool b_enableInt;

      /*---------------------------------------------------------------------*
       * Pointer to the function that gets called when an interrupt is
       * generated.
       *---------------------------------------------------------------------*/
      void (*pf_funPtr)( int16_t s_sample);

   }t_adcChanConf;/*The parameters pertain to configuration of an individual
                    ADC virtual channel.*/

   typedef volatile int16_t t_ADCCHANHNDL; /*Handle to the ADC virtual
                                             channel*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_adcError hal_configureAdcModule( t_adcModuleId t_module,
                                      t_adcModConf t_modConf);

   t_ADCCHANHNDL hal_requestAdcChannel( t_adcModuleId t_module,
                                        t_adcChanConf t_chanConf);

   t_adcError hal_releaseAdcChannel( t_ADCCHANHNDL t_handle);

   int16_t hal_getAdcSample( t_ADCCHANHNDL t_handle);

   t_adcError hal_startAdcConversion( t_ADCCHANHNDL t_handle);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef hal_adc_h*/

