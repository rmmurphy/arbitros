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
 * File Name   : hal_adc.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for a device
 *               specific ADC module.
 *
 * Last Update : Sept 22, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <avr\pgmspace.h>
#include "hal_adc.h"
#include "utl_linkedList.h"
#include "hal_pmic.h"
#include "hal_clocks.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define ADC_INTERNAL_INPUT_MAX_CONV_FREQ (125000)  /*hz*/
#define ADC_MAX_CONV_FREQ                (2000000) /*hz*/
#define ADC_MIN_CONV_FREQ                (100000)  /*hz*/
#define ADC_NUM_OF_MODULES               (2)
#define ADC_NUM_CHAN_PER_MOD             (4)

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef enum
{
   ADC1_CHAN0 = 0, /*ADCA channel 0*/
   ADC1_CHAN1,
   ADC1_CHAN2,
   ADC1_CHAN3,
   ADC2_CHAN0,     /*ADCB channel 0*/
   ADC2_CHAN1,
   ADC2_CHAN2,
   ADC2_CHAN3

}t_chanId;

typedef struct
{
   /*------------------------------------------------------------------------*
    * The module associated with this particular ADC virtual channel.
    *------------------------------------------------------------------------*/
   t_adcModuleId t_module;

   /*------------------------------------------------------------------------*
    * A unique identification number which represents the particular channel
    * this ADC is configured for.
    *------------------------------------------------------------------------*/
   t_chanId t_id;

   /*------------------------------------------------------------------------*
    * Pointer to the ADC virtual channel register this handle is accessing.
    *------------------------------------------------------------------------*/
   ADC_CH_t *pt_chan;

   /*------------------------------------------------------------------------*
    * The sample returned from the previous ADC conversion.
    *------------------------------------------------------------------------*/
   int16_t s_adcSample;

   /*------------------------------------------------------------------------*
    * Pointer to the conversion complete call-back function
    *------------------------------------------------------------------------*/
   void (*pf_funPtr)( int16_t s_sample);

}t_chanHandle;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_LINKHNDL createChanHandle( t_chanId t_id);

static t_adcError isGainValid( t_adcChanConf *pt_chan,
                               ADC_CH_GAIN_t *pt_gain);

static t_chanHandle *adc_findChannelElement( t_chanId t_id);

static t_LINKHNDL createChannelAdcA( void);

static t_LINKHNDL createChannelAdcB( void);

static int32_t getAdcModuleConvRate( t_adcModuleId t_module);

static uint8_t ReadCalibrationByte( uint8_t index);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * If true, a particular ADC module has been configured
 *---------------------------------------------------------------------------*/
static bool gt_adcModConfigured[ADC_NUM_OF_MODULES] = {false, false};

/*---------------------------------------------------------------------------*
 * Linked list of all the ADC channels opened up against a particular ADC
 * module.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_adcChanList);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
ISR( ADCA_CH0_vect)
{
   t_chanHandle *pt_element = NULL;

   pt_element = adc_findChannelElement( ADC1_CHAN0);
   if( pt_element != NULL)
   {
      /*---------------------------------------------------------------------*
       * Execute the call-back function.
       *---------------------------------------------------------------------*/
       pt_element->s_adcSample = ADCA.CH0.RES;
       pt_element->pf_funPtr( pt_element->s_adcSample);

   }/*End if( pt_element != NULL)*/

}

ISR( ADCA_CH1_vect)
{
   t_chanHandle *pt_element = NULL;

   pt_element = adc_findChannelElement( ADC1_CHAN1);
   if( pt_element != NULL)
   {
      /*---------------------------------------------------------------------*
       * Execute the call-back function.
       *---------------------------------------------------------------------*/
       pt_element->s_adcSample = ADCA.CH1.RES;
       pt_element->pf_funPtr( pt_element->s_adcSample);

   }/*End if( pt_element != NULL)*/

}

ISR( ADCA_CH2_vect)
{
   t_chanHandle *pt_element = NULL;

   pt_element = adc_findChannelElement( ADC1_CHAN2);
   if( pt_element != NULL)
   {
      /*---------------------------------------------------------------------*
       * Execute the call-back function.
       *---------------------------------------------------------------------*/
       pt_element->s_adcSample = ADCA.CH2.RES;
       pt_element->pf_funPtr( pt_element->s_adcSample);

   }/*End if( pt_element != NULL)*/

}

ISR( ADCA_CH3_vect)
{
   t_chanHandle *pt_element = NULL;

   pt_element = adc_findChannelElement( ADC1_CHAN3);
   if( pt_element != NULL)
   {
      /*---------------------------------------------------------------------*
       * Execute the call-back function.
       *---------------------------------------------------------------------*/
       pt_element->s_adcSample = ADCA.CH3.RES;
       pt_element->pf_funPtr( pt_element->s_adcSample);

   }/*End if( pt_element != NULL)*/

}

ISR( ADCB_CH0_vect)
{
   t_chanHandle *pt_element = NULL;

   pt_element = adc_findChannelElement( ADC2_CHAN0);
   if( pt_element != NULL)
   {
      /*---------------------------------------------------------------------*
       * Execute the call-back function.
       *---------------------------------------------------------------------*/
       pt_element->s_adcSample = ADCB.CH0.RES;
       pt_element->pf_funPtr( pt_element->s_adcSample);

   }/*End if( pt_element != NULL)*/

}

ISR( ADCB_CH1_vect)
{
   t_chanHandle *pt_element = NULL;

   pt_element = adc_findChannelElement( ADC2_CHAN1);
   if( pt_element != NULL)
   {
      /*---------------------------------------------------------------------*
       * Execute the call-back function.
       *---------------------------------------------------------------------*/
       pt_element->s_adcSample = ADCB.CH1.RES;
       pt_element->pf_funPtr( pt_element->s_adcSample);

   }/*End if( pt_element != NULL)*/

}

ISR( ADCB_CH2_vect)
{
   t_chanHandle *pt_element = NULL;

   pt_element = adc_findChannelElement( ADC2_CHAN2);
   if( pt_element != NULL)
   {
      /*---------------------------------------------------------------------*
       * Execute the call-back function.
       *---------------------------------------------------------------------*/
       pt_element->s_adcSample = ADCB.CH2.RES;
       pt_element->pf_funPtr( pt_element->s_adcSample);

   }/*End if( pt_element != NULL)*/

}

ISR( ADCB_CH3_vect)
{
   t_chanHandle *pt_element = NULL;

   pt_element = adc_findChannelElement( ADC2_CHAN3);
   if( pt_element != NULL)
   {
      /*---------------------------------------------------------------------*
       * Execute the call-back function.
       *---------------------------------------------------------------------*/
       pt_element->s_adcSample = ADCB.CH3.RES;
       pt_element->pf_funPtr( pt_element->s_adcSample);

   }/*End if( pt_element != NULL)*/

}

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static uint8_t ReadCalibrationByte( uint8_t index)
{
   uint8_t result;

   /* Load the NVM Command register to read the calibration row. */
   NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
   result = pgm_read_byte(index);

   /* Clean up NVM Command register. */
   NVM_CMD = NVM_CMD_NO_OPERATION_gc;

   return( result );
}/*End ReadCalibrationByte*/

static t_LINKHNDL createChanHandle( t_chanId t_id)
{
   t_chanHandle *pt_element;
   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Allocated memory for the link where the ADC channel information will be
    * stored.
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof(t_chanHandle));

   if( t_linkHndl < 0)
   {
      return (t_LINKHNDL)ADC_OUT_OF_HEAP;
   }/*End if( t_linkHndl < 0)*/

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the ADC channel
    * information is being stored.
    *------------------------------------------------------------------------*/
   pt_element = (t_chanHandle *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);

   pt_element->t_id        = t_id;
   pt_element->s_adcSample = 0;
   pt_element->pf_funPtr   = NULL;

   switch( t_id)
   {
      case ADC1_CHAN0:
         pt_element->pt_chan = &ADCA.CH0;
      break;

      case ADC1_CHAN1:
         pt_element->pt_chan = &ADCA.CH1;
      break;

      case ADC1_CHAN2:
         pt_element->pt_chan = &ADCA.CH2;
      break;

      case ADC1_CHAN3:
         pt_element->pt_chan = &ADCA.CH3;
      break;

      case ADC2_CHAN0:
         pt_element->pt_chan = &ADCB.CH0;
      break;

      case ADC2_CHAN1:
         pt_element->pt_chan = &ADCB.CH1;
      break;

      case ADC2_CHAN2:
         pt_element->pt_chan = &ADCB.CH2;
      break;

      case ADC2_CHAN3:
         pt_element->pt_chan = &ADCB.CH3;
      break;

   }/*End switch( t_id)*/

   /*---------------------------------------------------------------------*
    * Add the ADC channel link onto the list open channels.
    *---------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_adcChanList,
                           t_linkHndl,
                           true);

   return t_linkHndl;

}/*End createChanHandle*/

static t_chanHandle *adc_findChannelElement( t_chanId t_id)
{
   t_LINKHNDL t_linkHndl;
   t_chanHandle *pt_element;
   int16_t s_count;

   /*------------------------------------------------------------------------*
    * Find the ADC channel with an ID of t_id
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_adcChanList, s_count)
   {
      pt_element = (t_chanHandle *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( pt_element->t_id == t_id)
      {
         return pt_element;
      }

   }

   /*------------------------------------------------------------------------*
    * If we make it this far the ID has not been found in the open ADC
    * channel list.
    *------------------------------------------------------------------------*/
   return NULL;

}/*End adc_findChannelElement*/

static t_adcError isGainValid( t_adcChanConf *pt_chan,
                               ADC_CH_GAIN_t *pt_gain)
{
   switch( pt_chan->t_gain)
   {
      case GAIN_1X:
         *pt_gain = ADC_CH_GAIN_1X_gc;
      break;
      case GAIN_2X:
         *pt_gain = ADC_CH_GAIN_2X_gc;
      break;
      case GAIN_4X:
         *pt_gain = ADC_CH_GAIN_4X_gc;
      break;
      case GAIN_8X:
         *pt_gain = ADC_CH_GAIN_8X_gc;
      break;
      case GAIN_16X:
         *pt_gain = ADC_CH_GAIN_16X_gc;
      break;
      case GAIN_32X:
         *pt_gain = ADC_CH_GAIN_32X_gc;
      break;
      case GAIN_64X:
         *pt_gain = ADC_CH_GAIN_64X_gc;
      break;
      default:
         return ADC_INVALID_GAIN;
      break;
   }/*End switch( pt_chan->t_gain)*/

   return ADC_PASSED;

}/*End isGainValid*/

t_adcError hal_configureAdcModule( t_adcModuleId t_module,
                                   t_adcModConf  t_modConf)
{

   ADC_t *pt_adcReg = NULL;
   t_adcError t_err = ADC_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( t_module == ADC1_MODULE)
      pt_adcReg = &ADCA;
   else
      pt_adcReg = &ADCB;

   /*------------------------------------------------------------------------*
    * Disable the ADC module
    *------------------------------------------------------------------------*/
   pt_adcReg->CTRLA &= ~ADC_ENABLE_bm;

   switch( t_modConf.t_mode)
   {
      case SIGNED:
         pt_adcReg->CTRLB |= ADC_CONMODE_bm;
      break;

      case UNSIGNED:
         pt_adcReg->CTRLB &= ~ADC_CONMODE_bm;
      break;

      default:
         t_err = ADC_INVALID_CONMODE;
      break;

   }/*End switch( pt_adc->t_mode)*/

   switch( t_modConf.t_res)
   {
      case RES_12BIT:
         pt_adcReg->CTRLB &= ~ADC_RESOLUTION_gm; /*Clear old setting*/
         pt_adcReg->CTRLB |= ADC_RESOLUTION_12BIT_gc;
      break;

      case RES_8BIT:
         pt_adcReg->CTRLB &= ~ADC_RESOLUTION_gm; /*Clear old setting*/
         pt_adcReg->CTRLB |= ADC_RESOLUTION_8BIT_gc;
      break;

      default:
         t_err = ADC_INVALID_RES;
      break;

   }/*End switch( t_modConf.t_res)*/

   switch( t_modConf.t_ref)
   {
      case INTERNAL_1VOLT:
         pt_adcReg->REFCTRL &= ~ADC_REFSEL_gm; /*Clear old setting*/
         pt_adcReg->REFCTRL |= ADC_REFSEL_INT1V_gc;
      break;

      case INTERNAL_VCC_OVER_1Pnt6:
         pt_adcReg->REFCTRL &= ~ADC_REFSEL_gm; /*Clear old setting*/
         pt_adcReg->REFCTRL |= ADC_REFSEL_VCC_gc;
      break;

      case EXTERNAL_PORTA:
         pt_adcReg->REFCTRL &= ~ADC_REFSEL_gm; /*Clear old setting*/
         pt_adcReg->REFCTRL |= ADC_REFSEL_AREFA_gc;
      break;

      case EXTERNAL_PORTB:
         pt_adcReg->REFCTRL &= ~ADC_REFSEL_gm; /*Clear old setting*/
         pt_adcReg->REFCTRL |= ADC_REFSEL_AREFB_gc;
      break;

      default:
         t_err = ADC_INVALID_REF;
      break;

   }/*End switch( t_modConf.t_ref)*/

   switch( t_modConf.t_clock)
   {
      case PERIPH_CLOCK_OVER_4:

         if( ((hal_getCpuFreq() >> 2) <= ADC_MAX_CONV_FREQ)
            && ((hal_getCpuFreq() >> 2) >= ADC_MIN_CONV_FREQ))
         {
            pt_adcReg->PRESCALER &= ~ADC_PRESCALER_gm;
            pt_adcReg->PRESCALER |= ADC_PRESCALER_DIV4_gc;
         }
         else
            t_err = ADC_INVALID_CLOCK;

      break;

      case PERIPH_CLOCK_OVER_8:

         if( ((hal_getCpuFreq() >> 3) <= ADC_MAX_CONV_FREQ)
            && ((hal_getCpuFreq() >> 3) >= ADC_MIN_CONV_FREQ))
         {
            pt_adcReg->PRESCALER &= ~ADC_PRESCALER_gm;
            pt_adcReg->PRESCALER |= ADC_PRESCALER_DIV8_gc;
         }
         else
            t_err = ADC_INVALID_CLOCK;

      break;

      case PERIPH_CLOCK_OVER_16:

         if( ((hal_getCpuFreq() >> 4) <= ADC_MAX_CONV_FREQ)
            && ((hal_getCpuFreq() >> 4) >= ADC_MIN_CONV_FREQ))
         {
            pt_adcReg->PRESCALER &= ~ADC_PRESCALER_gm;
            pt_adcReg->PRESCALER |= ADC_PRESCALER_DIV16_gc;
         }
         else
            t_err = ADC_INVALID_CLOCK;

      break;

      case PERIPH_CLOCK_OVER_32:

         if( ((hal_getCpuFreq() >> 5) <= ADC_MAX_CONV_FREQ)
            && ((hal_getCpuFreq() >> 5) >= ADC_MIN_CONV_FREQ))
         {
            pt_adcReg->PRESCALER &= ~ADC_PRESCALER_gm;
            pt_adcReg->PRESCALER |= ADC_PRESCALER_DIV32_gc;
         }
         else
            t_err = ADC_INVALID_CLOCK;

      break;

      case PERIPH_CLOCK_OVER_64:

         if( ((hal_getCpuFreq() >> 6) <= ADC_MAX_CONV_FREQ)
            && ((hal_getCpuFreq() >> 6) >= ADC_MIN_CONV_FREQ))
         {
            pt_adcReg->PRESCALER &= ~ADC_PRESCALER_gm;
            pt_adcReg->PRESCALER |= ADC_PRESCALER_DIV64_gc;
         }
         else
            t_err = ADC_INVALID_CLOCK;

      break;

      case PERIPH_CLOCK_OVER_128:

         if( ((hal_getCpuFreq() >> 7) <= ADC_MAX_CONV_FREQ)
            && ((hal_getCpuFreq() >> 7) >= ADC_MIN_CONV_FREQ))
         {
            pt_adcReg->PRESCALER &= ~ADC_PRESCALER_gm;
            pt_adcReg->PRESCALER |= ADC_PRESCALER_DIV128_gc;
         }
         else
            t_err = ADC_INVALID_CLOCK;

      break;

      case PERIPH_CLOCK_OVER_256:

         if( ((hal_getCpuFreq() >> 8) <= ADC_MAX_CONV_FREQ)
            && ((hal_getCpuFreq() >> 8) >= ADC_MIN_CONV_FREQ))
         {
            pt_adcReg->PRESCALER &= ~ADC_PRESCALER_gm;
            pt_adcReg->PRESCALER |= ADC_PRESCALER_DIV256_gc;
         }
         else
            t_err = ADC_INVALID_CLOCK;

      break;

      case PERIPH_CLOCK_OVER_512:

         if( ((hal_getCpuFreq() >> 9) <= ADC_MAX_CONV_FREQ)
            && ((hal_getCpuFreq() >> 9) >= ADC_MIN_CONV_FREQ))
         {
            pt_adcReg->PRESCALER &= ~ADC_PRESCALER_gm;
            pt_adcReg->PRESCALER |= ADC_PRESCALER_DIV512_gc;
         }
         else
            t_err = ADC_INVALID_CLOCK;

      break;

      default:

         t_err = ADC_INVALID_CLOCK;

      break;

   }/*End switch( t_modConf.t_clock)*/

   if( t_err == ADC_PASSED)
   {

      /*---------------------------------------------------------------------*
       * Create a region of memory that holds the active channels opened
       * against this particular ADC module.
       *---------------------------------------------------------------------*/
      gt_adcModConfigured[t_module] = true;

      /*---------------------------------------------------------------------*
       * Enable the ADC module
       *---------------------------------------------------------------------*/
      pt_adcReg->CTRLA |= ADC_ENABLE_bm;

   }

   HAL_END_CRITICAL(); //Enable interrupts

   return t_err;

}/*End hal_configureAdcModule*/

static t_LINKHNDL createChannelAdcA( void)
{
   t_chanId t_chan  = ADC1_CHAN0;
   t_chanHandle *pt_chanElement = NULL;
   t_LINKHNDL t_linkHndl;

   /*------------------------------------------------------------------------*
    * The first channel to return NULL means that the channel is open.
    *------------------------------------------------------------------------*/
   for( t_chan = ADC1_CHAN0; t_chan <= ADC1_CHAN3; t_chan++)
   {
      pt_chanElement = adc_findChannelElement( t_chan);
      if( pt_chanElement == NULL) /*Didn't find the channel*/
         break;

   }/*End for( t_chan = ADC1_CHAN0; t_chan <= ADC1_CHAN3; t_chan++)*/

   /*------------------------------------------------------------------------*
    * Is there an open channel?
    *------------------------------------------------------------------------*/
   if( pt_chanElement == NULL) /*Yes*/
   {
      t_linkHndl = createChanHandle( t_chan);
      if( t_linkHndl < 0)
         return (t_LINKHNDL)ADC_OUT_OF_HEAP;

   }
   else
      return (t_LINKHNDL)ADC_NO_CHAN_AVAILABLE;

   return t_linkHndl;

}/*End createChannelAdcA*/

static t_LINKHNDL createChannelAdcB( void)
{
   t_chanId t_chan  = ADC2_CHAN0;
   t_chanHandle *pt_chanElement = NULL;
   t_LINKHNDL t_linkHndl;

   /*------------------------------------------------------------------------*
    * The first channel to return NULL means that the channel is open.
    *------------------------------------------------------------------------*/
   for( t_chan = ADC2_CHAN0; t_chan <= ADC2_CHAN3; t_chan++)
   {
      pt_chanElement = adc_findChannelElement( t_chan);
      if( pt_chanElement == NULL) /*Didn't find the channel*/
         break;

   }/*End for( t_chan = ADC2_CHAN0; t_chan <= ADC2_CHAN3; t_chan++)*/

   /*------------------------------------------------------------------------*
    * Is there an open channel?
    *------------------------------------------------------------------------*/
   if( pt_chanElement == NULL) /*Yes*/
   {
      t_linkHndl = createChanHandle( t_chan);
      if( t_linkHndl < 0)
         return (t_LINKHNDL)ADC_OUT_OF_HEAP;

   }
   else
      return (t_LINKHNDL)ADC_NO_CHAN_AVAILABLE;

   return t_linkHndl;

}/*End createChannelAdcB*/

static int32_t getAdcModuleConvRate( t_adcModuleId t_module)
{
   ADC_t *pt_adcReg = NULL;
   int32_t i_rate;

   if( t_module == ADC1_MODULE)
   {
      pt_adcReg = &ADCA;
   }
   else if( t_module == ADC2_MODULE)
   {
      pt_adcReg = &ADCB;
   }
   else
   {
      return (int32_t)ADC_INVALID_MODULE;
   }

   if( pt_adcReg->PRESCALER == ADC_PRESCALER_DIV4_gc)
      i_rate = hal_getCpuFreq() >> 2;
   else if ( pt_adcReg->PRESCALER == PERIPH_CLOCK_OVER_8)
      i_rate = hal_getCpuFreq() >> 3;
   else if ( pt_adcReg->PRESCALER == PERIPH_CLOCK_OVER_16)
      i_rate = hal_getCpuFreq() >> 4;
   else if ( pt_adcReg->PRESCALER == PERIPH_CLOCK_OVER_32)
      i_rate = hal_getCpuFreq() >> 5;
   else if ( pt_adcReg->PRESCALER == PERIPH_CLOCK_OVER_64)
      i_rate = hal_getCpuFreq() >> 6;
   else if ( pt_adcReg->PRESCALER == PERIPH_CLOCK_OVER_128)
      i_rate = hal_getCpuFreq() >> 7;
   else if ( pt_adcReg->PRESCALER == PERIPH_CLOCK_OVER_256)
      i_rate = hal_getCpuFreq() >> 8;
   else
      i_rate = hal_getCpuFreq() >> 9;

   return i_rate;

}/*End getAdcModuleConvRate*/

/*---------------------------------------------------------------------------*
 * Grab the next available open channel on a particular ADC module (ADCA or
 * ADCB).
 *---------------------------------------------------------------------------*/
t_ADCCHANHNDL hal_requestAdcChannel( t_adcModuleId t_module,
                                     t_adcChanConf t_chanConf)
{

   ADC_t *pt_adcReg                = NULL;
   t_chanHandle *pt_element        = NULL;
   t_LINKHNDL t_linkHndl;
   PORT_t *pt_port                 = NULL;
   t_adcError t_err                = ADC_PASSED;
   ADC_CH_MUXPOS_t t_posMux        = 0;
   ADC_CH_MUXNEG_t t_negMux        = 0;
   ADC_CH_MUXINT_t t_internalInput = 0;
   ADC_CH_INPUTMODE_t t_inputMode  = 0;
   ADC_CH_GAIN_t t_gain            = 0;
   ADC_CH_INTLVL_t t_intLevel      = 0;
   ADC_CH_INTMODE_t t_intMode      = 0;
   t_linkedListError t_lErr;
   t_gpioConf t_conf;
   t_gpioError t_gErr;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( t_module >= ADC_NUM_OF_MODULES)
   {
      HAL_END_CRITICAL(); //Enable interrupts
      return (t_ADCCHANHNDL)ADC_INVALID_MODULE;
   }
   else
   {
      if( gt_adcModConfigured[t_module] == false)
      {
         HAL_END_CRITICAL(); //Enable interrupts
         return (t_ADCCHANHNDL)ADC_MODULE_NOT_INIT;
      }
   }

   if( t_module == ADC1_MODULE)
   {
      /*---------------------------------------------------------------------*
       * Search for the next available ADCA channel.
       *---------------------------------------------------------------------*/
      t_linkHndl = createChannelAdcA();
      if( t_linkHndl < 0)
      {
         HAL_END_CRITICAL(); //Enable interrupts
         return (t_ADCCHANHNDL)ADC_OUT_OF_HEAP;
      }

      pt_adcReg = &ADCA;
      pt_port   = &PORTA;

   }/*End if( t_module == ADC1_MODULE)*/
   else if( t_module == ADC2_MODULE)
   {
      /*---------------------------------------------------------------------*
       * Search for the next available ADCB channel.
       *---------------------------------------------------------------------*/
      t_linkHndl = createChannelAdcB();
      if( t_linkHndl < 0)
      {
         HAL_END_CRITICAL(); //Enable interrupts
         return (t_ADCCHANHNDL)ADC_OUT_OF_HEAP;
      }

      pt_adcReg = &ADCB;
      pt_port   = &PORTB;

   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the ADC channel
    * information is being stored.
    *------------------------------------------------------------------------*/
   pt_element = (t_chanHandle *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);

   switch( t_chanConf.t_inMode)
   {
      case SINGLE_ENDED_EXT:
         t_inputMode  = ADC_CH_INPUTMODE_SINGLEENDED_gc;
      break;

      case DIFFERENTIAL_NO_GAIN_EXT:

         if( (pt_adcReg->CTRLB & ADC_CONMODE_bm) > 0)
         {
           t_inputMode = ADC_CH_INPUTMODE_DIFF_gc;
         }
         else
            t_err = ADC_INVALID_CONMODE;

      break;

      case DIFFERENTIAL_WITH_GAIN_EXT:

         if( (pt_adcReg->CTRLB & ADC_CONMODE_bm) > 0)
         {
             if( isGainValid( &t_chanConf, &t_gain) == 0)
             {
                t_inputMode = ADC_CH_INPUTMODE_DIFFWGAIN_gc;
             }
             else
                t_err = ADC_INVALID_GAIN;
         }
         else
            t_err = ADC_INVALID_CONMODE;

      break;

      case INTERNAL_TEMP:

         if( getAdcModuleConvRate( t_module) <= ADC_INTERNAL_INPUT_MAX_CONV_FREQ)
         {
            t_inputMode     = ADC_CH_INPUTMODE_INTERNAL_gc;
            t_internalInput = ADC_CH_MUXINT_TEMP_gc;
         }
         else
            t_err = ADC_INVALID_CLOCK;

      break;

      case INTERNAL_BANDGAP:

         if( getAdcModuleConvRate( t_module) <= ADC_INTERNAL_INPUT_MAX_CONV_FREQ)
         {
            t_inputMode     = ADC_CH_INPUTMODE_INTERNAL_gc;
            t_internalInput = ADC_CH_MUXINT_BANDGAP_gc;
         }
         else
            t_err = ADC_INVALID_CLOCK;
      break;

      case INTERNAL_ONE_TENTH_VCC:

         if( getAdcModuleConvRate( t_module) <= ADC_INTERNAL_INPUT_MAX_CONV_FREQ)
         {
            t_inputMode     = ADC_CH_INPUTMODE_INTERNAL_gc;
            t_internalInput = ADC_CH_MUXINT_SCALEDVCC_gc;
         }
         else
            t_err = ADC_INVALID_CLOCK;

      break;

      case INTERNAL_DAC:

         if( getAdcModuleConvRate( t_module) <= ADC_INTERNAL_INPUT_MAX_CONV_FREQ)
         {
            t_inputMode = ADC_CH_INPUTMODE_INTERNAL_gc;
            t_internalInput = ADC_CH_MUXINT_DAC_gc;
         }
         else
            t_err = ADC_INVALID_CLOCK;

      break;

      default:
         t_err = ADC_INVALID_INMODE;
      break;

   }/*End switch( t_chanConf.t_inMode)*/

   if( t_inputMode != ADC_CH_INPUTMODE_INTERNAL_gc)
   {
      switch( t_chanConf.c_posPin)
      {
         case PIN_0:
            t_posMux = ADC_CH_MUXPOS_PIN0_gc;
         break;

         case PIN_1:
            t_posMux = ADC_CH_MUXPOS_PIN1_gc;
         break;

         case PIN_2:
            t_posMux = ADC_CH_MUXPOS_PIN2_gc;
         break;

         case PIN_3:
            t_posMux = ADC_CH_MUXPOS_PIN3_gc;
         break;

         case PIN_4:
            t_posMux = ADC_CH_MUXPOS_PIN4_gc;
         break;

         case PIN_5:
            t_posMux = ADC_CH_MUXPOS_PIN5_gc;
         break;

         case PIN_6:
            t_posMux = ADC_CH_MUXPOS_PIN6_gc;
         break;

         case PIN_7:
            t_posMux = ADC_CH_MUXPOS_PIN7_gc;
         break;

         default:
            t_err = ADC_INVALID_PIN;
         break;

      }/*End switch( t_chanConf.c_posPin)*/

      /*---------------------------------------------------------------------*
       * Configure the input pins the ADC will use.
       *---------------------------------------------------------------------*/
      if( t_module == ADC1_MODULE)
      {
         t_conf.c_inputMask  = t_chanConf.c_posPin | t_chanConf.c_negPin;
         t_conf.c_outputMask = 0;
         t_conf.t_inConf     = TOTEM;
         t_conf.t_outConf    = TOTEM;
         t_gErr = hal_configureGpioPort( GPIO_PORTA, t_conf);

         ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
         ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
      }
      else
      {
         t_conf.c_inputMask  = t_chanConf.c_posPin | t_chanConf.c_negPin;
         t_conf.c_outputMask = 0;
         t_conf.t_inConf     = TOTEM;
         t_conf.t_outConf    = TOTEM;
         t_gErr = hal_configureGpioPort( GPIO_PORTB, t_conf);

         ADCB.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCBCAL0) );
         ADCB.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCBCAL1) );
      }

      /*---------------------------------------------------------------------*
       * Negative pin configuration is needed if we are in differential mode.
       *---------------------------------------------------------------------*/
      if( t_chanConf.t_inMode == DIFFERENTIAL_NO_GAIN_EXT)
      {
         //if( t_chanConf.c_posPin == t_chanConf.c_negPin)
         //   t_err = ADC_POSNEG_PINS_EQUAL;

         switch( t_chanConf.c_negPin)
         {
            case PIN_0:
               t_negMux = ADC_CH_MUXNEG_PIN0_gc;
            break;

            case PIN_1:
               t_negMux = ADC_CH_MUXNEG_PIN1_gc;
            break;

            case PIN_2:
               t_negMux = ADC_CH_MUXNEG_PIN2_gc;
            break;

            case PIN_3:
               t_negMux = ADC_CH_MUXNEG_PIN3_gc;
            break;

            default:
               t_err = ADC_INVALID_PIN;
            break;

         }/*End switch( t_chanConf.c_negPin)*/

      }
      else if( t_chanConf.t_inMode == DIFFERENTIAL_WITH_GAIN_EXT)
      {
         switch( t_chanConf.c_negPin)
         {
            case PIN_4:
               t_negMux = ADC_CH_MUXNEG_PIN0_gc;
            break;

            case PIN_5:
               t_negMux = ADC_CH_MUXNEG_PIN1_gc;
            break;

            case PIN_6:
               t_negMux = ADC_CH_MUXNEG_PIN2_gc;
            break;

            case PIN_7:
               t_negMux = ADC_CH_MUXNEG_PIN3_gc;
            break;

            default:
               t_err = ADC_INVALID_PIN;
            break;

         }/*End switch( t_chanConf.c_negPin)*/

      }/*End else if( t_chanConf.t_inMode == DIFFERENTIAL_WITH_GAIN_EXT)*/

   }/*End if( t_inputMode != ADC_CH_INPUTMODE_INTERNAL_gc)*/

   if( t_chanConf.b_enableInt == true)
   {
      if( t_chanConf.pf_funPtr == NULL)
      {
         t_err = ADC_NULL_PTR;
      }
      else
      {
         t_intLevel = ADC_CH_INTLVL_HI_gc;
         t_intMode  = ADC_CH_INTMODE_COMPLETE_gc;
      }
   }
   else
   {
      t_intLevel = ADC_CH_INTLVL_OFF_gc;
   }

   if( t_err != ADC_PASSED)
   {
      /*---------------------------------------------------------------------*
       * Remove the device driver from the driver list.
       *---------------------------------------------------------------------*/
      t_lErr = utl_destroyLink( gt_adcChanList,
                                (t_LINKHNDL)t_linkHndl);

      HAL_END_CRITICAL(); //Enable interrupts

      return (t_ADCCHANHNDL)t_err;

   }/*End if( t_err != ADC_PASSED)*/
   else
   {
      /*---------------------------------------------------------------------*
       * Disable band gap and temp
       *---------------------------------------------------------------------*/
      pt_adcReg->REFCTRL &= ~(ADC_TEMPREF_bm | ADC_BANDGAP_bm);

      pt_element->pt_chan->CTRL &= ~ADC_CH_INPUTMODE_gm;
      pt_element->pt_chan->CTRL = t_inputMode;

      pt_element->pt_chan->CTRL &= ~ADC_CH_GAINFAC_gm;
      pt_element->pt_chan->CTRL |= t_gain;

      /*---------------------------------------------------------------------*
       * Map interrupt and interrupt handler.
       *---------------------------------------------------------------------*/
      if( t_intLevel == ADC_CH_INTLVL_OFF_gc)
      {
         pt_element->pt_chan->INTCTRL = ADC_CH_INTLVL_OFF_gc;
      }
      else
      {
         pt_element->pt_chan->INTCTRL &= ~ADC_CH_INTLVL_gm;
         pt_element->pt_chan->INTCTRL |= t_intLevel;

         pt_element->pt_chan->INTCTRL &= ~ADC_CH_INTMODE_gm;
         pt_element->pt_chan->INTCTRL |= t_intMode;

         pt_element->pf_funPtr = t_chanConf.pf_funPtr;
      }

      /*---------------------------------------------------------------------*
       * Map external pins.
       *---------------------------------------------------------------------*/
      if( t_inputMode != ADC_CH_INPUTMODE_INTERNAL_gc)
      {
         pt_element->pt_chan->MUXCTRL &= ~ADC_CH_MUXPOS_gm;
         pt_element->pt_chan->MUXCTRL |= t_posMux;

         pt_element->pt_chan->MUXCTRL &= ~ADC_CH_MUXNEG_gm;
         pt_element->pt_chan->MUXCTRL |= t_negMux;

      }/*End if( t_inputMode != ADC_CH_INPUTMODE_INTERNAL_gc)*/
      else
      {
         pt_element->pt_chan->MUXCTRL &= ~ADC_CH_MUXINT_gm;
         pt_element->pt_chan->MUXCTRL = t_internalInput;

         if( t_internalInput == ADC_CH_MUXINT_TEMP_gc)
         {
            /*---------------------------------------------------------------*
             * Enable the temperature reference.
             *---------------------------------------------------------------*/
            pt_adcReg->REFCTRL |= ADC_TEMPREF_bm;
         }/*End if( t_internalInput == ADC_CH_MUXINT_TEMP_gc)*/
         else if( t_internalInput == ADC_CH_MUXINT_BANDGAP_gc)
         {
            /*---------------------------------------------------------------*
             * Enable the band gap reference.
             *---------------------------------------------------------------*/
            pt_adcReg->REFCTRL |= ADC_BANDGAP_bm;
         }/*End else if( t_internalInput == ADC_CH_MUXINT_BANDGAP_gc)*/

      }

      pt_element->t_module = t_module;

      HAL_END_CRITICAL(); //Enable interrupts

      return (t_ADCCHANHNDL)t_linkHndl;
   }

}/*End hal_requestAdcChannel*/

/*---------------------------------------------------------------------------*
 * Release a currently open channel on a particular ADC module.
 *---------------------------------------------------------------------------*/
t_adcError hal_releaseAdcChannel( t_ADCCHANHNDL t_handle)
{

   t_linkedListError t_lErr;

   t_lErr = utl_destroyLink( gt_adcChanList,
                             (t_LINKHNDL)t_handle);

   if( t_lErr == LINKEDLIST_INVALID_LINK)
      return ADC_INVALID_HNDL;

   return ADC_PASSED;

}/*End hal_releaseAdcChannel*/

/*---------------------------------------------------------------------------*
 * Returns the last known sample gathered from the channel pointed to
 * by the handle.
 *---------------------------------------------------------------------------*/
int16_t hal_getAdcSample( t_ADCCHANHNDL t_handle)
{
   t_chanHandle *pt_element = NULL;

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the ADC channel
    * information is being stored.
    *------------------------------------------------------------------------*/
   pt_element = (t_chanHandle *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

   /*------------------------------------------------------------------------*
    * Are interrupts disabled?
    *------------------------------------------------------------------------*/
   return pt_element->s_adcSample;

}/*End hal_getAdcSample*/

t_adcError hal_startAdcConversion( t_ADCCHANHNDL t_handle)
{
   t_chanHandle *pt_element = NULL;

   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_adcChanList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return ADC_INVALID_HNDL;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the ADC
    * channel information is being stored.
    *------------------------------------------------------------------------*/
   pt_element = (t_chanHandle *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);
   pt_element->pt_chan->CTRL |= ADC_CH_START_bm;

   HAL_END_CRITICAL();//Enable interrupts

   return ADC_PASSED;

}/*End hal_startAdcConversion*/

