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
 * File Name   : hal_gpio.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for configuring the
 *               input/output pins and managing interrupts on the eleven
 *               XMEGA gpios (PORTA, PORTB, ...PORTR). Interrupts are granted
 *               on a first-come first-serve basis and memory for them is
 *               held in a structure placed on a linked-list. The information
 *               in the structure contains the gpio ID (i.e. PORTA_INT0 = 0,
 *               PORTA_INT1 = 1), pin designated as interruptible, and ptrs
 *               to the adjacent entries in the list. There are two interrupts
 *               allowed per port (INT0 and INT1), each of which can be mapped
 *               to only one pin. The interrupts are configured in order-
 *               meaning the first interruptible pin on port 'x' will be
 *               mapped to 'int 0'. The second interruptible pin on the same
 *               port will be mapped to 'int 1'. Upon interrupt reception, the
 *               device searches the list for a gpio Id that matches that of
 *               the interrupt in question. Once found, the device signals
 *               the waiting external routine through a call-back function.
 *
 * Last Update : Sept 9, 2011
 *---------------------------------------------------------------------------*/
#ifndef hal_gpio_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define hal_gpio_h
   #define PINS_IGNORE (0) /*Pins are left undefined*/
   #define PIN_0       (1 << 0)
   #define PIN_1       (1 << 1)
   #define PIN_2       (1 << 2)
   #define PIN_3       (1 << 3)
   #define PIN_4       (1 << 4)
   #define PIN_5       (1 << 5)
   #define PIN_6       (1 << 6)
   #define PIN_7       (1 << 7)

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
    #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      GPIO_INVALID_HANDLE = -8, /*Handle not found*/
      GPIO_INTS_MAPPED    = -7, /*Both interrupts for the port area already
                                 mapped to a handle.*/
      GPIO_PIN_IS_OUTPUT  = -6, /*Trying to access a pin as an input that is
                                 configured as an output.*/
      GPIO_PIN_IS_INPUT   = -5, /*Trying to access a pin as an output that is
                                 configured as an input.*/
      GPIO_INVALID_CMD    = -4, /*Invalid command.*/
      GPIO_NULL_PTR       = -3, /*Pointer is not mapped to a valid address.*/
      GPIO_OUT_OF_HEAP    = -2, /*No more memory.*/
      GPIO_INVALID_PIN    = -1, /*Trying to access an invalid pin.*/
      GPIO_PASSED         = 0   /*Configuration good.*/

   }t_gpioError;

   typedef enum
   {
      GPIO_BOTH_EDGES = 0, /*Generate interrupt on both edges.*/
      GPIO_RISING,         /*Generate interrupt on rising edge.*/
      GPIO_FALLING,        /*Generate interrupt on falling edge.*/
      GPIO_LEVEL,          /*Generate interrupt on low level.*/
      GPIO_INPUT_DISABLE   /*Disable input buffer.*/

   }t_inputSense;

   typedef enum
   {
      TOTEM = 0, /*The output is driven hard to either VCC or grouns as
                   commanded by the bit in the OUT register.*/
      WIREDOR,   /*Writing a 1 causes a pin to be driven to VCC, otherwise its
                   released and allowed to be pulled to ground by an external
                   pull down resister.*/
      WIREDAND,  /*Writing a 0 causes a pin to be driven to GND, otherwise its
                   released and allowed to be pulled high by an external pull
                   up resister.*/
      BUSKEEPER, /*Keeps a pin at the same logic level when a pin is no longer
                   driven to a logic state.*/
      PULLDOWN,  /*Enables internal pull down resister for the pin.*/
      PULLUP     /*Enables internal pull up resister for the pin.*/

   }t_pullConf;

   typedef enum
   {
      GPIO_PORTA = 0,
      GPIO_PORTB,
      GPIO_PORTC,
      GPIO_PORTD,
      GPIO_PORTE,
      GPIO_PORTF,
      GPIO_PORTH,
      GPIO_PORTJ,
      GPIO_PORTK,
      GPIO_PORTQ,
      GPIO_PORTR

   }t_gpioPort;

   typedef struct
   {
      /*---------------------------------------------------------------------*
       * The pin that is being requested as interruptible
       *---------------------------------------------------------------------*/
      uint8_t c_pin;

      /*---------------------------------------------------------------------*
       * Sets the criteria for interrupt generation on pin given by c_pin
       *---------------------------------------------------------------------*/
      t_inputSense t_inSense;

      /*---------------------------------------------------------------------*
       * Pointer to the function that gets called when an interrupt is
       * generated.
       *---------------------------------------------------------------------*/
      void (*pf_funPtr)( t_gpioPort t_port,
                         uint8_t c_pin);

   }t_intConf; /*Interrupt configuration*/

   typedef struct
   {

      /*---------------------------------------------------------------------*
       * If a bit in this mask is a 1, then the pin is configured as an
       * input.
       *---------------------------------------------------------------------*/
      uint8_t c_inputMask;

      /*---------------------------------------------------------------------*
       * If a bit in this mask is a 1, then the pin is configured as an
       * output.
       *---------------------------------------------------------------------*/
      uint8_t c_outputMask;

      /*---------------------------------------------------------------------*
       * If true, the initial value of the output pins is low.
       *---------------------------------------------------------------------*/
      bool b_setOutputLow;

      /*---------------------------------------------------------------------*
       * Sets the pull configuration for ALL pins designated as an input by
       * the input mask (c_inputMask)
       *---------------------------------------------------------------------*/
      t_pullConf t_inConf;

      /*---------------------------------------------------------------------*
       * Sets the output configuration for ALL pins designated as an output by
       * the output mask (c_outMask)
       *---------------------------------------------------------------------*/
      t_pullConf t_outConf;

   }t_gpioConf; /*PORT configuration*/

   typedef volatile int16_t t_GPIOHNDL;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/
   static inline t_gpioError __attribute__ ( (always_inline)) hal_gpioOn(
   t_gpioPort t_port, uint8_t    c_pin)
   {
      t_gpioError t_err = GPIO_PASSED;

      switch( t_port)
      {
         case GPIO_PORTA:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTA.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTA.OUTSET = c_pin;

         break;/*End case GPIO_PORTA:*/

         case GPIO_PORTB:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTB.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTB.OUTSET = c_pin;

         break;/*End case GPIO_PORTB:*/

         case GPIO_PORTC:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTC.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTC.OUTSET = c_pin;

         break;/*End case GPIO_PORTC:*/

         case GPIO_PORTD:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTD.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTD.OUTSET = c_pin;

         break;/*End case GPIO_PORTD:*/

         case GPIO_PORTE:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTE.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTE.OUTSET = c_pin;

         break;/*End case GPIO_PORTE:*/

         case GPIO_PORTF:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTF.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTF.OUTSET = c_pin;

         break;/*End case GPIO_PORTF:*/

         case GPIO_PORTH:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTH.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTH.OUTSET = c_pin;

         break;/*End case GPIO_PORTH:*/

         case GPIO_PORTJ:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTJ.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTJ.OUTSET = c_pin;

         break;/*End case GPIO_PORTJ:*/

         case GPIO_PORTK:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTK.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTK.OUTSET = c_pin;

         break;/*End case GPIO_PORTK:*/

         case GPIO_PORTQ:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTQ.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTQ.OUTSET = c_pin;

         break;/*End case GPIO_PORTQ:*/

         case GPIO_PORTR:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTR.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTR.OUTSET = c_pin;

         break;/*End case GPIO_PORTR:*/

         default:
            t_err = GPIO_INVALID_CMD;
         break;

      }/*End switch( t_port)*/

      return t_err;

   }/*End hal_gpioOn*/

   static inline t_gpioError __attribute__ ( (always_inline)) hal_gpioOff(
   t_gpioPort t_port, uint8_t    c_pin)
   {

      t_gpioError t_err = GPIO_PASSED;

      switch( t_port)
      {
         case GPIO_PORTA:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTA.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTA.OUTCLR = c_pin;

         break;/*End case GPIO_PORTA:*/

         case GPIO_PORTB:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTB.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTB.OUTCLR = c_pin;

         break;/*End case GPIO_PORTB:*/

         case GPIO_PORTC:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTC.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTC.OUTCLR = c_pin;

         break;/*End case GPIO_PORTC:*/

         case GPIO_PORTD:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTD.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTD.OUTCLR = c_pin;

         break;/*End case GPIO_PORTD:*/

         case GPIO_PORTE:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTE.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTE.OUTCLR = c_pin;

         break;/*End case GPIO_PORTE:*/

         case GPIO_PORTF:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTF.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTF.OUTCLR = c_pin;

         break;/*End case GPIO_PORTF:*/

         case GPIO_PORTH:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTH.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTH.OUTCLR = c_pin;

         break;/*End case GPIO_PORTH:*/

         case GPIO_PORTJ:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTJ.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTJ.OUTCLR = c_pin;

         break;/*End case GPIO_PORTJ:*/

         case GPIO_PORTK:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTK.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTK.OUTCLR = c_pin;

         break;/*End case GPIO_PORTK:*/

         case GPIO_PORTQ:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTQ.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTQ.OUTCLR = c_pin;

         break;/*End case GPIO_PORTQ:*/

         case GPIO_PORTR:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTR.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTR.OUTCLR = c_pin;

         break;/*End case GPIO_PORTR:*/

         default:
            t_err = GPIO_INVALID_CMD;
         break;

      }/*End switch( t_port)*/

      return t_err;

   }/*End hal_gpioOff*/

   static inline bool __attribute__ ( (always_inline)) hal_isGpioHigh(
   t_gpioPort t_port, uint8_t    c_pin)
   {
      switch( t_port)
      {
         case GPIO_PORTA:

            return (bool)(PORTA.IN & c_pin);

         break;/*End case GPIO_PORTA:*/

         case GPIO_PORTB:

            return (bool)(PORTB.IN & c_pin);

         break;/*End case GPIO_PORTB:*/

         case GPIO_PORTC:

            return (bool)(PORTC.IN & c_pin);

         break;/*End case GPIO_PORTC:*/

         case GPIO_PORTD:

            return (bool)(PORTD.IN & c_pin);

         break;/*End case GPIO_PORTD:*/

         case GPIO_PORTE:

            return (bool)(PORTE.IN & c_pin);

         break;/*End case GPIO_PORTE:*/

         case GPIO_PORTF:

            return (bool)(PORTF.IN & c_pin);

         break;/*End case GPIO_PORTF:*/

         case GPIO_PORTH:

            return (bool)(PORTH.IN & c_pin);

         break;/*End case GPIO_PORTH:*/

         case GPIO_PORTJ:

            return (bool)(PORTJ.IN & c_pin);

         break;/*End case GPIO_PORTJ:*/

         case GPIO_PORTK:

            return (bool)(PORTK.IN & c_pin);

         break;/*End case GPIO_PORTK:*/

         case GPIO_PORTQ:

            return (bool)(PORTQ.IN & c_pin);

         break;/*End case GPIO_PORTQ:*/

         case GPIO_PORTR:

            return (bool)(PORTR.IN & c_pin);

         break;/*End case GPIO_PORTR:*/

         default:
            return false;
         break;

      }/*End switch( t_port)*/

   }/*End hal_isGpioHigh*/

   static inline t_gpioError __attribute__ ( (always_inline)) hal_gpioToggle(
   t_gpioPort t_port, uint8_t    c_pin)
   {
      t_gpioError t_err = GPIO_PASSED;

      switch( t_port)
      {
         case GPIO_PORTA:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTA.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTA.OUTTGL = c_pin;

         break;/*End case GPIO_PORTA:*/

         case GPIO_PORTB:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTB.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTB.OUTTGL = c_pin;

         break;/*End case GPIO_PORTB:*/

         case GPIO_PORTC:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTC.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTC.OUTTGL = c_pin;

         break;/*End case GPIO_PORTC:*/

         case GPIO_PORTD:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTD.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTD.OUTTGL = c_pin;

         break;/*End case GPIO_PORTD:*/

         case GPIO_PORTE:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTE.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTE.OUTTGL = c_pin;

         break;/*End case GPIO_PORTE:*/

         case GPIO_PORTF:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTF.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTF.OUTTGL = c_pin;

         break;/*End case GPIO_PORTF:*/

         case GPIO_PORTH:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTH.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTH.OUTTGL = c_pin;

         break;/*End case GPIO_PORTH:*/

         case GPIO_PORTJ:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTJ.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTJ.OUTTGL = c_pin;

         break;/*End case GPIO_PORTJ:*/

         case GPIO_PORTK:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTK.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTK.OUTTGL = c_pin;

         break;/*End case GPIO_PORTK:*/

         case GPIO_PORTQ:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTQ.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTQ.OUTTGL = c_pin;

         break;/*End case GPIO_PORTQ:*/

         case GPIO_PORTR:

            /*---------------------------------------------------------------*
             * Make sure the requested pin is configured as an output
             *---------------------------------------------------------------*/
            if( !(PORTR.DIR & c_pin))
               t_err = GPIO_PIN_IS_INPUT;
            else
               PORTR.OUTTGL = c_pin;

         break;/*End case GPIO_PORTR:*/

         default:
            t_err = GPIO_INVALID_CMD;
         break;

      }/*End switch( t_port)*/

      return t_err;

   }/*End hal_gpioToggle*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_gpioError hal_configureGpioPort( t_gpioPort t_port,
                                      t_gpioConf t_conf);

   t_GPIOHNDL hal_requestGpioInt( t_gpioPort t_port,
                                  t_intConf  t_intConf);

   t_gpioError hal_releaseGpioInt( t_GPIOHNDL t_handle);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef hal_gpio_h*/
