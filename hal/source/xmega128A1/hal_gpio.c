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
 * File Name   : hal_gpio.c
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

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "hal_gpio.h"
#include "utl_linkedlist.h"
#include "hal_pmic.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef enum
{
   PORTA_INT0 = 0, /*Interrupt 0 on PORTA*/
   PORTA_INT1,     /*Interrupt 1 on PORTA*/
   PORTB_INT0,
   PORTB_INT1,
   PORTC_INT0,
   PORTC_INT1,
   PORTD_INT0,
   PORTD_INT1,
   PORTE_INT0,
   PORTE_INT1,
   PORTF_INT0,
   PORTF_INT1,
   PORTH_INT0,
   PORTH_INT1,
   PORTJ_INT0,
   PORTJ_INT1,
   PORTK_INT0,
   PORTK_INT1,
   PORTQ_INT0,
   PORTQ_INT1,
   PORTR_INT0,
   PORTR_INT1

}t_portIntId;

typedef struct
{

   /*------------------------------------------------------------------------*
    * A unique number referring to one of the eleven ports and one of its two
    * configurable interrupts.
    *------------------------------------------------------------------------*/
   t_portIntId t_id;

   /*------------------------------------------------------------------------*
    * The pin mapped to the interrupt given by t_portIntId
    *------------------------------------------------------------------------*/
   uint8_t c_pin;

   /*------------------------------------------------------------------------*
    * Pointer to the call-back function
    *------------------------------------------------------------------------*/
   void (*pf_funPtr)( t_gpioPort t_port,
                      uint8_t c_pin);

}t_gpioIntHndl;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_LINKHNDL createIntHandle( void);

static void confPort( PORT_t *pt_port,
                      t_gpioConf *pt_conf);

static t_gpioIntHndl *findGpioIntElement( t_portIntId t_id);

static t_LINKHNDL confInt( PORT_t *pt_port,
                           t_intConf *pt_intConf,
                           t_portIntId t_int0,
                           t_portIntId t_int1);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Linked-list of all the currently active GPIO interrupts.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_gpioHndlList);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
ISR( PORTA_INT0_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTA_INT0);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTA,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTA_INT0_vect)*/

ISR( PORTA_INT1_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTA_INT1);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTA,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTA_INT1_vect)*/

ISR( PORTB_INT0_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTB_INT0);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTB,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTB_INT0_vect)*/

ISR( PORTB_INT1_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTB_INT1);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTB,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTB_INT1_vect)*/

ISR( PORTC_INT0_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( (uint8_t)PORTC_INT0);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTC,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTC_INT0_vect)*/

ISR( PORTC_INT1_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTC_INT1);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTC,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTC_INT1_vect)*/

ISR( PORTD_INT0_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTD_INT0);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTD,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTD_INT0_vect)*/

ISR( PORTD_INT1_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTD_INT1);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTD,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTD_INT1_vect)*/

ISR( PORTE_INT0_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTE_INT0);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTE,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTE_INT0_vect)*/

ISR( PORTE_INT1_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTE_INT1);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTE,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTE_INT1_vect)*/

ISR( PORTF_INT0_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTF_INT0);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTF,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTF_INT0_vect)*/

ISR( PORTF_INT1_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTF_INT1);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTF,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTF_INT1_vect)*/

ISR( PORTH_INT0_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTH_INT0);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTH,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTH_INT0_vect)*/

ISR( PORTH_INT1_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTH_INT1);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTH,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTH_INT1_vect)*/

ISR( PORTJ_INT0_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTJ_INT0);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTJ,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTJ_INT0_vect)*/

ISR( PORTJ_INT1_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTJ_INT1);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTJ,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTJ_INT1_vect)*/

ISR( PORTK_INT0_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTK_INT0);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTK,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTK_INT0_vect)*/

ISR( PORTK_INT1_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTK_INT1);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTK,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTK_INT1_vect)*/

ISR( PORTQ_INT0_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTQ_INT0);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTQ,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTQ_INT0_vect)*/

ISR( PORTQ_INT1_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTQ_INT1);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTQ,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTQ_INT1_vect)*/

ISR( PORTR_INT0_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTR_INT0);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTR,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTR_INT0_vect)*/

ISR( PORTR_INT1_vect)
{
   t_gpioIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this port interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findGpioIntElement( PORTR_INT1);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr( GPIO_PORTR,
                                pt_intHndl->c_pin);
   }

}/*End ISR( PORTR_INT1_vect)*/

static t_gpioIntHndl *findGpioIntElement( t_portIntId t_id)
{
   t_gpioIntHndl *pt_element;
   t_LINKHNDL t_linkHndl;
   int16_t s_count;

   /*------------------------------------------------------------------------*
    * Search the GPIO interrupt list for the requested ID
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_gpioHndlList, s_count)
   {
      pt_element = (t_gpioIntHndl *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( pt_element->t_id == t_id)
      {
         return pt_element;
      }

   }

   /*------------------------------------------------------------------------*
    * If we make it this far the ID has not been found in the open GPIO
    * interrupt list.
    *------------------------------------------------------------------------*/
   return NULL;

}/*End findGpioIntElement*/

/*---------------------------------------------------------------------------*
 * This function is called the very first time a user-space application
 * requests an interrupt for a given pin on a port. The driver only allows
 * two pins mapped to interrupts for the given port.
 *---------------------------------------------------------------------------*/
static t_LINKHNDL createIntHandle( void)
{
   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Allocated memory for the link where the DMA interrupt information will be
    * stored.
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof( t_gpioIntHndl));

   if( t_linkHndl < 0)
   {
      return (t_LINKHNDL)GPIO_OUT_OF_HEAP;
   }

   /*------------------------------------------------------------------------*
    * Add the GPIO interrupt link onto the list open GPIO interrupts.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_gpioHndlList,
                           t_linkHndl,
                           true);

   return t_linkHndl;

}/*End createIntHandle*/

static void confPort( PORT_t *pt_port,
                      t_gpioConf *pt_conf)
{

   /*------------------------------------------------------------------------*
    * Set the output configuration for the given pins
    *------------------------------------------------------------------------*/
   if( pt_conf->c_outputMask > 0) /*Are there any output pins?*/
   {

      /*---------------------------------------------------------------------*
       * Set the output pins to the required initial value
       *---------------------------------------------------------------------*/
      if( pt_conf->b_setOutputLow)
      {
         pt_port->OUTCLR = pt_conf->c_outputMask;
      }
      else
         pt_port->OUTSET = pt_conf->c_outputMask;

      pt_port->DIRSET = pt_conf->c_outputMask;

      /*---------------------------------------------------------------------*
       * Use the pin mask register as a means of setting a large group
       * of pins to the same value.
       *---------------------------------------------------------------------*/
      PORTCFG.MPCMASK = pt_conf->c_outputMask;

      /*---------------------------------------------------------------------*
       * Writing to any one of the pins in the port configuration register
       * will trigger all the port configuration registers to be written with
       * the value contained in the pin mask register.
       *---------------------------------------------------------------------*/
      switch( pt_conf->t_outConf)
      {
         case TOTEM:
            pt_port->PIN0CTRL = PORT_OPC_TOTEM_gc;
         break; /*End case TOTEM:*/

         case WIREDOR:
            pt_port->PIN0CTRL = PORT_OPC_WIREDOR_gc;
         break; /*End case WIREDOR:*/

         case WIREDAND:
            pt_port->PIN0CTRL = PORT_OPC_WIREDAND_gc;
         break; /*End case WIREDAND:*/

         default:
           pt_port->PIN0CTRL = PORT_OPC_TOTEM_gc;
         break;/*End case BUSKEEPER:*/

      }/*End switch( pt_conf->t_outConf)*/

   }/*End if( pt_conf->c_outputMask > 0)*/

   /*------------------------------------------------------------------------*
    * Set the pull configuration for the given input pins
    *------------------------------------------------------------------------*/
   if( pt_conf->c_inputMask > 0) /*Are there any input pins?*/
   {

      /*---------------------------------------------------------------------*
       * Interruptible pins are not allowed until an interrupt is requested.
       *---------------------------------------------------------------------*/
      pt_port->INTCTRL = PORT_INT0LVL_OFF_gc;

      pt_port->DIRCLR = pt_conf->c_inputMask;

      /*---------------------------------------------------------------------*
       * Use the pin mask register as a means of setting a large group of
       * pins to the same value.
       *---------------------------------------------------------------------*/
      PORTCFG.MPCMASK = pt_conf->c_inputMask;

      switch( pt_conf->t_inConf)
      {
         case PULLDOWN:
            pt_port->PIN0CTRL = PORT_OPC_PULLDOWN_gc;
         break; /*End case TOTEM:*/

         case PULLUP:
            pt_port->PIN0CTRL = PORT_OPC_PULLUP_gc;
         break; /*End case WIREDOR:*/

         case BUSKEEPER:
            pt_port->PIN0CTRL = PORT_OPC_BUSKEEPER_gc;
         break;/*End case BUSKEEPER:*/

         default:
            pt_port->PIN0CTRL = PORT_OPC_TOTEM_gc;
         break;

      }/*End switch( pt_conf->t_inConf)*/

   }/*End if( pt_conf->c_inputMask > 0)*/

}/*End confPort*/

static t_LINKHNDL confInt( PORT_t *pt_port,
                           t_intConf *pt_intConf,
                           t_portIntId t_int0,
                           t_portIntId t_int1)
{
   t_gpioIntHndl *pt_int0Hndl;
   t_gpioIntHndl *pt_int1Hndl;
   t_LINKHNDL t_linkHndl;

   /*------------------------------------------------------------------------*
    * Make sure the user-space application is attempting to attach one pin
    * and one pin ONLY to one of the two port interrupts.
    *------------------------------------------------------------------------*/
   switch( pt_intConf->c_pin)
   {
      case PIN_0:
      case PIN_1:
      case PIN_2:
      case PIN_3:
      case PIN_4:
      case PIN_5:
      case PIN_6:
      case PIN_7:
      break;

      default:

         return (t_LINKHNDL)GPIO_INVALID_PIN; /*Illegal pin configuration*/

      break;

   }/*End switch( pt_intConf->c_pin)*/

   /*------------------------------------------------------------------------*
    * Make sure the requested pin is configured as an input
    *------------------------------------------------------------------------*/
   if( pt_port->DIR & pt_intConf->c_pin)
      return (t_LINKHNDL)GPIO_PIN_IS_OUTPUT;
   else
   {
      /*---------------------------------------------------------------------*
       * Search the list and see if there is an active handle for interrupt 0.
       *---------------------------------------------------------------------*/
      pt_int0Hndl = findGpioIntElement( t_int0);

      if( pt_int0Hndl == NULL) /*No Active handle*/
      {

         /*------------------------------------------------------------------*
          * Create the new interrupt handle.
          *------------------------------------------------------------------*/
         t_linkHndl = createIntHandle();

         if( t_linkHndl < 0)
            return (t_LINKHNDL)GPIO_OUT_OF_HEAP;
         else
         {
            /*----------------------------------------------------------------*
             * Get a ptr to the link's element- which is the area where the
             * GPIO interrupt information is being stored.
             *----------------------------------------------------------------*/
            pt_int0Hndl = (t_gpioIntHndl *)
            UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
            pt_int0Hndl->t_id      = t_int0;
            pt_int0Hndl->c_pin     = pt_intConf->c_pin;
            pt_int0Hndl->pf_funPtr = pt_intConf->pf_funPtr;
         }

         switch( pt_intConf->c_pin)
         {
            case PIN_0:
               pt_port->PIN0CTRL |= pt_intConf->t_inSense;
            break;

            case PIN_1:
               pt_port->PIN1CTRL |= pt_intConf->t_inSense;
            break;

            case PIN_2:
               pt_port->PIN2CTRL |= pt_intConf->t_inSense;
            break;

            case PIN_3:
               pt_port->PIN3CTRL |= pt_intConf->t_inSense;
            break;

            case PIN_4:
               pt_port->PIN4CTRL |= pt_intConf->t_inSense;
            break;

            case PIN_5:
               pt_port->PIN5CTRL |= pt_intConf->t_inSense;
            break;

            case PIN_6:
               pt_port->PIN6CTRL |= pt_intConf->t_inSense;
            break;

            case PIN_7:
               pt_port->PIN7CTRL |= pt_intConf->t_inSense;
            break;

         }/*End switch( pt_intConf->c_pin)*/

         /*------------------------------------------------------------------*
          * This value must match that set in the PMIC control register
          *------------------------------------------------------------------*/
         pt_port->INTCTRL  |= PORT_INT0LVL_HI_gc;
         pt_port->INT0MASK = pt_intConf->c_pin;

      }/*End if( pt_int0Hndl == NULL)*/
      else /*Handle already open*/
      {

         /*---------------------------------------------------------------*
          * Search the list and see if there is an active handle for
          * interrupt 1.
          *---------------------------------------------------------------*/
         pt_int1Hndl = findGpioIntElement( t_int1);

         if( pt_int1Hndl == NULL) /*No Active handle*/
         {

            /*---------------------------------------------------------------*
             * Create the new interrupt handle.
             *---------------------------------------------------------------*/
            t_linkHndl = createIntHandle();

            if( t_linkHndl < 0)
               return (t_LINKHNDL)GPIO_OUT_OF_HEAP;
            else
            {
               /*-------------------------------------------------------------*
                * Get a ptr to the link's element- which is the area where the
                * GPIO interrupt information is being stored.
                *-------------------------------------------------------------*/
               pt_int1Hndl = (t_gpioIntHndl *)
               UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
               pt_int1Hndl->t_id      = t_int1;
               pt_int1Hndl->c_pin     = pt_intConf->c_pin;
               pt_int1Hndl->pf_funPtr = pt_intConf->pf_funPtr;
            }

            switch( pt_intConf->c_pin)
            {
               case PIN_0:
                  pt_port->PIN0CTRL |= pt_intConf->t_inSense;
               break;

               case PIN_1:
                  pt_port->PIN1CTRL |= pt_intConf->t_inSense;
               break;

               case PIN_2:
                  pt_port->PIN2CTRL |= pt_intConf->t_inSense;
               break;

               case PIN_3:
                  pt_port->PIN3CTRL |= pt_intConf->t_inSense;
               break;

               case PIN_4:
                  pt_port->PIN4CTRL |= pt_intConf->t_inSense;
               break;

               case PIN_5:
                  pt_port->PIN5CTRL |= pt_intConf->t_inSense;
               break;

               case PIN_6:
                  pt_port->PIN6CTRL |= pt_intConf->t_inSense;
               break;

               case PIN_7:
                  pt_port->PIN7CTRL |= pt_intConf->t_inSense;
               break;

            }/*End switch( pt_intConf->c_pin)*/

            /*---------------------------------------------------------------*
             * This value must match that set in the PMIC control register
             *---------------------------------------------------------------*/
            pt_port->INTCTRL |= PORT_INT1LVL_HI_gc;
            pt_port->INT1MASK = pt_intConf->c_pin;

         }/*End if( pt_int1Hndl == NULL)*/
         else /*Handle already open*/
         {
            /*---------------------------------------------------------------*
             * Handles have already been created for int 0 and int 1 on this
             * port.
             *---------------------------------------------------------------*/
            return (t_LINKHNDL)GPIO_INTS_MAPPED;
         }
      }

   }/*End if( t_err == GPIO_PASSED)*/

   return t_linkHndl;

}/*End confInt*/

t_gpioError hal_configureGpioPort( t_gpioPort t_port,
                                   t_gpioConf t_conf)
{
   t_gpioError t_err = GPIO_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   switch( t_port)
   {
      case GPIO_PORTA:

         confPort( &PORTA,
                   &t_conf);

      break;/*End case GPIO_PORTA:*/

      case GPIO_PORTB:

         confPort( &PORTB,
                   &t_conf);

      break;/*End case GPIO_PORTB:*/

      case GPIO_PORTC:

         confPort( &PORTC,
                   &t_conf);

      break;/*End case GPIO_PORTC:*/

      case GPIO_PORTD:

         confPort( &PORTD,
                   &t_conf);

      break;/*End case GPIO_PORTD:*/

      case GPIO_PORTE:

         confPort( &PORTE,
                   &t_conf);

      break;/*End case GPIO_PORTE:*/

      case GPIO_PORTF:

         confPort( &PORTF,
                   &t_conf);

      break;/*End case GPIO_PORTF:*/

      case GPIO_PORTH:

         confPort( &PORTH,
                   &t_conf);

      break;/*End case GPIO_PORTH:*/

      case GPIO_PORTJ:

         confPort( &PORTJ,
                   &t_conf);

      break;/*End case GPIO_PORTJ:*/

      case GPIO_PORTK:

         confPort( &PORTK,
                   &t_conf);

      break;/*End case GPIO_PORTK:*/

      case GPIO_PORTQ:

         confPort( &PORTQ,
                   &t_conf);

      break;/*End case GPIO_PORTQ:*/

      case GPIO_PORTR:

         confPort( &PORTR,
                   &t_conf);

      break;/*End case GPIO_PORTR:*/

      default:
         t_err = GPIO_INVALID_CMD;
      break;

   }/*End switch( t_port)*/

   HAL_END_CRITICAL();//Enable interrupts

   return t_err;

}/*End hal_configureGpioPort*/

t_GPIOHNDL hal_requestGpioInt( t_gpioPort t_port,
                               t_intConf  t_conf)
{
   t_LINKHNDL t_linkHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   switch( t_port)
   {
      case GPIO_PORTA:

         t_linkHndl = confInt( &PORTA,
                               &t_conf,
                               PORTA_INT0,
                               PORTA_INT1);

      break;/*End case GPIO_PORTA:*/

      case GPIO_PORTB:

         t_linkHndl = confInt( &PORTB,
                               &t_conf,
                               PORTB_INT0,
                               PORTB_INT1);

      break;/*End case GPIO_PORTB:*/

      case GPIO_PORTC:

         t_linkHndl = confInt( &PORTC,
                               &t_conf,
                               PORTC_INT0,
                               PORTC_INT1);

      break;/*End case GPIO_PORTC:*/

      case GPIO_PORTD:

         t_linkHndl = confInt( &PORTD,
                               &t_conf,
                               PORTD_INT0,
                               PORTD_INT1);

      break;/*End case GPIO_PORTD:*/

      case GPIO_PORTE:

         t_linkHndl = confInt( &PORTE,
                               &t_conf,
                               PORTE_INT0,
                               PORTE_INT1);

      break;/*End case GPIO_PORTE:*/

      case GPIO_PORTF:

         t_linkHndl = confInt( &PORTF,
                               &t_conf,
                               PORTF_INT0,
                               PORTF_INT1);

      break;/*End case GPIO_PORTF:*/

      case GPIO_PORTH:

         t_linkHndl = confInt( &PORTH,
                               &t_conf,
                               PORTH_INT0,
                               PORTH_INT1);

      break;/*End case GPIO_PORTH:*/

      case GPIO_PORTJ:

         t_linkHndl = confInt( &PORTJ,
                               &t_conf,
                               PORTJ_INT0,
                               PORTJ_INT1);

      break;/*End case GPIO_PORTJ:*/

      case GPIO_PORTK:

         t_linkHndl = confInt( &PORTK,
                               &t_conf,
                               PORTK_INT0,
                               PORTK_INT1);

      break;/*End case GPIO_PORTK:*/

      case GPIO_PORTQ:

         t_linkHndl = confInt( &PORTQ,
                               &t_conf,
                               PORTQ_INT0,
                               PORTQ_INT1);

      break;/*End case GPIO_PORTQ:*/

      case GPIO_PORTR:

         t_linkHndl = confInt( &PORTR,
                               &t_conf,
                               PORTR_INT0,
                               PORTR_INT1);

      break;/*End case GPIO_PORTR:*/

      default:

         HAL_END_CRITICAL();//Enable interrupts
         return (t_GPIOHNDL)GPIO_INVALID_CMD;

      break;

   }/*End switch( t_port)*/

   HAL_END_CRITICAL();//Enable interrupts

   return (t_GPIOHNDL)t_linkHndl;

}/*End hal_requestGpioInt*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
t_gpioError hal_releaseGpioInt( t_GPIOHNDL t_handle)
{
   t_gpioIntHndl *pt_gpioIntHndl;
   t_linkedListError t_lErr;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_gpioHndlList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return GPIO_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the GPIO
       * interrupt information is being stored.
       *---------------------------------------------------------------------*/
      pt_gpioIntHndl = (t_gpioIntHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Disable the interrupt attached to this handle.
       *---------------------------------------------------------------------*/
      switch( pt_gpioIntHndl->t_id)
      {
         case PORTA_INT0:
            PORTA.INTCTRL &= ~PORT_INT0LVL_HI_gc;
            PORTA.INT0MASK = 0;
         break;

         case PORTA_INT1:
            PORTA.INTCTRL &= ~PORT_INT1LVL_HI_gc;
            PORTA.INT1MASK = 0;
         break;

         case PORTB_INT0:
            PORTB.INTCTRL &= ~PORT_INT0LVL_HI_gc;
            PORTB.INT0MASK = 0;
         break;

         case PORTB_INT1:
            PORTB.INTCTRL &= ~PORT_INT1LVL_HI_gc;
            PORTB.INT1MASK = 0;
         break;

         case PORTC_INT0:
            PORTC.INTCTRL &= ~PORT_INT0LVL_HI_gc;
            PORTC.INT0MASK = 0;
         break;

         case PORTC_INT1:
            PORTC.INTCTRL &= ~PORT_INT1LVL_HI_gc;
            PORTC.INT1MASK = 0;
         break;

         case PORTD_INT0:
            PORTD.INTCTRL &= ~PORT_INT0LVL_HI_gc;
            PORTD.INT0MASK = 0;
         break;

         case PORTD_INT1:
            PORTD.INTCTRL &= ~PORT_INT1LVL_HI_gc;
            PORTD.INT1MASK = 0;
         break;

         case PORTE_INT0:
            PORTE.INTCTRL &= ~PORT_INT0LVL_HI_gc;
            PORTE.INT0MASK = 0;
         break;

         case PORTE_INT1:
            PORTE.INTCTRL &= ~PORT_INT1LVL_HI_gc;
            PORTE.INT1MASK = 0;
         break;

         case PORTF_INT0:
            PORTF.INTCTRL &= ~PORT_INT0LVL_HI_gc;
            PORTF.INT0MASK = 0;
         break;

         case PORTF_INT1:
            PORTF.INTCTRL &= ~PORT_INT1LVL_HI_gc;
            PORTF.INT1MASK = 0;
         break;

         case PORTH_INT0:
            PORTH.INTCTRL &= ~PORT_INT0LVL_HI_gc;
            PORTH.INT0MASK = 0;
         break;

         case PORTH_INT1:
            PORTH.INTCTRL &= ~PORT_INT1LVL_HI_gc;
            PORTH.INT1MASK = 0;
         break;

         case PORTJ_INT0:
            PORTJ.INTCTRL &= ~PORT_INT0LVL_HI_gc;
            PORTJ.INT0MASK = 0;
         break;

         case PORTJ_INT1:
            PORTJ.INTCTRL &= ~PORT_INT1LVL_HI_gc;
            PORTJ.INT1MASK = 0;
         break;

         case PORTK_INT0:
            PORTK.INTCTRL &= ~PORT_INT0LVL_HI_gc;
            PORTK.INT0MASK = 0;
         break;

         case PORTK_INT1:
            PORTK.INTCTRL &= ~PORT_INT1LVL_HI_gc;
            PORTK.INT1MASK = 0;
         break;

         case PORTQ_INT0:
            PORTQ.INTCTRL &= ~PORT_INT0LVL_HI_gc;
            PORTQ.INT0MASK = 0;
         break;

         case PORTQ_INT1:
            PORTQ.INTCTRL &= ~PORT_INT1LVL_HI_gc;
            PORTQ.INT1MASK = 0;
         break;

         case PORTR_INT0:
            PORTR.INTCTRL &= ~PORT_INT0LVL_HI_gc;
            PORTR.INT0MASK = 0;
         break;

         case PORTR_INT1:
            PORTR.INTCTRL &= ~PORT_INT1LVL_HI_gc;
            PORTR.INT1MASK = 0;
         break;

      }/*End switch( (pt_gpioIntHndl->t_id)*/

      t_lErr = utl_destroyLink( gt_gpioHndlList,
                                (t_LINKHNDL)t_handle);

   }/*End if( pt_gpioIntHndl != NULL)*/

   HAL_END_CRITICAL();//Enable interrupts

   return GPIO_PASSED;

}/*End hal_releaseGpioInt*/
