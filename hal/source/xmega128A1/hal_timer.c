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
 * File Name   : hal_timer.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for controlling
 *               the timer module.
 *
 * Last Update : Oct 12, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include "utl_linkedList.h"
#include "hal_timer.h"
#include "hal_pmic.h"
#include "hal_gpio.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define MAX_TIMER_RANGE (32767) /*Max number of timer ticks*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef enum
{
   TIMER_1_COMP_A = 0, /*Timer/counter C0*/
   TIMER_1_COMP_B,
   TIMER_1_COMP_C,
   TIMER_1_COMP_D,
   TIMER_1_OVERFLOW,
   TIMER_2_COMP_A,     /*Timer/counter C1*/
   TIMER_2_COMP_B,
   TIMER_2_NA_C,       /*Not available used as place holder*/
   TIMER_2_NA_D,
   TIMER_2_OVERFLOW,
   TIMER_3_COMP_A,
   TIMER_3_COMP_B,
   TIMER_3_COMP_C,
   TIMER_3_COMP_D,
   TIMER_3_OVERFLOW,
   TIMER_4_COMP_A,
   TIMER_4_COMP_B,
   TIMER_4_NA_C,       /*Not available used as place holder*/
   TIMER_4_NA_D,
   TIMER_4_OVERFLOW,
   TIMER_5_COMP_A,
   TIMER_5_COMP_B,
   TIMER_5_COMP_C,
   TIMER_5_COMP_D,
   TIMER_5_OVERFLOW,
   TIMER_6_COMP_A,
   TIMER_6_COMP_B,
   TIMER_6_NA_C,       /*Not available used as place holder*/
   TIMER_6_NA_D,
   TIMER_6_OVERFLOW,
   TIMER_7_COMP_A,
   TIMER_7_COMP_B,
   TIMER_7_COMP_C,
   TIMER_7_COMP_D,
   TIMER_7_OVERFLOW,
   TIMER_8_COMP_A,
   TIMER_8_COMP_B,
   TIMER_8_NA_C,       /*Not available used as place holder*/
   TIMER_8_NA_D,
   TIMER_8_OVERFLOW

}t_timerIntId;

typedef struct timerMod
{
   /*------------------------------------------------------------------------*
    * A unique number refering to one of the 8 possible timers
    *------------------------------------------------------------------------*/
   t_timerModId t_id;

   /*------------------------------------------------------------------------*
    * If true, then 'configTimer' successfully completed
    *------------------------------------------------------------------------*/
   bool b_validConfig;

   /*------------------------------------------------------------------------*
    * Keeps track of the interrupts opened against this handle
    *------------------------------------------------------------------------*/
   uint8_t c_intCount;

   /*------------------------------------------------------------------------*
    * Clock selection for this particular timer module
    *------------------------------------------------------------------------*/
   TC_CLKSEL_t t_clockSelection;

   /*------------------------------------------------------------------------*
    * Pointer to the timer 0 register for this particular handle
    *------------------------------------------------------------------------*/
   TC0_t *pt_timer0;

   /*------------------------------------------------------------------------*
    * Pointer to the timer 1 register for this particular handle
    *------------------------------------------------------------------------*/
   TC1_t *pt_timer1;

}t_timerModHndl;

typedef struct timerInt
{

   /*------------------------------------------------------------------------*
    * A unique number refering to one of the 32 possible timer interrupts
    *------------------------------------------------------------------------*/
   t_timerIntId t_id;

   /*------------------------------------------------------------------------*
    * Pointer to the call-back function
    *------------------------------------------------------------------------*/
   void (*pf_funPtr)( void);

}t_timerIntHndl;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_LINKHNDL createIntHandle( void);

static t_LINKHNDL createTimerHandle( void);

static t_timerIntHndl *findTimerIntElement( t_timerIntId t_id);

static t_timerModHndl *findTimerElement( t_timerModId t_id);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Linked-list of all the currently active Timer channels.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_timerHndlList);

/*---------------------------------------------------------------------------*
 * Linked-list of all the currently active Timer interrupts.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_timerIntHndlList);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
ISR( TCC0_ERR_vect)
{

}/*End ISR( TCC0_ERR_vect)*/

ISR( TCC0_CCA_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_1_COMP_A);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCC0_CCA_vect)*/

ISR( TCC0_CCB_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_1_COMP_B);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCC0_CCB_vect)*/

ISR( TCC0_CCC_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_1_COMP_C);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCC0_CCC_vect)*/

ISR( TCC0_CCD_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_1_COMP_D);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCC0_CCD_vect)*/

ISR( TCC0_OVF_vect)//, ISR_NAKED)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_1_OVERFLOW);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCC0_OVF_vect)*/

ISR( TCC1_CCA_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_2_COMP_A);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCC1_CCA_vect)*/

ISR( TCC1_CCB_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_2_COMP_B);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCC1_CCB_vect)*/

ISR( TCC1_OVF_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_2_OVERFLOW);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCC1_OVF_vect)*/

ISR( TCD0_CCA_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_3_COMP_A);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCD0_CCA_vect)*/

ISR( TCD0_CCB_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_3_COMP_B);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCD0_CCB_vect)*/

ISR( TCD0_CCC_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_3_COMP_C);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCD0_CCC_vect)*/

ISR( TCD0_CCD_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_3_COMP_D);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCD0_CCD_vect)*/

ISR( TCD0_OVF_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_3_OVERFLOW);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCD0_OVF_vect)*/

ISR( TCD1_CCA_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_4_COMP_A);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCD1_CCA_vect)*/

ISR( TCD1_CCB_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_4_COMP_B);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCD1_CCB_vect)*/

ISR( TCD1_OVF_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_4_OVERFLOW);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCD1_OVF_vect)*/

ISR( TCE0_CCA_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_5_COMP_A);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCE0_CCA_vect)*/

ISR( TCE0_CCB_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_5_COMP_B);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCE0_CCB_vect)*/

ISR( TCE0_CCC_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_5_COMP_C);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCE0_CCC_vect)*/

ISR( TCE0_CCD_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_5_COMP_D);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCE0_CCD_vect)*/

ISR( TCE0_OVF_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_5_OVERFLOW);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCE0_OVF_vect)*/

ISR( TCE1_CCA_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_6_COMP_A);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCE1_CCA_vect)*/

ISR( TCE1_CCB_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_6_COMP_B);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCE1_CCB_vect)*/

ISR( TCE1_OVF_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_6_OVERFLOW);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCE1_OVF_vect)*/

ISR( TCF0_CCA_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_7_COMP_A);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCF0_CCA_vect)*/

ISR( TCF0_CCB_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_7_COMP_B);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCF0_CCB_vect)*/

ISR( TCF0_CCC_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_7_COMP_C);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCF0_CCC_vect)*/

ISR( TCF0_CCD_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_7_COMP_D);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCF0_CCD_vect)*/

ISR( TCF0_OVF_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_7_OVERFLOW);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCF0_OVF_vect)*/

ISR( TCF1_CCA_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_8_COMP_A);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCF1_CCA_vect)*/

ISR( TCF1_CCB_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_8_COMP_B);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCF1_CCB_vect)*/

ISR( TCF1_OVF_vect)
{
   t_timerIntHndl *pt_intHndl = NULL;

   /*---------------------------------------------------------------------*
    * See if there is a handle on the list for this timer interrupt.
    *---------------------------------------------------------------------*/
   pt_intHndl = findTimerIntElement( TIMER_8_OVERFLOW);

   if( pt_intHndl != NULL)
   {
      if( pt_intHndl->pf_funPtr != NULL)
         pt_intHndl->pf_funPtr();
   }

}/*End ISR( TCF1_OVF_vect)*/

static t_timerIntHndl *findTimerIntElement( t_timerIntId t_id)
{
   t_timerIntHndl *pt_element;
   t_LINKHNDL t_linkHndl;
   int16_t s_count;

   /*------------------------------------------------------------------------*
    * Search the Timer interrupt list for the requested ID
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_timerIntHndlList, s_count)
   {
      pt_element = (t_timerIntHndl *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( pt_element->t_id == t_id)
      {
         return pt_element;
      }

   }

   /*------------------------------------------------------------------------*
    * If we make it this far the ID has not been found in the open Timer
    * interrupt list.
    *------------------------------------------------------------------------*/
   return NULL;

}/*End findTimerIntElement*/

static t_timerModHndl *findTimerElement( t_timerModId t_id)
{
   t_timerModHndl *pt_element;
   t_LINKHNDL t_linkHndl;
   int16_t s_count;

   /*------------------------------------------------------------------------*
    * Search the Timer list for the requested ID
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_timerHndlList, s_count)
   {
      pt_element = (t_timerModHndl *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( pt_element->t_id == t_id)
      {
         return pt_element;
      }

   }

   /*------------------------------------------------------------------------*
    * If we make it this far the ID has not been found in the open Timer
    * list.
    *------------------------------------------------------------------------*/
   return NULL;

}/*End findTimerElement*/

static t_LINKHNDL createIntHandle( void)
{
   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Allocated memory for the link where the Timer interrupt information 
    * will be stored.
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof( t_timerIntHndl));

   if( t_linkHndl < 0)
   {
      return (t_LINKHNDL)TIMER_OUT_OF_HEAP;
   }

   /*------------------------------------------------------------------------*
    * Add the Timer interrupt link onto the list open Timer interrupts.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_timerIntHndlList,
                           t_linkHndl,
                           true);

   return t_linkHndl;

}/*End createIntHandle*/

static t_LINKHNDL createTimerHandle( void)
{
   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Allocated memory for the link where the Timer information will be stored.
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof( t_timerModHndl));

   if( t_linkHndl < 0)
   {
      return (t_LINKHNDL)TIMER_OUT_OF_HEAP;
   }

   /*------------------------------------------------------------------------*
    * Add the Timer link onto the list open Timers.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_timerHndlList,
                           t_linkHndl,
                           true);

   return t_linkHndl;

}/*End createTimerHandle*/

t_timerError hal_requestTimerInterrupt( t_TIMERHNDL t_handle,
                                        t_compType t_type,
                                        void (*pf_funPtr)( void))
{
   t_timerModHndl *pt_timerHndl;
   t_timerIntHndl *pt_timerIntHndl;
   t_timerIntId t_intId;
   t_LINKHNDL t_linkHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Use the timer module id and interrrupt type as an index into the list
       * of possible interrupts.
       *---------------------------------------------------------------------*/
      t_intId = (t_timerIntId)pt_timerHndl->t_id + (t_timerIntId)t_type;

      if( (t_intId == TIMER_2_NA_C) || (t_intId == TIMER_2_NA_D) ||
          (t_intId == TIMER_4_NA_C) || (t_intId == TIMER_4_NA_D) ||
          (t_intId == TIMER_6_NA_C) || (t_intId == TIMER_6_NA_D) ||
          (t_intId == TIMER_8_NA_C) || (t_intId == TIMER_8_NA_D) ||
          (t_intId < TIMER_1_COMP_A) || (t_intId > TIMER_8_OVERFLOW))
      {
         HAL_END_CRITICAL();//Enable interrupts
         return TIMER_INVALID_COMP;
      }

      pt_timerIntHndl = findTimerIntElement( t_intId);

      /*---------------------------------------------------------------------*
       * Is there already and open handle for this interrupt?
       *---------------------------------------------------------------------*/
      if( pt_timerIntHndl == NULL) /*No*/
      {
         t_linkHndl = createIntHandle();
         if( t_linkHndl < 0)
         {
            HAL_END_CRITICAL();//Enable interrupts
            return TIMER_OUT_OF_HEAP;
         }

         /*------------------------------------------------------------------*
          * Get a ptr to the link's element- which is the area where the Timer 
          * interrupt information is being stored.
          *------------------------------------------------------------------*/
         pt_timerIntHndl = (t_timerIntHndl *)
         UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_linkHndl);

         pt_timerIntHndl->t_id = t_intId;
         pt_timerIntHndl->pf_funPtr = pf_funPtr;

         /*------------------------------------------------------------------*
          * Keep track of how many interrupts this handle has open.
          *------------------------------------------------------------------*/
         pt_timerHndl->c_intCount++;

         if( t_type == OVERFLOW)
         {
            if( pt_timerHndl->pt_timer0 != NULL)
               pt_timerHndl->pt_timer0->INTCTRLA |= TC_OVFINTLVL_HI_gc;
            else
               pt_timerHndl->pt_timer1->INTCTRLA |= TC_OVFINTLVL_HI_gc;

         }/*End if( t_type == OVERFLOW)*/
         else if( t_type == COMPAREA)
         {
            if( pt_timerHndl->pt_timer0 != NULL)
               pt_timerHndl->pt_timer0->INTCTRLB |= TC_CCAINTLVL_HI_gc;
            else
               pt_timerHndl->pt_timer1->INTCTRLB |= TC_CCAINTLVL_HI_gc;

         }
         else if( t_type == COMPAREB)
         {
            if( pt_timerHndl->pt_timer0 != NULL)
               pt_timerHndl->pt_timer0->INTCTRLB |= TC_CCBINTLVL_HI_gc;
            else
               pt_timerHndl->pt_timer1->INTCTRLB |= TC_CCBINTLVL_HI_gc;
         }
         else if( t_type == COMPAREC)
         {
            if( pt_timerHndl->pt_timer0 != NULL)
               pt_timerHndl->pt_timer0->INTCTRLB |= TC_CCCINTLVL_HI_gc;
            else
               pt_timerHndl->pt_timer1->INTCTRLB |= TC_CCCINTLVL_HI_gc;
         }
         else if( t_type == COMPARED)
         {
            if( pt_timerHndl->pt_timer0 != NULL)
               pt_timerHndl->pt_timer0->INTCTRLB |= TC_CCDINTLVL_HI_gc;
            else
               pt_timerHndl->pt_timer1->INTCTRLB |= TC_CCDINTLVL_HI_gc;
         }

      }/*End if( pt_timerIntHndl == NULL)*/
      else /*Yes*/
      {
         HAL_END_CRITICAL();//Enable interrupts
         return TIMER_INTERRUPT_OPEN;
      }

   }/*End if( pt_timerHndl != NULL)*/

   HAL_END_CRITICAL();//Enable interrupts

   return TIMER_PASSED;

}/*End hal_requestTimerInterrupt*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
t_timerError hal_releaseTimerInterrupt( t_TIMERHNDL t_handle,
                                        t_compType   t_type)
{
   static t_LINKHNDL t_linkHndl;
   t_timerModHndl *pt_timerHndl;
   t_timerIntId t_intId;
   int16_t s_count;
   t_linkedListError t_lErr;
   t_timerIntHndl *pt_timerIntHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Use the timer module id and interrrupt type as an index into the list
       * of possible interrupts.
       *---------------------------------------------------------------------*/
      t_intId = (t_timerIntId)pt_timerHndl->t_id + (t_timerIntId)t_type;

      if( (t_intId == TIMER_2_NA_C) || (t_intId == TIMER_2_NA_D) ||
          (t_intId == TIMER_4_NA_C) || (t_intId == TIMER_4_NA_D) ||
          (t_intId == TIMER_6_NA_C) || (t_intId == TIMER_6_NA_D) ||
          (t_intId == TIMER_8_NA_C) || (t_intId == TIMER_8_NA_D) ||
          (t_intId < TIMER_1_COMP_A) || (t_intId > TIMER_8_OVERFLOW))
      {
         HAL_END_CRITICAL();//Enable interrupts
         return TIMER_INVALID_COMP;
      }

      /*---------------------------------------------------------------------*
       * Search the Timer interrupt list for the requested ID
       *---------------------------------------------------------------------*/
      UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_timerIntHndlList, s_count)
      {
         pt_timerIntHndl = (t_timerIntHndl *)
         UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
         if( pt_timerIntHndl->t_id == t_intId)
         {

            /*---------------------------------------------------------------*
             * One less timer opened against this handle.
             *---------------------------------------------------------------*/
            pt_timerHndl->c_intCount--;

            if( t_type == OVERFLOW)
            {
               if( pt_timerHndl->pt_timer0 != NULL)
                  pt_timerHndl->pt_timer0->INTCTRLA &= ~TC_OVFINTLVL_HI_gc;
               else
                  pt_timerHndl->pt_timer1->INTCTRLA &= ~TC_OVFINTLVL_HI_gc;

            }/*End if( t_type == OVERFLOW)*/
            else if( t_type == COMPAREA)
            {
               if( pt_timerHndl->pt_timer0 != NULL)
                  pt_timerHndl->pt_timer0->INTCTRLB &= ~TC_CCAINTLVL_HI_gc;
               else
                  pt_timerHndl->pt_timer1->INTCTRLB &= ~TC_CCAINTLVL_HI_gc;

            }
            else if( t_type == COMPAREB)
            {
               if( pt_timerHndl->pt_timer0 != NULL)
                  pt_timerHndl->pt_timer0->INTCTRLB &= ~TC_CCBINTLVL_HI_gc;
               else
                  pt_timerHndl->pt_timer1->INTCTRLB &= ~TC_CCBINTLVL_HI_gc;
            }
            else if( t_type == COMPAREC)
            {
               if( pt_timerHndl->pt_timer0 != NULL)
                  pt_timerHndl->pt_timer0->INTCTRLB &= ~TC_CCCINTLVL_HI_gc;
               else
                  pt_timerHndl->pt_timer1->INTCTRLB &= ~TC_CCCINTLVL_HI_gc;
            }
            else if( t_type == COMPARED)
            {
               if( pt_timerHndl->pt_timer0 != NULL)
                  pt_timerHndl->pt_timer0->INTCTRLB &= ~TC_CCDINTLVL_HI_gc;
               else
                  pt_timerHndl->pt_timer1->INTCTRLB &= ~TC_CCDINTLVL_HI_gc;
            }

            t_lErr = utl_destroyLink( gt_timerIntHndlList,
                                      t_linkHndl);

            HAL_END_CRITICAL();//Enable interrupts
            return TIMER_PASSED;

         }

      }/*End UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_timerIntHndlList, 
      s_count)*/

   }

   HAL_END_CRITICAL();//Enable interrupts

   return TIMER_INT_NOT_OPEN;

}/*End hal_releaseTimerInterrupt*/

/*---------------------------------------------------------------------------*
 * Request access to a particular timer module
 *---------------------------------------------------------------------------*/
t_TIMERHNDL hal_requestTimer( t_timerModId t_id)
{
   t_timerModHndl *pt_timerHndl;
   t_LINKHNDL t_linkHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( (t_id < TIMER_1) || (t_id > TIMER_8))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_TIMERHNDL)TIMER_INVALID_MODULE;
   }

   pt_timerHndl = findTimerElement( t_id);

   /*------------------------------------------------------------------------*
    * Has this timer already been opened?
    *------------------------------------------------------------------------*/
   if( pt_timerHndl == NULL) /*No*/
   {

      t_linkHndl = createTimerHandle();
      if( t_linkHndl < 0)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return (t_TIMERHNDL)TIMER_OUT_OF_HEAP;

      }

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);

      pt_timerHndl->b_validConfig = false;

      pt_timerHndl->t_id = t_id;

      /*---------------------------------------------------------------------*
       * Clock isn't configured until configureTimer gets called.
       *---------------------------------------------------------------------*/
      pt_timerHndl->t_clockSelection = TC_CLKSEL_OFF_gc;

      /*---------------------------------------------------------------------*
       * No interrupts opened against this handle.
       *---------------------------------------------------------------------*/
      pt_timerHndl->c_intCount = 0;

      switch( t_id)
      {
         case TIMER_1:

            pt_timerHndl->pt_timer0 = &TCC0;
            pt_timerHndl->pt_timer1 = NULL;

         break;

         case TIMER_2:

            pt_timerHndl->pt_timer0 = NULL;
            pt_timerHndl->pt_timer1 = &TCC1;

         break;

         case TIMER_3:

            pt_timerHndl->pt_timer0 = &TCD0;
            pt_timerHndl->pt_timer1 = NULL;

         break;

         case TIMER_4:

            pt_timerHndl->pt_timer0 = NULL;
            pt_timerHndl->pt_timer1 = &TCD1;

         break;

         case TIMER_5:

            pt_timerHndl->pt_timer0 = &TCE0;
            pt_timerHndl->pt_timer1 = NULL;

         break;

         case TIMER_6:

            pt_timerHndl->pt_timer0 = NULL;
            pt_timerHndl->pt_timer1 = &TCE1;

         break;

         case TIMER_7:

            pt_timerHndl->pt_timer0 = &TCF0;
            pt_timerHndl->pt_timer1 = NULL;

         break;

         case TIMER_8:

            pt_timerHndl->pt_timer0 = NULL;
            pt_timerHndl->pt_timer1 = &TCF1;

         break;

      }/*End switch( t_id)*/

   }/*End if( pt_timerHndl == NULL)*/
   else /*Yes*/
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_TIMERHNDL)TIMER_MODULE_OPEN;
   }

   HAL_END_CRITICAL();//Enable interrupts

   return (t_TIMERHNDL)t_linkHndl;

}/*End hal_requestTimer*/

t_timerError hal_releaseTimer( t_TIMERHNDL t_handle)
{
   static t_LINKHNDL t_linkHndl;
   static t_LINKHNDL t_prevLinkHndl;
   t_timerModHndl *pt_timerHndl;
   t_timerIntHndl *pt_timerIntHndl;
   t_timerIntId t_intId;
   t_compType t_type;
   int16_t s_count;
   t_linkedListError t_lErr;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Remove all the interrupts associated with this particular timer 
       * channel. Since UTL_TRAVERSE_CONTAINER traverses through the entire 
       * interrupt list changing the value of t_linkHndl on each iteration, 
       * once 't_linkHndl' has been deleted information about the next position
       * on the list is lost.  This issue is resolved by using a previous ptr 
       * to change 't_linkHndl' back to a valid location once an item has been
       * removed.
       *---------------------------------------------------------------------*/
      for( t_type = COMPAREA; t_type <= OVERFLOW; t_type++)
      {
         /*------------------------------------------------------------------*
          * Use the timer module id and interrrupt type as an index into the
          * list of possible interrupts.
          *------------------------------------------------------------------*/
         t_intId = (t_timerIntId)pt_timerHndl->t_id + (t_timerIntId)t_type;

         /*------------------------------------------------------------------*
          * Search the timer interrupt list for the requested ID
          *------------------------------------------------------------------*/
         UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_timerIntHndlList, s_count)
         {
            pt_timerIntHndl = (t_timerIntHndl *)
            UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);

            t_prevLinkHndl = UTL_GET_PREV_LINK( t_linkHndl);
            if( pt_timerIntHndl->t_id == t_intId)
            {
               pt_timerHndl->c_intCount--;

               if( t_type == OVERFLOW)
               {
                  if( pt_timerHndl->pt_timer0 != NULL)
                     pt_timerHndl->pt_timer0->INTCTRLA &= ~TC_OVFINTLVL_HI_gc;
                  else
                     pt_timerHndl->pt_timer1->INTCTRLA &= ~TC_OVFINTLVL_HI_gc;

               }/*End if( t_type == OVERFLOW)*/
               else if( t_type == COMPAREA)
               {
                  if( pt_timerHndl->pt_timer0 != NULL)
                     pt_timerHndl->pt_timer0->INTCTRLB &= ~TC_CCAINTLVL_HI_gc;
                  else
                     pt_timerHndl->pt_timer1->INTCTRLB &= ~TC_CCAINTLVL_HI_gc;

               }
               else if( t_type == COMPAREB)
               {
                  if( pt_timerHndl->pt_timer0 != NULL)
                     pt_timerHndl->pt_timer0->INTCTRLB &= ~TC_CCBINTLVL_HI_gc;
                  else
                     pt_timerHndl->pt_timer1->INTCTRLB &= ~TC_CCBINTLVL_HI_gc;
               }
               else if( t_type == COMPAREC)
               {
                  if( pt_timerHndl->pt_timer0 != NULL)
                     pt_timerHndl->pt_timer0->INTCTRLB &= ~TC_CCCINTLVL_HI_gc;
                  else
                     pt_timerHndl->pt_timer1->INTCTRLB &= ~TC_CCCINTLVL_HI_gc;
               }
               else if( t_type == COMPARED)
               {
                  if( pt_timerHndl->pt_timer0 != NULL)
                     pt_timerHndl->pt_timer0->INTCTRLB &= ~TC_CCDINTLVL_HI_gc;
                  else
                     pt_timerHndl->pt_timer1->INTCTRLB &= ~TC_CCDINTLVL_HI_gc;
               }

               t_lErr = utl_destroyLink( gt_timerIntHndlList,
                                         t_linkHndl);

               t_linkHndl = t_prevLinkHndl;

            }/*End if( pt_timerIntHndl->t_id == t_intId)*/

         }/*End UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_timerIntHndlList, 
         s_count)*/

      }/*End for( t_type = COMPAREA; t_type <= OVERFLOW; t_type++)*/

      pt_timerHndl->c_intCount = 0;

      /*---------------------------------------------------------------------*
       * Which timer is be used?
       *---------------------------------------------------------------------*/
      if( pt_timerHndl->pt_timer0 != NULL)
      {
         /*------------------------------------------------------------------*
          * Reset the timer module
          *------------------------------------------------------------------*/
         pt_timerHndl->pt_timer0->CTRLA &= ~TC0_CLKSEL_gm;
         pt_timerHndl->pt_timer0->CTRLA |= TC_CLKSEL_OFF_gc;
         pt_timerHndl->pt_timer0->CTRLFSET = TC_CMD_RESET_gc;

      }
      else
      {
         /*------------------------------------------------------------------*
          * Reset the timer module
          *------------------------------------------------------------------*/
         pt_timerHndl->pt_timer1->CTRLA &= ~TC0_CLKSEL_gm;
         pt_timerHndl->pt_timer1->CTRLA |= TC_CLKSEL_OFF_gc;
         pt_timerHndl->pt_timer1->CTRLFSET = TC_CMD_RESET_gc;

      }

      t_lErr = utl_destroyLink( gt_timerHndlList,
                                (t_LINKHNDL)t_handle);

   }

   HAL_END_CRITICAL();//Enable interrupts

   return TIMER_PASSED;

}/*End hal_releaseTimer*/

t_timerError hal_configureTimer( t_TIMERHNDL t_handle,
                                 t_timerConfig t_conf)
{
   t_timerModHndl *pt_timerHndl;
   int32_t i_topTicks = 0;
   uint16_t as_divisor[7] = {1,2,4,8,64,256,1024};
   uint8_t c_index = 0;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( (t_conf.t_mode < NORMAL) || (t_conf.t_mode > DUAL_SLOPE_BOTTOM)
          || (t_conf.t_mode == UNDEFINED))
      {
         HAL_END_CRITICAL();//Enable interrupts
         return TIMER_INVALID_WF_MODE;
      }

      if( (t_conf.t_dir < DIRECTION_UP) || (t_conf.t_dir > DIRECTION_DOWN))
      {
         HAL_END_CRITICAL();//Enable interrupts
         return TIMER_INVALID_DIR;
      }

      /*---------------------------------------------------------------------*
       * Find a clock divisor that gives enough resolution for the requested
       * period.
       *---------------------------------------------------------------------*/
      for( c_index = 0; c_index < 7; c_index++)
      {
         i_topTicks = (int32_t)(((float)F_CPU*t_conf.f_period) /
         as_divisor[c_index]);

         if( i_topTicks < MAX_TIMER_RANGE)
            break;

      }/*End for( c_index = 0; c_index < 7; c_index++)*/

      switch( as_divisor[c_index])
      {
         case 1:
            pt_timerHndl->t_clockSelection = TC_CLKSEL_DIV1_gc;
         break;

         case 2:
            pt_timerHndl->t_clockSelection = TC_CLKSEL_DIV2_gc;
         break;

         case 4:
            pt_timerHndl->t_clockSelection = TC_CLKSEL_DIV4_gc;
         break;

         case 8:
            pt_timerHndl->t_clockSelection = TC_CLKSEL_DIV8_gc;
         break;

         case 64:
            pt_timerHndl->t_clockSelection = TC_CLKSEL_DIV64_gc;
         break;

         case 256:
            pt_timerHndl->t_clockSelection = TC_CLKSEL_DIV256_gc;
         break;

         case 1024:
            pt_timerHndl->t_clockSelection = TC_CLKSEL_DIV1024_gc;
         break;

         default:

            HAL_END_CRITICAL();//Enable interrupts
            return TIMER_INVALID_PERIOD;

         break;

      }/*End switch( as_divisor[c_index])*/

      /*---------------------------------------------------------------------*
       * Which timer is being used?
       *---------------------------------------------------------------------*/
      if( pt_timerHndl->pt_timer0 != NULL)
      {
         /*------------------------------------------------------------------*
          * Reset the timer module
          *------------------------------------------------------------------*/
         pt_timerHndl->pt_timer0->CTRLA &= ~TC0_CLKSEL_gm;
         pt_timerHndl->pt_timer0->CTRLA |= TC_CLKSEL_OFF_gc;
         pt_timerHndl->pt_timer0->CTRLFSET = TC_CMD_RESET_gc;

         /*------------------------------------------------------------------*
          * Set the waveform generation mode
          *------------------------------------------------------------------*/
         pt_timerHndl->pt_timer0->CTRLB &= ~TC0_WGMODE_gm;
         pt_timerHndl->pt_timer0->CTRLB |= (TC_WGMODE_t)t_conf.t_mode;

         /*------------------------------------------------------------------*
          * Set the clock period
          *------------------------------------------------------------------*/
         pt_timerHndl->pt_timer0->PER = i_topTicks;

         /*------------------------------------------------------------------*
          * Set the clock direction.
          *------------------------------------------------------------------*/
         if( t_conf.t_dir == DIRECTION_DOWN)
            pt_timerHndl->pt_timer0->CTRLFSET = TC0_DIR_bm;
         else
            pt_timerHndl->pt_timer0->CTRLFCLR = TC0_DIR_bm;
      }
      else
      {
         /*------------------------------------------------------------------*
          * Reset the timer module
          *------------------------------------------------------------------*/
         pt_timerHndl->pt_timer1->CTRLA &= ~TC1_CLKSEL_gm;
         pt_timerHndl->pt_timer1->CTRLA |= TC_CLKSEL_OFF_gc;
         pt_timerHndl->pt_timer1->CTRLFSET = TC_CMD_RESET_gc;

         /*------------------------------------------------------------------*
          * Set the waveform generation mode
          *------------------------------------------------------------------*/
         pt_timerHndl->pt_timer1->CTRLB &= ~TC0_WGMODE_gm;
         pt_timerHndl->pt_timer1->CTRLB |= (TC_WGMODE_t)t_conf.t_mode;

         /*------------------------------------------------------------------*
          * Set the clock period
          *------------------------------------------------------------------*/
         pt_timerHndl->pt_timer1->PER = i_topTicks;

         /*------------------------------------------------------------------*
          * Set the clock direction.
          *------------------------------------------------------------------*/
         if( t_conf.t_dir == DIRECTION_DOWN)
            pt_timerHndl->pt_timer0->CTRLFSET = TC1_DIR_bm;
         else
            pt_timerHndl->pt_timer0->CTRLFCLR = TC1_DIR_bm;
      }

      pt_timerHndl->b_validConfig = true;

   }

   HAL_END_CRITICAL();//Enable interrupts

   return TIMER_PASSED;

}/*End hal_configureTimer*/

t_timerError hal_startTimer( t_TIMERHNDL t_handle)
{
   t_timerModHndl *pt_timerHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_timerHndl->b_validConfig == false)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return TIMER_NO_CONFIG;
      }

      if( pt_timerHndl->pt_timer0 != NULL)
      {
         pt_timerHndl->pt_timer0->CTRLA &= ~TC0_CLKSEL_gm;
         pt_timerHndl->pt_timer0->CTRLA |= pt_timerHndl->t_clockSelection;
      }
      else
      {
         pt_timerHndl->pt_timer1->CTRLA &= ~TC1_CLKSEL_gm;
         pt_timerHndl->pt_timer1->CTRLA |= pt_timerHndl->t_clockSelection;
      }

   }/*End if( pt_timerHndl != NULL)*/

   HAL_END_CRITICAL();//Enable interrupts

   return TIMER_PASSED;

}/*End hal_startTimer*/

t_timerError hal_stopTimer( t_TIMERHNDL t_handle)
{
   t_timerModHndl *pt_timerHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_timerHndl->pt_timer0 != NULL)
      {
         pt_timerHndl->pt_timer0->CTRLA &= ~TC0_CLKSEL_gm;
         pt_timerHndl->pt_timer0->CTRLA |= TC_CLKSEL_OFF_gc;
      }
      else
      {
         pt_timerHndl->pt_timer1->CTRLA &= ~TC1_CLKSEL_gm;
         pt_timerHndl->pt_timer1->CTRLA |= TC_CLKSEL_OFF_gc;
      }

   }

   HAL_END_CRITICAL();//Enable interrupts

   return TIMER_PASSED;

}/*End hal_stopTimer*/

int32_t hal_getTimerCount( t_TIMERHNDL t_handle)
{
   t_timerModHndl *pt_timerHndl;
   int32_t i_count;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      return (int32_t)TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_timerHndl->pt_timer0 != NULL)
      {
         i_count = pt_timerHndl->pt_timer0->CNT;
      }
      else
      {
         i_count = pt_timerHndl->pt_timer1->CNT;
      }

   }

   return i_count;

}/*End hal_getTimerCount*/

t_timerError hal_setCompareValue( t_TIMERHNDL t_handle,
                                  t_compType  t_type,
                                  uint16_t    s_value)
{

   t_timerModHndl *pt_timerHndl;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      return TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_timerHndl->pt_timer0 != NULL)
      {
         if( s_value > pt_timerHndl->pt_timer0->PER)
            return TIMER_COMP_INVALID;

         switch( t_type)
         {
            case COMPAREA:
               pt_timerHndl->pt_timer0->CCA = s_value;
            break;

            case COMPAREB:
               pt_timerHndl->pt_timer0->CCB = s_value;
            break;

            case COMPAREC:
               pt_timerHndl->pt_timer0->CCC = s_value;
            break;

            case COMPARED:
               pt_timerHndl->pt_timer0->CCD = s_value;
            break;

            default:
               return TIMER_INVALID_COMP;
            break;

         }/*End switch( t_type)*/

      }
      else
      {
         if( s_value > pt_timerHndl->pt_timer1->PERBUF)
            return TIMER_COMP_INVALID;

         switch( t_type)
         {
            case COMPAREA:
               pt_timerHndl->pt_timer1->CCA = s_value;
            break;

            case COMPAREB:
               pt_timerHndl->pt_timer1->CCB = s_value;
            break;

            default:
               return TIMER_INVALID_COMP;
            break;

         }/*End switch( t_type)*/
      }

   }

   return TIMER_PASSED;

}/*End hal_setCompareValue*/

int32_t hal_getPeriodValue( t_TIMERHNDL t_handle)
{
   t_timerModHndl *pt_timerHndl;
   int32_t i_value;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      return (int32_t)TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_timerHndl->pt_timer0 != NULL)
      {
 
         i_value = pt_timerHndl->pt_timer0->PER;
      }
      else
      {

         i_value = pt_timerHndl->pt_timer1->PER;
      }

   }

   return i_value;

}/*End hal_getPeriodValue*/

int32_t hal_getCompareValue( t_TIMERHNDL t_handle,
                             t_compType  t_type)
{
   t_timerModHndl *pt_timerHndl;
   int32_t i_value;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      return (int32_t)TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_timerHndl->pt_timer0 != NULL)
      {
         switch( t_type)
         {
            case COMPAREA:
               i_value = pt_timerHndl->pt_timer0->CCA;
            break;

            case COMPAREB:
               i_value = pt_timerHndl->pt_timer0->CCB;
            break;

            case COMPAREC:
               i_value = pt_timerHndl->pt_timer0->CCC;
            break;

            case COMPARED:
               i_value = pt_timerHndl->pt_timer0->CCD;
            break;

            default:
               i_value = (int32_t)TIMER_INVALID_COMP;
            break;

         }/*End switch( t_type)*/

      }
      else
      {
         switch( t_type)
         {
            case COMPAREA:
               i_value = pt_timerHndl->pt_timer1->CCA;
            break;

            case COMPAREB:
               i_value = pt_timerHndl->pt_timer1->CCB;
            break;

            default:
               i_value = (int32_t)TIMER_INVALID_COMP;
            break;

         }/*End switch( t_type)*/
      }

   }

   return i_value;

}/*End hal_getCompareValue*/

int16_t hal_getIntStatus( t_TIMERHNDL t_handle,
                          t_compType   t_type)
{
   t_timerModHndl *pt_timerHndl;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      return (int16_t)TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_timerHndl->pt_timer0 != NULL)
      {
         switch( t_type)
         {
            case COMPAREA:
               return ((int16_t)pt_timerHndl->pt_timer0->INTFLAGS &
               (int16_t)TC0_CCAIF_bm);
            break;

            case COMPAREB:
               return ((int16_t)pt_timerHndl->pt_timer0->INTFLAGS &
               (int16_t)TC0_CCBIF_bm);
            break;

            case COMPAREC:
               return ((int16_t)pt_timerHndl->pt_timer0->INTFLAGS &
               (int16_t)TC0_CCCIF_bm);
            break;

            case COMPARED:
               return ((int16_t)pt_timerHndl->pt_timer0->INTFLAGS &
               (int16_t)TC0_CCDIF_bm);
            break;

            case OVERFLOW:
               return ((int16_t)pt_timerHndl->pt_timer0->INTFLAGS &
               (int16_t)TC0_OVFIF_bm);
            break;

            default:

            break;

         }/*End switch( t_type)*/

      }
      else
      {
         switch( t_type)
         {
            case COMPAREA:
               return ((int16_t)pt_timerHndl->pt_timer1->INTFLAGS &
               (int16_t)TC1_CCAIF_bm);
            break;

            case COMPAREB:
               return ((int16_t)pt_timerHndl->pt_timer1->INTFLAGS &
               (int16_t)TC1_CCBIF_bm);
            break;

            case OVERFLOW:
               return ((int16_t)pt_timerHndl->pt_timer1->INTFLAGS &
               (int16_t)TC1_OVFIF_bm);
            break;

            default:

            break;

         }/*End switch( t_type)*/
      }

   }

   return (int16_t)TIMER_INVALID_COMP;

}/*End hal_getIntStatus*/

t_timerError hal_clearIntStatus( t_TIMERHNDL t_handle,
                                 t_compType   t_type)
{
   t_timerModHndl *pt_timerHndl;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      return TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_timerHndl->pt_timer0 != NULL)
      {
         switch( t_type)
         {
            case COMPAREA:
               pt_timerHndl->pt_timer0->INTFLAGS |= TC0_CCAIF_bm;
            break;

            case COMPAREB:
               pt_timerHndl->pt_timer0->INTFLAGS |= TC0_CCBIF_bm;
            break;

            case COMPAREC:
               pt_timerHndl->pt_timer0->INTFLAGS |= TC0_CCCIF_bm;
            break;

            case COMPARED:
               pt_timerHndl->pt_timer0->INTFLAGS |= TC0_CCDIF_bm;
            break;

            case OVERFLOW:
               pt_timerHndl->pt_timer0->INTFLAGS |= TC0_OVFIF_bm;
            break;

            default:
               return TIMER_INVALID_COMP;
            break;

         }/*End switch( t_type)*/

      }
      else
      {
         switch( t_type)
         {
            case COMPAREA:
               pt_timerHndl->pt_timer1->INTFLAGS |= TC1_CCAIF_bm;
            break;

            case COMPAREB:
               pt_timerHndl->pt_timer1->INTFLAGS |= TC1_CCBIF_bm;
            break;

            case OVERFLOW:
               pt_timerHndl->pt_timer1->INTFLAGS |= TC1_OVFIF_bm;
            break;

            default:
               return TIMER_INVALID_COMP;
            break;

         }/*End switch( t_type)*/
      }

   }

   return TIMER_PASSED;

}/*End hal_clearIntStatus*/

t_timerError hal_enableCompareChannel( t_TIMERHNDL t_handle,
                                       t_compType t_type,
                                       bool b_ouputOnPin)
{
   t_timerModHndl *pt_timerHndl;
   uint8_t c_gpioPin;
   t_gpioPort t_port = GPIO_PORTC;
   t_gpioConf t_conf;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      return TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_timerHndl->pt_timer0 != NULL)
      {
         switch( t_type)
         {
            case COMPAREA:
               pt_timerHndl->pt_timer0->CTRLB |= TC0_CCAEN_bm;
               c_gpioPin = PIN_0;
            break;

            case COMPAREB:
               pt_timerHndl->pt_timer0->CTRLB |= TC0_CCBEN_bm;
               c_gpioPin = PIN_1;
            break;

            case COMPAREC:
               pt_timerHndl->pt_timer0->CTRLB |= TC0_CCCEN_bm;
               c_gpioPin = PIN_2;
            break;

            case COMPARED:
               pt_timerHndl->pt_timer0->CTRLB |= TC0_CCDEN_bm;
               c_gpioPin = PIN_3;
            break;

            default:
               return TIMER_INVALID_COMP;
            break;

         }/*End switch( t_type)*/

      }
      else
      {
         switch( t_type)
         {
            case COMPAREA:
               pt_timerHndl->pt_timer1->CTRLB |= TC1_CCAEN_bm;
               c_gpioPin = PIN_4;
            break;

            case COMPAREB:
               pt_timerHndl->pt_timer1->CTRLB |= TC1_CCBIF_bm;
               c_gpioPin = PIN_5;
            break;

            default:
               return TIMER_INVALID_COMP;
            break;

         }/*End switch( t_type)*/

      }

      switch( pt_timerHndl->t_id)
      {
         case TIMER_1:
         case TIMER_2:
            t_port = GPIO_PORTC;
         break;

         case TIMER_3:
         case TIMER_4:
            t_port = GPIO_PORTD;
         break;

         case TIMER_5:
         case TIMER_6:
            t_port = GPIO_PORTE;
         break;

         case TIMER_7:
         case TIMER_8:
            t_port = GPIO_PORTF;
         break;

      }/*End switch( pt_timerHndl->t_id)*/

      /*---------------------------------------------------------------------*
       * If required, output the compare channel for the given port pin.
       *---------------------------------------------------------------------*/
      if( b_ouputOnPin == true)
      {
         t_conf.c_outputMask   = c_gpioPin;
         t_conf.b_setOutputLow = true;
         t_conf.c_inputMask    = 0;
         t_conf.t_outConf      = TOTEM;
      }
      else
      {
         t_conf.c_outputMask = 0;
         t_conf.c_inputMask  = c_gpioPin;
         t_conf.t_inConf     = PULLDOWN;
      }

      if( hal_configureGpioPort( t_port, t_conf) < 0)
      {
         return TIMER_INV_GPIO_CONFIG;
      }
 
      /*---------------------------------------------------------------------*
       * If output require, make sure the pin is initially low...
       *---------------------------------------------------------------------*/ 
		if( b_ouputOnPin == true)
		   hal_gpioOff( t_port, c_gpioPin);
   }

   return TIMER_PASSED;

}/*End hal_enableCompareChannel*/

t_timerError hal_disableCompareChannel( t_TIMERHNDL t_handle,
                                        t_compType  t_type)
{
   t_timerModHndl *pt_timerHndl;
   uint8_t c_gpioPin;
   t_gpioPort t_port = GPIO_PORTC;
   t_gpioConf t_conf;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a Timer channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_timerHndlList) == false)
   {
      return TIMER_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the Timer 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_timerHndl = (t_timerModHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_timerHndl->pt_timer0 != NULL)
      {
         switch( t_type)
         {
            case COMPAREA:
               pt_timerHndl->pt_timer0->CTRLB &= ~TC0_CCAEN_bm;
               c_gpioPin = PIN_0;
            break;

            case COMPAREB:
               pt_timerHndl->pt_timer0->CTRLB &= ~TC0_CCBEN_bm;
               c_gpioPin = PIN_1;
            break;

            case COMPAREC:
               pt_timerHndl->pt_timer0->CTRLB &= ~TC0_CCCEN_bm;
               c_gpioPin = PIN_2;
            break;

            case COMPARED:
               pt_timerHndl->pt_timer0->CTRLB &= ~TC0_CCDEN_bm;
               c_gpioPin = PIN_3;
            break;

            default:
               return TIMER_INVALID_COMP;
            break;

         }/*End switch( t_type)*/

      }
      else
      {
         switch( t_type)
         {
            case COMPAREA:
               pt_timerHndl->pt_timer1->CTRLB &= ~TC1_CCAEN_bm;
               c_gpioPin = PIN_4;
            break;

            case COMPAREB:
               pt_timerHndl->pt_timer1->CTRLB &= ~TC1_CCBIF_bm;
               c_gpioPin = PIN_5;
            break;

            default:
               return TIMER_INVALID_COMP;
            break;

         }/*End switch( t_type)*/
      }

      switch( pt_timerHndl->t_id)
      {
         case TIMER_1:
         case TIMER_2:
            t_port = GPIO_PORTC;
         break;

         case TIMER_3:
         case TIMER_4:
            t_port = GPIO_PORTD;
         break;

         case TIMER_5:
         case TIMER_6:
            t_port = GPIO_PORTE;
         break;

         case TIMER_7:
         case TIMER_8:
            t_port = GPIO_PORTF;
         break;

      }/*End switch( pt_timerHndl->t_id)*/

      /*---------------------------------------------------------------------*
       * Shut off the output by configuring the pin as an input.
       *---------------------------------------------------------------------*/
      t_conf.c_inputMask    = c_gpioPin;
      t_conf.c_outputMask   = 0;
      t_conf.b_setOutputLow = false;
      t_conf.t_inConf       = PULLDOWN;

      if( hal_configureGpioPort( t_port, t_conf) < 0)
      {
         return TIMER_INV_GPIO_CONFIG;
      }

   }

   return TIMER_PASSED;

}/*End hal_disableCompareChannel*/
