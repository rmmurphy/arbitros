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
 * File Name   : utl_linkedList.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides an abstraction layer for the creation and
 *               control over linked-list objects.
 *
 * Last Update : June 11, 2012
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "arb_memory.h"
#include "utl_linkedList.h"
#include "hal_pmic.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define LINKEDLIST_LINK_UNMAPPED (0)

/*---------------------------------------------------------------------------*
 * Private Type Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
static uint32_t gi_dynListMemUsage = 0; /*Keeps track of total dyn mem used*/

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
uint32_t utl_getDynListMemUsage( void)
{
   return gi_dynListMemUsage;
}/*End utl_getDynListMemUsage*/

t_CONTHNDL utl_createContainer( void)
{
   t_listContainer *pt_cont;

   /*------------------------------------------------------------------------*
    * We are going to be adding an element to a shared list so enforce
    * mutual exclusion.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Allocated memory the new linked-list 'container'
    *------------------------------------------------------------------------*/
   arb_malloc( sizeof( t_listContainer),
               (void **)&pt_cont);

   //pt_cont = (t_listContainer *)malloc( sizeof( t_listContainer));

   if( pt_cont == NULL)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_CONTHNDL)LINKEDLIST_OUT_OF_HEAP;
   }/*End if( pt_cont == NULL)*/

   pt_cont->s_checkSum      = LINKEDLIST_CHECKSUM;
   pt_cont->s_numLinks      = 0;
   pt_cont->s_contSizeBytes = (uint16_t)sizeof( t_listContainer);
   pt_cont->pt_head         = NULL;
   pt_cont->pt_tail         = NULL;

   /*------------------------------------------------------------------------*
    * Keep track of the total memory usage...
    *------------------------------------------------------------------------*/
   gi_dynListMemUsage += pt_cont->s_contSizeBytes;

   HAL_END_CRITICAL();//Enable interrupts

   return (t_CONTHNDL)pt_cont;

}/*End utl_createContainer*/

t_linkedListError utl_destroyContainer( t_CONTHNDL t_contHndl)
{
   t_listContainer *pt_cont;

   /*------------------------------------------------------------------------*
    * We are going to be adding an element to a shared list so enforce
    * mutual exclusion.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   pt_cont = (t_listContainer *)t_contHndl;

   if( pt_cont->s_checkSum == LINKEDLIST_CHECKSUM)
   {
      /*---------------------------------------------------------------------*
       * Keep track of the total amount of memory being consumed...
       *---------------------------------------------------------------------*/
      gi_dynListMemUsage -= pt_cont->s_contSizeBytes;

      /*---------------------------------------------------------------------*
       * Delete this 'link' from memory.
       *---------------------------------------------------------------------*/
      arb_free( (void **)&pt_cont);
      //free( (void *)pt_cont);

      /*---------------------------------------------------------------------*
       * Clear the old memory region.
       *---------------------------------------------------------------------*/
      memset( pt_cont, 0, sizeof( t_listContainer));

   }/*End if( pt_cont->s_checkSum == LINKEDLIST_CHECKSUM)*/
   else
   {
      HAL_END_CRITICAL();//Enable interrupts
      return LINKEDLIST_INVALID_CONT;
   }

   HAL_END_CRITICAL();//Enable interrupts

   return LINKEDLIST_PASSED;

}/*End utl_destroyContainer*/

t_LINKHNDL utl_createLink( uint16_t s_elementSizeBytes)
{
   t_listLink *pt_link;
   void *pv_element;

   /*------------------------------------------------------------------------*
    * We are going to be adding an element to a shared list so enforce
    * mutual exclusion.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Allocated memory for the new linked-list 'link'
    *------------------------------------------------------------------------*/
   arb_malloc( sizeof(t_listLink),
               (void **)&pt_link);

   //pt_link = (t_listLink *)malloc( sizeof(t_listLink));

   if( pt_link == NULL)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_LINKHNDL)LINKEDLIST_OUT_OF_HEAP;
   }/*End if( pt_link == NULL)*/

   /*------------------------------------------------------------------------*
    * Allocated memory for the new linked-list 'element'
    *------------------------------------------------------------------------*/
   arb_malloc( (size_t)s_elementSizeBytes,
               (void **)&pv_element);

   //pv_element = (void *)malloc( s_elementSizeBytes);

   if( pv_element == NULL)
   {
      /*---------------------------------------------------------------------*
       * Delete this 'link' from memory.
       *---------------------------------------------------------------------*/
      arb_free( (void **)&pt_link);
      //free( (void *)pt_link);
      HAL_END_CRITICAL();//Enable interrupts
      return (t_LINKHNDL)LINKEDLIST_OUT_OF_HEAP;
   }/*End if( pv_element == NULL)*/

   /*------------------------------------------------------------------------*
    * Initialize the element memory with 0xFF's
    *------------------------------------------------------------------------*/
   memset( pv_element, 0xFF, s_elementSizeBytes);

   pt_link->pv_element         = pv_element;
   pt_link->s_contAddr         = LINKEDLIST_LINK_UNMAPPED;
   pt_link->s_elementSizeBytes = s_elementSizeBytes;
   pt_link->s_linkSizeBytes    = (uint16_t)sizeof( t_listLink) +
   s_elementSizeBytes;
   pt_link->pt_next            = NULL;
   pt_link->pt_prev            = NULL;

   /*------------------------------------------------------------------------*
    * Keep track of the total memory usage...
    *------------------------------------------------------------------------*/
   gi_dynListMemUsage += pt_link->s_linkSizeBytes;

   HAL_END_CRITICAL();//Enable interrupts

   return (t_LINKHNDL)pt_link;

}/*End utl_createLink*/

t_linkedListError utl_insertLink( t_CONTHNDL t_contHndl,
                                  t_LINKHNDL t_linkHndl,
                                  bool b_tail)
{
   t_listContainer *pt_cont;
   t_listLink *pt_link;

   /*------------------------------------------------------------------------*
    * We are going to be adding an element to a shared list so enforce
    * mutual exclusion.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   pt_cont = (t_listContainer *)t_contHndl;

   /*------------------------------------------------------------------------*
    * Is this a valid container?
    *------------------------------------------------------------------------*/
   if( pt_cont->s_checkSum == LINKEDLIST_CHECKSUM) /*Yes*/
   {

      pt_link = (t_listLink *)t_linkHndl;

      /*---------------------------------------------------------------------*
       * Is this 'link' already on this list or another one?
       *---------------------------------------------------------------------*/
      if( pt_link->s_contAddr != LINKEDLIST_LINK_UNMAPPED) /*Yes*/
      {
         HAL_END_CRITICAL();//Enable interrupts
         return LINKEDLIST_LINK_MAPPED;
      }

      /*---------------------------------------------------------------------*
       * Keep track of the 'container' where this 'link' belongs...
       *---------------------------------------------------------------------*/
      pt_link->s_contAddr = t_contHndl;

      /*---------------------------------------------------------------------*
       * Are we adding the very first node on the list
       *---------------------------------------------------------------------*/
      if( (pt_cont->pt_head == NULL) && (pt_cont->pt_tail == NULL))
      {
         /*------------------------------------------------------------------*
          * Establish circular link
          *------------------------------------------------------------------*/
         pt_link->pt_prev = pt_link;
         pt_link->pt_next = pt_link;

         /*------------------------------------------------------------------*
          * Move head and tail location
          *------------------------------------------------------------------*/
         (pt_cont->pt_head) = pt_link;
         (pt_cont->pt_tail) = pt_link;

      }/*End if( (pt_cont->pt_head == NULL) && (pt_cont->pt_tail == NULL))*/
      else
      {

         pt_link->pt_prev  = (pt_cont->pt_tail);
         pt_link->pt_next  = (pt_cont->pt_head);
         (pt_cont->pt_head)->pt_prev = pt_link;
         (pt_cont->pt_tail)->pt_next = pt_link;

         if( b_tail == true)
         {
            /*---------------------------------------------------------------*
             * Move tail location
             *---------------------------------------------------------------*/
            (pt_cont->pt_tail) = pt_link;
         }
         else
         {
             /*---------------------------------------------------------------*
             * Move head location
             *---------------------------------------------------------------*/
            (pt_cont->pt_head) = pt_link;
         }
      }

      /*---------------------------------------------------------------------*
       * Update the status for this container...
       *---------------------------------------------------------------------*/
      pt_cont->s_numLinks++;
      pt_cont->s_contSizeBytes += pt_link->s_linkSizeBytes;

   }/*End if( pt_cont->s_checkSum == LINKEDLIST_CHECKSUM)*/
   else
   {
      HAL_END_CRITICAL();//Enable interrupts
      return LINKEDLIST_INVALID_CONT;
   }

   HAL_END_CRITICAL();//Enable interrupts

   return LINKEDLIST_PASSED;

}/*End utl_insertLink*/

t_linkedListError utl_destroyLink( t_CONTHNDL t_contHndl,
                                   t_LINKHNDL t_linkHndl)
{
   t_listContainer *pt_cont;
   t_listLink *pt_curr;
   static t_listLink *pt_prev = NULL;
   static t_listLink *pt_next = NULL;

   /*------------------------------------------------------------------------*
    * Since we are about to act on global variables, protect this region
    * of code against higher priority threads interrupting us while we are
    * trying to register.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   pt_cont = (t_listContainer *)t_contHndl;

   /*------------------------------------------------------------------------*
    * Is this a valid container?
    *------------------------------------------------------------------------*/
   if( pt_cont->s_checkSum == LINKEDLIST_CHECKSUM) /*Yes*/
   {
      pt_curr = (t_listLink *)t_linkHndl;

      /*---------------------------------------------------------------------*
       * Is this 'link' on the list?
       *---------------------------------------------------------------------*/
      if( UTL_IS_LINK_ON_LIST( t_linkHndl, t_contHndl) == false) /*No*/
      {
         HAL_END_CRITICAL();//Enable interrupts
         return LINKEDLIST_INVALID_LINK;
      }

      pt_prev = pt_curr->pt_prev;
      pt_next = pt_curr->pt_next;

      /*---------------------------------------------------------------------*
       * Are there other 'links' still on the list?
       *---------------------------------------------------------------------*/
      if( pt_curr != pt_next)
      {
         /*------------------------------------------------------------------*
          * Remove the node from this list.
          *------------------------------------------------------------------*/
         pt_prev->pt_next = pt_next;
         pt_next->pt_prev = pt_prev;

         /*------------------------------------------------------------------*
          * Were we at the start or end of the list?
          *------------------------------------------------------------------*/
         if( pt_curr == pt_cont->pt_head)
            pt_cont->pt_head = pt_next;
         else if( pt_curr == pt_cont->pt_tail)
            pt_cont->pt_tail = pt_prev;

      }
      else /*Last node*/
      {
         pt_cont->pt_head = NULL;
         pt_cont->pt_tail = NULL;

      }

      /*---------------------------------------------------------------------*
       * Keep track of our memory usage.
       *---------------------------------------------------------------------*/
      pt_cont->s_numLinks--;
      pt_cont->s_contSizeBytes -= pt_curr->s_linkSizeBytes;

      /*---------------------------------------------------------------------*
       * Keep track of the total amount of memory being consumed...
       *---------------------------------------------------------------------*/
      gi_dynListMemUsage -= pt_curr->s_linkSizeBytes;

      /*---------------------------------------------------------------------*
       * Delete the 'element' from memory.
       *---------------------------------------------------------------------*/
      arb_free( (void **)&pt_curr->pv_element);
      //free( pt_curr->pv_element);

      /*---------------------------------------------------------------------*
       * Clear the old memory region.
       *---------------------------------------------------------------------*/
      memset( pt_curr->pv_element, 0, pt_curr->s_elementSizeBytes);

      /*---------------------------------------------------------------------*
       * Delete this 'link' from memory.
       *---------------------------------------------------------------------*/
      arb_free( (void **)&pt_curr);
      //free( (void *)pt_curr);

      /*---------------------------------------------------------------------*
       * Clear the old memory region.
       *---------------------------------------------------------------------*/
      memset( pt_curr, 0, sizeof( t_listLink));

   }/*End if( pt_cont->s_checkSum == LINKEDLIST_CHECKSUM)*/
   else
   {
      HAL_END_CRITICAL();//Enable interrupts
      return LINKEDLIST_INVALID_CONT;
   }

   HAL_END_CRITICAL();//Enable interrupts

   return LINKEDLIST_PASSED;

}/*End utl_destroyLink*/

t_linkedListError utl_removeLink( t_CONTHNDL t_contHndl,
                                  t_LINKHNDL t_linkHndl)
{
   t_listContainer *pt_cont;
   t_listLink *pt_curr;
   static t_listLink *pt_prev = NULL;
   static t_listLink *pt_next = NULL;

   /*------------------------------------------------------------------------*
    * Since we are about to act on global variables, protect this region
    * of code against higher priority threads interrupting us while we are
    * trying to register.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   pt_cont = (t_listContainer *)t_contHndl;

   /*------------------------------------------------------------------------*
    * Is this a valid container handle?
    *------------------------------------------------------------------------*/
   if( pt_cont->s_checkSum == LINKEDLIST_CHECKSUM) /*Yes*/
   {
      pt_curr = (t_listLink *)t_linkHndl;

      /*---------------------------------------------------------------------*
       * Is this 'link' on the list?
       *---------------------------------------------------------------------*/
      if( UTL_IS_LINK_ON_LIST( t_linkHndl, t_contHndl) == false) /*No*/
      {
         HAL_END_CRITICAL();//Enable interrupts
         return LINKEDLIST_INVALID_LINK;
      }

      /*---------------------------------------------------------------------*
       * Make sure the link is no longer mapped to this particular container.
       *---------------------------------------------------------------------*/
      pt_curr->s_contAddr = LINKEDLIST_LINK_UNMAPPED;

      pt_prev = pt_curr->pt_prev;
      pt_next = pt_curr->pt_next;

      /*---------------------------------------------------------------------*
       * Are there other 'links' still on the list?
       *---------------------------------------------------------------------*/
      if( pt_curr != pt_next)
      {
         /*------------------------------------------------------------------*
          * Remove the node from this list.
          *------------------------------------------------------------------*/
         pt_prev->pt_next = pt_next;
         pt_next->pt_prev = pt_prev;

         /*------------------------------------------------------------------*
          * Were we at the start or end of the list?
          *------------------------------------------------------------------*/
         if( pt_curr == pt_cont->pt_head)
            pt_cont->pt_head = pt_next;
         else if( pt_curr == pt_cont->pt_tail)
            pt_cont->pt_tail = pt_prev;

      }
      else /*Last node*/
      {
         pt_cont->pt_head = NULL;
         pt_cont->pt_tail = NULL;

      }

      /*---------------------------------------------------------------------*
       * This link is no longer mapped to a list...
       *---------------------------------------------------------------------*/
      pt_curr->s_contAddr = LINKEDLIST_LINK_UNMAPPED;
      pt_curr->pt_next    = NULL;
      pt_curr->pt_prev    = NULL;

      /*---------------------------------------------------------------------*
       * Keep track of the size of the container...
       *---------------------------------------------------------------------*/
      pt_cont->s_numLinks--;
      pt_cont->s_contSizeBytes -= pt_curr->s_linkSizeBytes;

   }/*End if( pt_cont->s_checkSum == LINKEDLIST_CHECKSUM)*/
   else
   {
      HAL_END_CRITICAL();//Enable interrupts
      return LINKEDLIST_INVALID_CONT;
   }

   return LINKEDLIST_PASSED;

}/*End utl_removeLink*/
