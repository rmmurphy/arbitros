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
 * File Name   : arb_mailbox.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for mailbox creation and control
 *               which can be used for inter-thread or interrupt to thread 
 *               communication. In the case of inter-thread comm the read
 *               and write operations can be setup for either blocking or non-
 *               blocking calls; conversely, in the case where an interrupt
 *               is writing, the writer should be configured for non-blocking
 *               operation only.
 *
 * References  : 1) http://en.wikipedia.org/wiki/Producer-consumer_problem
 *
 * Last Update : Jan, 5, 2013
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <string.h>
#include "arb_scheduler.h"
#include "arb_thread.h"
#include "arb_semaphore.h"
#include "arb_mailbox.h"
#include "hal_dma.h"
#include "utl_linkedlist.h"
#include "hal_pmic.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define MBX_HDR_FIELD_BYTES (2)

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct
{

   /*------------------------------------------------------------------------*
    * Mutual exclusion of the shared resources 's_wrPtr', 's_rdPtr', and
    * 's_numMessages'.
    *------------------------------------------------------------------------*/       
   t_SEMHANDLE t_mutex;
   /*------------------------------------------------------------------------*
    * The number of messages in the queue
    *------------------------------------------------------------------------*/       
   t_SEMHANDLE t_semFillCount;
   /*------------------------------------------------------------------------*
    * The number of open slots in the queue
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_semEmptyCount;
   /*------------------------------------------------------------------------*
    * The max message size in bytes
    *------------------------------------------------------------------------*/
   uint16_t s_queueSize;
   /*------------------------------------------------------------------------*
    * The max number of messages in the queue
    *------------------------------------------------------------------------*/
   uint16_t s_queueDepth;
   /*------------------------------------------------------------------------*
    * Current write position
    *------------------------------------------------------------------------*/
   uint16_t s_wrPtr;
   /*------------------------------------------------------------------------*
    * Current read position
    *------------------------------------------------------------------------*/
   uint16_t s_rdPtr;
   /*------------------------------------------------------------------------*
    * The number of messages contained in the queue.
    *------------------------------------------------------------------------*/
   int16_t s_numMessages;
   /*------------------------------------------------------------------------*
    * Determines whether or not the writing process blocks when the queue
    * is full.
    *------------------------------------------------------------------------*/   
   t_semMode t_writeMode;
   /*------------------------------------------------------------------------*
    * Determines whether or not the reading process blocks with the queue
    * is empty.
    *------------------------------------------------------------------------*/
   t_semMode t_readMode;
   /*------------------------------------------------------------------------*
    * If true, 'arb_mbxWrite' can be made via an interrupt (in which case
    * t_writeMode has to be set to NONBLOCKING). During this mode multiple
    * readers and writers is always assumed.
    *------------------------------------------------------------------------*/
   bool b_wrtFromInt;
   /*------------------------------------------------------------------------*
    * If true, there can be multiple readers and writers requiring locks 
    * during the update of shared memory variables. This will increase the 
    * processing overhead of the primary mailbox calls. If 'b_wrtFromInt' is 
    * true, this variable is ignored.
    *------------------------------------------------------------------------*/
   bool b_multRdWr;
   /*------------------------------------------------------------------------*
    * Pointer to the start of the mailbox queue where each location is 
    * prefixed with a 16-bit header field representing the amount of user data
    * available for that given location. In order to allow for multiple 
    * readers, an ID field could be added to the header (as well as an input
    * parameter to arb_mbxRead and arb_mbxWrite) which would allow
    * the mailbox to pop off messages specific only to that particular ID. 
    *
    *             |<-s_queueSize + MBX_HDR_FIELD_BYTES->|
    *             .-------------------.-----------------. ---------
    * pc_queue -> |            header | data            |     ^
    *             '-------------------'-----------------'     |
    *  s_rdPtr -> |            header | data            |     |
    *             '-------------------'-----------------'     |
    *             |            header | data            |  
    *             '-------------------'-----------------' s_queueDepth
    *                                 o             
    *                                 o                       |
    *                                 o                       |
    *             .-------------------'-----------------.     |
    *  s_wrPtr -> |            header | data            |     v
    *             '-------------------'-----------------' ---------
    *------------------------------------------------------------------------*/   
   int8_t *pc_queue;

}t_mailbox;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Linked list of all the mailbox's running on the system.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_activeMbx);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
int16_t arb_mailboxRead( t_MAILBOXHNDL t_mbxHandle,
                         /*int8_t c_ID,-Todo allow for multiple readers*/
                         int8_t *pc_buf,
                         uint16_t s_size)
{
   t_mailbox *pt_mbx;
   t_error t_err;
   int16_t s_size1 = 0;

   /*------------------------------------------------------------------------*
    * Is this a valid mailbox handle?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_mbxHandle, gt_activeMbx) == 
   false)
   {
      return (int16_t)ARB_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where information
       * about the mailbox is stored.
       *---------------------------------------------------------------------*/
      pt_mbx = (t_mailbox *)UTL_GET_LINK_ELEMENT_PTR( t_mbxHandle);

      /*---------------------------------------------------------------------*
       * Can the user-space buffer hold the data?
       *---------------------------------------------------------------------*/
      if( s_size < pt_mbx->s_queueSize) /*No*/
      {
         return (int16_t)ARB_READ_ERROR; /*User-space buffer too small*/
      }

      /*---------------------------------------------------------------------*
       * Wait for data to become available on the queue
       *---------------------------------------------------------------------*/
      t_err = arb_wait( pt_mbx->t_semFillCount, pt_mbx->t_readMode);
      if( t_err == ARB_PASSED)
      {
         int16_t s_index;

         /*------------------------------------------------------------------*
          * The buffer has data...Copy data from the mailbox into pc_buf
          *------------------------------------------------------------------*/
         if( pt_mbx->b_wrtFromInt == true) /*Always assume multiple 
                                             readers/writers*/
         {
            /*---------------------------------------------------------------*
             * We can't perform a mutex inside an interrupt so make sure this 
             * call is atomic by another means.
             *---------------------------------------------------------------*/
            HAL_BEGIN_CRITICAL(); //Disable interrupts

            /*---------------------------------------------------------------*
             * Grab the size of this data field from the first two bytes of 
             * this queue location.
             *---------------------------------------------------------------*/
            s_index = (pt_mbx->s_rdPtr)*(pt_mbx->s_queueSize + \
            MBX_HDR_FIELD_BYTES);
            s_size1 = (uint16_t)pt_mbx->pc_queue[ s_index];

            /*---------------------------------------------------------------*
             * Store the data in the remaining bytes...
             *---------------------------------------------------------------*/
            memcpy( (void *)pc_buf, 
                    (void *)&pt_mbx->pc_queue[ s_index + MBX_HDR_FIELD_BYTES], 
                    s_size1);

            /*---------------------------------------------------------------*
             * Increment the read pointer
             *---------------------------------------------------------------*/
            pt_mbx->s_rdPtr++;
            if( pt_mbx->s_rdPtr == pt_mbx->s_queueDepth)
               pt_mbx->s_rdPtr = 0;

            /*---------------------------------------------------------------*
             * Keep track of the number of open slots on the queue.
             *---------------------------------------------------------------*/
            pt_mbx->s_numMessages--;

            HAL_END_CRITICAL(); //Enable interrupts

         }/*End if( pt_mbx->b_wrtFromInt == true)*/
         else
         {
            if( pt_mbx->b_multRdWr == true)
               t_err = arb_wait( pt_mbx->t_mutex, BLOCKING); /*Lock shared
                                                               memory*/

            /*---------------------------------------------------------------*
             * Grab the size of this data field from the first two bytes of 
             * this queue location.
             *---------------------------------------------------------------*/
            s_index = (pt_mbx->s_rdPtr)*(pt_mbx->s_queueSize + \
            MBX_HDR_FIELD_BYTES);
            s_size1 = (uint16_t)pt_mbx->pc_queue[ s_index];

            /*---------------------------------------------------------------*
             * Store the data in the remaining bytes...
             *---------------------------------------------------------------*/
            memcpy( (void *)pc_buf, 
                    (void *)&pt_mbx->pc_queue[ s_index + MBX_HDR_FIELD_BYTES], 
                    s_size1);

            /*---------------------------------------------------------------*
             * Increment the read pointer
             *---------------------------------------------------------------*/
            pt_mbx->s_rdPtr++;
            if( pt_mbx->s_rdPtr == pt_mbx->s_queueDepth)
               pt_mbx->s_rdPtr = 0;

            /*---------------------------------------------------------------*
             * Keep track of the number of open slots on the queue.
             *---------------------------------------------------------------*/
            pt_mbx->s_numMessages--;

            if( pt_mbx->b_multRdWr == true)
               t_err = arb_signal( pt_mbx->t_mutex);
         }

         /*------------------------------------------------------------------*
          * Tell the producer to put more data in the buffer.
          *------------------------------------------------------------------*/
         arb_signal( pt_mbx->t_semEmptyCount); /*Increment the empty count*/

      }/*End if( t_err == ARB_PASSED)*/
      else if( t_err == ARB_SEM_DEC_ERROR) /*Non blocking mode*/
      {
         /*------------------------------------------------------------------*
          * Tell the consumer that the buffer is empty
          *------------------------------------------------------------------*/
         return (int16_t)ARB_MBX_EMPTY;
      }
   }

   return (int16_t)s_size1; /*Return the number of bytes read*/

}/*End arb_mailboxRead*/

int16_t arb_mailboxWrite( t_MAILBOXHNDL t_mbxHandle, 
                          /*int8_t c_ID,-Todo allow for multiple readers*/
                          int8_t *pc_buf, 
                          uint16_t s_size) /*Size of the message in pc_buf*/
{
   t_mailbox *pt_mbx;
   t_error t_err;

   /*------------------------------------------------------------------------*
    * Is this a valid mailbox handle?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_mbxHandle, gt_activeMbx) == 
   false)
   {
      return (int16_t)ARB_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where information
       * about the mailbox is stored.
       *---------------------------------------------------------------------*/
      pt_mbx = (t_mailbox *)UTL_GET_LINK_ELEMENT_PTR( t_mbxHandle);

      /*---------------------------------------------------------------------*
       * Can the data fit on the queue?
       *---------------------------------------------------------------------*/
      if( s_size > pt_mbx->s_queueSize) /*No*/
      {
         return (int16_t)ARB_WRITE_ERROR; /*User-space buffer too large*/
      }

      /*---------------------------------------------------------------------*
       * Wait for room on the queue..
       *---------------------------------------------------------------------*/
      t_err = arb_wait( pt_mbx->t_semEmptyCount, pt_mbx->t_writeMode);
      if( t_err == ARB_PASSED)
      {
         int8_t *pc_size;
         int16_t s_index;

         if( pt_mbx->b_wrtFromInt == true) /*Always assume multiple 
                                             readers/writers*/
         {
            /*---------------------------------------------------------------*
             * We can't perform a mutex inside an interrupt so make sure this 
             * call is atomic by another means.
             *---------------------------------------------------------------*/
            HAL_BEGIN_CRITICAL(); //Disable interrupts

            /*---------------------------------------------------------------*
             * The buffer has room...Copy data from the pc_buf into the mailbox. 
             *---------------------------------------------------------------*/
            pc_size = (int8_t *)&s_size;

            /*---------------------------------------------------------------*
             * Store the size in the first two bytes of the current queue
             * location.
             *---------------------------------------------------------------*/
            s_index = (pt_mbx->s_wrPtr)*(pt_mbx->s_queueSize + \
            MBX_HDR_FIELD_BYTES);
            pt_mbx->pc_queue[s_index]     = pc_size[0];
            pt_mbx->pc_queue[s_index + 1] = pc_size[1];

            /*---------------------------------------------------------------*
             * Store the data in the remaining bytes...
             *---------------------------------------------------------------*/
            memcpy( (void *)&pt_mbx->pc_queue[ s_index + MBX_HDR_FIELD_BYTES], 
                    (void *)pc_buf,  
                    s_size);

            /*---------------------------------------------------------------*
             * Increment the write pointer
             *---------------------------------------------------------------*/
            pt_mbx->s_wrPtr++;
            if( pt_mbx->s_wrPtr == pt_mbx->s_queueDepth)
               pt_mbx->s_wrPtr = 0;

            /*---------------------------------------------------------------*
             * Keep track of the number of open slots on the queue.
             *---------------------------------------------------------------*/
            pt_mbx->s_numMessages++;

            HAL_END_CRITICAL(); //Enable interrupts

         }/*End if( pt_mbx->b_wrtFromInt == true)*/
         else
         {
            if( pt_mbx->b_multRdWr == true)
               t_err = arb_wait( pt_mbx->t_mutex, BLOCKING); /*Lock shared 
                                                               memory*/

            /*---------------------------------------------------------------*
             * The buffer has room...Copy data from the pc_buf into the mailbox. 
             *---------------------------------------------------------------*/
            pc_size = (int8_t *)&s_size;

            /*---------------------------------------------------------------*
             * Store the size in the first two bytes of the current queue
             * location.
             *---------------------------------------------------------------*/
            s_index = (pt_mbx->s_wrPtr)*(pt_mbx->s_queueSize + \
            MBX_HDR_FIELD_BYTES);
            pt_mbx->pc_queue[s_index]     = pc_size[0];
            pt_mbx->pc_queue[s_index + 1] = pc_size[1];

            /*---------------------------------------------------------------*
             * Store the data in the remaining bytes...
             *---------------------------------------------------------------*/
            memcpy( (void *)&pt_mbx->pc_queue[ s_index + MBX_HDR_FIELD_BYTES], 
                    (void *)pc_buf,  
                    s_size);

            /*---------------------------------------------------------------*
             * Increment the write pointer
             *---------------------------------------------------------------*/
            pt_mbx->s_wrPtr++;
            if( pt_mbx->s_wrPtr == pt_mbx->s_queueDepth)
               pt_mbx->s_wrPtr = 0;

            /*---------------------------------------------------------------*
             * Keep track of the number of open slots on the queue.
             *---------------------------------------------------------------*/
            pt_mbx->s_numMessages++;

            if( pt_mbx->b_multRdWr == true)
               t_err = arb_signal( pt_mbx->t_mutex);
         }

         /*------------------------------------------------------------------*
          * Tell the consumer to read the new message
          *------------------------------------------------------------------*/
         arb_signal( pt_mbx->t_semFillCount); /*Increment the fill count*/

      }/*End if( t_err == ARB_PASSED)*/
      else if( t_err == ARB_SEM_DEC_ERROR) /*Non blocking mode*/
      {
         /*------------------------------------------------------------------*
          * Tell the producer that the buffer is full.
          *------------------------------------------------------------------*/
         return (int16_t)ARB_MBX_FULL;
      }
   }

   return (int16_t)s_size; /*Return the number of bytes written*/

}/*End arb_mailboxWrite*/

t_MAILBOXHNDL arb_mailboxCreate( t_mailboxConfig t_config)
{
   t_mailbox *pt_newMbx;
   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Is this a bad configuration?
    *------------------------------------------------------------------------*/
   if( (t_config.b_wrtFromInt == true) && (t_config.t_writeMode == BLOCKING))
   {
      return (t_MAILBOXHNDL)ARB_INVALID_ARG;
   }/*End if( (t_config.b_wrtFromInt == true) && (t_config.t_writeMode == 
   BLOCKING))*/

   /*------------------------------------------------------------------------*
    * Create a new link handle that will store information about this
    * particular mailbox
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof(t_mailbox) + (t_config.s_queueSize + \
   MBX_HDR_FIELD_BYTES)*t_config.s_queueDepth);

   if( t_linkHndl < 0)
   {
      return (t_MAILBOXHNDL)ARB_OUT_OF_HEAP;
   }/*End if( t_linkHndl < 0)*/

   /*------------------------------------------------------------------------*
    * Add the mailbox 'link' onto the list containing all the open mailbox's 
    * on the system.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_activeMbx,
                           t_linkHndl,
                           true);

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where information
    * about the mailbox is stored.
    *------------------------------------------------------------------------*/
   pt_newMbx = (t_mailbox *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);

   pt_newMbx->t_semFillCount = arb_semaphoreCreate( COUNTING);

   if( pt_newMbx->t_semFillCount < 0)
   {
      return (t_MAILBOXHNDL)pt_newMbx->t_semFillCount;

   }/*End if( pt_newMbx->t_semFillCount < 0)*/

   arb_semaphoreInit( pt_newMbx->t_semFillCount,
                      0); /*No data in mailbox*/

   pt_newMbx->t_semEmptyCount = arb_semaphoreCreate( COUNTING);

   if( pt_newMbx->t_semEmptyCount < 0)
   {
      return (t_MAILBOXHNDL)pt_newMbx->t_semEmptyCount;

   }/*End if( pt_newMbx->t_semEmptyCount < 0)*/

   arb_semaphoreInit( pt_newMbx->t_semEmptyCount,
                      t_config.s_queueDepth); /*All slots open*/

   pt_newMbx->t_mutex = arb_semaphoreCreate( MUTEX);

   if( pt_newMbx->t_mutex < 0)
   {
      return (t_MAILBOXHNDL)pt_newMbx->t_mutex;

   }/*End if( pt_newMbx->t_mutex < 0)*/

   pt_newMbx->t_writeMode   = t_config.t_writeMode;
   pt_newMbx->t_readMode    = t_config.t_readMode;
   pt_newMbx->s_queueSize   = t_config.s_queueSize;
   pt_newMbx->s_queueDepth  = t_config.s_queueDepth;
   pt_newMbx->s_numMessages = 0;
   pt_newMbx->s_wrPtr       = 0;
   pt_newMbx->s_rdPtr       = 0;
   pt_newMbx->b_wrtFromInt  = t_config.b_wrtFromInt;
   pt_newMbx->b_multRdWr    = t_config.b_multRdWr;

   /*------------------------------------------------------------------------*
    * Initialize the queue ptr to the location of the queue which starts
    * with the adjacent address location.
    *------------------------------------------------------------------------*/
   pt_newMbx->pc_queue = (int8_t *)((&pt_newMbx->pc_queue) + 1);

   /*------------------------------------------------------------------------*
    * Return a handle (or address ) to this semaphore.
    *------------------------------------------------------------------------*/
   return (t_MAILBOXHNDL)t_linkHndl;

}/*End arb_mailboxCreate*/

t_error arb_mailboxDestroy( t_MAILBOXHNDL t_mbxHandle)
{
   t_mailbox *pt_mbx;
   t_error t_err;

   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_mbxHandle, gt_activeMbx) ==
   false)
   {
      return ARB_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where information
    * about the mailbox is stored.
    *------------------------------------------------------------------------*/
   pt_mbx = (t_mailbox *)UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_mbxHandle);

   t_err = arb_semaphoreDestroy( pt_mbx->t_semFillCount);
   if( t_err < 0)
      return t_err;

   t_err = arb_semaphoreDestroy( pt_mbx->t_semEmptyCount);
   if( t_err < 0)
      return t_err;

   t_err = arb_semaphoreDestroy( pt_mbx->t_mutex);
   if( t_err < 0)
      return t_err;

   /*------------------------------------------------------------------------*
    * Destroy the mailbox
    *------------------------------------------------------------------------*/
   t_err = utl_destroyLink( gt_activeMbx, (t_LINKHNDL)t_mbxHandle);

   if( t_err < 0)
      return t_err;

   return ARB_PASSED;

}/*End arb_mailboxDestroy*/

int16_t arb_mailboxGetQueueMaxSize( t_MAILBOXHNDL t_mbxHandle)
{
   t_mailbox *pt_mbx;

   /*------------------------------------------------------------------------*
    * Is this a valid mailbox handle?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_mbxHandle, gt_activeMbx) == 
   false)
   {
      return (int16_t)ARB_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where information
    * about the mailbox is stored.
    *------------------------------------------------------------------------*/
   pt_mbx = (t_mailbox *)UTL_GET_LINK_ELEMENT_PTR( t_mbxHandle);

   return pt_mbx->s_queueSize;

}/*End arb_mailboxGetQueueMaxSize*/

int16_t arb_mailboxGetNumMessages( t_MAILBOXHNDL t_mbxHandle)
{
   t_mailbox *pt_mbx;

   /*------------------------------------------------------------------------*
    * Is this a valid mailbox handle?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_mbxHandle, gt_activeMbx) == 
   false)
   {
      return (int16_t)ARB_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where information
    * about the mailbox is stored.
    *------------------------------------------------------------------------*/
   pt_mbx = (t_mailbox *)UTL_GET_LINK_ELEMENT_PTR( t_mbxHandle);

   return pt_mbx->s_numMessages;

}/*End arb_mailboxGetNumMessages*/

int16_t arb_mailboxGetQueueMaxDepth( t_MAILBOXHNDL t_mbxHandle)
{
   t_mailbox *pt_mbx;

   /*------------------------------------------------------------------------*
    * Is this a valid mailbox handle?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_mbxHandle, gt_activeMbx) == 
   false)
   {
      return (int16_t)ARB_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where information
    * about the mailbox is stored.
    *------------------------------------------------------------------------*/
   pt_mbx = (t_mailbox *)UTL_GET_LINK_ELEMENT_PTR( t_mbxHandle);

   return pt_mbx->s_queueDepth;

}/*End arb_mailboxGetQueueMaxDepth*/
