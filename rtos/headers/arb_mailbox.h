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
 *               which is used for inter-thread communication between a single
 *               writer and single reader. Both the writing and reading
 *               process can be individually configured to block or not block
 *               depending on the the status of data on the queue.
 *
 * References  : 1) http://en.wikipedia.org/wiki/Producer-consumer_problem
 *
 * Last Update : Jan, 5, 2013
 *---------------------------------------------------------------------------*/
#ifndef arb_mailbox_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define arb_mailbox_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "arb_error.h"
   #include "arb_thread.h"
   #include "arb_semaphore.h"
   #include "utl_linkedList.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef struct
   {
      /*---------------------------------------------------------------------*
       * The max message size in bytes
       *---------------------------------------------------------------------*/
      uint16_t s_queueSize;
      /*---------------------------------------------------------------------*
       * The max number of messages in the queue
       *---------------------------------------------------------------------*/
      uint16_t s_queueDepth;
      /*---------------------------------------------------------------------*
       * Determines whether or not the writing process blocks when the queue
       * is full.
       *---------------------------------------------------------------------*/
      t_semMode t_writeMode;
      /*---------------------------------------------------------------------*
       * Determines whether or not the reading process blocks with the queue
       * is empty.
       *---------------------------------------------------------------------*/
      t_semMode t_readMode;
      /*---------------------------------------------------------------------*
       * If true, 'arb_mbxWrite' can be made via an interrupt (in which case
       * t_writeMode has to be set to NONBLOCKING). During this mode multiple
       * readers and writers is always assumed.
       *---------------------------------------------------------------------*/
      bool b_wrtFromInt;
      /*---------------------------------------------------------------------*
       * If true, there can be multiple readers and writers requiring
       * locks during the update of shared memory variables. This will
       * increase the processing overhead of the primary mailbox calls. If
       * b_wrtFromInt is true, this variable is ignored.
       *---------------------------------------------------------------------*/
      bool b_multRdWr;

   }t_mailboxConfig; /*Configuration for a particular mailbox*/

   typedef volatile int16_t t_MAILBOXHNDL; /*Mailbox handle type*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_MAILBOXHNDL arb_mailboxCreate( t_mailboxConfig t_config);

   t_error arb_mailboxDestroy( t_MAILBOXHNDL t_mbxHandle);

   int16_t arb_mailboxWrite( t_MAILBOXHNDL t_mbxHandle,
                             int8_t *pc_buf,
                             uint16_t s_size)  __attribute__ ((noinline));

   int16_t arb_mailboxRead( t_MAILBOXHNDL t_mbxHandle,
                            int8_t *pc_buf,
                            uint16_t s_size) __attribute__ ((noinline));

   int16_t arb_mailboxGetQueueMaxSize( t_MAILBOXHNDL t_mbxHandle);

   int16_t arb_mailboxGetNumMessages( t_MAILBOXHNDL t_mbxHandle);

   int16_t arb_mailboxGetQueueMaxDepth( t_MAILBOXHNDL t_mbxHandle);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef arb_mailbox_h*/
