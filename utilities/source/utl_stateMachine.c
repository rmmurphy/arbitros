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
 * File Name   : utl_stateMachine.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides an abstraction layer for the
 *               implementation of a state-machine.
 *
 * Last Update : Oct 27, 2012
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "utl_linkedList.h"
#include "hal_pmic.h"
#include "utl_stateMachine.h"
#include "arb_semaphore.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef void (*funPtr_t)(int32_t, uint8_t);

typedef struct
{
   /*------------------------------------------------------------------------*
    * The next state to execute.
    *------------------------------------------------------------------------*/
   uint8_t c_nextState;
   /*------------------------------------------------------------------------*
    * The current state executing.
    *------------------------------------------------------------------------*/
   uint8_t c_currState;
   /*------------------------------------------------------------------------*
    * The last state that executed.
    *------------------------------------------------------------------------*/
   uint8_t c_prevState;
   /*------------------------------------------------------------------------*
    * The number of states defined by this state-machine object.
    *------------------------------------------------------------------------*/
   uint8_t c_numStates;
   /*------------------------------------------------------------------------*
    * Data passed into a particular state.
    *------------------------------------------------------------------------*/
   int32_t i_stateData;
   /*------------------------------------------------------------------------*
    * Array of function ptrs, one for each of the 'c_numStates' possible
    * states.
    *------------------------------------------------------------------------*/
   funPtr_t *pv_funcMap;

   /*------------------------------------------------------------------------*
    * Matrix that defines the possible transitions from one state to another.
    * Where a value of 1 means that a transition from 'c_currState' to
    * 'c_nextState' is possible. Where *pc_transMap pts to the first location
    * after the space that has been allocated
    *------------------------------------------------------------------------*/
   uint8_t *pc_transMap;

   /*------------------------------------------------------------------------*
    * Resources that can be shared amongst multiple users (either a global
    * buffer or IO device) need to be protected against race conditions. We
    * use this semaphore for just that purpose.
    *------------------------------------------------------------------------*/
   //t_SEMHANDLE t_mutex;

}t_stMnObject;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_LINKHNDL utl_createStMnObject( uint8_t c_numStates);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * List of currently active state-machine objects
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_stMnHndlList);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static t_LINKHNDL utl_createStMnObject( uint8_t c_numStates)
{

   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;
   int16_t s_sizeFuncMap = sizeof( funPtr_t)*(int16_t)c_numStates;
   int16_t s_sizeTransMap = (int16_t)c_numStates*(int16_t)c_numStates;

   /*------------------------------------------------------------------------*
    * Allocated memory for the link (and element) that contains information
    * specific to this particular state-machine object
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof( t_stMnObject) + s_sizeFuncMap +
   s_sizeTransMap);

   if( t_linkHndl < 0)
   {
      return (t_LINKHNDL)STMN_OUT_OF_HEAP;
   }

   /*------------------------------------------------------------------------*
    * Add the state-machine object link onto the list open state-machine
    * objects.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_stMnHndlList,
                           t_linkHndl,
                           true);

   return t_linkHndl;

}/*End utl_createStMnObject*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
t_stmnError utl_stMnChangeState( t_STMNHNDL t_handle,
                                 int8_t c_newState,
                                 int32_t i_stateData)
{
   t_stMnObject *pt_stMnObject;
   t_stmnError t_err = STMN_PASSED;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a state-machine object?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_stMnHndlList) ==
   false)
   {
      t_err = STMN_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      uint8_t c_row;

      //arb_wait( pt_stMnObject->t_mutex, 0);

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the state-
       * machine object information is being stored.
       *---------------------------------------------------------------------*/
      pt_stMnObject = (t_stMnObject *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      c_row = pt_stMnObject->c_currState;
      if( pt_stMnObject->pc_transMap[((int16_t)c_row*(int16_t)pt_stMnObject->
      c_numStates) + (int16_t)c_newState])
      {
         pt_stMnObject->c_nextState = c_newState;
         pt_stMnObject->i_stateData = i_stateData;

      }
      else
         t_err = STMN_INVALID_TRANS;

      //arb_signal( pt_stMnObject->t_mutex);

   }

   return t_err;

}/*End utl_stMnChangeState*/

t_stmnError utl_stMnEngine( t_STMNHNDL t_handle)
{
   t_stMnObject *pt_stMnObject;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a state-machine object?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_stMnHndlList) ==
   false)
   {
      return STMN_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      //arb_wait( pt_stMnObject->t_mutex, 0);

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the state-
       * machine object information is being stored.
       *---------------------------------------------------------------------*/
      pt_stMnObject = (t_stMnObject *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Update to reflect the current state...
       *---------------------------------------------------------------------*/
      pt_stMnObject->c_prevState = pt_stMnObject->c_currState;
      pt_stMnObject->c_currState = pt_stMnObject->c_nextState;

      /*---------------------------------------------------------------------*
       * Call the next state to run...
       *---------------------------------------------------------------------*/
      pt_stMnObject->pv_funcMap[pt_stMnObject->c_nextState]( pt_stMnObject->\
      i_stateData, pt_stMnObject->c_prevState);

      //arb_signal( pt_stMnObject->t_mutex);
   }

   return STMN_PASSED;

}/*End utl_stMnEngine*/

t_stmnError utl_stMnPopFunMap( t_STMNHNDL t_handle,
                               void (*pf_funPtr)( int32_t, uint8_t),
                               uint8_t c_state)
{
   t_stMnObject *pt_stMnObject;

   /*------------------------------------------------------------------------*
    * Since we are about to act on global variables, protect this region
    * of code against higher priority threads interrupting us while we are
    * trying to register.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a state-machine object?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_stMnHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return STMN_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the state-
       * machine object information is being stored.
       *---------------------------------------------------------------------*/
      pt_stMnObject = (t_stMnObject *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( c_state >= pt_stMnObject->c_numStates)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return STMN_INVALID_STATE;
      }

      /*---------------------------------------------------------------------*
       * Store the call-back function for this particular state.
       *---------------------------------------------------------------------*/
      pt_stMnObject->pv_funcMap[c_state] = pf_funPtr;

   }

   HAL_END_CRITICAL();//Enable interrupts
   return STMN_PASSED;

}/*End utl_stMnPopFunMap*/

t_stmnError utl_stMnPopTransMap( t_STMNHNDL t_handle,
                                 uint8_t *pc_transMap)
{
   t_stMnObject *pt_stMnObject;
   uint8_t c_row;
   uint8_t c_col;

   /*------------------------------------------------------------------------*
    * Since we are about to act on global variables, protect this region
    * of code against higher priority threads interrupting us while we are
    * trying to register.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a state-machine object?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_stMnHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return STMN_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the state-
       * machine object information is being stored.
       *---------------------------------------------------------------------*/
      pt_stMnObject = (t_stMnObject *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      for( c_row = 0; c_row < pt_stMnObject->c_numStates; c_row++)
      {
         for( c_col = 0; c_col < pt_stMnObject->c_numStates; c_col++)
         {
            pt_stMnObject->pc_transMap[((int16_t)c_row*(int16_t)pt_stMnObject->
            c_numStates) + (int16_t)c_col] = pc_transMap[((int16_t)c_row*
            (int16_t)pt_stMnObject->c_numStates) + (int16_t)c_col];
         }
      }

   }

   HAL_END_CRITICAL();//Enable interrupts
   return STMN_PASSED;

}/*End utl_stMnPopTransMap*/

t_STMNHNDL utl_requestStMnObject( uint8_t c_numStates,
                                  uint8_t c_startingState)
{
   t_stMnObject *pt_stMnObject;
   t_LINKHNDL t_linkHndl;

   /*------------------------------------------------------------------------*
    * Since we are about to act on global variables, protect this region
    * of code against higher priority threads interrupting us while we are
    * trying to register.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   if( c_startingState >= c_numStates)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_STMNHNDL)STMN_INVALID_STATE;
   }

   t_linkHndl = utl_createStMnObject( c_numStates);
   if( t_linkHndl < 0)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_STMNHNDL)STMN_OUT_OF_HEAP;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the state-
    * machine object information is being stored.
    *------------------------------------------------------------------------*/
   pt_stMnObject = (t_stMnObject *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);

   /*------------------------------------------------------------------------*
    * Clear the state-machine object.
    *------------------------------------------------------------------------*/
   memset( (void *)pt_stMnObject, 0, sizeof( t_stMnObject));

//    pt_stMnObject->t_mutex = arb_semaphoreCreate( MUTEX);
// 
//    if( pt_stMnObject->t_mutex < 0)
//    {
//       utl_releaseStMnObject( t_linkHndl);
//       HAL_END_CRITICAL();//Enable interrupts
//       return (t_STMNHNDL)STMN_OUT_OF_HEAP;
//    }

   /*------------------------------------------------------------------------*
    * Point to the location of the function ptr buffer which starts
    * immediately after the transition map ptr.
    *------------------------------------------------------------------------*/
   pt_stMnObject->pv_funcMap = (funPtr_t *)(&pt_stMnObject->pv_funcMap + 2);

   /*------------------------------------------------------------------------*
    * Point to the location of the translation buffer which starts
    * immediately after the function ptr buffer.
    *------------------------------------------------------------------------*/
   pt_stMnObject->pc_transMap = (uint8_t *)(&pt_stMnObject->pv_funcMap + 2
   + (int16_t)c_numStates);

   pt_stMnObject->c_numStates = c_numStates;
   pt_stMnObject->c_nextState = c_startingState;
   pt_stMnObject->c_currState = c_numStates;
   /*------------------------------------------------------------------------*
    * We are in a valid state as of yet, so initialize to an unknown state...
    *------------------------------------------------------------------------*/
   pt_stMnObject->c_prevState = c_numStates;

   HAL_END_CRITICAL();//Enable interrupts

   return (t_STMNHNDL)t_linkHndl;

}/*End utl_requestStMnObject*/

t_stmnError utl_releaseStMnObject( t_STMNHNDL t_handle)
{

   if( utl_destroyLink( gt_stMnHndlList, (t_LINKHNDL)t_handle) < 0)
      return STMN_INVALID_HANDLE;

   return STMN_PASSED;

}/*End utl_releaseStMnObject*/

