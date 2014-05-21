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
 * File Name   : utl_stateMachine.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides an abstraction layer for the
 *               implementation of a state-machine.
 *
 * Last Update : Oct 27, 2012
 *---------------------------------------------------------------------------*/
#ifndef utl_stateMachine_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define utl_stateMachine_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      STMN_INVALID_STATE  = -4, /*State out of range*/
      STMN_INVALID_TRANS  = -3, /*An invalid transition was requested*/
      STMN_INVALID_HANDLE = -2, /*Handle doesn't map to an open state-machine*/
      STMN_OUT_OF_HEAP    = -1, /*No more memory.*/
      STMN_PASSED         = 0   /*Configuration good.*/

   }t_stmnError;

   typedef volatile int16_t t_STMNHNDL; /*Handle to a particular
                                          state-machine*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * This function transitions the state-machine object defined by
    * 't_handle' to a new state given by 'c_newState'.
    *------------------------------------------------------------------------*/
   t_stmnError utl_stMnChangeState( t_STMNHNDL t_handle,
                                    int8_t c_newState,
                                    int32_t i_stateData);

   /*------------------------------------------------------------------------*
    * This function vectors to the specific states as defined by 't_handle'.
    *------------------------------------------------------------------------*/
   t_stmnError utl_stMnEngine( t_STMNHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Creates a state-machine object with 'c_numStates' number of states.
    *------------------------------------------------------------------------*/
   t_STMNHNDL utl_requestStMnObject( uint8_t c_numStates,
                                     uint8_t c_startingState);

   /*------------------------------------------------------------------------*
    * Releases a state-machine object from memory.
    *------------------------------------------------------------------------*/
   t_stmnError utl_releaseStMnObject( t_STMNHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Populates the state-machine function map with the particular call-back
    * function requested for a given state.
    *------------------------------------------------------------------------*/
   t_stmnError utl_stMnPopFunMap( t_STMNHNDL t_handle,
                                  void (*pf_funPtr)( int32_t, uint8_t),
                                  uint8_t c_state);

   /*------------------------------------------------------------------------*
    * Populates the state-machine transition map which defines the rules for
    * moving between states.
    *------------------------------------------------------------------------*/
   t_stmnError utl_stMnPopTransMap( t_STMNHNDL t_handle,
                                    uint8_t *pc_transMap);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef utl_stateMachine_h*/
