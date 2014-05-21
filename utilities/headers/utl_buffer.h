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
 * File Name   : utl_buffer.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for the creation and
 *               control over circular buffers.
 *
 * Last Update : Dec 3, 2011
 *---------------------------------------------------------------------------*/
#ifndef utl_buffer_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define utl_buffer_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {

      BUFFER_INVALID_HNDL = -3, /*Handle not found*/
      BUFFER_NULL_PTR     = -2, /*Pointer is not mapped to a valid address.*/
      BUFFER_OUT_OF_HEAP  = -1, /*No more memory.*/
      BUFFER_PASSED       = 0   /*Configuration good.*/

   }t_bufferError;

   typedef volatile int16_t t_BUFFHANDLE; /*Handle to an 8-bit buffer*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_BUFFHANDLE utl_createBuffer( int16_t s_sizeBytes);

   t_bufferError utl_destroyBuffer( t_BUFFHANDLE t_handle);

   bool utl_isBufferFull( t_BUFFHANDLE t_handle);

   bool utl_isBufferEmpty( t_BUFFHANDLE t_handle);

   bool utl_hasBufferOverflowed( t_BUFFHANDLE t_handle);

   bool utl_hasBufferUnderflowed( t_BUFFHANDLE t_handle);

   int16_t utl_getBufferFullLevel( t_BUFFHANDLE t_handle);

   int16_t utl_getBufferEmptyLevel( t_BUFFHANDLE t_handle);

   int16_t utl_getBufferSize( t_BUFFHANDLE t_handle);

   int8_t *utl_getBufferPtr( t_BUFFHANDLE t_handle);

   void utl_writeByte( t_BUFFHANDLE t_handle,
                       int8_t c_byte);

   void utl_writeBlock( t_BUFFHANDLE t_handle,
                        int8_t *pc_buff,
                        uint16_t s_size);

   int8_t utl_readByte( t_BUFFHANDLE t_handle);

   void utl_readBlock( t_BUFFHANDLE t_handle,
                       int8_t *pc_buff,
                       uint16_t s_size);

   void utl_incrBufWrtPtr( t_BUFFHANDLE t_handle,
                           uint16_t s_size);

   void utl_incrBufRdPtr( t_BUFFHANDLE t_handle,
                          uint16_t s_size);

   int16_t utl_getBufRdPtr( t_BUFFHANDLE t_handle);

   int16_t utl_getBufWrtPtr( t_BUFFHANDLE t_handle);

   /*------------------------------------------------------------------------*
    * This function erases a byte at the tail of the buffer.
    *------------------------------------------------------------------------*/
   void utl_buffEraseTailByte( t_BUFFHANDLE t_handle);

   void ult_resetBuffer( t_BUFFHANDLE t_handle);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef utl_buffer_h*/
