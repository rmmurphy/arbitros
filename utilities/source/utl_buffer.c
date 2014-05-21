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
 * File Name   : utl_buffer.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for the creation and 
 *               control over circular buffers.
 *
 * Last Update : Dec 3, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "utl_linkedList.h"
#include "utl_buffer.h"
#include "hal_pmic.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct
{

   /*------------------------------------------------------------------------*
    * Location where the next element will be added to the buffer.
    *------------------------------------------------------------------------*/
   int16_t s_wrIndex;

   /*------------------------------------------------------------------------*
    * Location where the next element will be read from the buffer.
    *------------------------------------------------------------------------*/
   int16_t s_rdIndex;

   /*------------------------------------------------------------------------*
    * The number of elements in the buffer. When s_fillCount < 0 the
    * consuming process has underflowed the buffer or read too many elements.
    * When s_fillCount > s_sizeWords the producing process has overflowed
    * the buffer or wrote too many words.
    *------------------------------------------------------------------------*/
   int16_t s_fillCount;

   /*------------------------------------------------------------------------*
    * The size of the buffer in 8-bit words
    *------------------------------------------------------------------------*/
   uint16_t s_sizeBytes;

   /*------------------------------------------------------------------------*
    * Pointer to the circular buffer
    *------------------------------------------------------------------------*/
   int8_t *pc_buffer;

}t_bufferHandle;

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * This variable keeps track of all the open circular buffers on the system.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_bufferList);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
int16_t utl_getBufferFullLevel( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle =
   (t_bufferHandle *)UTL_GET_LINK_ELEMENT_PTR(t_handle);

   return pt_handle->s_fillCount; /*Return the number of bytes in the buffer*/

}/*End utl_getBufferFullLevel*/

int16_t utl_getBufferEmptyLevel( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);

   /*------------------------------------------------------------------------*
    * Return the number of bytes of space left in the buffer.
    *------------------------------------------------------------------------*/
   return ((int16_t)pt_handle->s_sizeBytes - pt_handle->s_fillCount);

}/*End utl_getBufferEmptyLevel*/

int16_t utl_getBufferSize( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);

   return ((int16_t)pt_handle->s_sizeBytes);

}/*End utl_getBufferSize*/

bool utl_isBufferFull( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);

   return (bool)(pt_handle->s_fillCount == pt_handle->s_sizeBytes);
}/*End utl_isBufferFull*/

bool utl_isBufferEmpty( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);

   return (bool)(pt_handle->s_fillCount == 0);
}/*End utl_isBufferEmpty*/

bool utl_hasBufferOverflowed( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);

   return (bool)(pt_handle->s_fillCount > pt_handle->s_sizeBytes);
}/*End utl_hasBufferOverflowed*/

bool utl_hasBufferUnderflowed( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);
   return (bool)(pt_handle->s_fillCount < 0);
}/*End hasBufferUnderflowed*/

int8_t *utl_getBufferPtr( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);
   return pt_handle->pc_buffer;
}/*End utl_getBufferPtr*/

void utl_writeByte( t_BUFFHANDLE t_handle,
                    int8_t c_byte)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);

   pt_handle->pc_buffer[pt_handle->s_wrIndex] = c_byte;
   pt_handle->s_wrIndex++;
   if( pt_handle->s_wrIndex == pt_handle->s_sizeBytes)
      pt_handle->s_wrIndex = 0;

   pt_handle->s_fillCount++;

}/*End utl_writeByte*/

void utl_incrBufWrtPtr( t_BUFFHANDLE t_handle,
                        uint16_t s_size)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);
   int16_t *ps_wrIndex       = &pt_handle->s_wrIndex;

   *ps_wrIndex += s_size;
   if( *ps_wrIndex >= pt_handle->s_sizeBytes)
   {
      *ps_wrIndex -= pt_handle->s_sizeBytes;
   }

   pt_handle->s_fillCount += (int16_t)s_size;

}/*End utl_incrBufWrtPtr*/

void utl_incrBufRdPtr( t_BUFFHANDLE t_handle,
                       uint16_t s_size)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);
   int16_t *ps_rdIndex       = &pt_handle->s_rdIndex;

   *ps_rdIndex += s_size;
   if( *ps_rdIndex >= pt_handle->s_sizeBytes)
   {
      *ps_rdIndex -= pt_handle->s_sizeBytes;
   }

   pt_handle->s_fillCount -= (int16_t)s_size;

}/*End utl_incrBufRdPtr*/

int16_t utl_getBufRdPtr( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);

   return pt_handle->s_rdIndex;
}/*End utl_getBufRdPtr*/

int16_t utl_getBufWrtPtr( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);

   return pt_handle->s_wrIndex;
}/*End utl_getBufWrtPtr*/

void utl_writeBlock( t_BUFFHANDLE t_handle,
                     int8_t *pc_buff,
                     uint16_t s_size)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);
   int8_t *pc_buffer         = pt_handle->pc_buffer;
   int16_t *ps_wrIndex       = &pt_handle->s_wrIndex;
   uint16_t s_room           = 0;

   s_room = pt_handle->s_sizeBytes - pt_handle->s_wrIndex;
   if( s_room < s_size)
   {
      /*---------------------------------------------------------------------*
       * Copy from current location to the end of the buffer.
       *---------------------------------------------------------------------*/
      memcpy( (void *)&pc_buffer[*ps_wrIndex], (void *)pc_buff, s_room);

      /*---------------------------------------------------------------------*
       * Copy the rest of the data starting at the beginning of the buffer.
       *---------------------------------------------------------------------*/
      memcpy( (void *)pc_buffer, (void *)&pc_buff[s_room], s_size - s_room);
   }
   else
   {
      memcpy( (void *)&pc_buffer[*ps_wrIndex], (void *)pc_buff, s_size);
   }

   *ps_wrIndex += s_size;
   if( *ps_wrIndex >= pt_handle->s_sizeBytes)
   {
      *ps_wrIndex -= pt_handle->s_sizeBytes;
   }

   pt_handle->s_fillCount += (int16_t)s_size;

}/*End utl_writeBlock*/

int8_t utl_readByte( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);
   int8_t c_byte = 0;

   c_byte = pt_handle->pc_buffer[pt_handle->s_rdIndex];
   pt_handle->s_rdIndex++;
   if( pt_handle->s_rdIndex == pt_handle->s_sizeBytes)
      pt_handle->s_rdIndex = 0;

   pt_handle->s_fillCount--;

   return c_byte;

}/*End utl_readByte*/

void utl_buffEraseTailByte( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);

   pt_handle->s_wrIndex--;
   if( pt_handle->s_wrIndex < 0)
      pt_handle->s_wrIndex = pt_handle->s_sizeBytes - 1;

   pt_handle->pc_buffer[pt_handle->s_wrIndex] = 0;

   pt_handle->s_fillCount--;

}/*End utl_buffEraseTailByte*/

void utl_readBlock( t_BUFFHANDLE t_handle,
                    int8_t *pc_buff,
                    uint16_t s_size)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);
   int8_t *pc_buffer         = pt_handle->pc_buffer;
   int16_t *ps_rdIndex       = &pt_handle->s_rdIndex;
   uint16_t s_room           = 0;

   s_room = pt_handle->s_sizeBytes - pt_handle->s_rdIndex;
   if( s_room < s_size)
   {
      /*---------------------------------------------------------------------*
       * Copy from current location to the end of the buffer.
       *---------------------------------------------------------------------*/
      memcpy( (void *)pc_buff, (void *)&pc_buffer[*ps_rdIndex], s_room);

      /*---------------------------------------------------------------------*
       * Copy the rest of the data starting at the beginning of the buffer.
       *---------------------------------------------------------------------*/
      memcpy( (void *)&pc_buff[s_room], (void *)pc_buffer, s_size - s_room);
   }
   else
   {
      memcpy( (void *)pc_buff, (void *)&pc_buffer[*ps_rdIndex], s_size);
   }

   *ps_rdIndex += s_size;
   if( *ps_rdIndex >= pt_handle->s_sizeBytes)
   {
      *ps_rdIndex -= pt_handle->s_sizeBytes;
   }

   pt_handle->s_fillCount -= (int16_t)s_size;

}/*End utl_readBlock*/

void ult_resetBuffer( t_BUFFHANDLE t_handle)
{
   t_bufferHandle *pt_handle = (t_bufferHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);

   pt_handle->s_rdIndex   = 0;
   pt_handle->s_wrIndex   = 0;
   pt_handle->s_fillCount = 0;

}/*End ult_resetBuffer*/

t_BUFFHANDLE utl_createBuffer( int16_t s_sizeBytes)
{
   t_bufferHandle *pt_newBuff;
   t_LINKHNDL t_newLinkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Since we are about to act on global variables, protect this region
    * of code against higher priority threads interrupting us while we are
    * trying to register.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   /*------------------------------------------------------------------------*
    * Create a new link handle with room at the end for the buffer..
    *------------------------------------------------------------------------*/
   t_newLinkHndl = utl_createLink( sizeof(t_bufferHandle) + s_sizeBytes);

   if( t_newLinkHndl < 0)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_BUFFHANDLE)BUFFER_OUT_OF_HEAP;
   }/*End if( t_newLinkHndl < 0)*/

   /*------------------------------------------------------------------------*
    * Initialize the read and write index's.
    *------------------------------------------------------------------------*/
   pt_newBuff = (t_bufferHandle *)UTL_GET_LINK_ELEMENT_PTR(t_newLinkHndl);

   pt_newBuff->s_wrIndex   = 0;
   pt_newBuff->s_rdIndex   = 0;
   pt_newBuff->s_fillCount = 0;

   /*------------------------------------------------------------------------*
    * Initialize the buffer ptr to the location of the buffer which starts
    * at the next address location as the address of the buffer pointer.
    *------------------------------------------------------------------------*/
   pt_newBuff->pc_buffer = (int8_t *)(&pt_newBuff->pc_buffer + 1);

   /*------------------------------------------------------------------------*
    * Initialize the buffer to all NULL's.
    *------------------------------------------------------------------------*/
   memset( (void *)pt_newBuff->pc_buffer, '\0', s_sizeBytes);

   /*------------------------------------------------------------------------*
    * The size of the buffer in 8-bit words.
    *------------------------------------------------------------------------*/
   pt_newBuff->s_sizeBytes = s_sizeBytes;

   t_err = utl_insertLink( gt_bufferList,
                           t_newLinkHndl,
                           true);

   HAL_END_CRITICAL();//Enable interrupts

   return (t_BUFFHANDLE)t_newLinkHndl;

}/*End utl_createBuffer*/

t_bufferError utl_destroyBuffer( t_BUFFHANDLE t_handle)
{
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Since we are about to act on global variables, protect this region
    * of code against higher priority threads interrupting us while we are
    * trying to register.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   t_err = utl_destroyLink( gt_bufferList,
                            (t_LINKHNDL)t_handle);

   if( t_err < 0)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return BUFFER_INVALID_HNDL;
   }

   HAL_END_CRITICAL();//Enable interrupts
   return BUFFER_PASSED;

}/*End utl_destroyBuffer*/
