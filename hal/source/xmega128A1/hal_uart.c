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
 * File Name   : hal_uart.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for controlling
 *               the UART module.
 *
 * Last Update : Oct 26, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "hal_uart.h"
#include "hal_clocks.h"
#include "utl_linkedlist.h"
#include "hal_gpio.h"
#include "hal_pmic.h"
#include "hal_dma.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define BSEL_MAX_RANGE  ((1 << 12) - 1)
#define BSCALE_MAX_RAGE (7)

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct
{
   /*------------------------------------------------------------------------*
    * A unique number referring to one of the 8 possible uarts
    *------------------------------------------------------------------------*/
   t_uartChanId t_id;

   /*------------------------------------------------------------------------*
    * The current buad rate for the UART connected to this handle
    *------------------------------------------------------------------------*/
   int32_t i_baudRate;

   /*------------------------------------------------------------------------*
    * Error between the desired baud rate and the one we were able to
    * actually configure the UART to use.
    *------------------------------------------------------------------------*/
   float f_percentBaudError;

   /*------------------------------------------------------------------------*
    * Pointer to the UART channel connected to this particular handle
    *------------------------------------------------------------------------*/
   USART_t *pt_uart;

   /*------------------------------------------------------------------------*
    * If true, there is a transaction in progress on the tx channel.
    *------------------------------------------------------------------------*/
	bool b_txBusLocked;

   /*------------------------------------------------------------------------*
    * If true, there is a transaction in progress on the rx channel.
    *------------------------------------------------------------------------*/
	bool b_rxBusLocked;

   /*------------------------------------------------------------------------*
    * Pointer to the spi tx buffer for the user connected to this channel.
    *------------------------------------------------------------------------*/
   int8_t *pc_txData;

   /*------------------------------------------------------------------------*
    * Pointer to the spi rx buffer for the user connected to this channel.
    *------------------------------------------------------------------------*/
   int8_t *pc_rxData;

   /*------------------------------------------------------------------------*
    * Length of the tx buffer.
    *------------------------------------------------------------------------*/
   uint16_t s_txBufLength;

   /*------------------------------------------------------------------------*
    * Length of the rx buffer.
    *------------------------------------------------------------------------*/
   uint16_t s_rxBufLength;

   /*------------------------------------------------------------------------*
    * Current index into the tx buffer.
    *------------------------------------------------------------------------*/
   uint16_t s_txBufIndex;

   /*------------------------------------------------------------------------*
    * Current index into the rx buffer.
    *------------------------------------------------------------------------*/
   uint16_t s_rxBufIndex;

   /*------------------------------------------------------------------------*
    * Address of the dma channel tied to the transmit buffer of this
    * particular UART channel.
    *------------------------------------------------------------------------*/
   t_DMAHNDL t_txDmaHndl;

   /*------------------------------------------------------------------------*
    * Address of the dma channel tied to the receive buffer of this
    * particular UART channel.
    *------------------------------------------------------------------------*/
   t_DMAHNDL t_rxDmaHndl;

   /*------------------------------------------------------------------------*
    * Pointer to the function called upon completion of a receiving data over
    * this particular UART channel.
    *------------------------------------------------------------------------*/
   void (*pf_rxCallBack)( uint16_t s_data);

   /*------------------------------------------------------------------------*
    * Pointer to the function called upon completion of a transmitting a
    * block of data over this particular UART channel.
    *------------------------------------------------------------------------*/
   void (*pf_txCallBack)( uint16_t s_size);

}t_uartChanHndl;

typedef struct
{
   t_uartChanHndl *pt_uart1Chan;
   t_uartChanHndl *pt_uart2Chan;
   t_uartChanHndl *pt_uart3Chan;
   t_uartChanHndl *pt_uart4Chan;
   t_uartChanHndl *pt_uart5Chan;
   t_uartChanHndl *pt_uart6Chan;
   t_uartChanHndl *pt_uart7Chan;
   t_uartChanHndl *pt_uart8Chan;

}t_intChanMap;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_LINKHNDL createUartHandle( void);
static t_uartChanHndl *findUartElement( t_uartChanId t_id);
static void uart1RxDmaInt( void);
static void uart2RxDmaInt( void);
static void uart3RxDmaInt( void);
static void uart4RxDmaInt( void);
static void uart5RxDmaInt( void);
static void uart6RxDmaInt( void);
static void uart7RxDmaInt( void);
static void uart8RxDmaInt( void);
static void uart1TxDmaInt( void);
static void uart2TxDmaInt( void);
static void uart3TxDmaInt( void);
static void uart4TxDmaInt( void);
static void uart5TxDmaInt( void);
static void uart6TxDmaInt( void);
static void uart7TxDmaInt( void);
static void uart8TxDmaInt( void);
static void updateRxBuffer( t_uartChanHndl *pt_handle);
static void updateTxBuffer( t_uartChanHndl *pt_handle);
static void updateRxDmaInt( t_uartChanHndl *pt_handle);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
static t_intChanMap gt_intChanMap = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};

/*---------------------------------------------------------------------------*
 * List of currently active UART modules.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_uartChanHndlList);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void updateRxDmaInt( t_uartChanHndl *pt_handle)
{
   /*------------------------------------------------------------------------*
    * Disable the DMA channel.
    *------------------------------------------------------------------------*/
   hal_dmaDisableChannel( pt_handle->t_rxDmaHndl);

   pt_handle->s_rxBufIndex = hal_getDmaTransferCount( pt_handle->t_rxDmaHndl);

   /*------------------------------------------------------------------------*
    * Execute the call-back function, returning the size of the data.
    *------------------------------------------------------------------------*/
	if( pt_handle->pf_rxCallBack != NULL)
      pt_handle->pf_rxCallBack( pt_handle->s_rxBufIndex);

   /*------------------------------------------------------------------------*
    * Release the RX UART bus.
    *------------------------------------------------------------------------*/
	pt_handle->b_rxBusLocked = false;
   pt_handle->pc_rxData = NULL;
   pt_handle->s_rxBufIndex = 0;
   pt_handle->s_rxBufLength = 0;

   /*------------------------------------------------------------------------*
    * Enable RX interrupts - so that the RX channel can be used for non DMA
    * transfers.
    *------------------------------------------------------------------------*/
   pt_handle->pt_uart->CTRLA |= USART_RXCINTLVL_HI_gc;

}/*End updateRxDmaInt*/

static void updateRxBuffer( t_uartChanHndl *pt_handle)
{
	int8_t *pc_rxBuf = pt_handle->pc_rxData;
	uint16_t *ps_bufIndex = &pt_handle->s_rxBufIndex;

   /*------------------------------------------------------------------------*
    * Is there a block transfer being performed?
    *------------------------------------------------------------------------*/
   if( pc_rxBuf == NULL) /*No*/
   {

      /*---------------------------------------------------------------------*
       * Execute the call-back function returning the byte in the UART
       * buffer.
       *---------------------------------------------------------------------*/
      if( pt_handle->pf_rxCallBack != NULL)
         pt_handle->pf_rxCallBack( (uint16_t)pt_handle->pt_uart->DATA);
   }
   else
   {
      /*---------------------------------------------------------------------*
       * Store any received bytes...
       *---------------------------------------------------------------------*/
      pc_rxBuf[(*ps_bufIndex)] = pt_handle->pt_uart->DATA;
      (*ps_bufIndex)++;

      /*---------------------------------------------------------------------*
       * Is there more room in the buffer
       *---------------------------------------------------------------------*/
	   if( (*ps_bufIndex) == pt_handle->s_rxBufLength) /*No*/
	   {
         /*------------------------------------------------------------------*
          * Execute the call-back function returning the size of the
          * transfer.
          *------------------------------------------------------------------*/
         if( pt_handle->pf_rxCallBack != NULL)
            pt_handle->pf_rxCallBack( (*ps_bufIndex));

         pc_rxBuf = NULL;
         (*ps_bufIndex) = 0;
         pt_handle->s_rxBufLength = 0;
         pt_handle->b_rxBusLocked = false;
	   }
   }

}/*End updateRxBuffer*/

static void updateTxBuffer( t_uartChanHndl *pt_handle)
{
	int8_t *pc_txBuf = pt_handle->pc_txData;
	uint16_t *ps_bufIndex = &pt_handle->s_txBufIndex;

   if( pc_txBuf != NULL)
   {
	   /*------------------------------------------------------------------------*
	    * Send the data to the UART.
	    *------------------------------------------------------------------------*/
      pt_handle->pt_uart->DATA = pc_txBuf[(*ps_bufIndex)];
      (*ps_bufIndex)++;

	   /*------------------------------------------------------------------------*
	    * Is the buffer empty?
	    *------------------------------------------------------------------------*/
	   if( (*ps_bufIndex) == pt_handle->s_txBufLength)
	   {
	      /*---------------------------------------------------------------------*
          * Disable interrupts until another interrupt driven block transfer has
          * been requested.
          *---------------------------------------------------------------------*/
         pt_handle->pt_uart->CTRLA &= ~USART_DREINTLVL_gm;

         if( pt_handle->pf_txCallBack != NULL)
            pt_handle->pf_txCallBack( (*ps_bufIndex));

         /*---------------------------------------------------------------------*
          * Release the TX UART bus.
          *---------------------------------------------------------------------*/
         pt_handle->pc_txData = NULL;
         pt_handle->s_txBufIndex = 0;
         pt_handle->s_txBufLength = 0;
         pt_handle->b_txBusLocked = false;
	   }

   }/*End if( pc_txBuf != NULL)*/

}/*End updateTxBuffer*/

static void uart1RxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart1Chan;

   if( pt_handle != NULL)
   {
      updateRxDmaInt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End uart1RxDmaInt*/

static void uart2RxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart2Chan;

   if( pt_handle != NULL)
   {
      updateRxDmaInt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End uart2RxDmaInt*/

static void uart3RxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart3Chan;

   if( pt_handle != NULL)
   {
      updateRxDmaInt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End uart3RxDmaInt*/

static void uart4RxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart4Chan;

   if( pt_handle != NULL)
   {
      updateRxDmaInt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End uart4RxDmaInt*/

static void uart5RxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart5Chan;

   if( pt_handle != NULL)
   {
      updateRxDmaInt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End uart5RxDmaInt*/

static void uart6RxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart6Chan;

   if( pt_handle != NULL)
   {
      updateRxDmaInt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End uart6RxDmaInt*/

static void uart7RxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart7Chan;

   if( pt_handle != NULL)
   {
      updateRxDmaInt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End uart7RxDmaInt*/

static void uart8RxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart8Chan;

   if( pt_handle != NULL)
   {
      updateRxDmaInt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End uart8RxDmaInt*/

static void uart1TxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart1Chan;

   if( pt_handle != NULL)
   {

      if( pt_handle->pf_txCallBack != NULL)
         pt_handle->pf_txCallBack( pt_handle->s_txBufLength);

      pt_handle->pc_txData = NULL;
      pt_handle->s_txBufIndex = 0;
      pt_handle->s_txBufLength = 0;
      pt_handle->b_txBusLocked = false;
   }

}/*End uart1TxDmaInt*/

static void uart2TxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart2Chan;

   if( pt_handle != NULL)
   {

      if( pt_handle->pf_txCallBack != NULL)
         pt_handle->pf_txCallBack( pt_handle->s_txBufLength);

      pt_handle->pc_txData = NULL;
      pt_handle->s_txBufIndex = 0;
      pt_handle->s_txBufLength = 0;
      pt_handle->b_txBusLocked = false;
   }

}/*End uart2TxDmaInt*/

static void uart3TxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart3Chan;

   if( pt_handle != NULL)
   {

      if( pt_handle->pf_txCallBack != NULL)
         pt_handle->pf_txCallBack( pt_handle->s_txBufLength);

      pt_handle->pc_txData = NULL;
      pt_handle->s_txBufIndex = 0;
      pt_handle->s_txBufLength = 0;
      pt_handle->b_txBusLocked = false;
   }

}/*End uart3TxDmaInt*/

static void uart4TxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart4Chan;

   if( pt_handle != NULL)
   {

      if( pt_handle->pf_txCallBack != NULL)
         pt_handle->pf_txCallBack( pt_handle->s_txBufLength);

      pt_handle->pc_txData = NULL;
      pt_handle->s_txBufIndex = 0;
      pt_handle->s_txBufLength = 0;
      pt_handle->b_txBusLocked = false;
   }

}/*End uart4TxDmaInt*/

static void uart5TxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart5Chan;

   if( pt_handle != NULL)
   {

      if( pt_handle->pf_txCallBack != NULL)
         pt_handle->pf_txCallBack( pt_handle->s_txBufLength);

      pt_handle->pc_txData = NULL;
      pt_handle->s_txBufIndex = 0;
      pt_handle->s_txBufLength = 0;
      pt_handle->b_txBusLocked = false;
   }

}/*End uart5TxDmaInt*/

static void uart6TxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart6Chan;

   if( pt_handle != NULL)
   {

      if( pt_handle->pf_txCallBack != NULL)
         pt_handle->pf_txCallBack( pt_handle->s_txBufLength);

      pt_handle->pc_txData = NULL;
      pt_handle->s_txBufIndex = 0;
      pt_handle->s_txBufLength = 0;
      pt_handle->b_txBusLocked = false;
   }

}/*End uart6TxDmaInt*/

static void uart7TxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart7Chan;

   if( pt_handle != NULL)
   {

      if( pt_handle->pf_txCallBack != NULL)
         pt_handle->pf_txCallBack( pt_handle->s_txBufLength);

      pt_handle->pc_txData = NULL;
      pt_handle->s_txBufIndex = 0;
      pt_handle->s_txBufLength = 0;
      pt_handle->b_txBusLocked = false;
   }

}/*End uart7TxDmaInt*/

static void uart8TxDmaInt( void)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart8Chan;

   if( pt_handle != NULL)
   {

      if( pt_handle->pf_txCallBack != NULL)
         pt_handle->pf_txCallBack( pt_handle->s_txBufLength);

      pt_handle->pc_txData = NULL;
      pt_handle->s_txBufIndex = 0;
      pt_handle->s_txBufLength = 0;
      pt_handle->b_txBusLocked = false;
   }

}/*End uart8TxDmaInt*/

ISR( USARTC0_RXC_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart1Chan;

   if( pt_handle != NULL)
   {
		updateRxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTC0_RXC_vect)*/

ISR( USARTC0_DRE_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart1Chan;

   if( pt_handle != NULL)
   {
      updateTxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTC0_DRE_vect)*/

ISR( USARTC1_RXC_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart2Chan;

   if( pt_handle != NULL)
   {
		updateRxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTC1_RXC_vect)*/

ISR( USARTC1_DRE_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart2Chan;

   if( pt_handle != NULL)
   {
      updateTxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTC1_DRE_vect)*/

ISR( USARTD0_RXC_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart3Chan;

   if( pt_handle != NULL)
   {
		updateRxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTD0_RXC_vect)*/

ISR( USARTD0_DRE_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart3Chan;

   if( pt_handle != NULL)
   {
      updateTxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTD0_DRE_vect)*/

ISR( USARTD1_RXC_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart4Chan;

   if( pt_handle != NULL)
   {
		updateRxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTD1_RXC_vect)*/

ISR( USARTD1_DRE_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart4Chan;

   if( pt_handle != NULL)
   {
      updateTxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTD1_DRE_vect)*/

ISR( USARTE0_RXC_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart5Chan;

   if( pt_handle != NULL)
   {
		updateRxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTE0_RXC_vect)*/

ISR( USARTE0_DRE_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart5Chan;

   if( pt_handle != NULL)
   {
      updateTxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTE0_DRE_vect)*/

ISR( USARTE1_RXC_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart6Chan;

   if( pt_handle != NULL)
   {
		updateRxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTE1_RXC_vect)*/

ISR( USARTE1_DRE_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart6Chan;

   if( pt_handle != NULL)
   {
      updateTxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTE1_DRE_vect)*/

ISR( USARTF0_RXC_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart7Chan;

   if( pt_handle != NULL)
   {
		updateRxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTF0_RXC_vect)*/

ISR( USARTF0_DRE_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart7Chan;

   if( pt_handle != NULL)
   {
      updateTxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTF0_DRE_vect)*/

ISR( USARTF1_RXC_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart8Chan;

   if( pt_handle != NULL)
   {
		updateRxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTF1_RXC_vect)*/

ISR( USARTF1_DRE_vect)
{
   t_uartChanHndl *pt_handle = NULL;

   pt_handle = gt_intChanMap.pt_uart8Chan;

   if( pt_handle != NULL)
   {
      updateTxBuffer( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( USARTF1_DRE_vect)*/

static t_uartChanHndl *findUartElement( t_uartChanId t_id)
{
   t_uartChanHndl *pt_element;
   t_LINKHNDL t_linkHndl;
   int16_t s_count;

   /*------------------------------------------------------------------------*
    * Search the UART list for the requested ID
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_uartChanHndlList, s_count)
   {
      pt_element = (t_uartChanHndl *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( pt_element->t_id == t_id)
      {
         return pt_element;
      }

   }

   /*------------------------------------------------------------------------*
    * If we make it this far the ID has not been found in the open UART module
    * list.
    *------------------------------------------------------------------------*/
   return NULL;

}/*End findUartElement*/

static t_LINKHNDL createUartHandle( void)
{

   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Allocated memory for the link (and element) that contains information
    * specific to this particular UART module
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof( t_uartChanHndl));

   if( t_linkHndl < 0)
   {
      return (t_LINKHNDL)UART_OUT_OF_HEAP;
   }

   /*------------------------------------------------------------------------*
    * Add the UART module link onto the list open UART modules.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_uartChanHndlList,
                           t_linkHndl,
                           true);

   return t_linkHndl;

}/*End createUartHandle*/

t_uartError hal_enableUartRxInt( t_UARTHNDL t_handle)
{
   t_uartChanHndl *pt_uartChanHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a UART module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_uartChanHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return UART_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the UART
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl = (t_uartChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      pt_uartChanHndl->pt_uart->CTRLA &= ~USART_RXCINTLVL_gm;
      pt_uartChanHndl->pt_uart->CTRLA |= USART_RXCINTLVL_HI_gc;

   }

   HAL_END_CRITICAL();//Enable interrupts

   return UART_PASSED;

}/*End hal_enableUartRxInt*/

t_uartError hal_disableUartRxInt( t_UARTHNDL t_handle)
{
   t_uartChanHndl *pt_uartChanHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a UART module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_uartChanHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return UART_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the UART
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl = (t_uartChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      pt_uartChanHndl->pt_uart->CTRLA &= ~USART_RXCINTLVL_gm;

   }

   HAL_END_CRITICAL();//Enable interrupts

   return UART_PASSED;

}/*End hal_disableUartRxInt*/

t_uartError hal_uartWriteByte( t_UARTHNDL t_handle,
                               uint8_t c_byte)
{
   t_uartChanHndl *pt_uartChanHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a UART module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_uartChanHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return UART_INVALID_HANDLE;
   }

   pt_uartChanHndl = (t_uartChanHndl *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

	/*------------------------------------------------------------------------*
	 * Is the bus locked by someone other than this user?
	 *------------------------------------------------------------------------*/
	if( pt_uartChanHndl->b_txBusLocked == true)
	{
      HAL_END_CRITICAL();//Enable interrupts
   	return UART_BUSY; /*Yes*/
   }

   pt_uartChanHndl->b_txBusLocked = true;

   HAL_END_CRITICAL();//Enable interrupts

   /*------------------------------------------------------------------------*
    * Wait for room to become available in the TX buffer.
    *------------------------------------------------------------------------*/
   while( !(pt_uartChanHndl->pt_uart->STATUS & USART_DREIF_bm));

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the UART
    * module information is being stored.
    *------------------------------------------------------------------------*/
   pt_uartChanHndl->pt_uart->DATA = c_byte;

   /*------------------------------------------------------------------------*
    * Release bus lock
    *------------------------------------------------------------------------*/
   pt_uartChanHndl->b_txBusLocked = false;

   return UART_PASSED;

}/*End hal_uartWriteByte*/

/*---------------------------------------------------------------------------*
 * Read a block of data from a particular UART channel.
 *---------------------------------------------------------------------------*/
t_uartError hal_uartReadBlock( t_UARTHNDL t_handle,
								       int8_t *pc_rxBuffer,
								       uint16_t s_numBytes)
{
   t_uartChanHndl *pt_uartChanHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a UART module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_uartChanHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return UART_INVALID_HANDLE;
   }

   pt_uartChanHndl = (t_uartChanHndl *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

	/*------------------------------------------------------------------------*
	 * Is the bus locked by someone other than this user?
	 *------------------------------------------------------------------------*/
	if( pt_uartChanHndl->b_rxBusLocked == true)
	{
      HAL_END_CRITICAL();//Enable interrupts
   	return UART_BUSY; /*Yes*/
   }

   pt_uartChanHndl->b_rxBusLocked = true;

	/*------------------------------------------------------------------------*
    * Disable interrupts until its known whether the transaction uses DMA.
    *------------------------------------------------------------------------*/
   pt_uartChanHndl->pt_uart->CTRLA &= ~USART_RXCINTLVL_gm;

   HAL_END_CRITICAL();//Enable interrupts

   pt_uartChanHndl->pc_rxData = pc_rxBuffer;
   pt_uartChanHndl->s_rxBufLength = s_numBytes;
   pt_uartChanHndl->s_rxBufIndex = 0;

	/*------------------------------------------------------------------------*
    * Does this transaction use DMA?
    *------------------------------------------------------------------------*/
   if( pt_uartChanHndl->t_rxDmaHndl > 0)
   {
      t_dmaChanConfig t_chanConf;
      t_dmaTriggerSource t_trigger;

      switch( pt_uartChanHndl->t_id)
      {
         case UART_1:
            t_trigger = UART1_RX_COMPLETE;
         break;

         case UART_2:
            t_trigger = UART2_RX_COMPLETE;
         break;

         case UART_3:
            t_trigger = UART3_RX_COMPLETE;
         break;

         case UART_4:
            t_trigger = UART4_RX_COMPLETE;
         break;

         case UART_5:
            t_trigger = UART5_RX_COMPLETE;
         break;

         case UART_6:
            t_trigger = UART6_RX_COMPLETE;
         break;

         case UART_7:
            t_trigger = UART7_RX_COMPLETE;
         break;

         case UART_8:
            t_trigger = UART8_RX_COMPLETE;
         break;

         default:
            t_trigger = UART1_RX_COMPLETE;
         break;

      }/*End switch( pt_uartChanHndl->t_id)*/

      t_chanConf.pi_srcAddress   = (uint32_t *)&pt_uartChanHndl->pt_uart->DATA;
      t_chanConf.pi_destAddress  = (uint32_t *)pc_rxBuffer;
      t_chanConf.t_srcAddDir     = FIXED;
      t_chanConf.t_destAddDir    = INCREMENT;
      t_chanConf.t_srcAddReload  = NO_RELOAD;
      t_chanConf.t_destAddReload = RELOAD_END_OF_BLOCK;
      t_chanConf.s_blockSize     = s_numBytes;
      t_chanConf.t_burstMode     = ONE_BYTE;
      t_chanConf.t_transferType  = SINGLE_SHOT;
      t_chanConf.t_triggerSrc    = t_trigger;
      t_chanConf.c_repeatCount   = 0;

      hal_configureDmaChannel( pt_uartChanHndl->t_rxDmaHndl,
                               t_chanConf);

      hal_dmaEnableChannel( pt_uartChanHndl->t_rxDmaHndl);

   }/*End if( pt_uartChanHndl->t_rxDmaHndl > 0)*/
   else
   {
      /*---------------------------------------------------------------------*
       * Receive data via the RX complete interrupt.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pt_uart->CTRLA |= USART_RXCINTLVL_HI_gc;
   }

   return UART_PASSED;

}/*End hal_uartReadBlock*/

/*---------------------------------------------------------------------------*
 * Write a block of data to a particular UART channel.
 *---------------------------------------------------------------------------*/
t_uartError hal_uartWriteBlock( t_UARTHNDL t_handle,
								        int8_t *pc_txBuffer,
								        uint16_t s_numBytes)
{
   t_uartChanHndl *pt_uartChanHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a UART module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_uartChanHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return UART_INVALID_HANDLE;
   }

   pt_uartChanHndl = (t_uartChanHndl *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

	/*------------------------------------------------------------------------*
	 * Is the bus locked by someone other than this user?
	 *------------------------------------------------------------------------*/
	if( pt_uartChanHndl->b_txBusLocked == true)
	{
      HAL_END_CRITICAL();//Enable interrupts
   	return UART_BUSY; /*Yes*/
   }

   pt_uartChanHndl->b_txBusLocked = true;

   pt_uartChanHndl->pt_uart->CTRLA &= ~USART_DREINTLVL_HI_gc;

   HAL_END_CRITICAL();//Enable interrupts

   pt_uartChanHndl->pc_txData = pc_txBuffer;
   pt_uartChanHndl->s_txBufLength = s_numBytes;
   pt_uartChanHndl->s_txBufIndex = 0;

	/*------------------------------------------------------------------------*
    * Are interrupts enabled, is this function being called from an
    * active int, or is the tx callback function not defined? If so, then
    * transmit the block of data "in-place".
    *------------------------------------------------------------------------*/
   if( (HAL_ARE_INTS_EN() == 0) ||
       (HAL_IS_ACTIVE_INT() == 1) ||
       (pt_uartChanHndl->pf_txCallBack == NULL))
   {
      uint16_t s_index;
   
      for( s_index = 0; s_index < pt_uartChanHndl->s_txBufLength; s_index++)
      {
         /*------------------------------------------------------------------*
          * Wait for room to become available in the TX buffer.
          *------------------------------------------------------------------*/
         while( !(pt_uartChanHndl->pt_uart->STATUS & USART_DREIF_bm));

         /*------------------------------------------------------------------*
          * Get a ptr to the link's element- which is the area where the UART
          * module information is being stored.
          *------------------------------------------------------------------*/
         pt_uartChanHndl->pt_uart->DATA = pt_uartChanHndl->pc_txData[s_index];

      }/*End for( s_index = 0; s_index < pt_uartChanHndl->s_txBufLength;
         s_index++)*/

      if( pt_uartChanHndl->pf_txCallBack != NULL)
         pt_uartChanHndl->pf_txCallBack( pt_uartChanHndl->s_txBufLength);

      pt_uartChanHndl->s_txBufLength = 0;
      pt_uartChanHndl->pc_txData = NULL;
      pt_uartChanHndl->b_txBusLocked = false;

   }/*End if( (HAL_ARE_INTS_EN() == 0) || (HAL_IS_ACTIVE_INT() == 1)*/
   else
   {
	   /*---------------------------------------------------------------------*
       * Does this transaction use DMA?
       *---------------------------------------------------------------------*/
      if( pt_uartChanHndl->t_txDmaHndl > 0)
      {
         t_dmaChanConfig t_chanConf;
         t_dmaTriggerSource t_trigger;

         switch( pt_uartChanHndl->t_id)
         {
            case UART_1:
               t_trigger = UART1_DATA_REG_EMPTY;
            break;

            case UART_2:
               t_trigger = UART2_DATA_REG_EMPTY;
            break;

            case UART_3:
               t_trigger = UART3_DATA_REG_EMPTY;
            break;

            case UART_4:
               t_trigger = UART4_DATA_REG_EMPTY;
            break;

            case UART_5:
               t_trigger = UART5_DATA_REG_EMPTY;
            break;

            case UART_6:
               t_trigger = UART6_DATA_REG_EMPTY;
            break;

            case UART_7:
               t_trigger = UART7_DATA_REG_EMPTY;
            break;

            case UART_8:
               t_trigger = UART8_DATA_REG_EMPTY;
            break;

            default:
               t_trigger = UART1_DATA_REG_EMPTY;
            break;

         }/*End switch( pt_uartChanHndl->t_id)*/

         t_chanConf.pi_srcAddress   = (uint32_t *)&pt_uartChanHndl->pc_txData[0];
         t_chanConf.pi_destAddress  = (uint32_t *)&pt_uartChanHndl->pt_uart->DATA;
         t_chanConf.t_srcAddDir     = INCREMENT;
         t_chanConf.t_destAddDir    = FIXED;
         t_chanConf.t_srcAddReload  = RELOAD_END_OF_BLOCK;
         t_chanConf.t_destAddReload = NO_RELOAD;
         t_chanConf.s_blockSize     = s_numBytes;
         t_chanConf.t_burstMode     = ONE_BYTE;
         t_chanConf.t_transferType  = SINGLE_SHOT;
         t_chanConf.t_triggerSrc    = t_trigger;
         t_chanConf.c_repeatCount   = 0;

         hal_configureDmaChannel( pt_uartChanHndl->t_txDmaHndl,
                                  t_chanConf);

         hal_dmaEnableChannel( pt_uartChanHndl->t_txDmaHndl);

      }/*End if( pt_uartChanHndl->t_txDmaHndl > 0)*/
      else
      {
         /*------------------------------------------------------------------*
          * Transmit data via the DRE interrupt.
          *------------------------------------------------------------------*/
         pt_uartChanHndl->pt_uart->CTRLA |= USART_DREINTLVL_HI_gc;
      }
   }

   return UART_PASSED;

}/*End hal_uartWriteBlock*/

int32_t hal_uartGetBaudRate( t_UARTHNDL t_handle)
{
   t_uartChanHndl *pt_uartChanHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a UART module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_uartChanHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (int32_t)UART_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the UART
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl = (t_uartChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);
   }

   HAL_END_CRITICAL();//Enable interrupts

   return (int32_t)pt_uartChanHndl->i_baudRate;

}/*End hal_uartGetBaudRate*/

int32_t hal_uartGetDataAddress( t_UARTHNDL t_handle)
{
   t_uartChanHndl *pt_uartChanHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a UART module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_uartChanHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (int32_t)UART_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the UART
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl = (t_uartChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);
   }

   HAL_END_CRITICAL();//Enable interrupts

   return (int32_t)((int16_t)(void *)&pt_uartChanHndl->pt_uart->DATA);

}/*End hal_uartGetDataAddress*/

/*---------------------------------------------------------------------------*
 * Request access to a particular UART module
 *---------------------------------------------------------------------------*/
t_UARTHNDL hal_requestUartChannel( t_uartChanId t_chanId)
{
   t_uartChanHndl *pt_uartChanHndl;
   t_LINKHNDL t_linkHndl;
   t_gpioConf t_conf;
   t_gpioError t_gErr;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( (t_chanId < UART_1) || (t_chanId > UART_8))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_UARTHNDL)UART_INVALID_MODULE;
   }

   pt_uartChanHndl = findUartElement( t_chanId);

   /*------------------------------------------------------------------------*
    * Is there a channel available?
    *------------------------------------------------------------------------*/
   if( pt_uartChanHndl == NULL) /*Yes*/
   {

      t_linkHndl = createUartHandle();
      if( t_linkHndl < 0)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return (t_UARTHNDL)UART_OUT_OF_HEAP;

      }

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the UART
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl = (t_uartChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);

      pt_uartChanHndl->t_id = t_chanId;
      pt_uartChanHndl->i_baudRate = 0;

      switch( t_chanId)
      {
         case UART_1:

            pt_uartChanHndl->pt_uart = &USARTC0;
            /*---------------------------------------------------------------*
             * Configure the output pins the UART will use.
             *---------------------------------------------------------------*/
            t_conf.c_inputMask    = PIN_2;
            t_conf.c_outputMask   = PIN_3;
            t_conf.b_setOutputLow = true;
            t_conf.t_inConf       = TOTEM;
            t_conf.t_outConf      = TOTEM;
            t_gErr = hal_configureGpioPort( GPIO_PORTC, t_conf);
            gt_intChanMap.pt_uart1Chan = pt_uartChanHndl;

         break;

         case UART_2:

            pt_uartChanHndl->pt_uart = &USARTC1;
            /*---------------------------------------------------------------*
             * Configure the output pins the UART will use.
             *---------------------------------------------------------------*/
            t_conf.c_inputMask    = PIN_6;
            t_conf.c_outputMask   = PIN_7;
            t_conf.b_setOutputLow = true;
            t_conf.t_inConf       = TOTEM;
            t_conf.t_outConf      = TOTEM;
            t_gErr = hal_configureGpioPort( GPIO_PORTC, t_conf);
            gt_intChanMap.pt_uart2Chan = pt_uartChanHndl;

         break;

         case UART_3:

            pt_uartChanHndl->pt_uart = &USARTD0;
            /*---------------------------------------------------------------*
             * Configure the output pins the UART will use.
             *---------------------------------------------------------------*/
            t_conf.c_inputMask    = PIN_2;
            t_conf.c_outputMask   = PIN_3;
            t_conf.b_setOutputLow = true;
            t_conf.t_inConf       = TOTEM;
            t_conf.t_outConf      = TOTEM;
            t_gErr = hal_configureGpioPort( GPIO_PORTD, t_conf);
            gt_intChanMap.pt_uart3Chan = pt_uartChanHndl;

         break;

         case UART_4:

            pt_uartChanHndl->pt_uart = &USARTD1;
            /*---------------------------------------------------------------*
             * Configure the output pins the UART will use.
             *---------------------------------------------------------------*/
            t_conf.c_inputMask    = PIN_6;
            t_conf.c_outputMask   = PIN_7;
            t_conf.b_setOutputLow = true;
            t_conf.t_inConf       = TOTEM;
            t_conf.t_outConf      = TOTEM;
            t_gErr = hal_configureGpioPort( GPIO_PORTD, t_conf);
            gt_intChanMap.pt_uart4Chan = pt_uartChanHndl;

         break;

         case UART_5:

            pt_uartChanHndl->pt_uart = &USARTE0;
            /*---------------------------------------------------------------*
             * Configure the output pins the UART will use.
             *---------------------------------------------------------------*/
            t_conf.c_inputMask    = PIN_2;
            t_conf.c_outputMask   = PIN_3;
            t_conf.b_setOutputLow = true;
            t_conf.t_inConf       = PULLUP;//TOTEM;
            t_conf.t_outConf      = TOTEM;
            t_gErr = hal_configureGpioPort( GPIO_PORTE, t_conf);
            gt_intChanMap.pt_uart5Chan = pt_uartChanHndl;

         break;

         case UART_6:

            pt_uartChanHndl->pt_uart = &USARTE1;
            /*---------------------------------------------------------------*
             * Configure the output pins the UART will use.
             *---------------------------------------------------------------*/
            t_conf.c_inputMask    = PIN_6;
            t_conf.c_outputMask   = PIN_7;
            t_conf.b_setOutputLow = true;
            t_conf.t_inConf       = PULLUP;//TOTEM;
            t_conf.t_outConf      = TOTEM;
            t_gErr = hal_configureGpioPort( GPIO_PORTE, t_conf);
            gt_intChanMap.pt_uart6Chan = pt_uartChanHndl;

         break;

         case UART_7:

            pt_uartChanHndl->pt_uart = &USARTF0;
            /*---------------------------------------------------------------*
             * Configure the output pins the UART will use.
             *---------------------------------------------------------------*/
            t_conf.c_inputMask    = PIN_2;
            t_conf.c_outputMask   = PIN_3;
            t_conf.b_setOutputLow = true;
            t_conf.t_inConf       = PULLUP;//TOTEM;
            t_conf.t_outConf      = TOTEM;
            t_gErr = hal_configureGpioPort( GPIO_PORTF, t_conf);
            gt_intChanMap.pt_uart7Chan = pt_uartChanHndl;

         break;

         case UART_8:

            pt_uartChanHndl->pt_uart = &USARTF1;
            /*---------------------------------------------------------------*
             * Configure the output pins the console will use.
             *---------------------------------------------------------------*/
            t_conf.c_inputMask    = PIN_6;
            t_conf.c_outputMask   = PIN_7;
            t_conf.b_setOutputLow = true;
            t_conf.t_inConf       = PULLUP;//TOTEM;
            t_conf.t_outConf      = TOTEM;
            t_gErr = hal_configureGpioPort( GPIO_PORTF, t_conf);
            gt_intChanMap.pt_uart8Chan = pt_uartChanHndl;

         break;

      }/*End switch( t_chanId)*/

   }/*End if( pt_uartChanHndl == NULL)*/
   else /*No*/
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_UARTHNDL)UART_CHANNEL_OPEN;
   }

   HAL_END_CRITICAL();//Enable interrupts

   return (t_UARTHNDL)t_linkHndl;

}/*End hal_requestUartChannel*/

t_uartError hal_releaseUartChannel( t_UARTHNDL t_handle)
{
   t_uartChanHndl *pt_uartChanHndl;
   t_linkedListError t_lErr;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a UART module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_uartChanHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return UART_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the UART
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl = (t_uartChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      pt_uartChanHndl->pt_uart->CTRLA &= ~USART_RXCINTLVL_gm;
      pt_uartChanHndl->pt_uart->CTRLA &= ~USART_DREINTLVL_gm;
      pt_uartChanHndl->pt_uart->CTRLA &= ~USART_TXCINTLVL_gm;

      /*---------------------------------------------------------------------*
       * Disable and reset this UART channel.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pt_uart->CTRLA &= ~USART_RXCINTLVL_gm;
      pt_uartChanHndl->pt_uart->CTRLA &= ~USART_DREINTLVL_gm;
      pt_uartChanHndl->pt_uart->CTRLA &= ~USART_TXCINTLVL_gm;
      pt_uartChanHndl->pt_uart->CTRLB &= ~USART_RXEN_bm;
      pt_uartChanHndl->pt_uart->CTRLB &= ~USART_TXEN_bm;
      pt_uartChanHndl->pt_uart->CTRLC &= ~USART_CMODE_gm;
      pt_uartChanHndl->pt_uart->CTRLC &= ~USART_PMODE_gm;
      pt_uartChanHndl->pt_uart->CTRLC &= ~USART_CHSIZE_gm;
      pt_uartChanHndl->pt_uart->CTRLC &= ~USART_SBMODE_bm;
      pt_uartChanHndl->pt_uart->BAUDCTRLA &= ~USART_BSEL_gm;
      pt_uartChanHndl->pt_uart->BAUDCTRLB &= ~0x0F; /*BSEL in control B reg*/
      pt_uartChanHndl->pt_uart->BAUDCTRLB &= ~USART_BSCALE_gm;

      switch( pt_uartChanHndl->t_id)
      {
         case UART_1:
            gt_intChanMap.pt_uart1Chan = NULL;
         break;

         case UART_2:
            gt_intChanMap.pt_uart2Chan = NULL;
         break;

         case UART_3:
            gt_intChanMap.pt_uart3Chan = NULL;
         break;

         case UART_4:
            gt_intChanMap.pt_uart4Chan = NULL;
         break;

         case UART_5:
            gt_intChanMap.pt_uart5Chan = NULL;
         break;

         case UART_6:
            gt_intChanMap.pt_uart6Chan = NULL;
         break;

         case UART_7:
            gt_intChanMap.pt_uart7Chan = NULL;
         break;

         case UART_8:
            gt_intChanMap.pt_uart8Chan = NULL;
         break;

      }/*End switch( pt_uartChanHndl->t_id)*/

      t_lErr = utl_destroyLink( gt_uartChanHndlList,
                                (t_LINKHNDL)t_handle);

   }

   HAL_END_CRITICAL();//Enable interrupts

   return UART_PASSED;

}/*End hal_releaseUartChannel*/

t_uartError hal_configureUartChannel( t_UARTHNDL t_handle,
                                      t_uartConfig t_conf)
{
   t_uartChanHndl *pt_uartChanHndl;
   float f_bselPlus1 = 0.0f;
   uint32_t i_baud = 0;
   uint16_t s_bsel = 0;
   int8_t c_bscale = 0;
   void (*pf_rxDmaCallback)( void) = NULL;
   void (*pf_txDmaCallback)( void) = NULL;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * At this time only async mode is supported!
    *------------------------------------------------------------------------*/
   if( t_conf.t_comMd != ASYNC)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return UART_INVALID_COM_MODE;
   }/*End if( t_conf.t_comMd != ASYNC)*/

   if( (t_conf.t_parityMd < NO_PARITY) || (t_conf.t_parityMd > ODD_PARITY))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return UART_INVALID_PARITY_MODE;
   }/*End if( (t_conf.t_parityMd < NO_PARITY) || (t_conf.t_parityMd >
   ODD_PARITY))*/

   if( (t_conf.t_charSz < CHAR_5BIT) || (t_conf.t_charSz > CHAR_8BIT))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return UART_INVALID_CHAR_SIZE;
   }/*End if( (t_conf.t_charSz < CHAR_5BIT) || (t_conf.t_charSz > CHAR_8BIT))*/

   if( (t_conf.t_stopBitMd < ONE_STOP_BIT) || (t_conf.t_stopBitMd >
   TWO_STOP_BITS))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return UART_INVALID_STOPBIT_MODE;
   }/*End if( (t_conf.t_stopBitMd < ONE_STOP_BIT) || (t_conf.t_stopBitMd >
   TWO_STOP_BITS))*/

   if( ((float)t_conf.i_baudRate < ((float)hal_getCpuFreq() / 8388608.0f)) ||
   ((float)t_conf.i_baudRate > ((float)hal_getCpuFreq() / 16.0f)))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return UART_INVALID_BAUD_RATE;
   }

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a UART module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_uartChanHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return UART_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the UART
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl = (t_uartChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Disable all UART interrupts for this channel
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pt_uart->CTRLA &= ~USART_RXCINTLVL_gm;
      pt_uartChanHndl->pt_uart->CTRLA &= ~USART_DREINTLVL_gm;
      pt_uartChanHndl->pt_uart->CTRLA &= ~USART_TXCINTLVL_gm;

      /*---------------------------------------------------------------------*
       * Disable RX and TX UARTS.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pt_uart->CTRLB &= ~USART_RXEN_bm;
      pt_uartChanHndl->pt_uart->CTRLB &= ~USART_TXEN_bm;

      /*---------------------------------------------------------------------*
       * Reset and configure the UART mode.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pt_uart->CTRLC &= ~USART_CMODE_gm;
      pt_uartChanHndl->pt_uart->CTRLC |= (t_conf.t_comMd << 6);

      /*---------------------------------------------------------------------*
       * Reset and configure the parity mode.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pt_uart->CTRLC &= ~USART_PMODE_gm;

      switch( t_conf.t_parityMd)
      {
         case NO_PARITY:
            pt_uartChanHndl->pt_uart->CTRLC |= USART_PMODE_DISABLED_gc;
         break;

         case EVEN_PARITY:
            pt_uartChanHndl->pt_uart->CTRLC |= USART_PMODE_EVEN_gc;
         break;

         case ODD_PARITY:
            pt_uartChanHndl->pt_uart->CTRLC |= USART_PMODE_ODD_gc;
         break;

      }/*End switch( t_conf.t_parityMd)*/

      /*---------------------------------------------------------------------*
       * Reset and configure the character size.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pt_uart->CTRLC &= ~USART_CHSIZE_gm;
      pt_uartChanHndl->pt_uart->CTRLC |= t_conf.t_charSz;

      /*---------------------------------------------------------------------*
       * Reset and configure the number of stop bits.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pt_uart->CTRLC &= ~USART_SBMODE_bm;
      pt_uartChanHndl->pt_uart->CTRLC |= (t_conf.t_stopBitMd << 3);

      /*---------------------------------------------------------------------*
       * Reset and configure the baud rate
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pt_uart->BAUDCTRLA &= ~USART_BSEL_gm;
      pt_uartChanHndl->pt_uart->BAUDCTRLB &= ~0x0F; /*BSEL in control B reg*/
      pt_uartChanHndl->pt_uart->BAUDCTRLB &= ~USART_BSCALE_gm;

      /*---------------------------------------------------------------------*
       * For asynchronous mode of operation the baud rate is controlled
       * through two variables BSEL and BSCALE. BSEL is used for scaling the
       * peripheral clock and can range from 0 to 4095. Since BSEL has
       * limited range, BSCALE can be used for increasing/decreasing the
       * baud rate. BSCALE ranges from -7 to 7, where a negative value has
       * the effect of decreasing BSEL; conversely, a positive value has
       * the effect of increasing BSEL. The two equations for calculating
       * the baud rate are as follows;
       *
       * Equation 1: BSCALE >= 0
       *
       *                     fper                       BSCALE       fper
       *    fbaud = ------------------------ , (BSEL+1)*2      = -----------
       *              BSCALE                                       16*fbaud
       *             2       * 16*(BSEL + 1)
       *
       *
       * Equation 2: BSCALE < 0
       *
       *                     fper                    BSCALE      fper
       *    fbaud = ------------------------ , BSEL*2       = ---------- - 1
       *                  BSCALE                               16*fbaud
       *             16((2       *BSEL) + 1)
       *
       * The first thing that we need to do is decide whether BSCALE is
       * positive, negative or 0. In order to do this we use part 2 of
       * equation 1 with BSCALE = 0 and use the requested baud 't_conf.
       * i_baudRate' to calculate BSEL (in floating point arithmetic). If BSEL
       * is greater than BSEL_MAX_RANGE, then it needs to be reduced below
       * BSEL_MAX_RANGE using 2^BSCALE. If BSEL is less than BSEL_MAX_RANGE then
       * one could just use the truncated the value as is - however losing the
       * fractional values will increase the error in baud rate calculation.
       * This is where BSCALE is useful, by scaling BSEL up by 2^BSCALE we are
       * changing the fractional values into fixed-point ones. The more we
       * move BSEL to the right the more precision we gain. The scaling
       * continues until BSEL <= BSEL_MAX_RANGE. At that point the truncated
       * value of BSEL and the final value BSCALE are loaded into the baud
       * rate registers and the overall error in the baud rate is minimized.
       *---------------------------------------------------------------------*/

      /*Using part 2 of equation 1*/
      f_bselPlus1 = ((float)hal_getCpuFreq() /
      ((float)t_conf.i_baudRate*16.0f));

      if( (f_bselPlus1 - 1.0f) >= (float)BSEL_MAX_RANGE) /* Too high need to
                                                            lower using
                                                            BSCALE*/
      {
         /*------------------------------------------------------------------*
          * Find the number of powers-of-2 'f_bselPlus1' is above 4095
          *------------------------------------------------------------------*/
         while( f_bselPlus1 > (float)BSEL_MAX_RANGE)
         {
            f_bselPlus1 = f_bselPlus1 / 2.0f;
            c_bscale++;
            if( c_bscale == 7)
               break;
         }

         s_bsel = (uint16_t)(f_bselPlus1 - 1.0f);

      }/*End if( (f_bselPlus1 - 1.0f) >= BSEL_MAX_RANGE)*/
      else if( (f_bselPlus1 - 1.0f) < (float)BSEL_MAX_RANGE) /* Lower than
                                                                maximum, increase
                                                                using BSCALE*/
      {
         /*Equation 2 part 2 references bsel not bsel + 1*/
         f_bselPlus1 = f_bselPlus1 - 1.0f;

         /*------------------------------------------------------------------*
          * Find the number of powers-of-2 'f_bselPlus1' is above 4095
          *------------------------------------------------------------------*/
         while( f_bselPlus1 < (float)BSEL_MAX_RANGE)
         {
            f_bselPlus1 = f_bselPlus1*2.0f;
            c_bscale--;
            if( c_bscale == -7)
               break;
         }

         if( f_bselPlus1 > (float)BSEL_MAX_RANGE)
         {
            /*One step too far*/
            f_bselPlus1 /= 2.0;
            c_bscale++;
         }

         s_bsel = (uint16_t)f_bselPlus1;

      }/*End else if( (f_bselPlus1 - 1.0f) < (float)BSEL_MAX_RANGE)*/

      /*---------------------------------------------------------------------*
       * Calculate the baud rate error
       *---------------------------------------------------------------------*/
      if( c_bscale > 0)
      {
          i_baud = hal_getCpuFreq() / ((uint32_t)16*((uint32_t)1 << c_bscale)*
          ((uint32_t)s_bsel + 1));
      }
      else
      {
          i_baud = ((uint32_t)hal_getCpuFreq()) / ((((uint32_t)16*
          (uint32_t)s_bsel) / ((uint32_t)1 << -c_bscale)) + 16);
      }

      pt_uartChanHndl->f_percentBaudError = (((float)i_baud -
      (float)t_conf.i_baudRate) / ((float)t_conf.i_baudRate))*100.0f;

      switch( pt_uartChanHndl->t_id)
      {
         case UART_1:
            pf_rxDmaCallback = &uart1RxDmaInt;
            pf_txDmaCallback = &uart1TxDmaInt;
         break;

         case UART_2:
            pf_rxDmaCallback = &uart2RxDmaInt;
            pf_txDmaCallback = &uart2TxDmaInt;
         break;

         case UART_3:
            pf_rxDmaCallback = &uart3RxDmaInt;
            pf_txDmaCallback = &uart3TxDmaInt;
         break;

         case UART_4:
            pf_rxDmaCallback = &uart4RxDmaInt;
            pf_txDmaCallback = &uart4TxDmaInt;
         break;

         case UART_5:
            pf_rxDmaCallback = &uart5RxDmaInt;
            pf_txDmaCallback = &uart5TxDmaInt;
         break;

         case UART_6:
            pf_rxDmaCallback = &uart6RxDmaInt;
            pf_txDmaCallback = &uart6TxDmaInt;
         break;

         case UART_7:
            pf_rxDmaCallback = &uart7RxDmaInt;
            pf_txDmaCallback = &uart7TxDmaInt;
         break;

         case UART_8:
            pf_rxDmaCallback = &uart8RxDmaInt;
            pf_txDmaCallback = &uart8TxDmaInt;
         break;

      }/*End switch( pt_uartChanHndl->t_id)*/

      /*---------------------------------------------------------------------*
       * Is DMA being used for tx or rx transactions?
       *---------------------------------------------------------------------*/
      if( t_conf.b_enRxDma == true)
      {
         pt_uartChanHndl->t_rxDmaHndl = hal_requestDmaChannel();
         if( pt_uartChanHndl->t_rxDmaHndl  < 0)
         {
            HAL_END_CRITICAL();//Enable interrupts
            if( pt_uartChanHndl->t_rxDmaHndl == DMA_NO_CHANNELS_OPEN)
               return UART_NO_DMA;
            else
               return UART_OUT_OF_HEAP;

         }/*End if( pt_uartChanHndl->t_rxDmaHndl  < 0)*/

         hal_requestDmaInterrupt( pt_uartChanHndl->t_rxDmaHndl,
                                  DMA_TRANSFER_COMPLETE,
                                  pf_rxDmaCallback);

      }/*End if( t_conf.b_enRxDma == true)*/

      if( t_conf.b_enTxDma == true)
      {
         pt_uartChanHndl->t_txDmaHndl = hal_requestDmaChannel();
         if( pt_uartChanHndl->t_txDmaHndl  < 0)
         {

            HAL_END_CRITICAL();//Enable interrupts
            if( pt_uartChanHndl->t_txDmaHndl == DMA_NO_CHANNELS_OPEN)
               return UART_NO_DMA;
            else
               return UART_OUT_OF_HEAP;

         }/*End if( pt_spiChanHndl->t_txDmaHndl  < 0)*/

         hal_requestDmaInterrupt( pt_uartChanHndl->t_txDmaHndl,
                                  DMA_TRANSFER_COMPLETE,
                                  pf_txDmaCallback);

      }/*End if( t_conf.b_enTxDma == true)*/

      /*---------------------------------------------------------------------*
       * Keep track of the current baud rate for this particular UART.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->i_baudRate = t_conf.i_baudRate;

      /*---------------------------------------------------------------------*
       * Configure the baud rate generator registers
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pt_uart->BAUDCTRLA = (uint8_t)s_bsel;
      pt_uartChanHndl->pt_uart->BAUDCTRLB = (c_bscale << USART_BSCALE0_bp) |
      (s_bsel >> 8);

      /*---------------------------------------------------------------------*
       * Enable RX and TX UARTS.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pt_uart->CTRLB |= USART_RXEN_bm;
      pt_uartChanHndl->pt_uart->CTRLB |= USART_TXEN_bm;

      /*---------------------------------------------------------------------*
       * No transactions in progress.
       *---------------------------------------------------------------------*/
		pt_uartChanHndl->b_txBusLocked = false;
      pt_uartChanHndl->b_rxBusLocked = false;

      /*---------------------------------------------------------------------*
       * Buffers are only used during block transfers.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pc_rxData = NULL;
      pt_uartChanHndl->pc_txData = NULL;
      pt_uartChanHndl->s_rxBufIndex = 0;
      pt_uartChanHndl->s_rxBufLength = 0;
      pt_uartChanHndl->s_txBufIndex = 0;
      pt_uartChanHndl->s_txBufLength = 0;

      /*---------------------------------------------------------------------*
       * Map the RX complete interrupt callback function.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pf_rxCallBack = t_conf.pf_rxCallBack;
      pt_uartChanHndl->pf_txCallBack = t_conf.pf_txCallBack;

      /*---------------------------------------------------------------------*
       * Enable RX complete interrupt.
       *---------------------------------------------------------------------*/
      pt_uartChanHndl->pt_uart->CTRLA |= USART_RXCINTLVL_HI_gc;

   }

   HAL_END_CRITICAL();//Enable interrupts

   return UART_PASSED;

}/*End hal_configureUartChannel*/
