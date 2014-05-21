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
 * File Name   : hal_dma.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides an abstraction layer for controlling a
 *               device specific DMA module. The functions herein are 
 *               responsible for arbitrating access to the four XMEGA DMA 
 *               channels. Channel requests are granted on a first-come 
 *               first-serve basis, meaning a user requests access to a 
 *               channel (using hal_requestDmaChannel) and is granted the
 *               next available resource - either channel 0,1,2,or 3. If
 *               successful (DMA_PASSED), a handle is returned
 *               (t_DMAHNDL) which is the primary mechanism for distinguishing
 *               one channel from another for all configuration changes
 *               (hal_configureDmaChannel) and
 *               interrupt requests (hal_requestDmaInterrupt).
 *
 * Last Update : Oct 23, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "hal_dma.h"
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
   DMA_CHAN0_TR_COMP = 0, /*DMA channel 0 transfer complete*/
   DMA_CHAN0_TR_ERROR,
   DMA_CHAN1_TR_COMP,
   DMA_CHAN1_TR_ERROR,
   DMA_CHAN2_TR_COMP,
   DMA_CHAN2_TR_ERROR,
   DMA_CHAN3_TR_COMP,
   DMA_CHAN3_TR_ERROR

}t_dmaIntId;

typedef struct
{
   /*------------------------------------------------------------------------*
    * A unique number referring to one of the 4 possible DMA's
    *------------------------------------------------------------------------*/
   t_dmaChanId t_id;

   /*------------------------------------------------------------------------*
    * If true, then 'configdma' successfully completed
    *------------------------------------------------------------------------*/
   bool b_validConfig;

   /*------------------------------------------------------------------------*
    * Keeps track of the interrupts opened against this handle
    *------------------------------------------------------------------------*/
   uint8_t c_intCount;

   /*------------------------------------------------------------------------*
    * Pointer to the DMA channel for this particular handle
    *------------------------------------------------------------------------*/
   DMA_CH_t *pt_dma;

}t_dmaChan;

typedef struct
{

   /*------------------------------------------------------------------------*
    * A unique number referring to one of the 32 possible DMA interrupts
    *------------------------------------------------------------------------*/
   t_dmaIntId t_id;

   /*------------------------------------------------------------------------*
    * Pointer to the call-back function
    *------------------------------------------------------------------------*/
   void (*pf_funPtr)( void);

}t_dmaIntHndl;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_LINKHNDL createIntHandle( void);

static t_LINKHNDL createDmaHandle( t_dmaChanId t_id);

static t_dmaIntHndl *findDmaIntElement( t_dmaIntId t_id);

static t_dmaChan *findDmaChanElement( t_dmaChanId t_id);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * After bootup, the DMA controller needs to be configured on the very first
 * request for a DMA handle (hal_requestDmaChannel)
 *---------------------------------------------------------------------------*/
static bool gb_configureDmaController = true;

/*---------------------------------------------------------------------------*
 * Linked-list of all the currently active DMA channels.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_dmaChanList);

/*---------------------------------------------------------------------------*
 * Linked-list of currently active DMA interrupts.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_dmaIntHndlList);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
ISR( DMA_CH0_vect)
{
   t_dmaIntHndl *pt_iElement;

   if( DMA.CH0.CTRLB & DMA_CH_ERRIF_bm)
   {
      DMA.CH0.CTRLB |= DMA_CH_ERRIF_bm;
      /*---------------------------------------------------------------------*
       * See if there is a handle on the list for this dma interrupt.
       *---------------------------------------------------------------------*/
      pt_iElement = findDmaIntElement( (uint8_t)DMA_CHAN0_TR_ERROR);
   }
   else
   {
      DMA.CH0.CTRLB |= DMA_CH_TRNIF_bm;
      /*---------------------------------------------------------------------*
       * See if there is a handle on the list for this dma interrupt.
       *---------------------------------------------------------------------*/
      pt_iElement = findDmaIntElement( (uint8_t)DMA_CHAN0_TR_COMP);

   }

   if( pt_iElement != NULL)
   {
      if( pt_iElement->pf_funPtr != NULL)
         pt_iElement->pf_funPtr();
   }

}/*End ISR( DMA_CH0_vect)*/

ISR( DMA_CH1_vect)
{
   t_dmaIntHndl *pt_iElement;

   if( DMA.CH1.CTRLB & DMA_CH_ERRIF_bm)
   {
      DMA.CH1.CTRLB |= DMA_CH_ERRIF_bm;
      /*---------------------------------------------------------------------*
       * See if there is a handle on the list for this dma interrupt.
       *---------------------------------------------------------------------*/
      pt_iElement = findDmaIntElement( (uint8_t)DMA_CHAN1_TR_ERROR);
   }
   else
   {
      DMA.CH1.CTRLB |= DMA_CH_TRNIF_bm;
      /*---------------------------------------------------------------------*
       * See if there is a handle on the list for this dma interrupt.
       *---------------------------------------------------------------------*/
      pt_iElement = findDmaIntElement( (uint8_t)DMA_CHAN1_TR_COMP);

   }

   if( pt_iElement != NULL)
   {
      if( pt_iElement->pf_funPtr != NULL)
         pt_iElement->pf_funPtr();
   }

}/*End ISR( DMA_CH1_vect)*/

ISR( DMA_CH2_vect)
{
   t_dmaIntHndl *pt_iElement;

   if( DMA.CH2.CTRLB & DMA_CH_ERRIF_bm)
   {
      DMA.CH2.CTRLB |= DMA_CH_ERRIF_bm;
      /*---------------------------------------------------------------------*
       * See if there is a handle on the list for this dma interrupt.
       *---------------------------------------------------------------------*/
      pt_iElement = findDmaIntElement( (uint8_t)DMA_CHAN2_TR_ERROR);
   }
   else
   {
      DMA.CH2.CTRLB |= DMA_CH_TRNIF_bm;
      /*---------------------------------------------------------------------*
       * See if there is a handle on the list for this dma interrupt.
       *---------------------------------------------------------------------*/
      pt_iElement = findDmaIntElement( (uint8_t)DMA_CHAN2_TR_COMP);

   }

   if( pt_iElement != NULL)
   {
      if( pt_iElement->pf_funPtr != NULL)
         pt_iElement->pf_funPtr();
   }

}/*End ISR( DMA_CH2_vect)*/

ISR( DMA_CH3_vect)
{
   t_dmaIntHndl *pt_iElement;

   if( DMA.CH3.CTRLB & DMA_CH_ERRIF_bm)
   {
      DMA.CH3.CTRLB |= DMA_CH_ERRIF_bm;
      /*---------------------------------------------------------------------*
       * See if there is a handle on the list for this dma interrupt.
       *---------------------------------------------------------------------*/
      pt_iElement = findDmaIntElement( (uint8_t)DMA_CHAN3_TR_ERROR);
   }
   else
   {
      DMA.CH3.CTRLB |= DMA_CH_TRNIF_bm;
      /*---------------------------------------------------------------------*
       * See if there is a handle on the list for this dma interrupt.
       *---------------------------------------------------------------------*/
      pt_iElement = findDmaIntElement( (uint8_t)DMA_CHAN3_TR_COMP);

   }

   if( pt_iElement != NULL)
   {
      if( pt_iElement->pf_funPtr != NULL)
         pt_iElement->pf_funPtr();
   }

}/*End ISR( DMA_CH3_vect)*/

static t_dmaError hal_configureDmaController( t_dmaCntrlConfig t_conf)
{

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Disable module
    *------------------------------------------------------------------------*/
   DMA.CTRL &= ~DMA_ENABLE_bm;

   if( (t_conf.t_buffMode < DOUBLE_BUFF_DISABLED) ||
       (t_conf.t_buffMode > DOUBLE_BUFF_ALL_CHAN))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_BUFF_MODE;
   }

   if( (t_conf.t_chanPriority < ROUND_ROBIN) ||
       (t_conf.t_chanPriority > CHAN_0_1_2_3))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_CHAN_PRIORITY;
   }

   /*------------------------------------------------------------------------*
    * Apply configuration
    *------------------------------------------------------------------------*/
   DMA.CTRL = (DMA.CTRL & ~DMA_DBUFMODE_gm ) |
   (DMA_DBUFMODE_t)t_conf.t_buffMode;
   DMA.CTRL = (DMA.CTRL & ~DMA_PRIMODE_gm ) |
   (DMA_PRIMODE_t)t_conf.t_chanPriority;

   /*------------------------------------------------------------------------*
    * Enable module
    *------------------------------------------------------------------------*/
   DMA.CTRL |= DMA_ENABLE_bm;

   HAL_END_CRITICAL();//Enable interrupts

   return DMA_PASSED;

}/*End hal_configureDmaController*/

static t_dmaIntHndl *findDmaIntElement( t_dmaIntId t_id)
{
   t_dmaIntHndl *pt_element;
   t_LINKHNDL t_linkHndl;
   int16_t s_count;

   /*------------------------------------------------------------------------*
    * Search the DMA interrupt list for the requested ID
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_dmaIntHndlList, s_count)
   {
      pt_element = (t_dmaIntHndl *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( pt_element->t_id == t_id)
      {
         return pt_element;
      }

   }

   /*------------------------------------------------------------------------*
    * If we make it this far the ID has not been found in the open DMA
    * interrupt list.
    *------------------------------------------------------------------------*/
   return NULL;

}/*End findDmaIntElement*/

static t_dmaChan *findDmaChanElement( t_dmaChanId t_id)
{
   t_dmaChan *pt_element;
   t_LINKHNDL t_linkHndl;
   int16_t s_count;

   /*------------------------------------------------------------------------*
    * Search the DMA channel list for the requested ID
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_dmaChanList, s_count)
   {
      pt_element = (t_dmaChan *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( pt_element->t_id == t_id)
      {
         return pt_element;
      }

   }

   /*------------------------------------------------------------------------*
    * If we make it this far the ID has not been found on the open DMA
    * channel list.
    *------------------------------------------------------------------------*/
   return NULL;

}/*End findDmaChanElement*/

static t_LINKHNDL createIntHandle( void)
{
   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Allocated memory for the link where the DMA interrupt information will be
    * stored.
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof(t_dmaIntHndl));

   if( t_linkHndl < 0)
   {
      return (t_LINKHNDL)DMA_OUT_OF_HEAP;
   }

   /*------------------------------------------------------------------------*
    * Add the DMA interrupt link onto the list open DMA interrupt.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_dmaIntHndlList,
                           t_linkHndl,
                           true);

   return t_linkHndl;

}/*End createIntHandle*/

static t_LINKHNDL createDmaHandle( t_dmaChanId t_id)
{
   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;
   t_dmaChan *pt_element;

   /*------------------------------------------------------------------------*
    * Allocated memory for the link where the DMA channel information will be
    * stored.
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof(t_dmaChan));

   if( t_linkHndl < 0)
   {
      return (t_LINKHNDL)DMA_OUT_OF_HEAP;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the DMA channel
    * information is being stored.
    *------------------------------------------------------------------------*/
   pt_element = (t_dmaChan *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);

   pt_element->b_validConfig = false;
   pt_element->t_id          = t_id;
   pt_element->c_intCount    = 0;

   switch( t_id)
   {
      case DMA_CHAN_0:
         pt_element->pt_dma = &DMA.CH0;
      break;

      case DMA_CHAN_1:
         pt_element->pt_dma = &DMA.CH1;
      break;

      case DMA_CHAN_2:
         pt_element->pt_dma = &DMA.CH2;
      break;

      case DMA_CHAN_3:
         pt_element->pt_dma = &DMA.CH3;
      break;

   }/*End switch( t_chanId)*/

   /*------------------------------------------------------------------------*
    * Add the DMA channel link onto the list open channels.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_dmaChanList,
                           t_linkHndl,
                           true);

   return t_linkHndl;

}/*End createDmaHandle*/

t_dmaError hal_requestDmaInterrupt( t_DMAHNDL t_handle,
                                    t_dmaIntType  t_type,
                                    void (*pf_funPtr)( void))
{
   t_dmaIntHndl *pt_iElement;
   t_dmaChan *pt_dElement;
   t_dmaIntId t_intId;
   t_LINKHNDL t_linkHndl;

   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA 
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Use the dma module id and interrupt type as an index into the list
       * of possible interrupts.
       *---------------------------------------------------------------------*/
      t_intId = (t_dmaIntId)pt_dElement->t_id + (t_dmaIntId)t_type;

      /*---------------------------------------------------------------------*
       * Is the requested interrupt within a valid range?
       *---------------------------------------------------------------------*/
      if( (t_intId < DMA_CHAN0_TR_COMP) || (t_intId > DMA_CHAN3_TR_ERROR))
      {
         HAL_END_CRITICAL();//Enable interrupts
         return DMA_INVALID_INT_TYPE;
      }

      pt_iElement = findDmaIntElement( t_intId);

      /*---------------------------------------------------------------------*
       * Is there already and open handle for this interrupt?
       *---------------------------------------------------------------------*/
      if( pt_iElement == NULL) /*No*/
      {
         /*------------------------------------------------------------------*
          * Allocate a new link member for this particular DMA interrupt.
          *------------------------------------------------------------------*/
         t_linkHndl = createIntHandle();
         if( t_linkHndl < 0)
         {
            HAL_END_CRITICAL();//Enable interrupts
            return DMA_OUT_OF_HEAP;
         }

         /*------------------------------------------------------------------*
          * Get a ptr to the link's element- which is the area where the DMA
          * interrupt information is being stored.
          *------------------------------------------------------------------*/
         pt_iElement = (t_dmaIntHndl *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);

         pt_iElement->t_id      = t_intId;
         pt_iElement->pf_funPtr = pf_funPtr;

         /*------------------------------------------------------------------*
          * Keep track of the number of interrupts attached to this channel.
          *------------------------------------------------------------------*/
         pt_dElement->c_intCount++;

         /*------------------------------------------------------------------*
          * Enable interrupt.
          *------------------------------------------------------------------*/
         if( t_type == DMA_TRANSFER_COMPLETE)
            pt_dElement->pt_dma->CTRLB |= DMA_CH_TRNINTLVL_gm;
         else
            pt_dElement->pt_dma->CTRLB |= DMA_CH_ERRINTLVL_gm;

      }/*End if( pt_iElement == NULL)*/
      else /*Yes*/
      {
         HAL_END_CRITICAL();//Enable interrupts
         return DMA_INTERRUPT_OPEN;
      }

   }

   HAL_END_CRITICAL();//Enable interrupts

   return DMA_PASSED;

}/*End hal_requestDmaInterrupt*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
t_dmaError hal_releaseDmaInterrupt( t_DMAHNDL t_handle,
                                    t_dmaIntType t_type)
{
   static t_LINKHNDL t_linkHndl;
   t_dmaChan *pt_dElement;
   t_dmaIntHndl *pt_iElement;
   t_dmaIntId t_intId;
   int16_t s_count;
   t_linkedListError t_lErr;

   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Use the dma module id and interrupt type as an index into the list
       * of possible interrupts.
       *---------------------------------------------------------------------*/
      t_intId = (t_dmaIntId)pt_dElement->t_id + (t_dmaIntId)t_type;

      /*---------------------------------------------------------------------*
       * Search the DMA interrupt list for the requested ID
       *---------------------------------------------------------------------*/
      UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_dmaIntHndlList, s_count)
      {
         pt_iElement = (t_dmaIntHndl *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
         if( pt_iElement->t_id == t_intId)
         {
            pt_dElement->c_intCount--;
            if( t_type == DMA_TRANSFER_COMPLETE)
               pt_dElement->pt_dma->CTRLB &= ~DMA_CH_TRNINTLVL_gm;
            else
               pt_dElement->pt_dma->CTRLB &= ~DMA_CH_ERRINTLVL_gm;

            t_lErr = utl_destroyLink( gt_dmaIntHndlList,
                                      t_linkHndl);

            HAL_END_CRITICAL();//Enable interrupts
            return DMA_PASSED;

         }

      }

      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_INT_TYPE;

   }

}/*End hal_releaseDmaInterrupt*/

/*---------------------------------------------------------------------------*
 * Request access to a particular dma module
 *---------------------------------------------------------------------------*/
t_DMAHNDL hal_requestDmaChannel( void)
{
   t_dmaChan *pt_dmaChan;
   t_LINKHNDL t_linkHndl;
   t_dmaChanId t_chanId;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( gb_configureDmaController == true)
   {
      t_dmaCntrlConfig t_cntrlConf;

      t_cntrlConf.t_buffMode = DOUBLE_BUFF_DISABLED;
      
      /*---------------------------------------------------------------------*
       * Channel priority is needed in order for the SPI module to operate
       * properly when using DMA for both the TX and RX transactions, as RX 
       * operations need to happen first.
       *---------------------------------------------------------------------*/
      t_cntrlConf.t_chanPriority = CHAN_0_1_2_3;

      if( hal_configureDmaController( t_cntrlConf) < 0)
         exit(0); /*This should not happen*/

      gb_configureDmaController = false;
   }

   /*------------------------------------------------------------------------*
    * Search for next available DMA channel.
    *------------------------------------------------------------------------*/
   for( t_chanId = DMA_CHAN_0; t_chanId <= DMA_CHAN_3; t_chanId+=2)
   {
      pt_dmaChan = findDmaChanElement( t_chanId);
      if( pt_dmaChan == NULL) /*Not open*/
         break;
   }

   /*------------------------------------------------------------------------*
    * Is there a channel available?
    *------------------------------------------------------------------------*/
   if( pt_dmaChan == NULL) /*Yes*/
   {
      t_linkHndl = createDmaHandle( t_chanId);
      if( t_linkHndl < 0)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return (t_DMAHNDL)DMA_OUT_OF_HEAP;

      }/*End if( t_linkHndl < 0)*/

   }
   else /*No*/
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_DMAHNDL)DMA_NO_CHANNELS_OPEN;
   }

   HAL_END_CRITICAL();//Enable interrupts

   return (t_DMAHNDL)t_linkHndl;

}/*End hal_requestDmaChannel*/

t_dmaError hal_releaseDmaChannel( t_DMAHNDL t_handle)
{
   static t_LINKHNDL t_linkHndl;
   static t_LINKHNDL t_prevLinkHndl;
   t_dmaChan *pt_dElement;
   t_dmaIntHndl *pt_iElement;
   t_dmaIntId t_intId;
   t_dmaIntType t_type;
   int16_t s_count;
   t_linkedListError t_lErr;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Remove all the interrupts associated with this particular DMA channel.
       * Since UTL_TRAVERSE_CONTAINER traverses through the entire interrupt
       * list changing the value of t_linkHndl on each iteration, once
       * 't_linkHndl' has been deleted information about the next position on
       * the list is lost.  This issue is resolved by using a previous ptr to
       * change 't_linkHndl' back to a valid location once an item has been
       * removed.
       *---------------------------------------------------------------------*/
      for( t_type = DMA_TRANSFER_COMPLETE; t_type <= DMA_TRANSFER_ERROR;
      t_type++)
      {
         /*------------------------------------------------------------------*
          * Use the DMA channel id and interrupt type as an index into the
          * list of possible interrupts.
          *------------------------------------------------------------------*/
         t_intId = (t_dmaIntId)pt_dElement->t_id + (t_dmaIntId)t_type;

         /*------------------------------------------------------------------*
          * Search the DMA interrupt list for the requested ID
          *------------------------------------------------------------------*/
         UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_dmaIntHndlList, s_count)
         {
            pt_iElement = (t_dmaIntHndl *)
            UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
            t_prevLinkHndl = UTL_GET_PREV_LINK( t_linkHndl);
            if( pt_iElement->t_id == t_intId)
            {
               pt_dElement->c_intCount--;
               if( t_type == DMA_TRANSFER_COMPLETE)
                  pt_dElement->pt_dma->CTRLB &= ~DMA_CH_TRNINTLVL_gm;
               else
                  pt_dElement->pt_dma->CTRLB &= ~DMA_CH_ERRINTLVL_gm;

               t_lErr = utl_destroyLink( gt_dmaIntHndlList,
                                         t_linkHndl);

               t_linkHndl = t_prevLinkHndl;

            }
         }

      }/*End for( t_type = DMA_TRANSFER_COMPLETE; t_type <= DMA_TRANSFER_ERROR;
      t_type++)*/

      /*---------------------------------------------------------------------*
       * Disable the channel.
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->CTRLA &= ~DMA_CH_ENABLE_bm;

      /*---------------------------------------------------------------------*
       * Reset all registers to default values
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->CTRLA &= ~DMA_CH_ENABLE_bm;
      pt_dElement->pt_dma->CTRLA |= DMA_CH_RESET_bm;
      pt_dElement->pt_dma->CTRLA &= ~DMA_CH_RESET_bm;

      t_lErr = utl_destroyLink( gt_dmaChanList,
                                (t_LINKHNDL)t_handle);

   }

   HAL_END_CRITICAL();//Enable interrupts

   return DMA_PASSED;

}/*End hal_releaseDmaChannel*/

t_dmaError hal_setDmaBlockSize( t_DMAHNDL t_handle,
                                uint16_t s_blockSize)
{
   t_dmaChan *pt_dElement;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( s_blockSize == 0)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_BLOCK_SIZE;
   }

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Disable the DMA channel
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->CTRLA &= ~DMA_CH_ENABLE_bm;

      /*---------------------------------------------------------------------*
       * Configure the block size
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->TRFCNT = s_blockSize;

   }

   HAL_END_CRITICAL();//Enable interrupts

   return DMA_PASSED;

}/*End hal_setDmaBlockSize*/

t_dmaError hal_setDmaSourceAddress( t_DMAHNDL t_handle,
                                    uint32_t i_address)
{
   t_dmaChan *pt_dElement;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Disable the DMA channel
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->CTRLA &= ~DMA_CH_ENABLE_bm;

      pt_dElement->pt_dma->SRCADDR0 = (i_address >> 0)  & 0xFF;
      pt_dElement->pt_dma->SRCADDR1 = (i_address >> 8)  & 0xFF;
      pt_dElement->pt_dma->SRCADDR2 = (i_address >> 16) & 0xFF;

   }

   HAL_END_CRITICAL();//Enable interrupts

   return DMA_PASSED;

}/*End hal_setDmaSourceAddress*/

t_dmaError hal_setDmaDestAddress( t_DMAHNDL t_handle,
                                  uint32_t i_address)
{
   t_dmaChan *pt_dElement;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Disable the DMA channel
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->CTRLA &= ~DMA_CH_ENABLE_bm;

      pt_dElement->pt_dma->DESTADDR0 = (i_address >> 0)  & 0xFF;
      pt_dElement->pt_dma->DESTADDR1 = (i_address >> 8)  & 0xFF;
      pt_dElement->pt_dma->DESTADDR2 = (i_address >> 16) & 0xFF;

   }

   HAL_END_CRITICAL();//Enable interrupts

   return DMA_PASSED;

}/*End hal_setDmaDestAddress*/

int16_t hal_getDmaStatus( t_DMAHNDL t_handle)
{
   t_dmaChan *pt_dElement;
   int16_t s_status = 0; /*0 - inactive, 1 = busy, -1 = error*/

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      return (int16_t)DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Is there an ongoing transaction?
       *---------------------------------------------------------------------*/
      s_status = 0x01 & ((pt_dElement->pt_dma->CTRLB & DMA_CH_CHBUSY_bm)
      >> DMA_CH_CHBUSY_bp);

   }

   return s_status;

}/*End hal_getDmaStatus*/

uint16_t hal_getDmaTransferCount( t_DMAHNDL t_handle)
{
   t_dmaChan *pt_dElement;
   uint16_t s_count = 0;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      return (int16_t)DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      s_count = pt_dElement->pt_dma->TRFCNT;

   }

   return s_count;

}/*End hal_getDmaTransferCount*/

t_dmaError hal_configureDmaChannel( t_DMAHNDL t_handle,
                                    t_dmaChanConfig t_conf)
{
   t_dmaChan *pt_dElement;
   int8_t *pc_src;
   int8_t *pc_dst;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( (t_conf.t_srcAddDir < FIXED) || (t_conf.t_srcAddDir >
   DECREMENT))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_ADDRESS_MODE;
   }

   if( (t_conf.t_destAddDir< FIXED) || (t_conf.t_destAddDir >
   DECREMENT))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_ADDRESS_MODE;
   }

   if( (t_conf.t_srcAddReload < NO_RELOAD) || (t_conf.t_srcAddReload >
   RELOAD_END_OF_TRANSACTION))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_ADD_RELOAD_MODE;
   }

   if( (t_conf.t_destAddReload < NO_RELOAD) || (t_conf.t_destAddReload >
   RELOAD_END_OF_TRANSACTION))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_ADD_RELOAD_MODE;
   }

   if( t_conf.s_blockSize == 0)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_BLOCK_SIZE;
   }

   if( (t_conf.t_burstMode < ONE_BYTE) || (t_conf.t_burstMode >
   EIGHT_BYTE))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_BURST_MODE;
   }

   if( (t_conf.t_transferType < BLOCK) || (t_conf.t_transferType >
   SINGLE_SHOT))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_TRANSFER_TYPE;
   }

   if( (t_conf.t_triggerSrc < SOFTWARE) || (t_conf.t_triggerSrc >
   UART8_DATA_REG_EMPTY))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_TRIGGER_SOURCE;
   }

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Disable the DMA channel
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->CTRLA &= ~DMA_CH_ENABLE_bm;

      pc_src = (int8_t *)&t_conf.pi_srcAddress;
      pc_dst = (int8_t *)&t_conf.pi_destAddress;

      /*---------------------------------------------------------------------*
       * Apply configuration
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->SRCADDR0 = (uint16_t)pc_src[0];
      pt_dElement->pt_dma->SRCADDR1 = (uint16_t)pc_src[1];

      /*---------------------------------------------------------------------*
       * Todo...this will need to be fixed if someone is attempting to 
       * access memory outside of a 16-bit memory range...
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->SRCADDR2 = 0;//(uint16_t)pc_src[2];

      pt_dElement->pt_dma->DESTADDR0 = (uint16_t)pc_dst[0];
      pt_dElement->pt_dma->DESTADDR1 = (uint16_t)pc_dst[1];

      /*---------------------------------------------------------------------*
       * Todo...this will need to be fixed if someone is attempting to 
       * access memory outside of a 16-bit memory range...
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->DESTADDR2 = 0;//(uint16_t)pc_dst[2];

      /*---------------------------------------------------------------------*
       * Configure the direction
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->ADDRCTRL &= ~DMA_CH_SRCDIR_gm;
      pt_dElement->pt_dma->ADDRCTRL |= (t_conf.t_srcAddDir << 4);

      pt_dElement->pt_dma->ADDRCTRL &= ~DMA_CH_DESTDIR_gm;
      pt_dElement->pt_dma->ADDRCTRL |= (t_conf.t_destAddDir << 0);

      /*---------------------------------------------------------------------*
       * Configure whether or not to reload.
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->ADDRCTRL &= ~DMA_CH_SRCRELOAD_gm;
      pt_dElement->pt_dma->ADDRCTRL |= (t_conf.t_srcAddReload << 6);

      pt_dElement->pt_dma->ADDRCTRL &= ~DMA_CH_DESTRELOAD_gm;
      pt_dElement->pt_dma->ADDRCTRL |= (t_conf.t_destAddReload << 2);

      /*---------------------------------------------------------------------*
       * Configure the block size
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->TRFCNT = t_conf.s_blockSize;

      /*---------------------------------------------------------------------*
       * Configure the channel transfer mode.
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->CTRLA &= ~DMA_CH_BURSTLEN_gm;
      pt_dElement->pt_dma->CTRLA |= t_conf.t_burstMode;

      /*---------------------------------------------------------------------*
       * Configure repeat mode
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->CTRLA &= ~DMA_CH_REPEAT_bm;
      pt_dElement->pt_dma->REPCNT = 0;
      if( t_conf.c_repeatCount > 0)
      {
         /*Fixed number of repeats*/
         pt_dElement->pt_dma->CTRLA |= DMA_CH_REPEAT_bm;
         pt_dElement->pt_dma->REPCNT = t_conf.c_repeatCount;

      }
      else if( t_conf.c_repeatCount < 0)
      {
         /*Infinite repeat*/
         pt_dElement->pt_dma->CTRLA |= DMA_CH_REPEAT_bm;
      }

      /*---------------------------------------------------------------------*
       * Configure the data transfer type
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->CTRLA &= ~DMA_CH_SINGLE_bm;
      pt_dElement->pt_dma->CTRLA |= t_conf.t_transferType << 2;

      /*---------------------------------------------------------------------*
       * Configure the DMA trigger source
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->TRIGSRC &= ~DMA_CH_TRIGSRC_gm;
      pt_dElement->pt_dma->TRIGSRC |= t_conf.t_triggerSrc;

      pt_dElement->b_validConfig = true;

   }

   HAL_END_CRITICAL();//Enable interrupts

   return DMA_PASSED;

}/*End hal_configureDmaChannel*/

void hal_resetDmaController( void)
{

   DMA.CTRL |= DMA_RESET_bm;

   /*------------------------------------------------------------------------*
    * Wait for the module to finish reseting.
    *------------------------------------------------------------------------*/
   while (DMA.CTRL & DMA_RESET_bm)
   {

   }/*End while (DMA.CTRL & DMA_RESET_bm)*/

}/*End hal_resetDmaController*/

t_dmaError hal_dmaStartTransfer( t_DMAHNDL t_handle)
{
   t_dmaChan *pt_dElement;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Has the channel been configured?
       *---------------------------------------------------------------------*/
      if( pt_dElement->b_validConfig == true)
      {

         /*------------------------------------------------------------------*
          * Make sure the channel is enabled.
          *------------------------------------------------------------------*/
         pt_dElement->pt_dma->CTRLA |= DMA_CH_ENABLE_bm;

         /*------------------------------------------------------------------*
          * Manually start the transfer.
          *------------------------------------------------------------------*/
         pt_dElement->pt_dma->CTRLA |= DMA_CH_TRFREQ_bm;
      }
      else
      {
         HAL_END_CRITICAL();//Enable interrupts
         return DMA_NO_CONFIG;
      }
   }

   HAL_END_CRITICAL();//Enable interrupts

   return DMA_PASSED;

}/*End hal_dmaStartTransfer*/

int16_t hal_getDmaIntStatus( t_DMAHNDL t_handle,
                             t_dmaIntType t_int)
{
   volatile t_dmaChan *pt_dElement;
   uint16_t s_status = 0;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      return (int16_t)DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( t_int == DMA_TRANSFER_COMPLETE)
      {
         s_status = 0x0001 & ((pt_dElement->pt_dma->CTRLB & DMA_CH_TRNIF_bm)
         >> DMA_CH_TRNIF_bp);
      }
      else
      {
         s_status = 0x0001 & ((pt_dElement->pt_dma->CTRLB & DMA_CH_ERRIF_bm)
         >> DMA_CH_ERRIF_bp);
      }
   }

   return s_status;

}/*End hal_getDmaIntStatus*/

/*------------------------------------------------------------------------*
   * Returns the channel Id for a particular DMA pointed to by 't_handle'.
   *------------------------------------------------------------------------*/
t_dmaChanId hal_getDmaChannelId( t_DMAHNDL t_handle)
{
   volatile t_dmaChan *pt_dElement;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      return (int16_t)DMA_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the DMA
    * channel information is being stored.
    *------------------------------------------------------------------------*/
   pt_dElement = (t_dmaChan *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

   return pt_dElement->t_id;

}/*End hal_getDmaChannelId*/

t_dmaError hal_clearDmaIntStatus( t_DMAHNDL t_handle,
                                  t_dmaIntType t_int)
{
   t_dmaChan *pt_dElement;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      return DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_dElement->t_id == DMA_CHAN_0)
      {
         if( t_int == DMA_TRANSFER_COMPLETE)
         {
            DMA.INTFLAGS |= DMA_CH0TRNIF_bm;
         }
         else
         {
            DMA.INTFLAGS |= DMA_CH0ERRIF_bm;
         }
      }
      else if( pt_dElement->t_id == DMA_CHAN_1)
      {
         if( t_int == DMA_TRANSFER_COMPLETE)
         {
            DMA.INTFLAGS |= DMA_CH1TRNIF_bm;
         }
         else
         {
            DMA.INTFLAGS |= DMA_CH1ERRIF_bm;
         }
      }
      else if( pt_dElement->t_id == DMA_CHAN_2)
      {
         if( t_int == DMA_TRANSFER_COMPLETE)
         {
            DMA.INTFLAGS |= DMA_CH2TRNIF_bm;
         }
         else
         {
            DMA.INTFLAGS |= DMA_CH2ERRIF_bm;
         }
      }
      else
      {
         if( t_int == DMA_TRANSFER_COMPLETE)
         {
            DMA.INTFLAGS |= DMA_CH3TRNIF_bm;
         }
         else
         {
            DMA.INTFLAGS |= DMA_CH3ERRIF_bm;
         }
      }
   }

   return DMA_PASSED;

}/*End hal_clearDmaIntStatus*/

t_dmaError hal_dmaEnableChannel( t_DMAHNDL t_handle)
{
   t_dmaChan *pt_dElement;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Enable the DMA channel
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->CTRLA |= DMA_CH_ENABLE_bm;

   }

   HAL_END_CRITICAL();//Enable interrupts

   return DMA_PASSED;

}/*End hal_dmaEnableChannel*/

t_dmaError hal_dmaDisableChannel( t_DMAHNDL t_handle)
{
   t_dmaChan *pt_dElement;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_dmaChanList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return DMA_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_dElement = (t_dmaChan *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Disable the DMA channel
       *---------------------------------------------------------------------*/
      pt_dElement->pt_dma->CTRLA &= ~DMA_CH_ENABLE_bm;

   }

   HAL_END_CRITICAL();//Enable interrupts

   return DMA_PASSED;

}/*End hal_dmaDisableChannel*/

