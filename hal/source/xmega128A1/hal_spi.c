/*---------------------------------------------------------------------------*
 * Copyright (C) 2013 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * File Name   : hal_spi.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for controlling
 *               the SPI module.
 *
 * Last Update : June 9, 2013
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "hal_spi.h"
#include "hal_clocks.h"
#include "utl_linkedlist.h"
#include "hal_gpio.h"
#include "hal_pmic.h"
#include "hal_dma.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct
{
   /*------------------------------------------------------------------------*
    * A unique number referring to one of the 4 possible spi channels
    *------------------------------------------------------------------------*/
   t_spiChanId t_id;

   /*------------------------------------------------------------------------*
    * The current buad rate for the spi connected to this handle
    *------------------------------------------------------------------------*/
   int32_t i_baudRate;

   /*------------------------------------------------------------------------*
    * The number of users connected to this particular spi channel.
    *------------------------------------------------------------------------*/
    uint8_t c_numUsers;

   /*------------------------------------------------------------------------*
    * The configuration of this particular spi channel - either master or
     * slave.
    *------------------------------------------------------------------------*/
    t_spiOperation t_spiOp;

   /*------------------------------------------------------------------------*
    * Pointer to the spi channel connected to this particular handle
    *------------------------------------------------------------------------*/
   SPI_t *pt_spi;

   /*------------------------------------------------------------------------*
    * If true, there is a transaction in progress.
    *------------------------------------------------------------------------*/
    bool b_busLocked;

   /*------------------------------------------------------------------------*
    * Points to the user currently in control of this particular spi channel.
    *------------------------------------------------------------------------*/
    void *pt_activeUser;

   /*------------------------------------------------------------------------*
    * Address of the link this 'element' is contained in - used when the
    * 'destroy' command is called.
    *------------------------------------------------------------------------*/
    t_LINKHNDL t_linkHndl;

   /*------------------------------------------------------------------------*
    * Address of the dma channel tied to the transmit buffer of this
    * particular spi channel.
    *------------------------------------------------------------------------*/
   t_DMAHNDL t_txDmaHndl;

   /*------------------------------------------------------------------------*
    * Address of the dma channel tied to the receive buffer of this
    * particular spi channel.
    *------------------------------------------------------------------------*/
   t_DMAHNDL t_rxDmaHndl;

}t_spiChanHndl;

typedef struct
{

   /*------------------------------------------------------------------------*
    * Pointer to the interrupt call-back function
    *------------------------------------------------------------------------*/
   void (*pf_funPtr)( int8_t *pc_data,
                      uint16_t s_length);

   /*------------------------------------------------------------------------*
    * Pointer to the spi tx buffer for the user connected to this channel.
    *------------------------------------------------------------------------*/
   int8_t *pc_txData;

   /*------------------------------------------------------------------------*
    * Pointer to the spi rx buffer for the user connected to this channel.
    *------------------------------------------------------------------------*/
   int8_t *pc_rxData;

   /*------------------------------------------------------------------------*
    * Length of the tx and rx buffers.
    *------------------------------------------------------------------------*/
   uint16_t s_bufLength;

   /*------------------------------------------------------------------------*
    * Current index into the rx or tx buffer.
    *------------------------------------------------------------------------*/
   uint16_t s_bufIndex;

   /*------------------------------------------------------------------------*
    * Pointer to the chip select port for the user connected to this channel.
    *------------------------------------------------------------------------*/
   t_gpioPort t_csPort;

   /*------------------------------------------------------------------------*
    * Pointer to the chip select pin for the user connected to this channel.
    *------------------------------------------------------------------------*/
    uint8_t c_csPin;

   /*------------------------------------------------------------------------*
    * Pointer to the particular spi channel this user handle is connected to.
    *------------------------------------------------------------------------*/
    t_spiChanHndl *pt_spiChanHndl;

   /*------------------------------------------------------------------------*
    * Handle to a particular gpio interrupt used for knowing when a chip
    * select has transitioned from negative to positive when a particular
    * spi channel is configured for slave operation.
    *------------------------------------------------------------------------*/
   t_GPIOHNDL t_csIntHndl;

   /*------------------------------------------------------------------------*
    * If true, this module autonomously controls the chip select pin.
    *------------------------------------------------------------------------*/
   bool b_enCs;

}t_spiUserHndl;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_LINKHNDL createSpiHandle( void);
static t_LINKHNDL createSpiUserHandle( void);
static t_spiChanHndl *findSpiElement( t_spiChanId t_id);
static bool updateBuffersMast( t_spiUserHndl *pt_handle);
static bool updateBuffersSlave( t_spiUserHndl *pt_handle);
static void slaveSpiCChipSelectInt( t_gpioPort t_port,
                                    uint8_t c_pin);
static void slaveSpiDChipSelectInt( t_gpioPort t_port,
                                    uint8_t c_pin);
static void slaveSpiEChipSelectInt( t_gpioPort t_port,
                                    uint8_t c_pin);
static void slaveSpiFChipSelectInt( t_gpioPort t_port,
                                    uint8_t c_pin);
static void masterSpiCRxDmaInt( void);
static void masterSpiDRxDmaInt( void);
static void masterSpiERxDmaInt( void);
static void masterSpiFRxDmaInt( void);
static void processSlaveSpiChipSelectInt( t_spiChanHndl *pt_chanHndl,
                                          t_spiUserHndl *pt_activeUser);
static void processMasterSpiRxDmaInt( t_spiChanHndl *pt_chanHndl,
                                      t_spiUserHndl *pt_activeUser);
static void processSpiInterrupt( t_spiChanHndl *pt_chanHndl,
                                 t_spiUserHndl *pt_activeUser);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
static uint8_t gc_tempBuf = 0xFF;

/*---------------------------------------------------------------------------*
 * List of currently active spi modules.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_spiChanHndlList);

/*---------------------------------------------------------------------------*
 * List of users attached to the spi modules.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_spiUserHndlList);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static bool updateBuffersMast( t_spiUserHndl *pt_handle)
{
    int8_t *pc_rxBuf = pt_handle->pc_rxData;
    int8_t *pc_txBuf = pt_handle->pc_txData;
    uint16_t *ps_bufIndex = &pt_handle->s_bufIndex;

    /*------------------------------------------------------------------------*
     * Store any received bytes...
     *------------------------------------------------------------------------*/
   if( pc_rxBuf != (int8_t *)&gc_tempBuf)
   {
       pc_rxBuf[(*ps_bufIndex)] = pt_handle->pt_spiChanHndl->pt_spi->DATA;
    }
   else
      pc_rxBuf[0] = pt_handle->pt_spiChanHndl->pt_spi->DATA;

   (*ps_bufIndex)++;

    /*------------------------------------------------------------------------*
     * Is there more data that needs to be tx'd?
     *------------------------------------------------------------------------*/
    if( (*ps_bufIndex) < pt_handle->s_bufLength)
    {
      if( pc_txBuf != (int8_t *)&gc_tempBuf)
           pt_handle->pt_spiChanHndl->pt_spi->DATA = pc_txBuf[(*ps_bufIndex)];
      else
         pt_handle->pt_spiChanHndl->pt_spi->DATA = pc_txBuf[0];

        return false;
    }

    return true; /*Burst complete*/

}/*End updateBuffersMast*/

static bool updateBuffersSlave( t_spiUserHndl *pt_handle)
{
    int8_t *pc_rxBuf = pt_handle->pc_rxData;
    int8_t *pc_txBuf = pt_handle->pc_txData;
    uint16_t *ps_bufIndex = &pt_handle->s_bufIndex;

    /*------------------------------------------------------------------------*
     * Store any received bytes...
     *------------------------------------------------------------------------*/
   if( pc_rxBuf != (int8_t *)&gc_tempBuf)
   {
      pc_rxBuf[(*ps_bufIndex)] = pt_handle->pt_spiChanHndl->pt_spi->DATA;
   }
   else
      pc_rxBuf[0] = pt_handle->pt_spiChanHndl->pt_spi->DATA;

   if( pc_txBuf != (int8_t *)&gc_tempBuf)
        pt_handle->pt_spiChanHndl->pt_spi->DATA = pc_txBuf[(*ps_bufIndex)];
   else
      pt_handle->pt_spiChanHndl->pt_spi->DATA = pc_txBuf[0];

   (*ps_bufIndex)++;
    /*------------------------------------------------------------------------*
     * Is there more data that needs to be tx'd?
     *------------------------------------------------------------------------*/
    if( (*ps_bufIndex) == pt_handle->s_bufLength)
    {
        return false;
    }

    return true; /*Burst complete*/

}/*End updateBuffersSlave*/

static void processMasterSpiRxDmaInt( t_spiChanHndl *pt_chanHndl,
                                      t_spiUserHndl *pt_activeUser)
{
   /*------------------------------------------------------------------------*
    * Grab the number of bytes transfered.
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Was this a transaction issued by a call to 'hal_spiWriteBlock'?
    *------------------------------------------------------------------------*/
   if( pt_activeUser->pc_rxData == (int8_t *)&gc_tempBuf) /*Yes*/
   {
      /*---------------------------------------------------------------------*
       * Receive buffer was fixed...
       *---------------------------------------------------------------------*/
      pt_activeUser->s_bufIndex = 1;
   }
   else /*No*/
   {
      pt_activeUser->s_bufIndex =
      hal_getDmaTransferCount( pt_chanHndl->t_rxDmaHndl);
   }

   /*------------------------------------------------------------------------*
    * Release spi channel
    *------------------------------------------------------------------------*/
    if( pt_activeUser->b_enCs == true)
   {
      hal_gpioOn( pt_activeUser->t_csPort,
                       pt_activeUser->c_csPin);
   }

   /*------------------------------------------------------------------------*
    * Disable the DMA channel.
    *------------------------------------------------------------------------*/
   hal_dmaDisableChannel( pt_chanHndl->t_rxDmaHndl);

   /*------------------------------------------------------------------------*
    * Execute the call-back function, returning the rx buffer and
    * the size of the data.
    *------------------------------------------------------------------------*/
    if( pt_activeUser->pf_funPtr != NULL)
      pt_activeUser->pf_funPtr( pt_activeUser->pc_rxData,
                                    pt_activeUser->s_bufIndex);

   /*------------------------------------------------------------------------*
    * Release the spi bus.
    *------------------------------------------------------------------------*/
    pt_chanHndl->b_busLocked = false;

   pt_activeUser->s_bufIndex = 0;

}/*End processMasterSpiRxDmaInt*/

static void processSlaveSpiChipSelectInt( t_spiChanHndl *pt_chanHndl,
                                          t_spiUserHndl *pt_activeUser)
{
   pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

   /*------------------------------------------------------------------------*
    * Is the DMA enabled?
    *------------------------------------------------------------------------*/
   if( pt_chanHndl->t_rxDmaHndl > 0)
   {
      /*---------------------------------------------------------------------*
       * Was this a transaction issued by a call to 'hal_spiWriteBlock'?
       *---------------------------------------------------------------------*/
      if( pt_activeUser->pc_rxData == (int8_t *)&gc_tempBuf) /*Yes*/
      {
         pt_activeUser->s_bufIndex =
         hal_getDmaTransferCount( pt_chanHndl->t_rxDmaHndl);
      }
      else
         pt_activeUser->s_bufIndex = 1;

      /*---------------------------------------------------------------------*
       * Disable the DMA channel
       *---------------------------------------------------------------------*/
      hal_dmaDisableChannel( pt_chanHndl->t_rxDmaHndl);

   }/*End if( pt_chanHndl->t_rxDmaHndl > 0)*/

   /*------------------------------------------------------------------------*
    * Release the spi bus.
    *------------------------------------------------------------------------*/
   if( pt_activeUser->s_bufIndex > 0)
   {
      /*---------------------------------------------------------------------*
       * Execute the call-back function.
       *---------------------------------------------------------------------*/
       if( pt_activeUser->pf_funPtr != NULL)
         pt_activeUser->pf_funPtr( pt_activeUser->pc_rxData,
                                       pt_activeUser->s_bufIndex);

      /*---------------------------------------------------------------------*
       * Release the spi bus.
       *---------------------------------------------------------------------*/
      pt_chanHndl->b_busLocked = false;
      pt_activeUser->s_bufIndex = 0;

   }/*End if( pt_activeUser->s_bufIndex > 0)*/

}/*End processSlaveSpiChipSelectInt*/

static void processSpiInterrupt( t_spiChanHndl *pt_chanHndl,
                                 t_spiUserHndl *pt_activeUser)
{

    if( pt_chanHndl->t_spiOp == SPI_MASTER)
    {
       /*------------------------------------------------------------------*
        * Store the received data and transmit the next byte
        *------------------------------------------------------------------*/
        if( updateBuffersMast( pt_activeUser) == true)
        {

          /*---------------------------------------------------------------*
           * Release spi channel
           *---------------------------------------------------------------*/
          if( pt_activeUser->b_enCs == true)
          {
             hal_gpioOn( pt_activeUser->t_csPort,
                         pt_activeUser->c_csPin);
          }

          /*---------------------------------------------------------------*
           * Execute the call-back function, returning the rx buffer and
           * the size of the data.
           *---------------------------------------------------------------*/
          if( pt_activeUser->pf_funPtr != NULL)
            pt_activeUser->pf_funPtr( pt_activeUser->pc_rxData,
                                           pt_activeUser->s_bufIndex);

          /*---------------------------------------------------------------*
           * Release the spi bus.
           *---------------------------------------------------------------*/
          pt_chanHndl->b_busLocked = false;

      }/*End if( updateBuffersMast( pt_activeUser) == true)*/

    }/*End if( pt_chanHndl->t_spiOp == SPI_MASTER)*/
    else
    {
       /*------------------------------------------------------------------*
        * Is the chip select low for this device? Fill data buffer until
        * the chip select goes high.
        *------------------------------------------------------------------*/
       if( hal_isGpioHigh( pt_activeUser->t_csPort,
                           pt_activeUser->c_csPin) == false)
       {
         updateBuffersSlave( pt_activeUser);

       }

    }

}/*End processSpiInterrupt*/

static void masterSpiCRxDmaInt( void)
{
   t_spiChanHndl *pt_chanHndl = NULL;
   t_spiUserHndl *pt_activeUser = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this spi interrupt.
    *------------------------------------------------------------------------*/
   pt_chanHndl = findSpiElement( (uint8_t)SPI_1);

   if( pt_chanHndl != NULL)
   {
      pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

      processMasterSpiRxDmaInt( pt_chanHndl,
                                pt_activeUser);

   }/*End if( pt_chanHndl != NULL)*/

}/*End masterSpiCRxDmaInt*/

static void masterSpiDRxDmaInt( void)
{
   t_spiChanHndl *pt_chanHndl = NULL;
   t_spiUserHndl *pt_activeUser = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this spi interrupt.
    *------------------------------------------------------------------------*/
   pt_chanHndl = findSpiElement( (uint8_t)SPI_2);

   if( pt_chanHndl != NULL)
   {
      pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

      processMasterSpiRxDmaInt( pt_chanHndl,
                                pt_activeUser);

   }/*End if( pt_chanHndl != NULL)*/

}/*End masterSpiDRxDmaInt*/

static void masterSpiERxDmaInt( void)
{
   t_spiChanHndl *pt_chanHndl = NULL;
   t_spiUserHndl *pt_activeUser = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this spi interrupt.
    *------------------------------------------------------------------------*/
   pt_chanHndl = findSpiElement( (uint8_t)SPI_3);

   if( pt_chanHndl != NULL)
   {
      pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

      processMasterSpiRxDmaInt( pt_chanHndl,
                                pt_activeUser);

   }/*End if( pt_chanHndl != NULL)*/

}/*End masterSpiERxDmaInt*/

static void masterSpiFRxDmaInt( void)
{
   t_spiChanHndl *pt_chanHndl = NULL;
   t_spiUserHndl *pt_activeUser = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this spi interrupt.
    *------------------------------------------------------------------------*/
   pt_chanHndl = findSpiElement( (uint8_t)SPI_4);

   if( pt_chanHndl != NULL)
   {
      pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

      processMasterSpiRxDmaInt( pt_chanHndl,
                                pt_activeUser);

   }/*End if( pt_chanHndl != NULL)*/

}/*End masterSpiFRxDmaInt*/

static void slaveSpiCChipSelectInt( t_gpioPort t_port,
                                    uint8_t c_pin)
{
   t_spiChanHndl *pt_chanHndl = NULL;
   t_spiUserHndl *pt_activeUser = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this spi interrupt.
    *------------------------------------------------------------------------*/
   pt_chanHndl = findSpiElement( (uint8_t)SPI_1);

   if( (pt_chanHndl != NULL) && (pt_chanHndl->t_spiOp == SPI_SLAVE))
   {
      pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

      processSlaveSpiChipSelectInt( pt_chanHndl,
                                    pt_activeUser);

   }/*End if( (pt_chanHndl != NULL) && (pt_chanHndl->t_spiOp == SPI_SLAVE))*/

}/*End slaveSpiDChipSelectInt*/

static void slaveSpiDChipSelectInt( t_gpioPort t_port,
                                    uint8_t c_pin)
{
   t_spiChanHndl *pt_chanHndl = NULL;
   t_spiUserHndl *pt_activeUser = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this spi interrupt.
    *------------------------------------------------------------------------*/
   pt_chanHndl = findSpiElement( (uint8_t)SPI_2);

   if( (pt_chanHndl != NULL) && (pt_chanHndl->t_spiOp == SPI_SLAVE))
   {
      pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

      processSlaveSpiChipSelectInt( pt_chanHndl,
                                    pt_activeUser);

   }/*End if( (pt_chanHndl != NULL) && (pt_chanHndl->t_spiOp == SPI_SLAVE))*/

}/*End slaveSpiDChipSelectInt*/

static void slaveSpiEChipSelectInt( t_gpioPort t_port,
                                    uint8_t c_pin)
{
   t_spiChanHndl *pt_chanHndl = NULL;
   t_spiUserHndl *pt_activeUser = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this spi interrupt.
    *------------------------------------------------------------------------*/
   pt_chanHndl = findSpiElement( (uint8_t)SPI_3);

   if( (pt_chanHndl != NULL) && (pt_chanHndl->t_spiOp == SPI_SLAVE))
   {
      pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

      processSlaveSpiChipSelectInt( pt_chanHndl,
                                    pt_activeUser);
   }/*End if( (pt_chanHndl != NULL) && (pt_chanHndl->t_spiOp == SPI_SLAVE))*/

}/*End slaveSpiEChipSelectInt*/

static void slaveSpiFChipSelectInt( t_gpioPort t_port,
                                    uint8_t c_pin)
{
   t_spiChanHndl *pt_chanHndl = NULL;
   t_spiUserHndl *pt_activeUser = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this spi interrupt.
    *------------------------------------------------------------------------*/
   pt_chanHndl = findSpiElement( (uint8_t)SPI_4);

   if( (pt_chanHndl != NULL) && (pt_chanHndl->t_spiOp == SPI_SLAVE))
   {
      pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

      processSlaveSpiChipSelectInt( pt_chanHndl,
                                    pt_activeUser);

   }/*End if( (pt_chanHndl != NULL) && (pt_chanHndl->t_spiOp == SPI_SLAVE))*/

}/*End slaveSpiFChipSelectInt*/

ISR( SPIC_INT_vect)
{
   t_spiChanHndl *pt_chanHndl = NULL;
   t_spiUserHndl *pt_activeUser = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this spi interrupt.
    *------------------------------------------------------------------------*/
   pt_chanHndl = findSpiElement( (uint8_t)SPI_1);

   if( pt_chanHndl != NULL)
   {
      pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

      processSpiInterrupt( pt_chanHndl,
                           pt_activeUser);

   }/*End if( pt_chanHndl != NULL)*/

}/*End ISR( SPIC_INT_vect)*/

ISR( SPID_INT_vect)
{
   t_spiChanHndl *pt_chanHndl = NULL;
   t_spiUserHndl *pt_activeUser = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this spi interrupt.
    *------------------------------------------------------------------------*/
   pt_chanHndl = findSpiElement( (uint8_t)SPI_2);

   if( pt_chanHndl != NULL)
   {
      pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

      processSpiInterrupt( pt_chanHndl,
                           pt_activeUser);

   }/*End if( pt_chanHndl != NULL)*/

}/*End ISR( SPID_INT_vect)*/

ISR( SPIE_INT_vect)
{
   t_spiChanHndl *pt_chanHndl = NULL;
   t_spiUserHndl *pt_activeUser = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this spi interrupt.
    *------------------------------------------------------------------------*/
   pt_chanHndl = findSpiElement( (uint8_t)SPI_3);

   if( pt_chanHndl != NULL)
   {
      pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

      processSpiInterrupt( pt_chanHndl,
                           pt_activeUser);

   }/*End if( pt_chanHndl != NULL)*/

}/*End ISR( SPIE_INT_vect)*/

ISR( SPIF_INT_vect)
{
   t_spiChanHndl *pt_chanHndl = NULL;
   t_spiUserHndl *pt_activeUser = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this spi interrupt.
    *------------------------------------------------------------------------*/
   pt_chanHndl = findSpiElement( (uint8_t)SPI_4);

   if( pt_chanHndl != NULL)
   {
      pt_activeUser = (t_spiUserHndl*)pt_chanHndl->pt_activeUser;

      processSpiInterrupt( pt_chanHndl,
                           pt_activeUser);

   }/*End if( pt_chanHndl != NULL)*/

}/*End ISR( SPIF_INT_vect)*/

static t_spiChanHndl *findSpiElement( t_spiChanId t_id)
{
   t_spiChanHndl *pt_element;
   t_LINKHNDL t_linkHndl;
   int16_t s_count;

   /*------------------------------------------------------------------------*
    * Search the spi list for the requested ID
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_spiChanHndlList, s_count)
   {
      pt_element = (t_spiChanHndl *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( pt_element->t_id == t_id)
      {
         return pt_element;
      }

   }

   /*------------------------------------------------------------------------*
    * If we make it this far the ID has not been found in the open spi module
    * list.
    *------------------------------------------------------------------------*/
   return NULL;

}/*End findSpiElement*/

static t_LINKHNDL createSpiHandle( void)
{

   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Allocated memory for the link (and element) that contains information
    * specific to this particular spi module
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof( t_spiChanHndl));

   if( t_linkHndl < 0)
   {
      return (t_LINKHNDL)SPI_OUT_OF_HEAP;
   }

   /*------------------------------------------------------------------------*
    * Add the spi module link onto the list open spi modules.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_spiChanHndlList,
                           t_linkHndl,
                           true);

   return t_linkHndl;

}/*End createSpiHandle*/

static t_LINKHNDL createSpiUserHandle( void)
{

   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Allocated memory for the link (and element) that contains information
    * specific to the user connected to particular spi module
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof( t_spiUserHndl));

   if( t_linkHndl < 0)
   {
      return (t_LINKHNDL)SPI_OUT_OF_HEAP;
   }

   /*------------------------------------------------------------------------*
    * Add the spi module link onto the list open spi modules.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_spiUserHndlList,
                           t_linkHndl,
                           true);

   return t_linkHndl;

}/*End createSpiUserHandle*/

t_spiError hal_spiConfCallBack( t_SPIHNDL t_handle,
                                  void (*pf_funPtr)( int8_t *pc_data,
                                                     uint16_t s_length))
{
   t_spiChanHndl *pt_spiChanHndl;
   t_spiUserHndl *pt_spiUserHndl;

   /*------------------------------------------------------------------------*
    * Mutual exclusion - operations being performed on global variables.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a spi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_spiUserHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the spi
    * user information is being stored.
    *------------------------------------------------------------------------*/
   pt_spiUserHndl = (t_spiUserHndl *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

   /*------------------------------------------------------------------------*
    * Grab the module associated with this user.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl = pt_spiUserHndl->pt_spiChanHndl;

   if( pt_spiChanHndl->b_busLocked == true)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_BUSY; /*Yes*/
   }

   pt_spiUserHndl->pf_funPtr = pf_funPtr;

   HAL_END_CRITICAL();//Enable interrupts

   return SPI_PASSED;

}/*End hal_spiConfCallBack*/

t_spiError hal_spiSetChipSelect( t_SPIHNDL t_handle,
                                 t_spiCsCntl t_csCntl)
{
   t_spiChanHndl *pt_spiChanHndl;
   t_spiUserHndl *pt_spiUserHndl;

   /*------------------------------------------------------------------------*
    * Mutual exclusion - operations being performed on global variables.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a spi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_spiUserHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the spi
    * user information is being stored.
    *------------------------------------------------------------------------*/
   pt_spiUserHndl = (t_spiUserHndl *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

   /*------------------------------------------------------------------------*
    * Grab the module associated with this user.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl = pt_spiUserHndl->pt_spiChanHndl;

   /*------------------------------------------------------------------------*
    * Is the bus locked by someone other than this user?
    *------------------------------------------------------------------------*/
   if( (pt_spiChanHndl->b_busLocked == true) &&
       (pt_spiChanHndl->pt_activeUser != pt_spiUserHndl))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_BUSY; /*Yes*/
   }

   /*------------------------------------------------------------------------*
    * Interrupts not used in manual mode
    *------------------------------------------------------------------------*/
   pt_spiChanHndl->pt_spi->INTCTRL = SPI_INTLVL_OFF_gc;

   /*------------------------------------------------------------------------*
    * Chip select is active low.
    *------------------------------------------------------------------------*/
   if( t_csCntl == SPI_CS_EN)
   {

      /*---------------------------------------------------------------------*
       * Lock access to the bus until the transaction is finished
       *---------------------------------------------------------------------*/
      pt_spiChanHndl->b_busLocked = true;
      pt_spiChanHndl->pt_activeUser = pt_spiUserHndl;

      hal_gpioOff( pt_spiUserHndl->t_csPort,
                   pt_spiUserHndl->c_csPin);
   }
   else
   {
      /*---------------------------------------------------------------------*
       * Release bus lock.
       *---------------------------------------------------------------------*/
      hal_gpioOn( pt_spiUserHndl->t_csPort,
                  pt_spiUserHndl->c_csPin);

      pt_spiChanHndl->b_busLocked = false;
   }

   HAL_END_CRITICAL();//Enable interrupts

   return SPI_PASSED;

}/*End hal_spiSetChipSelect*/

/*---------------------------------------------------------------------------*
 * Read a single character from a particular spi device pointed to by
 * 't_handle'
 *---------------------------------------------------------------------------*/
t_spiError hal_spiReadByte( t_SPIHNDL t_handle,
                            int8_t *pc_rxChar)
{
   t_spiChanHndl *pt_spiChanHndl;
   t_spiUserHndl *pt_spiUserHndl;
   uint8_t c_status;

   /*------------------------------------------------------------------------*
    * Mutual exclusion - operations being performed on global variables.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a spi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_spiUserHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the spi
    * user information is being stored.
    *------------------------------------------------------------------------*/
   pt_spiUserHndl = (t_spiUserHndl *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

   /*------------------------------------------------------------------------*
    * Grab the module associated with this user.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl = pt_spiUserHndl->pt_spiChanHndl;

   /*------------------------------------------------------------------------*
    * Is the bus locked by someone other than this user?
    *------------------------------------------------------------------------*/
   if( (pt_spiChanHndl->b_busLocked == true) &&
       (pt_spiChanHndl->pt_activeUser != pt_spiUserHndl))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_BUSY; /*Yes*/
   }

   /*------------------------------------------------------------------------*
    * Interrupts not used in manual mode
    *------------------------------------------------------------------------*/
   pt_spiChanHndl->pt_spi->INTCTRL = SPI_INTLVL_OFF_gc;

   /*------------------------------------------------------------------------*
    * Lock bus access
    *------------------------------------------------------------------------*/
   pt_spiChanHndl->b_busLocked = true;
   pt_spiChanHndl->pt_activeUser = pt_spiUserHndl;

   HAL_END_CRITICAL();//Enable interrupts

   /*------------------------------------------------------------------------*
    * Read status register to clear any pending transactions
    *------------------------------------------------------------------------*/
   c_status = pt_spiChanHndl->pt_spi->STATUS;

   /*------------------------------------------------------------------------*
    * Transmit a character of dummy data so that we can grab data out of the
    * rx buffer.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl->pt_spi->DATA = 0xFF;

   /*------------------------------------------------------------------------*
    * Wait for the data to clear the register
    *------------------------------------------------------------------------*/
   while( !(pt_spiChanHndl->pt_spi->STATUS & SPI_IF_bm));

   /*------------------------------------------------------------------------*
    * Read new data
    *------------------------------------------------------------------------*/
   *pc_rxChar = pt_spiChanHndl->pt_spi->DATA;

   return SPI_PASSED;

}/*End hal_spiReadByte*/

/*---------------------------------------------------------------------------*
 * Write a single character to a particular spi device pointed to by
 * 't_handle'
 *---------------------------------------------------------------------------*/
t_spiError hal_spiWriteByte( t_SPIHNDL t_handle,
                                     int8_t c_txChar)
{
   t_spiChanHndl *pt_spiChanHndl;
   t_spiUserHndl *pt_spiUserHndl;
   uint8_t c_temp;

   /*------------------------------------------------------------------------*
    * Mutual exclusion - operations being performed on global variables.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a spi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_spiUserHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the spi
    * user information is being stored.
    *------------------------------------------------------------------------*/
   pt_spiUserHndl = (t_spiUserHndl *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

   /*------------------------------------------------------------------------*
    * Grab the module associated with this user.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl = pt_spiUserHndl->pt_spiChanHndl;

   /*------------------------------------------------------------------------*
    * Is the bus locked by someone other than this user?
    *------------------------------------------------------------------------*/
   if( (pt_spiChanHndl->b_busLocked == true) &&
       (pt_spiChanHndl->pt_activeUser != pt_spiUserHndl))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_BUSY; /*Yes*/
   }

   /*------------------------------------------------------------------------*
    * Interrupts not used in manual mode
    *------------------------------------------------------------------------*/
   pt_spiChanHndl->pt_spi->INTCTRL = SPI_INTLVL_OFF_gc;

   /*------------------------------------------------------------------------*
    * Lock bus access
    *------------------------------------------------------------------------*/
   pt_spiChanHndl->b_busLocked = true;
   pt_spiChanHndl->pt_activeUser = pt_spiUserHndl;

   HAL_END_CRITICAL();//Enable interrupts

   /*------------------------------------------------------------------------*
    * Read status register to clear any pending transactions
    *------------------------------------------------------------------------*/
   c_temp = pt_spiChanHndl->pt_spi->STATUS;

   /*------------------------------------------------------------------------*
    * Transmit a character of dummy data so that we can grab data out of the
    * rx buffer.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl->pt_spi->DATA = (uint8_t)c_txChar;

   /*------------------------------------------------------------------------*
    * Wait for the data to clear the register
    *------------------------------------------------------------------------*/
   while( !(pt_spiChanHndl->pt_spi->STATUS & SPI_IF_bm));

   /*------------------------------------------------------------------------*
    * Read new data
    *------------------------------------------------------------------------*/
   c_temp = pt_spiChanHndl->pt_spi->DATA;

   return SPI_PASSED;

}/*End hal_spiWriteByte*/

t_spiError hal_spiSetBaudRate( t_SPIHNDL t_handle,
                               uint32_t i_baudRate)
{
   t_spiChanHndl *pt_spiChanHndl;
   t_spiUserHndl *pt_spiUserHndl;
   uint32_t i_maxSpiFreq;
   SPI_PRESCALER_t t_clockDivision;
   uint8_t c_doubleClock = 0;

   /*------------------------------------------------------------------------*
    * Mutual exclusion - operations being performed on global variables.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a spi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_spiUserHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the spi
    * user information is being stored.
    *------------------------------------------------------------------------*/
   pt_spiUserHndl = (t_spiUserHndl *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

   /*------------------------------------------------------------------------*
    * Grab the module associated with this user.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl = pt_spiUserHndl->pt_spiChanHndl;

   if( pt_spiChanHndl->t_spiOp == SPI_MASTER)
      i_maxSpiFreq = (hal_getCpuFreq() >> 1);
   else
      i_maxSpiFreq = (hal_getCpuFreq() >> 2);

   if( (i_baudRate < (hal_getCpuFreq() >> 7)) ||
       (i_baudRate > i_maxSpiFreq))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_INVALID_BAUD_RATE;
   }

   /*------------------------------------------------------------------------*
    * Find the clock divider closest to the requested baud rate.
    *------------------------------------------------------------------------*/
   if( (i_baudRate >= (hal_getCpuFreq() >> 7)) &&
       (i_baudRate < (hal_getCpuFreq() >> 6)))
   {
      t_clockDivision = SPI_PRESCALER_DIV128_gc;
      pt_spiChanHndl->i_baudRate = (hal_getCpuFreq() >> 7);
   }
   else if( (i_baudRate >= (hal_getCpuFreq() >> 6)) &&
            (i_baudRate < (hal_getCpuFreq() >> 4)))
   {
      t_clockDivision = SPI_PRESCALER_DIV64_gc;
      pt_spiChanHndl->i_baudRate = (hal_getCpuFreq() >> 6);
   }
   else if( (i_baudRate >= (hal_getCpuFreq() >> 4)) &&
            (i_baudRate < (hal_getCpuFreq() >> 2)))
   {
      t_clockDivision = SPI_PRESCALER_DIV16_gc;
      pt_spiChanHndl->i_baudRate = (hal_getCpuFreq() >> 4);
   }
   else if( (i_baudRate >= (hal_getCpuFreq() >> 2)) &&
            (i_baudRate < (hal_getCpuFreq() >> 1)))
   {
      t_clockDivision = SPI_PRESCALER_DIV4_gc;
      pt_spiChanHndl->i_baudRate = (hal_getCpuFreq() >> 2);
   }
   else
   {
      t_clockDivision = SPI_PRESCALER_DIV4_gc;
      pt_spiChanHndl->i_baudRate = (hal_getCpuFreq() >> 2);
      c_doubleClock = 0;
      if( pt_spiChanHndl->t_spiOp == SPI_MASTER)
      {
         pt_spiChanHndl->i_baudRate *=2;
         c_doubleClock = SPI_CLK2X_bm;
      }

   }

   /*------------------------------------------------------------------------*
    * Disable the spi channel.
    *------------------------------------------------------------------------*/
    pt_spiChanHndl->pt_spi->CTRL &= ~SPI_ENABLE_bm;

   /*------------------------------------------------------------------------*
    * Configure the spi baud rate.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl->pt_spi->CTRL &= ~(SPI_PRESCALER_gm << SPI_PRESCALER_gp);
   pt_spiChanHndl->pt_spi->CTRL |= t_clockDivision;
   pt_spiChanHndl->pt_spi->CTRL &= ~SPI_CLK2X_bm;
   pt_spiChanHndl->pt_spi->CTRL |= c_doubleClock;

   /*------------------------------------------------------------------------*
    * Enable the spi channel.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl->pt_spi->CTRL |= SPI_ENABLE_bm;

   HAL_END_CRITICAL();//Enable interrupts

   return SPI_PASSED;

}/*End hal_spiSetBaudRate*/

t_spiError hal_spiReadBlock( t_SPIHNDL t_handle,
                             bool b_enCs,
                             int8_t *pc_rxBuffer,
                             uint16_t s_numBytes)
{
   t_spiChanHndl *pt_spiChanHndl;
   t_spiUserHndl *pt_spiUserHndl;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a spi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_spiUserHndlList) ==
   false)
   {
      return SPI_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the spi
    * user information is being stored.
    *------------------------------------------------------------------------*/
   pt_spiUserHndl = (t_spiUserHndl *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

   /*------------------------------------------------------------------------*
    * Grab the module associated with this user.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl = pt_spiUserHndl->pt_spiChanHndl;

   /*------------------------------------------------------------------------*
    * Mutual exclusion - operations being performed on global variables.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is there a current transaction in progress by another user?
    *------------------------------------------------------------------------*/
   if( (pt_spiChanHndl->b_busLocked == true) &&
       (pt_spiChanHndl->pt_activeUser != pt_spiUserHndl))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_BUSY; /*Yes*/
   }
   else /*No*/
   {
      /*---------------------------------------------------------------------*
       * Lock bus access.
       *---------------------------------------------------------------------*/
      pt_spiChanHndl->b_busLocked = true;
      pt_spiChanHndl->pt_activeUser = pt_spiUserHndl;

      HAL_END_CRITICAL();//Enable interrupts

      /*---------------------------------------------------------------------*
       * Remember the address of the data associated with this transfer
       *---------------------------------------------------------------------*/
      gc_tempBuf = 0xFF;
      pt_spiUserHndl->pc_txData   = (int8_t *)&gc_tempBuf;
      pt_spiUserHndl->pc_rxData   = pc_rxBuffer;
      pt_spiUserHndl->s_bufLength = s_numBytes;

      /*---------------------------------------------------------------------*
       * Does this transaction use DMA?
       *---------------------------------------------------------------------*/
      if( pt_spiChanHndl->t_rxDmaHndl > 0)
      {
         t_dmaChanConfig t_chanConf;
         t_dmaTriggerSource t_trigger;
         t_dmaError t_err;

         pt_spiChanHndl->pt_spi->INTCTRL = SPI_INTLVL_OFF_gc;

         switch( pt_spiChanHndl->t_id)
         {
            case SPI_1:
               t_trigger = SPI1_TRANSFER_COMPLETE;
            break;

            case SPI_2:
               t_trigger = SPI2_TRANSFER_COMPLETE;
            break;

            case SPI_3:
               t_trigger = SPI3_TRANSFER_COMPLETE;
            break;

            case SPI_4:
               t_trigger = SPI4_TRANSFER_COMPLETE;
            break;

            default:
               t_trigger = SPI1_TRANSFER_COMPLETE;
            break;

         }/*End switch( pt_spiChanHndl->t_id)*/

         if( (pt_spiChanHndl->t_spiOp == SPI_MASTER) &&
            !(pt_spiUserHndl->pc_txData == (int8_t *)&gc_tempBuf))
            t_chanConf.pi_srcAddress = (uint32_t *)&pt_spiUserHndl->pc_txData[1];
         else
            t_chanConf.pi_srcAddress = (uint32_t *)&pt_spiUserHndl->pc_txData[0];
         t_chanConf.pi_destAddress  = (uint32_t *)&pt_spiChanHndl->pt_spi->DATA;
         t_chanConf.t_srcAddDir     = FIXED;
         t_chanConf.t_destAddDir    = FIXED;
         t_chanConf.t_srcAddReload  = RELOAD_END_OF_BLOCK;
         t_chanConf.t_destAddReload = NO_RELOAD;
         if( pt_spiChanHndl->t_spiOp == SPI_MASTER)
            t_chanConf.s_blockSize = s_numBytes - 1;
         else
            t_chanConf.s_blockSize = s_numBytes;
         t_chanConf.t_burstMode     = ONE_BYTE;
         t_chanConf.t_transferType  = SINGLE_SHOT;
         t_chanConf.t_triggerSrc    = t_trigger;
         t_chanConf.c_repeatCount   = 0;

         t_err = hal_configureDmaChannel( pt_spiChanHndl->t_txDmaHndl,
                                          t_chanConf);

         t_err = hal_dmaEnableChannel( pt_spiChanHndl->t_txDmaHndl);

         t_chanConf.pi_srcAddress   = (uint32_t *)&pt_spiChanHndl->pt_spi->DATA;
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

         t_err = hal_configureDmaChannel( pt_spiChanHndl->t_rxDmaHndl,
                                          t_chanConf);

         t_err = hal_dmaEnableChannel( pt_spiChanHndl->t_rxDmaHndl);

      }/*End if( pt_spiChanHndl->t_rxDmaHndl > 0)*/
      else
         pt_spiChanHndl->pt_spi->INTCTRL = SPI_INTLVL_HI_gc;

      /*---------------------------------------------------------------------*
       * Begin data transaction.
       *---------------------------------------------------------------------*/
      if( pt_spiChanHndl->t_spiOp == SPI_MASTER)
      {

         pt_spiUserHndl->s_bufIndex = 0;
         pt_spiUserHndl->b_enCs = b_enCs;

         /*------------------------------------------------------------------*
          * Chip select low...
          *------------------------------------------------------------------*/
         if( b_enCs == true)
         {
            hal_gpioOff( pt_spiUserHndl->t_csPort,
                         pt_spiUserHndl->c_csPin);
         }

         /*------------------------------------------------------------------*
          * Transfer first byte...
          *------------------------------------------------------------------*/
         pt_spiChanHndl->pt_spi->DATA = pt_spiUserHndl->pc_txData[0];

       }/*End if( pt_spiChanHndl->t_spiOp == SPI_MASTER)*/

   }

   return SPI_PASSED;

}/*End hal_spiReadBlock*/

t_spiError hal_spiWriteBlock( t_SPIHNDL t_handle,
                              bool b_enCs,
                              int8_t *pc_txBuffer,
                              uint16_t s_numBytes)
{
   t_spiChanHndl *pt_spiChanHndl;
   t_spiUserHndl *pt_spiUserHndl;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a spi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_spiUserHndlList) ==
   false)
   {
      return SPI_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the spi
    * user information is being stored.
    *------------------------------------------------------------------------*/
   pt_spiUserHndl = (t_spiUserHndl *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

   /*------------------------------------------------------------------------*
    * Grab the module associated with this user.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl = pt_spiUserHndl->pt_spiChanHndl;

   /*------------------------------------------------------------------------*
    * Mutual exclusion - operations being performed on global variables.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is there a current transaction in progress by another user?
    *------------------------------------------------------------------------*/
   if( (pt_spiChanHndl->b_busLocked == true) &&
       (pt_spiChanHndl->pt_activeUser != pt_spiUserHndl))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_BUSY; /*Yes*/
   }
   else /*No*/
   {
      /*---------------------------------------------------------------------*
       * Lock bus access.
       *---------------------------------------------------------------------*/
      pt_spiChanHndl->b_busLocked = true;
      pt_spiChanHndl->pt_activeUser = pt_spiUserHndl;

      HAL_END_CRITICAL();//Enable interrupts

      /*---------------------------------------------------------------------*
       * Remember the address of the data associated with this transfer
       *---------------------------------------------------------------------*/
      gc_tempBuf = 0xFF;
      pt_spiUserHndl->pc_txData   = pc_txBuffer;
      pt_spiUserHndl->pc_rxData   = (int8_t *)&gc_tempBuf;
      pt_spiUserHndl->s_bufLength = s_numBytes;

      /*---------------------------------------------------------------------*
       * Does this transaction use DMA?
       *---------------------------------------------------------------------*/
      if( pt_spiChanHndl->t_txDmaHndl > 0)
      {
         t_dmaChanConfig t_chanConf;
         t_dmaTriggerSource t_trigger;
         t_dmaError t_err;

         pt_spiChanHndl->pt_spi->INTCTRL = SPI_INTLVL_OFF_gc;

         switch( pt_spiChanHndl->t_id)
         {
            case SPI_1:
               t_trigger = SPI1_TRANSFER_COMPLETE;
            break;

            case SPI_2:
               t_trigger = SPI2_TRANSFER_COMPLETE;
            break;

            case SPI_3:
               t_trigger = SPI3_TRANSFER_COMPLETE;
            break;

            case SPI_4:
               t_trigger = SPI4_TRANSFER_COMPLETE;
            break;

            default:
               t_trigger = SPI1_TRANSFER_COMPLETE;
            break;

         }/*End switch( pt_spiChanHndl->t_id)*/

         if( pt_spiChanHndl->t_spiOp == SPI_MASTER)
            t_chanConf.pi_srcAddress   = (uint32_t *)&pc_txBuffer[1];
         else
            t_chanConf.pi_srcAddress   = (uint32_t *)&pc_txBuffer[0];
         t_chanConf.pi_destAddress  = (uint32_t *)&pt_spiChanHndl->pt_spi->DATA;
         t_chanConf.t_srcAddDir     = INCREMENT;
         t_chanConf.t_destAddDir    = FIXED;
         t_chanConf.t_srcAddReload  = RELOAD_END_OF_BLOCK;
         t_chanConf.t_destAddReload = NO_RELOAD;
         if( pt_spiChanHndl->t_spiOp == SPI_MASTER)
            t_chanConf.s_blockSize     = s_numBytes - 1;
         else
            t_chanConf.s_blockSize     = s_numBytes;
         t_chanConf.t_burstMode     = ONE_BYTE;
         t_chanConf.t_transferType  = SINGLE_SHOT;
         t_chanConf.t_triggerSrc    = t_trigger;
         t_chanConf.c_repeatCount   = 0;

         t_err = hal_configureDmaChannel( pt_spiChanHndl->t_txDmaHndl,
                                          t_chanConf);

         t_err = hal_dmaEnableChannel( pt_spiChanHndl->t_txDmaHndl);

         t_chanConf.pi_srcAddress   = (uint32_t *)&pt_spiChanHndl->pt_spi->DATA;
         t_chanConf.pi_destAddress  = (uint32_t *)pt_spiUserHndl->pc_rxData;
         t_chanConf.t_srcAddDir     = FIXED;
         t_chanConf.t_destAddDir    = FIXED;
         t_chanConf.t_srcAddReload  = NO_RELOAD;
         t_chanConf.t_destAddReload = RELOAD_END_OF_BLOCK;
         t_chanConf.s_blockSize     = s_numBytes;
         t_chanConf.t_burstMode     = ONE_BYTE;
         t_chanConf.t_transferType  = SINGLE_SHOT;
         t_chanConf.t_triggerSrc    = t_trigger;
         t_chanConf.c_repeatCount   = 0;

         t_err = hal_configureDmaChannel( pt_spiChanHndl->t_rxDmaHndl,
                                          t_chanConf);

         t_err = hal_dmaEnableChannel( pt_spiChanHndl->t_rxDmaHndl);

      }/*End if( pt_spiChanHndl->t_txDmaHndl > 0)*/
      else
         pt_spiChanHndl->pt_spi->INTCTRL = SPI_INTLVL_HI_gc;

      /*---------------------------------------------------------------------*
       * Begin data transaction.
       *---------------------------------------------------------------------*/
      if( pt_spiChanHndl->t_spiOp == SPI_MASTER)
      {

         pt_spiUserHndl->s_bufIndex = 0;
         pt_spiUserHndl->b_enCs = b_enCs;

         /*------------------------------------------------------------------*
          * Chip select low...
          *------------------------------------------------------------------*/
         if( b_enCs == true)
         {
            hal_gpioOff( pt_spiUserHndl->t_csPort,
                         pt_spiUserHndl->c_csPin);
         }

         /*------------------------------------------------------------------*
          * Transfer first byte...
          *------------------------------------------------------------------*/
         pt_spiChanHndl->pt_spi->DATA = pc_txBuffer[0];

       }/*End if( pt_spiChanHndl->t_spiOp == SPI_MASTER)*/

   }

   return SPI_PASSED;

}/*End hal_spiWriteBlock*/

t_spiError hal_spiReadWrite( t_SPIHNDL t_handle,
                             bool b_enCs,
                             int8_t *pc_txBuffer,
                             int8_t *pc_rxBuffer,
                             uint16_t s_numBytes)
{
   t_spiChanHndl *pt_spiChanHndl;
   t_spiUserHndl *pt_spiUserHndl;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a spi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_spiUserHndlList) ==
   false)
   {
      return SPI_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the spi
    * user information is being stored.
    *------------------------------------------------------------------------*/
   pt_spiUserHndl = (t_spiUserHndl *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

   /*------------------------------------------------------------------------*
    * Grab the module associated with this user.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl = pt_spiUserHndl->pt_spiChanHndl;

   /*------------------------------------------------------------------------*
    * Mutual exclusion - operations being performed on global variables.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is there a current transaction in progress by another user?
    *------------------------------------------------------------------------*/
   if( (pt_spiChanHndl->b_busLocked == true) &&
       (pt_spiChanHndl->pt_activeUser != pt_spiUserHndl))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_BUSY; /*Yes*/
   }
   else /*No*/
   {
      /*---------------------------------------------------------------------*
       * Lock bus access.
       *---------------------------------------------------------------------*/
      pt_spiChanHndl->b_busLocked = true;
      pt_spiChanHndl->pt_activeUser = pt_spiUserHndl;

      HAL_END_CRITICAL();//Enable interrupts

      /*---------------------------------------------------------------------*
       * Remember the address of the data associated with this transfer
       *---------------------------------------------------------------------*/
      pt_spiUserHndl->pc_txData   = pc_txBuffer;
      pt_spiUserHndl->pc_rxData   = pc_rxBuffer;
      pt_spiUserHndl->s_bufLength = s_numBytes;

      /*---------------------------------------------------------------------*
       * Does this transaction use DMA?
       *---------------------------------------------------------------------*/
      if( pt_spiChanHndl->t_txDmaHndl > 0)
      {
         t_dmaChanConfig t_chanConf;
         t_dmaTriggerSource t_trigger;
         t_dmaError t_err;

         pt_spiChanHndl->pt_spi->INTCTRL = SPI_INTLVL_OFF_gc;

         switch( pt_spiChanHndl->t_id)
         {
            case SPI_1:
               t_trigger = SPI1_TRANSFER_COMPLETE;
            break;

            case SPI_2:
               t_trigger = SPI2_TRANSFER_COMPLETE;
            break;

            case SPI_3:
               t_trigger = SPI3_TRANSFER_COMPLETE;
            break;

            case SPI_4:
               t_trigger = SPI4_TRANSFER_COMPLETE;
            break;

            default:
               t_trigger = SPI1_TRANSFER_COMPLETE;
            break;

         }/*End switch( pt_spiChanHndl->t_id)*/

         if( pt_spiChanHndl->t_spiOp == SPI_MASTER)
            t_chanConf.pi_srcAddress   = (uint32_t *)&pc_txBuffer[1];
         else
            t_chanConf.pi_srcAddress   = (uint32_t *)&pc_txBuffer[0];
         t_chanConf.pi_destAddress  = (uint32_t *)&pt_spiChanHndl->pt_spi->DATA;
         t_chanConf.t_srcAddDir     = INCREMENT;
         t_chanConf.t_destAddDir    = FIXED;
         t_chanConf.t_srcAddReload  = RELOAD_END_OF_BLOCK;
         t_chanConf.t_destAddReload = NO_RELOAD;
         if( pt_spiChanHndl->t_spiOp == SPI_MASTER)
            t_chanConf.s_blockSize     = s_numBytes - 1;
         else
            t_chanConf.s_blockSize     = s_numBytes;
         t_chanConf.t_burstMode     = ONE_BYTE;
         t_chanConf.t_transferType  = SINGLE_SHOT;
         t_chanConf.t_triggerSrc    = t_trigger;
         t_chanConf.c_repeatCount   = 0;

         t_err = hal_configureDmaChannel( pt_spiChanHndl->t_txDmaHndl,
                                          t_chanConf);

         t_err = hal_dmaEnableChannel( pt_spiChanHndl->t_txDmaHndl);

         t_chanConf.pi_srcAddress   = (uint32_t *)&pt_spiChanHndl->pt_spi->DATA;
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

         t_err = hal_configureDmaChannel( pt_spiChanHndl->t_rxDmaHndl,
                                          t_chanConf);

         t_err = hal_dmaEnableChannel( pt_spiChanHndl->t_rxDmaHndl);

      }/*End if( pt_spiChanHndl->t_txDmaHndl > 0)*/
      else
         pt_spiChanHndl->pt_spi->INTCTRL = SPI_INTLVL_HI_gc;

      /*---------------------------------------------------------------------*
       * Begin data transaction.
       *---------------------------------------------------------------------*/
      if( pt_spiChanHndl->t_spiOp == SPI_MASTER)
      {

         pt_spiUserHndl->s_bufIndex = 0;
         pt_spiUserHndl->b_enCs = b_enCs;

         /*------------------------------------------------------------------*
          * Chip select low...
          *------------------------------------------------------------------*/
         if( b_enCs == true)
         {
            hal_gpioOff( pt_spiUserHndl->t_csPort,
                         pt_spiUserHndl->c_csPin);
         }

         /*------------------------------------------------------------------*
          * Transfer first byte...
          *------------------------------------------------------------------*/
         pt_spiChanHndl->pt_spi->DATA = pc_txBuffer[0];

      }/*End if( pt_spiChanHndl->t_spiOp == SPI_MASTER)*/

   }

   return SPI_PASSED;

}/*End hal_spiReadWrite*/

/*---------------------------------------------------------------------------*
 * Request access to a particular spi module
 *---------------------------------------------------------------------------*/
t_SPIHNDL hal_requestSpiChannel( t_spiChanId t_chanId,
                                 void (*pf_funPtr)( int8_t *pc_data,
                                                    uint16_t s_length),
                                 t_gpioPort t_csPort,
                                 uint8_t c_csPin)
{
   t_spiChanHndl *pt_spiChanHndl;
   t_spiUserHndl *pt_spiUserHndl;
   static t_LINKHNDL t_linkHndl;
   t_gpioConf t_conf;
   t_gpioError t_gErr;
   t_intConf t_csIntConf;

   t_csIntConf.pf_funPtr = NULL;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( (t_chanId < SPI_1) ||
        (t_chanId > SPI_4))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_SPIHNDL)SPI_INVALID_MODULE;
   }

   if( (c_csPin < PIN_0) ||
       (c_csPin > PIN_7))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_SPIHNDL)SPI_INVALID_PIN;
   }

   if( (t_csPort < GPIO_PORTA) ||
       (t_csPort > GPIO_PORTR))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_SPIHNDL)SPI_INVALID_PORT;
   }

   /*------------------------------------------------------------------------*
    * Grab the handle associated with this channel ID.
    *------------------------------------------------------------------------*/
   pt_spiChanHndl = findSpiElement( t_chanId);

   /*------------------------------------------------------------------------*
    * Has a spi channel been configured?
    *------------------------------------------------------------------------*/
   if( pt_spiChanHndl != NULL) /*Yes*/
   {
      /*---------------------------------------------------------------------*
       * In slave operation there can be only one reader.
       *---------------------------------------------------------------------*/
      if( (pt_spiChanHndl->t_spiOp == SPI_SLAVE) &&
          (pt_spiChanHndl->c_numUsers > 0))
      {
         HAL_END_CRITICAL();//Enable interrupts
         return (t_SPIHNDL)SPI_ONLY_ONE_SLAVE;
      }

      /*---------------------------------------------------------------------*
       * Allocated memory for the link (and element) that contains information
       * specific to the user connected to particular spi module
       *---------------------------------------------------------------------*/
      t_linkHndl = createSpiUserHandle();
      if( t_linkHndl < 0)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return (t_SPIHNDL)SPI_OUT_OF_HEAP;

      }

      pt_spiUserHndl = (t_spiUserHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_linkHndl);

      /*---------------------------------------------------------------------*
       * Make sure all the elements of the spi user handle are reset.
       *---------------------------------------------------------------------*/
      memset( (void *)pt_spiUserHndl, 0, sizeof( t_spiUserHndl));

      /*---------------------------------------------------------------------*
       * Keep track of how many users are attached to this spi.
       *---------------------------------------------------------------------*/
      pt_spiChanHndl->c_numUsers++;

      /*---------------------------------------------------------------------*
       * Keep track of the spi channel associated with this user handle.
       *---------------------------------------------------------------------*/
      pt_spiUserHndl->pt_spiChanHndl = pt_spiChanHndl;

      /*---------------------------------------------------------------------*
       * Store the chip select location.
       *---------------------------------------------------------------------*/
      pt_spiUserHndl->t_csPort    = t_csPort;
      pt_spiUserHndl->c_csPin     = c_csPin;

      pt_spiUserHndl->s_bufIndex  = 0;
      pt_spiUserHndl->s_bufLength = 0;
      pt_spiUserHndl->pc_txData   = NULL;
      pt_spiUserHndl->pc_rxData   = NULL;

      pt_spiUserHndl->pf_funPtr   = pf_funPtr;

      /*---------------------------------------------------------------------*
       * Configure the chip-select pin register.
       *---------------------------------------------------------------------*/
      if( pt_spiChanHndl->t_spiOp == SPI_SLAVE)
      {

         pt_spiChanHndl->pt_activeUser = pt_spiUserHndl;
         t_conf.c_inputMask  = c_csPin;
         t_conf.c_outputMask = 0;

         /*------------------------------------------------------------------*
          * Configure chip select interrupt - allows the slave to know when
          * it has finished receiving data.
          *------------------------------------------------------------------*/
         t_csIntConf.c_pin = c_csPin;
         t_csIntConf.t_inSense = GPIO_RISING;

         switch( t_chanId)
         {
            case SPI_1:
               t_csIntConf.pf_funPtr = &slaveSpiCChipSelectInt;
            break;

            case SPI_2:
               t_csIntConf.pf_funPtr = &slaveSpiDChipSelectInt;
            break;

            case SPI_3:
               t_csIntConf.pf_funPtr = &slaveSpiEChipSelectInt;
            break;

            case SPI_4:
               t_csIntConf.pf_funPtr = &slaveSpiFChipSelectInt;
            break;

         }/*End switch( t_chanId)*/

      }
      else
      {
         t_conf.c_inputMask  = 0;
         t_conf.c_outputMask = c_csPin;
      }

      t_conf.t_inConf  = TOTEM;
      t_conf.t_outConf = TOTEM;
      t_gErr = hal_configureGpioPort( t_csPort, t_conf);

      if( pt_spiChanHndl->t_spiOp == SPI_SLAVE)
      {
         pt_spiUserHndl->t_csIntHndl =
         hal_requestGpioInt( t_csPort, t_csIntConf);
      }

      if( pt_spiUserHndl->t_csIntHndl < 0)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return (t_SPIHNDL)SPI_NO_CONFIG;
      }

      /*---------------------------------------------------------------------*
       * Chip select high...
       *---------------------------------------------------------------------*/
      if( pt_spiChanHndl->t_spiOp == SPI_MASTER)
      {
         hal_gpioOn( t_csPort,
                     c_csPin);
      }

   }/*End if( pt_spiChanHndl != NULL)*/
   else /*No*/
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_SPIHNDL)SPI_NO_CONFIG;
   }

   HAL_END_CRITICAL();//Enable interrupts

   return (t_SPIHNDL)t_linkHndl;

}/*End hal_requestSpiChannel*/

t_spiError hal_releaseSpiChannel( t_SPIHNDL t_handle)
{
   t_spiChanHndl *pt_spiChanHndl;
   t_spiUserHndl *pt_spiUserHndl;
   t_linkedListError t_lErr;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a spi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_spiUserHndlList) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the spi
       * user information is being stored.
       *---------------------------------------------------------------------*/
      pt_spiUserHndl = (t_spiUserHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_spiUserHndl->t_csIntHndl > 0)
      {
         hal_releaseGpioInt( pt_spiUserHndl->t_csIntHndl);
      }

      pt_spiChanHndl = pt_spiUserHndl->pt_spiChanHndl;

      t_lErr = utl_destroyLink( gt_spiUserHndlList,
                                (t_LINKHNDL)t_handle);

      pt_spiChanHndl->c_numUsers--;

      if( pt_spiChanHndl->c_numUsers == 0)
      {
         /*------------------------------------------------------------------*
          * Reset this particular spi channel.
          *------------------------------------------------------------------*/
         pt_spiChanHndl->pt_spi->CTRL = 0;

         /*------------------------------------------------------------------*
          * Disable and reset this spi channel.
          *------------------------------------------------------------------*/
         t_lErr = utl_destroyLink( gt_spiChanHndlList,
                                   pt_spiChanHndl->t_linkHndl);
       }/*End if( pt_spiChanHndl->c_numUsers == 0)*/

   }

   HAL_END_CRITICAL();//Enable interrupts

   return SPI_PASSED;

}/*End hal_releaseSpiChannel*/

t_spiError hal_configureSpiChannel( t_spiChanId t_chanId,
                                    t_spiConfig t_conf)
{
   t_spiChanHndl *pt_spiChanHndl;
   static t_LINKHNDL t_linkHndl;
   t_gpioConf t_gConf;
   t_gpioError t_gErr;
   t_gpioPort t_spiPort = GPIO_PORTC;
   SPI_PRESCALER_t t_clockDivision;
   void (*pf_dmaCallback)( void) = NULL;
   uint8_t c_doubleClock = 0;
   uint32_t i_maxSpiFreq;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   pt_spiChanHndl = findSpiElement( (uint8_t)t_chanId);

   /*------------------------------------------------------------------------*
    * Has this particular spi channel already been configured?
    *------------------------------------------------------------------------*/
   if( pt_spiChanHndl != NULL) /*Yes*/
   {
      HAL_END_CRITICAL();//Enable interrupts
      return SPI_MODULE_CON;
   }
   else /*No*/
   {

      if( (t_chanId < SPI_1) ||
          (t_chanId > SPI_4))
      {
         HAL_END_CRITICAL();//Enable interrupts
         return SPI_INVALID_MODULE;
      }

      if( (t_conf.t_spiMd < SPI_MODE_0) ||
          (t_conf.t_spiMd > SPI_MODE_3))
      {
         HAL_END_CRITICAL();//Enable interrupts
         return SPI_INVALID_MODE;
      }

      if( (t_conf.t_spiOp < SPI_MASTER) ||
          (t_conf.t_spiOp > SPI_SLAVE))
      {
         HAL_END_CRITICAL();//Enable interrupts
         return SPI_INVALID_OP;
      }

      if( (t_conf.t_spiOrder < SPI_LSB_FIRST) ||
          (t_conf.t_spiOrder > SPI_MSB_FIRST))
      {
         HAL_END_CRITICAL();//Enable interrupts
         return SPI_INVALID_ORDER;
      }

      if( t_conf.t_spiOp == SPI_MASTER)
         i_maxSpiFreq = (hal_getCpuFreq() >> 1);
      else
         i_maxSpiFreq = (hal_getCpuFreq() >> 2);

      if( (t_conf.i_baudRate < (hal_getCpuFreq() >> 7)) ||
          (t_conf.i_baudRate > i_maxSpiFreq))
      {
         HAL_END_CRITICAL();//Enable interrupts
         return SPI_INVALID_BAUD_RATE;
      }

      t_linkHndl = createSpiHandle();

      if( t_linkHndl < 0)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return (t_SPIHNDL)SPI_OUT_OF_HEAP;
      }

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the spi
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_spiChanHndl = (t_spiChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_linkHndl);

      /*---------------------------------------------------------------------*
       * Make sure all the elements of the spi channel handle are reset.
       *---------------------------------------------------------------------*/
      memset( (void *)pt_spiChanHndl, 0, sizeof( t_spiChanHndl));

      /*---------------------------------------------------------------------*
       * Keep track of the link address so that it can be 'destroyed' at a
       * later time.
       *---------------------------------------------------------------------*/
      pt_spiChanHndl->t_linkHndl = t_linkHndl;

      switch( t_chanId)
      {
         case SPI_1:

            pt_spiChanHndl->pt_spi = &SPIC;
            t_spiPort = GPIO_PORTC;
            pf_dmaCallback = &masterSpiCRxDmaInt;

         break;

         case SPI_2:

            pt_spiChanHndl->pt_spi = &SPID;
            t_spiPort = GPIO_PORTD;
            pf_dmaCallback = &masterSpiDRxDmaInt;

         break;

         case SPI_3:

            pt_spiChanHndl->pt_spi = &SPIE;
            t_spiPort = GPIO_PORTE;
            pf_dmaCallback = &masterSpiERxDmaInt;

         break;

         case SPI_4:

            pt_spiChanHndl->pt_spi = &SPIF;
            t_spiPort = GPIO_PORTF;
            pf_dmaCallback = &masterSpiFRxDmaInt;

         break;

      }/*End switch( t_chanId)*/

      /*---------------------------------------------------------------------*
       * Is DMA being used for tx and rx transactions?
       *---------------------------------------------------------------------*/
      if( t_conf.b_enDma == true)
      {

         pt_spiChanHndl->t_rxDmaHndl = hal_requestDmaChannel();
         if( pt_spiChanHndl->t_rxDmaHndl  < 0)
         {
            utl_destroyLink( gt_spiChanHndlList,
                             t_linkHndl);

            HAL_END_CRITICAL();//Enable interrupts
            if( pt_spiChanHndl->t_rxDmaHndl == DMA_NO_CHANNELS_OPEN)
               return (t_SPIHNDL)SPI_NO_DMA;
            else
               return (t_SPIHNDL)SPI_OUT_OF_HEAP;

         }/*End if( pt_spiChanHndl->t_rxDmaHndl  < 0)*/

         pt_spiChanHndl->t_txDmaHndl = hal_requestDmaChannel();
         if( pt_spiChanHndl->t_txDmaHndl  < 0)
         {
            utl_destroyLink( gt_spiChanHndlList,
                             t_linkHndl);
            hal_releaseDmaChannel( pt_spiChanHndl->t_rxDmaHndl);

            HAL_END_CRITICAL();//Enable interrupts
            if( pt_spiChanHndl->t_txDmaHndl == DMA_NO_CHANNELS_OPEN)
               return (t_SPIHNDL)SPI_NO_DMA;
            else
               return (t_SPIHNDL)SPI_OUT_OF_HEAP;

         }/*End if( pt_spiChanHndl->t_txDmaHndl  < 0)*/

         /*------------------------------------------------------------------*
          * For master operation with DMA use a transaction complete signal
          * for knowing when the burst has finished. For slave operations,
          * burst complete is signaled via the chip select pin transitioning
          * from low to high.
          *------------------------------------------------------------------*/
         if( t_conf.t_spiOp == SPI_MASTER)
         {
            hal_requestDmaInterrupt( pt_spiChanHndl->t_rxDmaHndl,
                                     DMA_TRANSFER_COMPLETE,
                                     pf_dmaCallback);
         }/*End if( t_conf.t_spiOp == SPI_MASTER)*/

         /*------------------------------------------------------------------*
          * Make sure the read DMA channel has higher priority than the write.
          *------------------------------------------------------------------*/
         if( hal_getDmaChannelId( pt_spiChanHndl->t_rxDmaHndl) >
             hal_getDmaChannelId( pt_spiChanHndl->t_txDmaHndl))
         {
            t_DMAHNDL t_temp;

            /*---------------------------------------------------------------*
             * Swap handles...
             *---------------------------------------------------------------*/
            t_temp = pt_spiChanHndl->t_rxDmaHndl;
            pt_spiChanHndl->t_rxDmaHndl = pt_spiChanHndl->t_txDmaHndl;
            pt_spiChanHndl->t_txDmaHndl = t_temp;
         }

      }/*End if( t_conf.b_enDma == true)*/

      /*---------------------------------------------------------------------*
       * Find the clock divider closest to the requested baud rate.
       *---------------------------------------------------------------------*/
      if( (t_conf.i_baudRate >= (hal_getCpuFreq() >> 7)) &&
          (t_conf.i_baudRate < (hal_getCpuFreq() >> 6)))
      {
         t_clockDivision = SPI_PRESCALER_DIV128_gc;
         pt_spiChanHndl->i_baudRate = (hal_getCpuFreq() >> 7);
      }
      else if( (t_conf.i_baudRate >= (hal_getCpuFreq() >> 6)) &&
               (t_conf.i_baudRate < (hal_getCpuFreq() >> 4)))
      {
         t_clockDivision = SPI_PRESCALER_DIV64_gc;
         pt_spiChanHndl->i_baudRate = (hal_getCpuFreq() >> 6);
      }
      else if( (t_conf.i_baudRate >= (hal_getCpuFreq() >> 4)) &&
               (t_conf.i_baudRate < (hal_getCpuFreq() >> 2)))
      {
         t_clockDivision = SPI_PRESCALER_DIV16_gc;
         pt_spiChanHndl->i_baudRate = (hal_getCpuFreq() >> 4);
      }
      else if( (t_conf.i_baudRate >= (hal_getCpuFreq() >> 2)) &&
               (t_conf.i_baudRate < (hal_getCpuFreq() >> 1)))
      {
         t_clockDivision = SPI_PRESCALER_DIV4_gc;
         pt_spiChanHndl->i_baudRate = (hal_getCpuFreq() >> 2);
      }
      else
      {
         t_clockDivision = SPI_PRESCALER_DIV4_gc;
         pt_spiChanHndl->i_baudRate = (hal_getCpuFreq() >> 2);
         c_doubleClock = 0;
         if( t_conf.t_spiOp == SPI_MASTER)
         {
            pt_spiChanHndl->i_baudRate *=2;
            c_doubleClock = SPI_CLK2X_bm;
         }

      }

      /*---------------------------------------------------------------------*
       * Configure the pins the SPI will use.
       *---------------------------------------------------------------------*/
      if( t_conf.t_spiOp == SPI_MASTER)
      {
         t_gConf.c_inputMask        = PIN_6;
         t_gConf.c_outputMask       = PIN_4|PIN_5|PIN_7;
         t_gConf.b_setOutputLow     = false;
      }/*End if( t_conf.t_spiOp == SPI_MASTER)*/
      else
      {
         t_gConf.c_inputMask    = PIN_4|PIN_5|PIN_7;
         t_gConf.c_outputMask   = PIN_6;
         t_gConf.b_setOutputLow = false;
      }

      t_gConf.t_inConf  = PULLUP;
      t_gConf.t_outConf = TOTEM;

      t_gErr = hal_configureGpioPort( t_spiPort, t_gConf);

      /*---------------------------------------------------------------------*
       * Reset this particular spi channel.
       *---------------------------------------------------------------------*/
      pt_spiChanHndl->pt_spi->CTRL = 0;

      /*---------------------------------------------------------------------*
       * Configure the spi channel.
       *---------------------------------------------------------------------*/
      pt_spiChanHndl->pt_spi->CTRL |= t_clockDivision;

      pt_spiChanHndl->pt_spi->CTRL |= c_doubleClock;

      if( t_conf.t_spiOp == SPI_MASTER)
         pt_spiChanHndl->pt_spi->CTRL |= SPI_MASTER_bm;

      if( t_conf.t_spiOrder == SPI_LSB_FIRST)
         pt_spiChanHndl->pt_spi->CTRL |= SPI_DORD_bm;

      pt_spiChanHndl->pt_spi->CTRL |= (t_conf.t_spiMd << 2);

      /*---------------------------------------------------------------------*
       * Store the channel ID.
       *---------------------------------------------------------------------*/
      pt_spiChanHndl->t_id = t_chanId;

      /*---------------------------------------------------------------------*
       * Number users attached at this time.
       *---------------------------------------------------------------------*/
        pt_spiChanHndl->c_numUsers = 0;

      /*---------------------------------------------------------------------*
       * Store the operational mode for this particular spi channel.
       *---------------------------------------------------------------------*/
        pt_spiChanHndl->t_spiOp = t_conf.t_spiOp;

      /*---------------------------------------------------------------------*
       * No transactions in progress.
       *---------------------------------------------------------------------*/
        pt_spiChanHndl->b_busLocked = false;

      /*---------------------------------------------------------------------*
       * Enable spi interrupts for block devices that aren't configured for
       * dma operation.
       *---------------------------------------------------------------------*/
      if( t_conf.b_enDma == true)
         pt_spiChanHndl->pt_spi->INTCTRL = SPI_INTLVL_OFF_gc;
      else
         pt_spiChanHndl->pt_spi->INTCTRL = SPI_INTLVL_HI_gc;

      /*---------------------------------------------------------------------*
       * Set the chip select pin high.
       *---------------------------------------------------------------------*/
      hal_gpioOn( t_spiPort,
                  PIN_4);

      /*---------------------------------------------------------------------*
       * Enable the spi channel.
       *---------------------------------------------------------------------*/
      pt_spiChanHndl->pt_spi->CTRL |= SPI_ENABLE_bm;

   }

   HAL_END_CRITICAL();//Enable interrupts

   return SPI_PASSED;

}/*End hal_configureSpiChannel*/

