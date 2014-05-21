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
 * File Name   : hal_twi.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for controlling
 *               the TWI module.
 *
 * Last Update : Jan 30, 2012
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "arb_sysTimer.h"
#include "hal_gpio.h"
#include "utl_linkedList.h"
#include "hal_pmic.h"
#include "hal_clocks.h"
#include "hal_twi.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define TWI_MAX_SLAVE_ADDRESS (127)

/*---------------------------------------------------------------------------*
 * If there has been inactivity on the bus for this period of time the
 * particular TWI channel needs to be reset.
 *---------------------------------------------------------------------------*/
#define TWI_BUS_RESET_TIMEOUT (500000) /*usec*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct twiChanHndl
{
   /*------------------------------------------------------------------------*
    * A unique number referring to one of the 4 possible twi
    *------------------------------------------------------------------------*/
   t_twiChanId t_id;

   /*------------------------------------------------------------------------*
    * If true, then 'configureTwi' successfully completed
    *------------------------------------------------------------------------*/
   bool b_validConfig;

   /*------------------------------------------------------------------------*
    * The current buad rate for the twi connected to this handle
    *------------------------------------------------------------------------*/
   int32_t i_baudRate;

   /*------------------------------------------------------------------------*
    * The current status of the TWI bus.
    *------------------------------------------------------------------------*/
   t_twiStatus t_status;

   /*------------------------------------------------------------------------*
    * Pointer to the twi master channel connected to this particular handle,
    * if this is a slave channel then this pointer is NULL.
    *------------------------------------------------------------------------*/
   TWI_MASTER_t *pt_master;

   /*------------------------------------------------------------------------*
    * Pointer to the twi slave channel connected to this particular handle,
    * if this is a master channel then this pointer is NULL.
    *------------------------------------------------------------------------*/
   TWI_SLAVE_t *pt_slave;

   /*------------------------------------------------------------------------*
    * Pointer to the slave transaction complete function which is used
    * for signaling an end-of-transaction or for reading/writing data to and
    * from a slave device.
    *------------------------------------------------------------------------*/
   int8_t (*pf_transComplete)( t_twiStatus t_status, int8_t c_data);

   /*------------------------------------------------------------------------*
    * Pointer to the buffer the TWI is writing/reading.
    *------------------------------------------------------------------------*/
   uint8_t *pc_data;

   /*------------------------------------------------------------------------*
    * Length of the buffer.
    *------------------------------------------------------------------------*/
   uint16_t s_length;

   /*------------------------------------------------------------------------*
    * Current index into the buffer.
    *------------------------------------------------------------------------*/
   uint16_t s_bufIndex;

   /*------------------------------------------------------------------------*
    * The time of the last known master/slave read/write interrupt.
    *------------------------------------------------------------------------*/
   t_sysTime t_lastIntTime;

}t_twiChanHndl;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_LINKHNDL createTwiHandle( void);

static t_twiChanHndl *findTwiElement( t_twiChanId t_id);

static void hal_resetTwiMaster( TWI_MASTER_t *pt_master);

static void hal_resetTwiSlave( TWI_SLAVE_t *pt_slave);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * List of currently active twi modules.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_twiChanHndlList);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void hal_masterInterrupt( volatile t_twiChanHndl *pt_handle)
{
   volatile uint8_t c_busStatus = 0;
   volatile uint8_t *pc_data     = NULL;
   volatile uint16_t s_length   = 0;
   volatile uint16_t *ps_wrPtr  = NULL;

   c_busStatus = pt_handle->pt_master->STATUS;
   pc_data     = pt_handle->pc_data;
   s_length    = pt_handle->s_length;
   ps_wrPtr   = &pt_handle->s_bufIndex;

   /*------------------------------------------------------------------------*
    * Check bus status
    *------------------------------------------------------------------------*/
   if( c_busStatus & TWI_MASTER_ARBLOST_bm) /*Arbitration lost*/
   {

      pt_handle->pt_master->CTRLC = TWI_MASTER_CMD_STOP_gc;
      pt_handle->pt_master->CTRLC = TWI_MASTER_CMD_STOP_gc;

      /*---------------------------------------------------------------------*
       * Clear interrupt flag and set the status of the bus back to idle...
       *---------------------------------------------------------------------*/
      pt_handle->pt_master->STATUS = c_busStatus | TWI_MASTER_ARBLOST_bm;
      pt_handle->t_status          = TWI_ARB_LOST;


   }/*End if( (c_busStatus & TWI_MASTER_ARBLOST_bm)*/
   else if( c_busStatus & TWI_MASTER_BUSERR_bm) /*Bus error*/
   {
      pt_handle->t_lastIntTime = arb_sysTimeNow();
      pt_handle->pt_master->CTRLC = TWI_MASTER_CMD_STOP_gc;
      pt_handle->pt_master->CTRLC = TWI_MASTER_CMD_STOP_gc;

      /*---------------------------------------------------------------------*
       * Clear interrupt flag and set the status of the bus back to idle...
       *---------------------------------------------------------------------*/
      pt_handle->pt_master->STATUS = c_busStatus | TWI_MASTER_ARBLOST_bm;
      pt_handle->t_status          = TWI_BUS_ERROR;

   }/*End else if( c_busStatus & TWI_MASTER_BUSERR_bm)*/
   else if( c_busStatus & TWI_MASTER_RXACK_bm) /*Nack from slave...*/
   {
      pt_handle->pt_master->CTRLC = TWI_MASTER_CMD_STOP_gc;
      pt_handle->pt_master->CTRLC = TWI_MASTER_CMD_STOP_gc;
      pt_handle->t_status         = TWI_NACK_RECEIVED;

   }/*End else if( c_busStatus & TWI_MASTER_RXACK_bm)*/
   else if (c_busStatus & TWI_MASTER_WIF_bm) /*Master write interrupt*/
   {
      pt_handle->t_lastIntTime = arb_sysTimeNow();

      if( (*ps_wrPtr) < s_length)
      {
         pt_handle->pt_master->DATA = pc_data[*ps_wrPtr];
         (*ps_wrPtr)++;

      }/*End if( (*ps_wrPtr) < s_length)*/
      else /*Transaction finished...*/
      {
         pt_handle->pt_master->CTRLC = TWI_MASTER_CMD_STOP_gc;
         pt_handle->pt_master->CTRLC = TWI_MASTER_CMD_STOP_gc;
         pt_handle->t_status         = TWI_TRANS_COMPLETE;

      }

   }/*End else if (c_busStatus & TWI_MASTER_WIF_bm)*/
   else if (c_busStatus & TWI_MASTER_RIF_bm) /*Master read interrupt*/
   {

      pt_handle->t_lastIntTime = arb_sysTimeNow();

      /*---------------------------------------------------------------------*
       * Store data if there is room in the buffer...
       *---------------------------------------------------------------------*/
      if( (*ps_wrPtr) < s_length)
      {
         pc_data[*ps_wrPtr] = pt_handle->pt_master->DATA;
         (*ps_wrPtr)++;

         /*------------------------------------------------------------------*
          * We still need to read more data out of the slave...
          *------------------------------------------------------------------*/
         pt_handle->pt_master->CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;

      }/*End if( (*ps_wrPtr) < s_length)*/
      else
      {
         pt_handle->pt_master->CTRLC = TWI_MASTER_ACKACT_bm |
         TWI_MASTER_CMD_STOP_gc;
         pt_handle->pt_master->CTRLC = TWI_MASTER_ACKACT_bm |
         TWI_MASTER_CMD_STOP_gc;
         pt_handle->t_status = TWI_TRANS_COMPLETE;

      }

   }/*End else if (c_busStatus & TWI_MASTER_RIF_bm)*/

}/*End hal_masterInterrupt*/

static void hal_slaveInterrupt( volatile t_twiChanHndl *pt_handle)
{
   volatile uint8_t c_busStatus = 0;
   volatile uint16_t *ps_wrPtr  = NULL;
   int8_t c_temp       = 0;

   c_busStatus = pt_handle->pt_slave->STATUS;
   ps_wrPtr    = &pt_handle->s_bufIndex;

   if( c_busStatus & TWI_SLAVE_BUSERR_bm) /*Bus error*/
   {
      pt_handle->t_status = TWI_BUS_ERROR;

      /*---------------------------------------------------------------------*
       * Signal that the transaction has finished...
       *---------------------------------------------------------------------*/
      if( pt_handle->pf_transComplete != NULL)
         c_temp = pt_handle->pf_transComplete( pt_handle->t_status, 0);

   }/*End if( c_busStatus & TWI_SLAVE_BUSERR_bm)*/
   else if( c_busStatus & TWI_SLAVE_COLL_bm) /*Bus collision*/
   {
      pt_handle->t_status = TWI_COLLISION;

      /*---------------------------------------------------------------------*
       * Signal that the transaction has finished...
       *---------------------------------------------------------------------*/
      if( pt_handle->pf_transComplete != NULL)
         c_temp = pt_handle->pf_transComplete( pt_handle->t_status, 0);

   }/*End else if( c_busStatus & TWI_SLAVE_COLL_bm)*/
   else if( (c_busStatus & TWI_SLAVE_APIF_bm) &&
            (c_busStatus & TWI_SLAVE_AP_bm)) /*Address match*/
   {
      pt_handle->t_status = TWI_TRANSACTION_BUSY;

      /*---------------------------------------------------------------------*
       * The buffer index is used during slave mode to keep track of how many
       * bytes have been read/written to and from the master device.
       *---------------------------------------------------------------------*/
      (*ps_wrPtr) = 0;

      /*---------------------------------------------------------------------*
       * Send ACK, and wait for another interrupt...
       *---------------------------------------------------------------------*/
      pt_handle->pt_slave->CTRLB = TWI_SLAVE_CMD_RESPONSE_gc;
   }
   else if( c_busStatus & TWI_SLAVE_APIF_bm) /*Stop*/
   {
      /*---------------------------------------------------------------------*
       * Clear APIF...
       *---------------------------------------------------------------------*/
      pt_handle->pt_slave->STATUS = c_busStatus | TWI_SLAVE_APIF_bm;

      pt_handle->t_status = TWI_TRANS_COMPLETE;

      /*---------------------------------------------------------------------*
       * Signal that the transaction has finished...
       *---------------------------------------------------------------------*/
      if( pt_handle->pf_transComplete != NULL)
         c_temp = pt_handle->pf_transComplete( pt_handle->t_status, 0);

   }/*End else if( c_busStatus & TWI_SLAVE_APIF_bm)*/
   else if( c_busStatus & TWI_SLAVE_DIF_bm) /*Data interrupt?*/
   {
      /*---------------------------------------------------------------------*
       * Is the master requesting a read operation?
       *---------------------------------------------------------------------*/
      if( c_busStatus & TWI_SLAVE_DIR_bm) /*Master reading from slave...*/
      {

         /*------------------------------------------------------------------*
          * Have we sent at least 1 byte and received a NACK from the master?
          *------------------------------------------------------------------*/
         if( ((*ps_wrPtr) > 0) && (pt_handle->pt_slave->STATUS &
         TWI_SLAVE_RXACK_bm))
         {
            /*---------------------------------------------------------------*
             * Transaction finished...
             *---------------------------------------------------------------*/
            pt_handle->pt_slave->CTRLB = TWI_SLAVE_CMD_COMPTRANS_gc;
         }
         else
         {
            pt_handle->t_status = TWI_SLAVE_READ;

            /*---------------------------------------------------------------*
             * Signal that the transaction has finished...
             *---------------------------------------------------------------*/
            if( pt_handle->pf_transComplete != NULL)
            {
               c_temp = pt_handle->pf_transComplete( pt_handle->t_status, 0);

               pt_handle->pt_slave->DATA = c_temp;

               /*------------------------------------------------------------*
                * Let the master know data is available...
                *------------------------------------------------------------*/
               pt_handle->pt_slave->CTRLB = TWI_SLAVE_CMD_RESPONSE_gc;

               /*------------------------------------------------------------*
                * Keep track of how much data we have sent to the master during
                * this transaction...
                *------------------------------------------------------------*/
               (*ps_wrPtr)++;
            }/*End if( pt_handle->pf_transComplete != NULL)*/

         }

      }/*End if( c_busStatus & TWI_SLAVE_DIR_bm)*/
      else /*Master writing to slave...*/
      {

         pt_handle->t_status = TWI_SLAVE_WRITE;

         /*------------------------------------------------------------------*
          * Signal that the transaction has finished...
          *------------------------------------------------------------------*/
         if( pt_handle->pf_transComplete != NULL)
         {
            c_temp = pt_handle->pf_transComplete( pt_handle->t_status,
            pt_handle->pt_slave->DATA);
         }

         /*------------------------------------------------------------------*
          * Send ACK to master...
          *------------------------------------------------------------------*/
         pt_handle->pt_slave->CTRLB = TWI_SLAVE_CMD_RESPONSE_gc;

      }

   }/*End else if( c_busStatus & TWI_SLAVE_DIF_bm)*/

}/*End hal_slaveInterrupt*/

ISR( TWIC_TWIM_vect)
{
   t_twiChanHndl *pt_handle = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this twi interrupt.
    *------------------------------------------------------------------------*/
   pt_handle = findTwiElement( (uint8_t)TWI_1);

   if( pt_handle != NULL)
   {
      hal_masterInterrupt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( TWIC_TWIM_vect)*/

ISR(TWIC_TWIS_vect)
{
   t_twiChanHndl *pt_handle = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this twi interrupt.
    *------------------------------------------------------------------------*/
   pt_handle = findTwiElement( (uint8_t)TWI_1);

   if( pt_handle != NULL)
   {
      hal_slaveInterrupt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR(TWIC_TWIS_vect)*/

ISR( TWID_TWIM_vect)
{
    t_twiChanHndl *pt_handle = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this twi interrupt.
    *------------------------------------------------------------------------*/
   pt_handle = findTwiElement( (uint8_t)TWI_2);

   if( pt_handle != NULL)
   {
      hal_masterInterrupt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( TWID_TWIM_vect)*/

ISR( TWID_TWIS_vect)
{
   t_twiChanHndl *pt_handle = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this twi interrupt.
    *------------------------------------------------------------------------*/
   pt_handle = findTwiElement( (uint8_t)TWI_2);

   if( pt_handle != NULL)
   {
      hal_slaveInterrupt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( TWID_TWIS_vect)*/

ISR( TWIE_TWIM_vect)
{
   t_twiChanHndl *pt_handle = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this twi interrupt.
    *------------------------------------------------------------------------*/
   pt_handle = findTwiElement( (uint8_t)TWI_3);

   if( pt_handle != NULL)
   {
      hal_masterInterrupt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( TWIE_TWIM_vect)*/

ISR( TWIE_TWIS_vect)
{
   t_twiChanHndl *pt_handle = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this twi interrupt.
    *------------------------------------------------------------------------*/
   pt_handle = findTwiElement( (uint8_t)TWI_3);

   if( pt_handle != NULL)
   {
      hal_slaveInterrupt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( TWIE_TWIS_vect)*/

ISR( TWIF_TWIM_vect)
{
   t_twiChanHndl *pt_handle = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this twi interrupt.
    *------------------------------------------------------------------------*/
   pt_handle = findTwiElement( (uint8_t)TWI_4);

   if( pt_handle != NULL)
   {
      hal_masterInterrupt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( TWIF_TWIM_vect)*/

ISR( TWIF_TWIS_vect)
{
   t_twiChanHndl *pt_handle = NULL;

   /*------------------------------------------------------------------------*
    * See if there is a handle on the list for this twi interrupt.
    *------------------------------------------------------------------------*/
   pt_handle = findTwiElement( (uint8_t)TWI_4);

   if( pt_handle != NULL)
   {
      hal_slaveInterrupt( pt_handle);
   }/*End if( pt_handle != NULL)*/

}/*End ISR( TWIF_TWIS_vect)*/

static t_twiChanHndl *findTwiElement( t_twiChanId t_id)
{
   t_twiChanHndl *pt_element;
   t_LINKHNDL t_linkHndl;
   int16_t s_count;

   /*------------------------------------------------------------------------*
    * Search the twi list for the requested ID
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_twiChanHndlList, s_count)
   {
      pt_element = (t_twiChanHndl *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( pt_element->t_id == t_id)
      {
         return pt_element;
      }

   }

   /*------------------------------------------------------------------------*
    * If we make it this far the ID has not been found in the open twi module
    * list.
    *------------------------------------------------------------------------*/
   return NULL;

}/*End findTwiElement*/

static t_LINKHNDL createTwiHandle( void)
{
   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Allocated memory for the link (and element) that contains information
    * specific to this particular twi module
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof( t_twiChanHndl));

   if( t_linkHndl < 0)
   {
      return (t_LINKHNDL)TWI_OUT_OF_HEAP;
   }

   /*------------------------------------------------------------------------*
    * Add the twi module link onto the list open twi modules.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_twiChanHndlList,
                           t_linkHndl,
                           true);

   return t_linkHndl;

}/*End createTwiHandle*/

static void hal_resetTwiMaster( TWI_MASTER_t *pt_master)
{
   pt_master->ADDR   = 0;
   pt_master->BAUD   = 0;
   pt_master->CTRLA  = 0;
   pt_master->CTRLB  = 0;
   pt_master->CTRLC  = 0;
   pt_master->DATA   = 0;
   pt_master->STATUS = 0;

}/*End hal_resetTwiMaster*/

static void hal_resetTwiSlave( TWI_SLAVE_t *pt_slave)
{
   pt_slave->ADDR     = 0;
   pt_slave->ADDRMASK = 0;
   pt_slave->CTRLA    = 0;
   pt_slave->CTRLB    = 0;
   pt_slave->DATA     = 0;
   pt_slave->STATUS   = 0;

}/*End hal_resetTwiSlave*/

int32_t hal_twiGetBaudRate( t_TWIHNDL t_handle)
{
   t_twiChanHndl *pt_twiChanHndl = NULL;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a twi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_twiChanHndlList) == 
   false)
   {
      return (int32_t)TWI_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where the twi module 
    * information is being stored.
    *------------------------------------------------------------------------*/
   pt_twiChanHndl = (t_twiChanHndl *)
   UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

   return (int32_t)pt_twiChanHndl->i_baudRate;

}/*End hal_twiGetBaudRate*/

/*---------------------------------------------------------------------------*
 * Request access to a particular twi module
 *---------------------------------------------------------------------------*/
t_TWIHNDL hal_requestTwiChannel( t_twiChanId t_chanId)
{
   t_twiChanHndl *pt_twiChanHndl;
   t_LINKHNDL t_linkHndl;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( (t_chanId < TWI_1) || (t_chanId > TWI_4))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_TWIHNDL)TWI_INVALID_CHAN;
   }

   pt_twiChanHndl = findTwiElement( t_chanId);

   /*------------------------------------------------------------------------*
    * Is this channel available?
    *------------------------------------------------------------------------*/
   if( pt_twiChanHndl == NULL) /*Yes*/
   {

      t_linkHndl = createTwiHandle();
      if( t_linkHndl < 0)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return (t_TWIHNDL)TWI_OUT_OF_HEAP;

      }

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the twi 
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_twiChanHndl = (t_twiChanHndl *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);

      pt_twiChanHndl->b_validConfig = false;
      pt_twiChanHndl->t_id          = t_chanId;
      pt_twiChanHndl->t_status      = TWI_IDLE;

   }/*End if( pt_twiChanHndl == NULL)*/
   else /*No*/
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_TWIHNDL)TWI_CHAN_UNAVAILABLE;
   }

   HAL_END_CRITICAL();//Enable interrupts

   return (t_TWIHNDL)t_linkHndl;

}/*End hal_requestTwiChannel*/

t_twiError hal_releaseTwiChannel( t_TWIHNDL t_handle)
{
   t_twiChanHndl *pt_twiChanHndl;
   t_linkedListError t_lErr;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a twi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_twiChanHndlList) == 
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return TWI_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the twi 
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_twiChanHndl = (t_twiChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Disable and reset this twi channel.
       *---------------------------------------------------------------------*/
      if( pt_twiChanHndl->pt_master == NULL)
         hal_resetTwiSlave( (TWI_SLAVE_t *)pt_twiChanHndl->pt_slave);
      else
         hal_resetTwiMaster( (TWI_MASTER_t *)pt_twiChanHndl->pt_master);

      t_lErr = utl_destroyLink( gt_twiChanHndlList,
                                (t_LINKHNDL)t_handle);

   }

   HAL_END_CRITICAL();//Enable interrupts

   return TWI_PASSED;

}/*End hal_releaseTwiChannel*/

t_twiError hal_configureTwiChannel( t_TWIHNDL t_handle,
                                    t_twiConfig t_conf)
{
   t_twiChanHndl *pt_twiChanHndl;
   int16_t s_twmbr = 0;
   t_gpioConf t_gConf;
   t_gpioError t_gErr;

   /*------------------------------------------------------------------------*
    * We are going to be configuring registers and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( (t_conf.t_mode < TWI_MASTER) || (t_conf.t_mode > TWI_SLAVE))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return TWI_INVALID_MODE;
   }

   if( (t_conf.t_mode == TWI_SLAVE) && 
       (t_conf.c_slaveAddress > TWI_MAX_SLAVE_ADDRESS))
   {
      HAL_END_CRITICAL();//Enable interrupts
      return TWI_INVALID_SLAVE_ADDR;
   }

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a twi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_twiChanHndlList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return TWI_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the twi 
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_twiChanHndl = (t_twiChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Configure the output pins the TWI will use.
       *---------------------------------------------------------------------*/
      if( t_conf.t_mode == TWI_MASTER)
      {
         t_gConf.c_inputMask    = 0;
         t_gConf.c_outputMask   = PIN_0 | PIN_1;
         t_gConf.b_setOutputLow = true;
         t_gConf.t_outConf      = TOTEM;
      }
      else
      {
         t_gConf.c_inputMask    = PIN_0 | PIN_1;
         t_gConf.c_outputMask   = 0;
         t_gConf.b_setOutputLow = true;
         t_gConf.t_inConf       = PULLUP;
      }

      switch( pt_twiChanHndl->t_id)
      {
         case TWI_1:

            pt_twiChanHndl->pt_master = &TWIC.MASTER;
            pt_twiChanHndl->pt_slave  = &TWIC.SLAVE;
            /*---------------------------------------------------------------*
             * Configure the output pins the TWI will use.
             *---------------------------------------------------------------*/
            t_gErr = hal_configureGpioPort( GPIO_PORTC, t_gConf);

         break;

         case TWI_2:

            pt_twiChanHndl->pt_master = &TWID.MASTER;
            pt_twiChanHndl->pt_slave  = &TWID.SLAVE;
            /*---------------------------------------------------------------*
             * Configure the output pins the TWI will use.
             *---------------------------------------------------------------*/
            t_gErr = hal_configureGpioPort( GPIO_PORTD, t_gConf);

         break;

         case TWI_3:

            pt_twiChanHndl->pt_master = &TWIE.MASTER;
            pt_twiChanHndl->pt_slave  = &TWIE.SLAVE;
            /*---------------------------------------------------------------*
             * Configure the output pins the TWI will use.
             *---------------------------------------------------------------*/
            t_gErr = hal_configureGpioPort( GPIO_PORTE, t_gConf);

         break;

         case TWI_4:

            pt_twiChanHndl->pt_master = &TWIF.MASTER;
            pt_twiChanHndl->pt_slave  = &TWIF.SLAVE;
            /*---------------------------------------------------------------*
             * Configure the output pins the TWI will use.
             *---------------------------------------------------------------*/
            t_gErr = hal_configureGpioPort( GPIO_PORTF, t_gConf);

         break;

      }/*End switch( pt_twiChanHndl->t_id)*/

      hal_resetTwiMaster( (TWI_MASTER_t *)pt_twiChanHndl->pt_master);
      hal_resetTwiSlave( (TWI_SLAVE_t *)pt_twiChanHndl->pt_slave);

      if( t_conf.t_mode == TWI_MASTER)
      {

         /*------------------------------------------------------------------*
          * Calculate the baud register setting which is defined in the users
          * manual. Where s_twmbr can range from 0 to 255, representing a max
          * frequency of getCpuFreq() / 5 and min frequency of getCpuFreq() /
          * 520.
          *------------------------------------------------------------------*/
         s_twmbr = (hal_getCpuFreq() / (2*t_conf.i_baud)) - 5;

         if( (s_twmbr <= 0) || (s_twmbr > 255))
         {
            HAL_END_CRITICAL();//Enable interrupts
            return TWI_INVALID_BAUD_RATE;
         }

         /*------------------------------------------------------------------*
          * We are not using the slave channel.
          *------------------------------------------------------------------*/
         pt_twiChanHndl->pt_slave = NULL;

         //if( t_conf.b_mastPolling == false)
         //{
            /*---------------------------------------------------------------*
             * Configure high level interrupts.
             *---------------------------------------------------------------*/
         //   pt_twiChanHndl->pt_master->CTRLA |= TWI_MASTER_INTLVL_HI_gc;

            /*---------------------------------------------------------------*
             * Enable read and write interrupts.
             *---------------------------------------------------------------*/
         //   pt_twiChanHndl->pt_master->CTRLA |= TWI_MASTER_RIEN_bm;
         //   pt_twiChanHndl->pt_master->CTRLA |= TWI_MASTER_WIEN_bm;

         //}/*End if( t_conf.b_mastPolling == false)*/

         /*------------------------------------------------------------------*
          * Enable master mode.
          *------------------------------------------------------------------*/
         pt_twiChanHndl->pt_master->CTRLA |= TWI_MASTER_ENABLE_bm;

         /*------------------------------------------------------------------*
          * Configure the baud rate.
          *------------------------------------------------------------------*/
         pt_twiChanHndl->pt_master->BAUD = s_twmbr;

         /*------------------------------------------------------------------*
          * Configure the state of the bus.
          *------------------------------------------------------------------*/
         pt_twiChanHndl->pt_master->STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;

         /*------------------------------------------------------------------*
          * Back calculate the 'true' baud by taking the reverse of the
          * equation used to calculate s_twmbr.
          *------------------------------------------------------------------*/
         pt_twiChanHndl->i_baudRate = (uint32_t)(hal_getCpuFreq() /
         ((uint16_t)2*((uint16_t)s_twmbr + (uint16_t)5)));

      }/*End if( t_conf.t_mode == TWI_MASTER)*/
      else
      {

         /*------------------------------------------------------------------*
          * We are not using the master channel.
          *------------------------------------------------------------------*/
         pt_twiChanHndl->pt_master = NULL;

         /*------------------------------------------------------------------*
          * Configure high level interrupts.
          *------------------------------------------------------------------*/
         pt_twiChanHndl->pt_slave->CTRLA |= TWI_SLAVE_INTLVL_HI_gc;

         /*------------------------------------------------------------------*
          * Enable data and stop interrupts.
          *------------------------------------------------------------------*/
         pt_twiChanHndl->pt_slave->CTRLA |= TWI_SLAVE_DIEN_bm;
         pt_twiChanHndl->pt_slave->CTRLA |= TWI_SLAVE_APIEN_bm;
         pt_twiChanHndl->pt_slave->CTRLA |= TWI_SLAVE_PIEN_bm;

         /*------------------------------------------------------------------*
          * Enable slave mode.
          *------------------------------------------------------------------*/
         pt_twiChanHndl->pt_slave->CTRLA |= TWI_SLAVE_ENABLE_bm;

         /*------------------------------------------------------------------*
          * Set the slave address.
          *------------------------------------------------------------------*/
         pt_twiChanHndl->pt_slave->ADDR =  t_conf.c_slaveAddress << 1;

         /*------------------------------------------------------------------*
          * Store the call-back function
          *------------------------------------------------------------------*/
         pt_twiChanHndl->pf_transComplete = t_conf.pf_transComplete;

      }

      /*---------------------------------------------------------------------*
       * Reset the bus status.
       *---------------------------------------------------------------------*/
      pt_twiChanHndl->t_status = TWI_IDLE;

      /*---------------------------------------------------------------------*
       * Configuration completed.
       *---------------------------------------------------------------------*/
      pt_twiChanHndl->b_validConfig = true;

   }

   HAL_END_CRITICAL();//Enable interrupts

   return TWI_PASSED;

}/*End hal_configureTwiChannel*/

t_twiError hal_twiMasterWrite( t_TWIHNDL t_handle,
                               uint8_t *pc_data,
                               uint16_t s_length,
                               uint8_t c_slaveAdd,
                               int8_t c_numRetries)
{
   volatile t_twiChanHndl *pt_twiChanHndl;
   volatile t_sysTime t_currTime;
   volatile int32_t i_deltaUsec;
   
   /*------------------------------------------------------------------------*
    * Is this a valid handle to a twi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_twiChanHndlList) == 
   false)
   {
      return TWI_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the twi 
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_twiChanHndl = (t_twiChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Is this handle opened against a master twi?
       *---------------------------------------------------------------------*/
      if( pt_twiChanHndl->pt_master != NULL)
      {
         
         pt_twiChanHndl->t_status = TWI_TRANSACTION_BUSY;

         /*------------------------------------------------------------------*
          * Keep retrying the transaction until it completes, or hits the
          * timeout condition.
          *------------------------------------------------------------------*/
         while( (pt_twiChanHndl->t_status != TWI_TRANS_COMPLETE) &&
                (c_numRetries >= 0))
         {

            /*---------------------------------------------------------------*
             * Wait for the bus to become available...
             *---------------------------------------------------------------*/
            while( !((pt_twiChanHndl->pt_master->STATUS & 
            TWI_MASTER_BUSSTATE_gm) == TWI_MASTER_BUSSTATE_IDLE_gc))
            {
            }

            /*---------------------------------------------------------------*
             * Store the address and length of the data buffer this particular
             * twi is accessing.
             *---------------------------------------------------------------*/
            pt_twiChanHndl->pc_data    = pc_data;
            pt_twiChanHndl->s_length   = s_length;
            pt_twiChanHndl->s_bufIndex = 0;

            pt_twiChanHndl->t_lastIntTime = arb_sysTimeNow();

            /*---------------------------------------------------------------*
             * Begin write operation by copying the slave address into this
             * particular twi's slave address register and setting the write 
             * bit.
             *---------------------------------------------------------------*/
            pt_twiChanHndl->pt_master->ADDR = (c_slaveAdd << 1) & ~0x01;

            while( !((pt_twiChanHndl->pt_master->STATUS & 
            TWI_MASTER_BUSSTATE_gm) == TWI_MASTER_BUSSTATE_IDLE_gc))
            {

               hal_masterInterrupt( pt_twiChanHndl);

               t_currTime = arb_sysTimeNow();

               i_deltaUsec = t_currTime.i_usec - pt_twiChanHndl->t_lastIntTime.
               i_usec;

               if( i_deltaUsec < 0)
                  i_deltaUsec = TWI_BUS_RESET_TIMEOUT;

               if( i_deltaUsec >= 1000000)
                  i_deltaUsec -= 1000000;

               /*------------------------------------------------------------*
                * Have we encountered a bus lock condition?
                *------------------------------------------------------------*/
               if( i_deltaUsec >= TWI_BUS_RESET_TIMEOUT)
               {
                  /*---------------------------------------------------------*
                   * Reset the module so that we can recover from the locked
                   * bus condition...
                   *---------------------------------------------------------*/
                  pt_twiChanHndl->pt_master->CTRLA &= ~TWI_MASTER_ENABLE_bm;
                  pt_twiChanHndl->pt_master->CTRLA |= TWI_MASTER_ENABLE_bm;
                  pt_twiChanHndl->t_status = TWI_BUS_LOCKED;

                  pt_twiChanHndl->pt_master->STATUS = 
                  TWI_MASTER_BUSSTATE_IDLE_gc;

               }/*End if( i_deltaUsec >= TWI_BUS_RESET_TIMEOUT)*/

            }

            c_numRetries--;
         }

      }/*End if( pt_twiChanHndl->pt_master != NULL)*/
      else
         return TWI_CHAN_NOT_MASTER;
      
   }

   return TWI_PASSED;

}/*End hal_twiMasterWrite*/

t_twiStatus hal_getTwiStatus( t_TWIHNDL t_handle)
{
   t_twiChanHndl *pt_twiChanHndl = NULL;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a twi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_twiChanHndlList) == 
   false)
   {
      return TWI_UNKNOWN;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the twi 
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_twiChanHndl = (t_twiChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      return pt_twiChanHndl->t_status;
   }

   return TWI_UNKNOWN;

}/*End hal_getTwiStatus*/

t_twiError hal_twiMasterRead( t_TWIHNDL t_handle,
                              uint8_t *pc_data,
                              uint16_t s_length,
                              uint8_t c_slaveAdd,
                              int8_t c_numRetries)
{
   volatile t_twiChanHndl *pt_twiChanHndl;
   volatile t_sysTime t_currTime;
   volatile int32_t i_deltaUsec;

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a twi module?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_twiChanHndlList) == 
   false)
   {
      return TWI_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the twi 
       * module information is being stored.
       *---------------------------------------------------------------------*/
      pt_twiChanHndl = (t_twiChanHndl *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Is this handle opened against a master twi?
       *---------------------------------------------------------------------*/
      if( pt_twiChanHndl->pt_master != NULL)
      {

         pt_twiChanHndl->t_status = TWI_TRANSACTION_BUSY;

         /*------------------------------------------------------------------*
          * Keep retrying the transaction until it completes, or hits the
          * timeout condition.
          *------------------------------------------------------------------*/
         while( (pt_twiChanHndl->t_status != TWI_TRANS_COMPLETE) &&
                (c_numRetries >= 0))
         {

            /*---------------------------------------------------------------*
             * Wait for the bus to become available...
             *---------------------------------------------------------------*/
            while( !((pt_twiChanHndl->pt_master->STATUS & 
            TWI_MASTER_BUSSTATE_gm) == TWI_MASTER_BUSSTATE_IDLE_gc))
            {
            }

            /*---------------------------------------------------------------*
             * Store the address and length of the data buffer this particular
             * twi is accessing.
             *---------------------------------------------------------------*/
            pt_twiChanHndl->pc_data    = pc_data;
            pt_twiChanHndl->s_length   = s_length;
            pt_twiChanHndl->s_bufIndex = 0;

            pt_twiChanHndl->t_lastIntTime = arb_sysTimeNow();

            /*---------------------------------------------------------------*
             * Begin read operation by copying the slave address into this
             * particular twi's slave address register and setting the read bit.
             *---------------------------------------------------------------*/
            pt_twiChanHndl->pt_master->ADDR = (c_slaveAdd << 1) | 0x01;

            while( !((pt_twiChanHndl->pt_master->STATUS & 
            TWI_MASTER_BUSSTATE_gm)
            == TWI_MASTER_BUSSTATE_IDLE_gc))
            {

               hal_masterInterrupt( pt_twiChanHndl);

               t_currTime = arb_sysTimeNow();

               i_deltaUsec = t_currTime.i_usec - pt_twiChanHndl->t_lastIntTime.
               i_usec;

               if( i_deltaUsec >= 1000000)
                  i_deltaUsec -= 1000000;

               /*------------------------------------------------------------*
                * Have we encountered a bus lock condition?
                *------------------------------------------------------------*/
               if( i_deltaUsec >= TWI_BUS_RESET_TIMEOUT)
               {

                  /*---------------------------------------------------------*
                   * Reset the module so that we can recover from the locked
                   * bus condition...
                   *---------------------------------------------------------*/
                  pt_twiChanHndl->pt_master->CTRLA &= ~TWI_MASTER_ENABLE_bm;
                  pt_twiChanHndl->pt_master->CTRLA |= TWI_MASTER_ENABLE_bm;
                  pt_twiChanHndl->t_status = TWI_BUS_LOCKED;

                  pt_twiChanHndl->pt_master->STATUS = 
                  TWI_MASTER_BUSSTATE_IDLE_gc;

               }/*End if( i_deltaUsec >= TWI_BUS_RESET_TIMEOUT)*/

            }
         }

      }/*End if( pt_twiChanHndl->pt_master != NULL)*/
      else
         return TWI_CHAN_NOT_MASTER;

   }

   return TWI_PASSED;

}/*End hal_twiMasterRead*/
