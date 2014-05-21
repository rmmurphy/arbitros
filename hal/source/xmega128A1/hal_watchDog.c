/*---------------------------------------------------------------------------*
 * Copyright (C) 2012 Ryan M. Murphy (ryan.m.murphy.77@gmail.com)
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
 * File Name   : hal_watchDog.c
 *
 * Project     : Arbitros 
 *               https://code.google.com/p/arbitros/
 *               
 * Description : This file is responsible for controlling access to the
 *               watchdog timer.
 *
 * Last Update : Nov 21, 2012
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include "utl_linkedList.h"
#include "hal_watchDog.h"
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
    * If true, then 'hal_configureWd' successfully completed
    *------------------------------------------------------------------------*/
   bool b_validConfig;

   /*------------------------------------------------------------------------*
    * The watchdog timer period.
    *------------------------------------------------------------------------*/
   uint32_t i_period;
    
}t_wdObject;

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
 
/*---------------------------------------------------------------------------*
 * Linked-list of all the currently active watchdog timer handles.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_wdAccessList);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
 
/*---------------------------------------------------------------------------*
 * Request access to the watchdog timer.
 *---------------------------------------------------------------------------*/
t_WDHNDL hal_requestWdAccess( void)
{
   t_wdObject *pt_element;
   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is the watchdog available?
    *------------------------------------------------------------------------*/
   if( UTL_GET_NUM_LINKS_CONT( gt_wdAccessList) == 0)
   {
      
      /*---------------------------------------------------------------------*
       * Allocated memory for the link where the DMA channel information will 
       * be stored.
       *---------------------------------------------------------------------*/
      t_linkHndl = utl_createLink( sizeof(t_wdObject));

      if( t_linkHndl < 0)
      {
         return (t_LINKHNDL)WD_OUT_OF_HEAP;
      }

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the WD 
       * timer information is being stored.
       *---------------------------------------------------------------------*/
      pt_element = (t_wdObject *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      pt_element->b_validConfig = false;
      pt_element->i_period = 0;

      /*---------------------------------------------------------------------*
       * Add the DMA channel link onto the list open channels.
       *---------------------------------------------------------------------*/
      t_err = utl_insertLink( gt_wdAccessList,
                              t_linkHndl,
                              true);	
   }
   else
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_WDHNDL)WD_UNAVAILABLE;	
   }
    
   HAL_END_CRITICAL();//Enable interrupts
   
   return (t_WDHNDL)t_linkHndl;

}/*End hal_requestWdAccess*/

t_wdError hal_releaseWdAccess( t_WDHNDL t_handle)
{
   t_wdObject *pt_element;
   t_linkedListError t_lErr;
   uint8_t c_temp;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Is this a valid handle to a DMA channel?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_wdAccessList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return WD_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_element = (t_wdObject *)
        UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Disable the watchdog timer.
       *---------------------------------------------------------------------*/
       CCP = CCP_IOREG_gc;

      /*---------------------------------------------------------------------*
       * Reset the watchdog timer 
       *---------------------------------------------------------------------*/
       c_temp = 0;
      WDT.CTRL = c_temp;

      t_lErr = utl_destroyLink( gt_wdAccessList,
                                (t_LINKHNDL)t_handle);
   }

   HAL_END_CRITICAL();//Enable interrupts

   return WD_PASSED;

}/*End hal_releaseWdAccess*/

/*---------------------------------------------------------------------------*
 * Configures the DMA transaction for the channel pointed to by
 * 't_handle'.
 *---------------------------------------------------------------------------*/
t_wdError hal_configureWd( t_WDHNDL t_handle,
                           t_wdConfig t_conf)
{
   t_wdObject *pt_element;
   WDT_PER_t t_per;
   uint8_t c_temp;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( t_conf.i_period <= 8) /*msec*/
    {
       t_per = WDT_PER_8CLK_gc;
    }
    else if( t_conf.i_period <= 16) /*msec*/
    {
       t_per = WDT_PER_16CLK_gc;
    }
    else if( t_conf.i_period <= 32) /*msec*/
    {
       t_per = WDT_PER_32CLK_gc;
    }
    else if( t_conf.i_period <= 64) /*msec*/
    {
       t_per = WDT_PER_64CLK_gc;
    }
    else if( t_conf.i_period <= 125) /*msec*/
    {
       t_per = WDT_PER_128CLK_gc;
    }
    else if( t_conf.i_period <= 250) /*msec*/
    {
       t_per = WDT_PER_256CLK_gc;
    }
    else if( t_conf.i_period <= 500) /*msec*/
    {
       t_per = WDT_PER_512CLK_gc;
    }
    else if( t_conf.i_period <= 1000) /*msec*/
    {
       t_per = WDT_PER_1KCLK_gc;
    }
    else if( t_conf.i_period <= 2000) /*msec*/
    {
       t_per = WDT_PER_2KCLK_gc;
    }
    else if( t_conf.i_period <= 4000) /*msec*/
    {
       t_per = WDT_PER_4KCLK_gc;
    }
    else if( t_conf.i_period <= 8000) /*msec*/
    {
       t_per = WDT_PER_8KCLK_gc;
    }
    else
    {
      HAL_END_CRITICAL();//Enable interrupts
      return WD_INVALID_PERIOD;	
    }
    
   /*------------------------------------------------------------------------*
    * Is this a valid handle to a watchdog timer?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_wdAccessList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return WD_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_element = (t_wdObject *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Reset the watchdog
       *---------------------------------------------------------------------*/
      c_temp = 0;
      CCP = CCP_IOREG_gc;
      WDT.CTRL = c_temp;

      /*---------------------------------------------------------------------*
       * Configure the period
       *---------------------------------------------------------------------*/
      c_temp = (uint8_t)(WDT_ENABLE_bm | WDT_CEN_bm | t_per);
      CCP = CCP_IOREG_gc;
      WDT.CTRL = c_temp;
 
      pt_element->i_period      = t_conf.i_period;
      pt_element->b_validConfig = true;

      /*---------------------------------------------------------------------*
       * Wait for WD to synchronize with new settings.
       *---------------------------------------------------------------------*/
      while(HAL_WD_IS_SYNC_BUSY())
      {

      }
      
   }

   HAL_END_CRITICAL();//Enable interrupts

   return WD_PASSED;

}
                                        
/*---------------------------------------------------------------------------*
 * This function disables the watchdog timer.
 *---------------------------------------------------------------------------*/	
t_wdError hal_wdDisable( t_WDHNDL t_handle)
{
   t_wdObject *pt_element;
    uint8_t c_temp;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts
    
   /*------------------------------------------------------------------------*
    * Is this a valid handle to a watchdog timer?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_wdAccessList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return WD_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_element = (t_wdObject *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      /*---------------------------------------------------------------------*
       * Disable the watchdog
       *---------------------------------------------------------------------*/
      c_temp = (~WDT_ENABLE_bm | WDT_CEN_bm);
      CCP = CCP_IOREG_gc;
      WDT.CTRL &= c_temp;
   }
    
   HAL_END_CRITICAL();//Enable interrupts

   return WD_PASSED;
    
}/*End hal_wdDisable*/

t_wdError hal_wdEnable( t_WDHNDL t_handle)
{
   t_wdObject *pt_element;
    uint8_t c_temp;

   /*------------------------------------------------------------------------*
    * We are going to be configuring pin register and can't have another
    * thread try and access them why the change is happening.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts
    
   /*------------------------------------------------------------------------*
    * Is this a valid handle to a watchdog timer?
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_handle, gt_wdAccessList) == false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return WD_INVALID_HANDLE;
   }
   else /*Yes...*/
   {
      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the DMA
       * channel information is being stored.
       *---------------------------------------------------------------------*/
      pt_element = (t_wdObject *)
      UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)t_handle);

      if( pt_element->b_validConfig == false)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return WD_NO_CONFIG;
      }

      /*---------------------------------------------------------------------*
       * Enable the watchdog
       *---------------------------------------------------------------------*/
      c_temp = (WDT_ENABLE_bm | WDT_CEN_bm);
      CCP = CCP_IOREG_gc;
      WDT.CTRL |= c_temp;
        
      /*---------------------------------------------------------------------*
       * Wait for WD to synchronize with new settings.
       *---------------------------------------------------------------------*/
      while(HAL_WD_IS_SYNC_BUSY())
      {

      }		
   }
    
   HAL_END_CRITICAL();//Enable interrupts

   return WD_PASSED;
    
}/*End hal_wdEnable*/


