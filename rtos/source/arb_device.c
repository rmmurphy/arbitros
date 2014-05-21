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
 * File Name   : arb_device.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file presents the standard API for interfacing between
 *               a user space application, kernel, and device driver.
 *
 * Last Update : June, 12, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <string.h>
#include "arb_device.h"
#include "utl_linkedList.h"
#include "hal_pmic.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static void arb_initDevice( t_device **pt_newDevice,
                            t_deviceOperations **pt_devOps,
                            char **pc_name,
                            t_deviceId t_devId);

static void arb_initDeviceHandle( t_devHandle **pt_new,
                                  t_device **pt_dev,
                                  char **pc_fileName,
                                  uint8_t c_flags);

static t_LINKHNDL arb_getDeviceLinkByName( char *pc_name);

static t_LINKHNDL arb_getDeviceLinkById( t_deviceId t_devId);

/*---------------------------------------------------------------------------*
 * Linked list of all the devices (drivers) that the kernel and user space
 * have access to. Every time a driver's 'init' routine is called a device
 * structure is added to this list.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_activeDevices);

/*---------------------------------------------------------------------------*
 * Linked list of all the open handles to devices on the system. Every time
 * a user space application invokes an 'open' call to a particular driver
 * the open handle is added to this list. It represents the primary mechanism
 * for interfacing how a user-space thread accesses the driver through the
 * kernel.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_activeHandles);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void arb_initDevice( t_device **pt_newDevice,
                            t_deviceOperations **pt_devOps,
                            char **pc_name,
                            t_deviceId t_devId)
{

   /*------------------------------------------------------------------------*
    * Initialize this new memory region to 0
    *------------------------------------------------------------------------*/
   memset( (*pt_newDevice), 0, sizeof( t_device));

   (*pt_newDevice)->t_devId = t_devId;

   (*pt_newDevice)->pt_devOps = (*pt_devOps);

   strncpy( (char *)(*pt_newDevice)->ac_deviceName, (char *)(*pc_name),
   MAX_DEVICE_NAME_BYTES);

}/*End arb_initDevice*/

static void arb_initDeviceHandle( t_devHandle **pt_new,
                                  t_device **pt_dev,
                                  char **pc_fileName,
                                  uint8_t c_flags)
{

   /*------------------------------------------------------------------------*
    * Initialize this new memory region to 0
    *------------------------------------------------------------------------*/
   memset( (*pt_new), 0, sizeof( t_devHandle));

   /*------------------------------------------------------------------------*
    * Increment the count of handles opened against this particular device
    * driver.
    *------------------------------------------------------------------------*/
   (*pt_dev)->c_numDevHandles++;

   (*pt_new)->pt_dev = (*pt_dev);

   (*pt_new)->c_flags = c_flags;

   /*------------------------------------------------------------------------*
    * Temporarily use the private data ptr to store the location of the
    * file name- this is only used for files being stored on the hard drive.
    *------------------------------------------------------------------------*/
   (*pt_new)->pv_privateData = (void *)(*pc_fileName);

}/*End arb_initDeviceHandle*/

static t_LINKHNDL arb_getDeviceLinkByName( char *pc_name)
{
   t_device *pt_curr;
   t_LINKHNDL t_linkHndl;
   int16_t s_count;

   /*------------------------------------------------------------------------*
    * Find the device with a name of pc_name
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_activeDevices, s_count)
   {
      pt_curr = (t_device *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( strcmp( (char *)pt_curr->ac_deviceName, (char *)pc_name) == 0)
      {
         return t_linkHndl;
      }

   }

   /*------------------------------------------------------------------------*
    * If we make it this far the name has not been found in the device list
    *------------------------------------------------------------------------*/
   return (t_LINKHNDL)ARB_DEVICE_NOT_FOUND;

}/*End arb_getDeviceLinkByName*/

static t_LINKHNDL arb_getDeviceLinkById( t_deviceId t_devId)
{
   t_device *pt_curr;
   t_LINKHNDL t_linkHndl;
   int16_t s_count;

   /*------------------------------------------------------------------------*
    * Find the device with an ID of t_devId
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_activeDevices, s_count)
   {
      pt_curr = (t_device *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( pt_curr->t_devId == t_devId)
      {
         return t_linkHndl;
      }

   }

   /*------------------------------------------------------------------------*
    * If we make it this far the name has not been found in the device list
    *------------------------------------------------------------------------*/
   return (t_LINKHNDL)ARB_DEVICE_NOT_FOUND;

}/*End arb_getDeviceLinkById*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * This function returns a handle to the list of active device drivers.
 *---------------------------------------------------------------------------*/
t_CONTHNDL arb_getDeviceList( void)
{
   return gt_activeDevices;
}/*End arb_getDeviceList*/

t_DEVHANDLE arb_open( char *pc_name,
                      uint8_t c_flags)
{
   t_devHandle *pt_devHandle;
   t_device *pt_device;
   t_error t_err;
   t_LINKHNDL t_devLink;
   t_LINKHNDL t_hndlLink;
   char *pc_devName;
   char *pc_fileName;
   int16_t s_size1;
   int16_t s_size2;

   /*------------------------------------------------------------------------*
    * A new handle is being dynamically created make sure operation acting on
    * the global variable are atomic.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * The device name is located at the begging of 'pc_name' and can be
    * found by extracting the characters up until the first '/'.
    *------------------------------------------------------------------------*/
   s_size1 = strlen( pc_name);

   pc_devName = strtok( pc_name, "/");

   s_size1 -= (strlen( pc_devName) + 1);

   /*------------------------------------------------------------------------*
    * The characters after the first '/' represent the file path
    *------------------------------------------------------------------------*/
   pc_fileName = strtok( NULL, "/");

   s_size2 = strlen( pc_fileName);

   /*------------------------------------------------------------------------*
    * Are there characters left after the second token?
    *------------------------------------------------------------------------*/
   if( (s_size1 > s_size2) && (pc_fileName != NULL)) /*Yes*/
   {

      /*---------------------------------------------------------------------*
       * Replace the backslash removed by strtok.
       *---------------------------------------------------------------------*/
      pc_fileName[s_size2] = '/';

   }

   /*------------------------------------------------------------------------*
    * Search the active device list for a device with this name...
    *------------------------------------------------------------------------*/
   t_devLink = arb_getDeviceLinkByName( pc_devName);

   if( t_devLink == (t_LINKHNDL)ARB_DEVICE_NOT_FOUND)
   {
      HAL_END_CRITICAL(); //Enable interrupts
      return (t_DEVHANDLE)ARB_DEVICE_NOT_FOUND;
   }/*End if( t_devLink == (t_LINKHNDL)ARB_DEVICE_NOT_FOUND)*/

   /*------------------------------------------------------------------------*
    * Create a new 'link' in the list which we will use as a  'user space'
    * handle to this device
    *-----------------------------------------------------------------------*/
   t_hndlLink = utl_createLink( sizeof(t_devHandle));

   if( t_hndlLink < 0)
   {
      HAL_END_CRITICAL(); //Enable interrupts
      return (t_DEVHANDLE)ARB_OUT_OF_HEAP;
   }/*End if( t_hndlLink < 0)*/

   /*------------------------------------------------------------------------*
    * Grab the record for this particular device driver the handle is being
    * opened against.
    *------------------------------------------------------------------------*/
   pt_device = (t_device *)UTL_GET_LINK_ELEMENT_PTR(t_devLink);

   /*------------------------------------------------------------------------*
    * Grab the a ptr to the record for the user-space handle
    *------------------------------------------------------------------------*/
   pt_devHandle = (t_devHandle *)UTL_GET_LINK_ELEMENT_PTR(t_hndlLink);

   /*------------------------------------------------------------------------*
    * Populate the record with information about the device it is going to
    * access.
    *------------------------------------------------------------------------*/
   arb_initDeviceHandle( &pt_devHandle,
                         &pt_device,
                         &pc_fileName,
                         c_flags);

   /*------------------------------------------------------------------------*
    * Add the new handle to this particular device on the list of open
    * user-space handles.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_activeHandles,
                           t_hndlLink,
                           true);

   /*------------------------------------------------------------------------*
    * Finished creating handle, enable interrupts
    *------------------------------------------------------------------------*/
   HAL_END_CRITICAL();

   /*------------------------------------------------------------------------*
    * Call the device driver's 'open' routine
    *------------------------------------------------------------------------*/
   if( pt_devHandle->pt_dev->pt_devOps != NULL)
   {
      t_err = pt_devHandle->pt_dev->pt_devOps->pf_open( (t_DEVHANDLE)
      pt_devHandle);

      if( t_err < 0)
      {

         /*------------------------------------------------------------------*
          * Decrement the count of handles opened against this particular
          * device driver.
          *------------------------------------------------------------------*/
         pt_devHandle->pt_dev->c_numDevHandles--;

         utl_destroyLink( gt_activeHandles,
                          t_hndlLink);

         return (t_DEVHANDLE)t_err;
      }
   }
   else
   {
      return (t_DEVHANDLE)ARB_NULL_PTR;
   }

   return (t_DEVHANDLE)t_hndlLink;

}/*End open*/

int16_t arb_read( t_DEVHANDLE t_handle,
                  int8_t *pc_buff,
                  uint16_t s_size)
{
   t_devHandle *pt_devHandle = (t_devHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);
   int16_t s_amountRead = 0;

   /*------------------------------------------------------------------------*
    * If the handle is valid, call the device's 'read' method
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( t_handle, gt_activeHandles))
   {
      if( pt_devHandle->pt_dev->pt_devOps != NULL)
      {
         s_amountRead = pt_devHandle->pt_dev->pt_devOps->pf_read(
         (t_DEVHANDLE)pt_devHandle, pc_buff, s_size);
      }
      else
         s_amountRead = (int16_t)ARB_NULL_PTR;

   }/*End if( UTL_IS_LINK_ON_LIST( t_handle, gt_activeHandles))*/
   else
      s_amountRead = (int16_t)ARB_INVALID_HANDLE;

   return s_amountRead;

}/*End arb_read*/

int16_t arb_write( t_DEVHANDLE t_handle,
                   int8_t *pc_buff,
                   uint16_t s_size)
{
   t_devHandle *pt_devHandle = (t_devHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);
   int16_t s_amountWrote     = (int16_t)ARB_PASSED;

   /*------------------------------------------------------------------------*
    * If the handle is valid, call the device's 'write' method
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( t_handle, gt_activeHandles))
   {
      if( pt_devHandle->pt_dev->pt_devOps != NULL)
      {
         s_amountWrote = pt_devHandle->pt_dev->pt_devOps->pf_write(
         (t_DEVHANDLE)pt_devHandle, pc_buff, s_size);
      }
      else
         s_amountWrote = (int16_t)ARB_NULL_PTR;

   }/*End if( UTL_IS_LINK_ON_LIST( t_handle, gt_activeHandles))*/
   else
      s_amountWrote = (int16_t)ARB_INVALID_HANDLE;

   return s_amountWrote;

}/*End arb_write*/

int8_t *arb_getDevName( t_DEVHANDLE t_handle)
{
   t_devHandle *pt_devHandle = (t_devHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);

   /*------------------------------------------------------------------------*
    * If the handle is valid, call the device's 'write' method
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( t_handle, gt_activeHandles))
   {
      return pt_devHandle->pt_dev->ac_deviceName;
   }/*End if( UTL_IS_LINK_ON_LIST( t_handle, gt_activeHandles))*/

   return NULL;

}/*End arb_getDevName*/

int32_t arb_ioctl( t_DEVHANDLE t_handle,
                   uint16_t s_command,
                   int32_t i_argument)
{
   t_devHandle *pt_devHandle = (t_devHandle *)
   UTL_GET_LINK_ELEMENT_PTR(t_handle);
   int32_t i_status = (int32_t)ARB_PASSED;

   /*------------------------------------------------------------------------*
    * If the handle is valid, call the device's 'ioctl' method
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( t_handle, gt_activeHandles))
   {
      if( pt_devHandle->pt_dev->pt_devOps != NULL)
      {
         i_status = pt_devHandle->pt_dev->pt_devOps->pf_ioctl( (t_DEVHANDLE)
         pt_devHandle, s_command, i_argument);
      }
      else
         i_status = (int32_t)ARB_NULL_PTR;

   }/*End if( UTL_IS_LINK_ON_LIST( t_handle, gt_activeHandles))*/
   else
      i_status = (int32_t)ARB_INVALID_HANDLE;

   return i_status;

}/*End arb_ioctl*/

t_error arb_close( t_DEVHANDLE t_handle)
{
   t_devHandle *pt_devHandle = (t_devHandle *)UTL_GET_LINK_ELEMENT_PTR(\
   (t_LINKHNDL)t_handle);
   t_error t_err;

   /*------------------------------------------------------------------------*
    * If this is a valid 'user-space' handle then close the connection...
    *------------------------------------------------------------------------*/
   if( UTL_IS_LINK_ON_LIST( t_handle, gt_activeHandles))
   {

      /*---------------------------------------------------------------------*
       * Call device driver's close method.
       *---------------------------------------------------------------------*/
      if( pt_devHandle->pt_dev->pt_devOps != NULL)
      {
         t_err = pt_devHandle->pt_dev->pt_devOps->pf_close( (t_DEVHANDLE)
         pt_devHandle);

         if( t_err < 0)
            return t_err;

         /*------------------------------------------------------------------*
          * Decrement the count of handles opened against this particular
          * device driver.
          *------------------------------------------------------------------*/
         pt_devHandle->pt_dev->c_numDevHandles--;

         t_err = utl_destroyLink( gt_activeHandles,
                                  (t_LINKHNDL)t_handle);
      }
      else
         return ARB_NULL_PTR;

   }/*End if( UTL_IS_LINK_ON_LIST( t_handle, gt_activeHandles))*/
   else
      return ARB_INVALID_HANDLE;

   return ARB_PASSED;

}/*End arb_close*/

t_error arb_registerDevice( char *pc_name,
                            t_deviceId t_devId,
                            t_deviceOperations *pt_devOps)
{
   t_device *pt_device;
   t_LINKHNDL t_devLink;
   t_LINKHNDL t_devLink2;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Since we are about to act on global variables- in this case the
    * handle to the device driver, we need to protect against interruption
    * from another process while the driver is being added to the list.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   if( strlen( pc_name) > MAX_DEVICE_NAME_BYTES)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return ARB_NAME_ERROR;
   }/*End if( s_length > MAX_DEVICE_NAME_BYTES)*/

   /*------------------------------------------------------------------------*
    * Check to see if there is another device with the same name
    *------------------------------------------------------------------------*/
   t_devLink = arb_getDeviceLinkByName( pc_name);

   /*------------------------------------------------------------------------*
    * Check to see if there is another device with the same ID
    *------------------------------------------------------------------------*/
   t_devLink2 = arb_getDeviceLinkById( t_devId);

   if( (t_devLink == (t_LINKHNDL)ARB_DEVICE_NOT_FOUND) && (t_devLink2
   == (t_LINKHNDL)ARB_DEVICE_NOT_FOUND))
   {

      /*---------------------------------------------------------------------*
       * Create a new link where the device driver information will be stored.
       *---------------------------------------------------------------------*/
      t_devLink = utl_createLink( sizeof(t_device));

      if( t_devLink < 0)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return ARB_OUT_OF_HEAP;
      }/*End if( t_devLink < 0)*/

      /*---------------------------------------------------------------------*
       * Get a ptr to the link's element- which is the area where the device
       * information is being stored.
       *---------------------------------------------------------------------*/
      pt_device = (t_device *)UTL_GET_LINK_ELEMENT_PTR( t_devLink);

      /*---------------------------------------------------------------------*
       * Configure the device by storing the callback functions, name, and
       * device ID.
       *---------------------------------------------------------------------*/
      arb_initDevice( &pt_device,
                      &pt_devOps,
                      &pc_name,
                      t_devId);

      /*---------------------------------------------------------------------*
       * Add the device onto the list of available device drivers.
       *---------------------------------------------------------------------*/
      t_err = utl_insertLink( gt_activeDevices,
                              t_devLink,
                              true);

   }
   else
   {
      HAL_END_CRITICAL();//Enable interrupts
      return ARB_DEVICE_PRESENT;
   }

   HAL_END_CRITICAL();//Enable interrupts

   return ARB_PASSED;

}/*End arb_registerDevice*/

t_error arb_destroyDevice( char *pc_name)
{
   static t_LINKHNDL t_devLink;
   static t_LINKHNDL t_hndlLink;
   static t_LINKHNDL t_prevHndlLink;
   t_devHandle *pt_handle;
   t_linkedListError t_err;
   t_device *pt_device;
   int16_t s_count;

   /*------------------------------------------------------------------------*
    * Since we are about to act on global variables- in this case the
    * handle to the device driver, we need to protect against interruption
    * from another process while the driver is being removed from the list.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   /*------------------------------------------------------------------------*
    * Search the active device list for this particular ID.
    *------------------------------------------------------------------------*/
   t_devLink = arb_getDeviceLinkByName( pc_name);

   if( t_devLink == (t_LINKHNDL)ARB_DEVICE_NOT_FOUND)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return ARB_DEVICE_NOT_FOUND;
   }/*End if( t_devLink == NULL)*/

   pt_device = (t_device *)UTL_GET_LINK_ELEMENT_PTR( t_devLink);

   /*------------------------------------------------------------------------*
    * Remove all the user-space handles pointing to the device driver from the
    * handle list. Since UTL_TRAVERSE_CONTAINER traverses through the entire
    * list changing the value of 't_hndlLink' on each iteration, once
    * 't_hndlLink' has been deleted information about the next position on the
    * list is lost. This issue is resolved by using a previous ptr to change
    * 't_hndlLink' back to a valid location once an item has been removed.
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_hndlLink, gt_activeHandles, s_count)
   {
      pt_handle = (t_devHandle *)UTL_GET_LINK_ELEMENT_PTR( t_hndlLink);
      t_prevHndlLink = UTL_GET_PREV_LINK( t_hndlLink);
      if( pt_handle->pt_dev == pt_device)
      {
         t_err = utl_destroyLink( gt_activeHandles, t_hndlLink);
         t_hndlLink = t_prevHndlLink;
      }
   }

   /*------------------------------------------------------------------------*
    * Remove the device driver from the driver list.
    *------------------------------------------------------------------------*/
   t_err = utl_destroyLink( gt_activeDevices,
                            (t_LINKHNDL)t_devLink);

   HAL_END_CRITICAL();//Enable interrupts

   return ARB_PASSED;

}/*End arb_destroyDevice*/
