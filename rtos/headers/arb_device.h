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
 * File Name   : arb_device.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file presents the standard API for interfacing between
 *               a user space application, kernel, and device driver.
 *
 * Last Update : June, 12, 2011
 *---------------------------------------------------------------------------*/
#ifndef arb_device_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define arb_device_h

   #define ARB_O_READ   (0x01) /*Open a file as read only*/
   #define ARB_O_WRITE  (0x02) /*Open as write only*/
   #define ARB_O_APPEND (0X04) /*Set the file offset to end before each write*/
   #define ARB_O_SYNC   (0X08) /*Synchronize writes with the hard drive*/
   #define ARB_O_TRUNC  (0X10) /*Truncate the file to zero length*/
   #define ARB_O_AT_END (0X20) /*Set the initial file position to end*/
   #define ARB_O_CREAT  (0X40) /*Create a file if non existent*/

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "arb_error.h"
   #include "arb_semaphore.h"
   #include "utl_linkedList.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   #define MAX_DEVICE_NAME_BYTES (16)

   /*------------------------------------------------------------------------*
    * This type def defines the handle for interfacing the user space
    * application and driver.
    *------------------------------------------------------------------------*/
   typedef volatile int16_t t_DEVHANDLE;

   typedef volatile int16_t t_deviceId;

   /*------------------------------------------------------------------------*
    * This structure contains a list of function pointers which define the
    * interface between the kernel and driver. These pointers must be
    * initialized an individual driver when it registers with the kernel.
    *------------------------------------------------------------------------*/
   typedef struct
   {
      t_error (*pf_open)( t_DEVHANDLE t_devHandle);

      int16_t (*pf_read)( t_DEVHANDLE t_devHandle,
                          int8_t *pc_buff,
                          uint16_t s_size);

      int16_t (*pf_write)( t_DEVHANDLE t_devHandle,
                           int8_t *pc_buff,
                           uint16_t s_size);

      int32_t (*pf_ioctl)( t_DEVHANDLE t_devHandle,
                           uint16_t s_command,
                           int32_t  i_arguments);

      t_error (*pf_close)( t_DEVHANDLE t_devHandle);

   }t_deviceOperations; /*Similar to 'file_operations' in linux*/

   /*------------------------------------------------------------------------*
    * This structure is where the kernel stores information about a specific
    * device on the system. It is created at the time of device initialization
    * when a specific driver registers with the kernel.
    *------------------------------------------------------------------------*/
   typedef struct
   {
      /*---------------------------------------------------------------------*
       * The identification number of the device registering with the kernel
       *---------------------------------------------------------------------*/
      t_deviceId t_devId; /*Major number upper 8 bits, Minor number lower 8*/

      /*---------------------------------------------------------------------*
       * The name of the device registering with the kernel
       *---------------------------------------------------------------------*/
      int8_t ac_deviceName[MAX_DEVICE_NAME_BYTES];

      /*---------------------------------------------------------------------*
       * The number of handles 'opened' against this specific device driver.
       *---------------------------------------------------------------------*/
      int8_t c_numDevHandles;

      /*---------------------------------------------------------------------*
       * List of functions that define the interface between the kernel and
       * device
       *---------------------------------------------------------------------*/
      t_deviceOperations *pt_devOps;

   }t_device; /*The interface between the kernel and driver, similar to
                'inode' in linux*/

   typedef struct
   {
      /*---------------------------------------------------------------------*
       * Pointer to the particular device this handle is accessing.
       *---------------------------------------------------------------------*/
      t_device *pt_dev;

      /*---------------------------------------------------------------------*
       * Pointer to static memory created by the device that is attached to
       * this handle. The driver has the option of using this variable for
       * storing data that is intended for a particular user that has
       * opened a handle to the device.
       *---------------------------------------------------------------------*/
      void *pv_privateData;

      /*---------------------------------------------------------------------*
       * Restricts file read/write operations (i.e. ARB_O_READ, ARB_O_WRITE,
       * etc).
       *---------------------------------------------------------------------*/
      uint8_t c_flags;

      /*---------------------------------------------------------------------*
       * Current read/write position within an open file.
       *---------------------------------------------------------------------*/
      uint32_t i_pos;

   }t_devHandle; /*The interface between user-space and the device via the
                   kernel - similar to 'file' in Linux*/

   /*------------------------------------------------------------------------*
    * Public Macros
    *------------------------------------------------------------------------*/
   #define ARB_GET_DEV_MAJOR(t_deviceId) (uint8_t)((t_deviceId) >> 8)

   #define ARB_GET_DEV_MINOR(t_deviceId) (uint8_t)(0x0000FFFF & (t_deviceId))

   /*------------------------------------------------------------------------*
    * Public Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/
   static inline t_deviceId __attribute__((always_inline))
   arb_createDevId( uint8_t c_major, uint8_t c_minor)
   {
      return( ((uint16_t)c_major << 8) | (uint16_t)c_minor);
   }/*End arb_createDevId*/

   static inline uint8_t __attribute__((always_inline))
   arb_getMajor( t_DEVHANDLE t_handle)
   {
      t_devHandle *pt_handle = (t_devHandle *)t_handle;
      uint8_t c_major = ARB_GET_DEV_MAJOR(pt_handle->pt_dev->t_devId);
      return( c_major);
   }/*End arb_getMajor*/

   static inline uint8_t __attribute__((always_inline))
   arb_getMinor( t_DEVHANDLE t_handle)
   {
      t_devHandle *pt_handle = (t_devHandle *)t_handle;
      uint8_t c_minor = ARB_GET_DEV_MINOR(pt_handle->pt_dev->t_devId);
      return( c_minor);
   }/*End arb_getMinor*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * User-space to kernel interface functions
    *------------------------------------------------------------------------*/
   t_DEVHANDLE arb_open( char *pc_name,
                         uint8_t c_flags);

   int16_t arb_read( t_DEVHANDLE t_handle,
                     int8_t *pc_buff,
                     uint16_t s_size);

   int16_t arb_write( t_DEVHANDLE t_handle,
                      int8_t *pc_buff,
                      uint16_t s_size);

   int32_t arb_ioctl( t_DEVHANDLE t_handle,
                      uint16_t s_command,
                      int32_t  i_argument);

   t_error arb_close( t_DEVHANDLE t_handle);

   /*------------------------------------------------------------------------*
    * Driver to kernel interface function
    *------------------------------------------------------------------------*/
   t_error arb_registerDevice( char *pc_name,
                               t_deviceId t_devId,
                               t_deviceOperations *pt_devOps);

   t_error arb_destroyDevice( char *pc_name);

   /*------------------------------------------------------------------------*
    * This function returns a handle to the list of active device drivers.
    *------------------------------------------------------------------------*/
   t_CONTHNDL arb_getDeviceList( void);

   int8_t *arb_getDevName( t_DEVHANDLE t_handle);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef arb_device_h*/
