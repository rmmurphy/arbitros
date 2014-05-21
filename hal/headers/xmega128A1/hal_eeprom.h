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
 * File Name   : hal_eeprom.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This module provides an abstraction layer for the control
 *               over a device specific EEPROM module. An external application
 *               can request access to a 'block' of memory by calling the
 *               function 'hal_requestEepromBlock' with a unique identifer
 *               'c_userId' and size of the data being stored 's_sizeBytes' in
 *               bytes. The function first checks to see if the identifier is
 *               already in use and if there is enough room to store the
 *               'block' of data. If either of these cases is invalid the
 *               function will return an error, otherwise it will allocate the
 *               required number of EEPROM pages (that makeup the 'block') by
 *               choosing their location randomly (random access helps increase
 *               the life-span of the EEPROM). The 'head' of the EEPROM block
 *               is stored on a RAM based linked-list along with the user ID,
 *               and size of the data in bytes. Upon completion, a
 *               handle to the list is returned and becomes the primary
 *               mechanism for granting read/write access for the particular
 *               module that initially requested the 'block' of EEPROM memory.
 *               Each new 'block' of EEPROM memory contains a page known as the
 *               'head' comprised of a 2-byte header, 1-byte  user ID, 2-byte
 *               size, and 1-byte index to the next page on the list. The
 *               subsequent EEPROM pages making up the 'block' are linked
 *               together via an index stored in the first byte of the page.
 *               The header acts as a marker for identifying the start of a new
 *               EEPROM memory 'block' and is used by the function
 *               'hal_eepromBuildList' for populating the linked-list
 *               containing information about all the allocated EEPROM
 *               'blocks' on the system. Since the list is volatile, it
 *               gets repopulated each time a power on reset has been detected.
 *
 * Makeup of the 'head' EEPROM block :
 *
 *               .------------.
 *               | Header 0   | - First byte of header
 *               '------------'
 *               | Header 1   | - Second byte of header
 *               '------------'
 *               | User ID    | - ID of the module that stored the data
 *               '------------'
 *               | Size 0     | - First byte of size
 *               '------------'
 *               | Size 1     | - Second byte of size
 *               '------------'
 *               | Next Page  | - Address of the next page associated with
 *               '------------'   this user ID.
 *               | Checksum   | - Checksum for making sure the data in this
 *               '------------'   particular 'block' is valid.
 *               | Userdata 0 | - First byte of user data
 *               '------------'
 *               |     o      |
 *               '------------'
 *               |UserdataN-1 | - Last byte of user data
 *               '------------'
 *
 * Last Update : Dec 3, 2011
 *---------------------------------------------------------------------------*/
#ifndef hal_eeprom_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define hal_eeprom_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      EEPROM_PAGE_ERROR            = -7, /*EEPROM page has been corrupted.*/
      EEPROM_INVALID_WRITE_SIZE    = -6, /*Not writing the amount allocated*/
      EEPROM_ID_NOT_FOUND          = -5, /*Can't find the user ID*/
      EEPROM_ALREADY_ACTIVE_USERID = -4, /*This ID is already being used.*/
      EEPROM_INVALID_HNDL          = -3, /*Handle not found*/
      EEPROM_NULL_PTR              = -2, /*Pointer is not mapped to a valid
                                           address.*/
      EEPROM_OUT_OF_HEAP           = -1, /*No more memory.*/
      EEPROM_PASSED                = 0   /*Configuration good.*/

   }t_eepromError;

   typedef volatile int16_t t_EEPROMHANDLE; /*Handle to an 8-bit buffer*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_EEPROMHANDLE hal_requestEepromBlock( uint8_t c_userId,
                                          int16_t s_sizeBytes);

   t_eepromError hal_destroyEepromBlock( t_EEPROMHANDLE t_handle);

   t_EEPROMHANDLE hal_getEepromHandle( uint8_t c_userId);

   t_eepromError hal_writeEeprom( t_EEPROMHANDLE t_handle,
                                  uint8_t *pc_data,
                                  uint16_t s_sizeBytes);

   t_eepromError hal_readEeprom( t_EEPROMHANDLE t_handle,
                                 uint8_t *pc_data,
                                 uint16_t s_sizeBytes);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef hal_eeprom_h*/
