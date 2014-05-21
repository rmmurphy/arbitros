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

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "arb_memory.h"
#include "hal_eeprom.h"
#include "hal_pmic.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define EEPROM_PAGESIZE              (32) //32-byte page size
#define EEPROM_START_ADDR            (0x1000)
#define EEPROM_NUM_PAGES             (64)
#define EEPROM_HEADER                (0xBEEF)
#define EEPROM_FIRST_PAGE_OVERHEAD   (7) //Bytes
#define EEPROM_NOMINAL_PAGE_OVERHEAD (2) //Bytes all other pages...
#define EEPROM_EEPROM_VALUE_AT_RESET (0xFF)

#define NVM_EXEC() asm("push r30"      "\n\t"   \
                       "push r31"      "\n\t"   \
                        "push r16"      "\n\t"   \
                        "push r18"      "\n\t"   \
                       "ldi r30, 0xCB" "\n\t"   \
                       "ldi r31, 0x01" "\n\t"   \
                       "ldi r16, 0xD8" "\n\t"   \
                       "ldi r18, 0x01" "\n\t"   \
                       "out 0x34, r16" "\n\t"   \
                       "st Z, r18"       "\n\t"   \
                        "pop r18"       "\n\t"   \
                       "pop r16"       "\n\t"   \
                       "pop r31"       "\n\t"   \
                       "pop r30"       "\n\t")

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct eeprom
{

   /*------------------------------------------------------------------------*
    * Unique number which maps a specifc block of eeprom memory to the device
    * that has requested access to it.
    *------------------------------------------------------------------------*/
   uint8_t c_userId;

   /*------------------------------------------------------------------------*
    * The page address where the data is being stored in the eeprom
    *------------------------------------------------------------------------*/
   uint8_t c_startPageAddr;

   /*------------------------------------------------------------------------*
    * The number of bytes of user data stored at the location given by
    * 'c_startPageAddr'
    *------------------------------------------------------------------------*/
   int16_t s_sizeBytes;

   /*------------------------------------------------------------------------*
    * Pointer to the next eeprom thats been allocated
    *------------------------------------------------------------------------*/
   struct eeprom *pt_next;

   /*------------------------------------------------------------------------*
    * Pointer to the previous eeprom thats been allocated
    *------------------------------------------------------------------------*/
   struct eeprom *pt_prev;

}t_eepromHandle;

typedef struct
{

   /*------------------------------------------------------------------------*
    * The number of modules storing data on the eeprom in this system.
    *------------------------------------------------------------------------*/
   uint8_t c_numUsers;

   /*------------------------------------------------------------------------*
    * The size of this list in bytes.
    *------------------------------------------------------------------------*/
   uint16_t s_listSizeBytes;

   /*------------------------------------------------------------------------*
    * Buffer that signals whether or not there is an open page in the eeprom.
    * The index into the buffer represents the page number and the value
    * contained in the index is the page status, where a value of 'true'
    * means that the page is free.
    *------------------------------------------------------------------------*/
   bool ab_freePages[EEPROM_NUM_PAGES];

   /*------------------------------------------------------------------------*
    * Pointer to the first eeprom handle on the list.
    *------------------------------------------------------------------------*/
   t_eepromHandle *pt_head;

   /*------------------------------------------------------------------------*
    * Pointer to the last eeprom handle on the list.
    *------------------------------------------------------------------------*/
   t_eepromHandle *pt_tail;

}t_eepromList;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static void eepromListInsert( t_eepromHandle **pt_new,
                              t_eepromHandle **pt_head,
                              t_eepromHandle **pt_tail);

static t_eepromHandle *hal_findHandle( t_EEPROMHANDLE t_handle);

static t_eepromError hal_eepromBuildList( void);

static void eepromWaitForNvm( void);

static t_eepromHandle *eepromCreateListMember( uint8_t c_userId,
                                               int16_t s_sizeBytes,
                                               uint8_t c_startPageAddr);

static void eepromEnableMemMapping( void);

static uint8_t eepromBytesToPages( uint16_t s_sizeBytes);

static t_eepromHandle *hal_findHandleFromId( uint8_t c_userId);

static uint8_t hal_getFreePageCount( void);

static void eepromWritePage( uint8_t c_page,
                             uint8_t c_startingIndex,
                             uint8_t *pc_data,
                             uint8_t s_size);

static uint8_t eepromGetOpenPage( void);

static void hal_eepromErasePage( uint8_t c_pageAdd);

static void hal_eepromResetBlock( uint8_t c_startPageAddr);

static void eepromReadPage( uint8_t c_pageIndex,
                            uint8_t c_startingIndex,
                            uint8_t *pc_data,
                            uint8_t s_size);

static uint8_t eepromComputeChecksum( uint8_t *pc_input,
                                      int16_t s_size);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * This variable keeps track of all the open circular eeproms on the system.
 *---------------------------------------------------------------------------*/
static t_eepromList gt_eepromList;
static bool gb_eepromListPopulated = false;
static uint8_t *gpc_eepromAddr = (uint8_t *)EEPROM_START_ADDR;

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static uint8_t eepromComputeChecksum( uint8_t *pc_input,
                                      int16_t s_size)
{
   uint8_t c_index = 0;
   uint16_t s_checkSum = 0;

   for( c_index = 0; c_index < s_size; c_index++)
   {
      s_checkSum += ((uint16_t)pc_input[c_index]);
   }

   return (uint8_t)(s_checkSum%255);

}/*End eepromComputeChecksum*/

static uint8_t eepromGetOpenPage( void)
{
   uint8_t c_nextPage = 0;

   /*------------------------------------------------------------------------*
    * Grab (randomly) the next available page address
    *------------------------------------------------------------------------*/
   do
   {

      c_nextPage = (uint8_t)(((double)rand()*(double)EEPROM_NUM_PAGES) /
      ((double)RAND_MAX + 1));
   }
   while( gt_eepromList.ab_freePages[c_nextPage] == false);

   return c_nextPage;

}/*End eepromGetOpenPage*/

static uint8_t eepromBytesToPages( uint16_t s_sizeBytes)
{
   uint8_t c_numPages       = 0;
   int16_t s_remainder      = 0;
   uint8_t c_bytesFirstPage = 0;
   uint8_t c_bytesNomPages  = 0;

   c_bytesFirstPage = (uint8_t)EEPROM_PAGESIZE -
   (uint8_t)EEPROM_FIRST_PAGE_OVERHEAD;

   /*------------------------------------------------------------------------*
    * Requesting more than one page?
    *------------------------------------------------------------------------*/
   if( s_sizeBytes > c_bytesFirstPage) //Yes
   {
      /*---------------------------------------------------------------------*
       * Remove the overhead from the other pages...
       *---------------------------------------------------------------------*/
      c_bytesNomPages = (uint8_t)EEPROM_PAGESIZE -
      (uint8_t)EEPROM_NOMINAL_PAGE_OVERHEAD;
      s_sizeBytes -= c_bytesFirstPage;
      c_numPages += (s_sizeBytes / c_bytesNomPages);

      s_remainder = (int16_t)s_sizeBytes -
      ((int16_t)c_numPages*(int16_t)c_bytesNomPages);

      if( s_remainder > 0)
         c_numPages++;
   }

   return (c_numPages + 1); //Add one for the first page

}/*End eepromBytesToPages*/

static void eepromWaitForNvm()
{

   /*------------------------------------------------------------------------*
    * Wait for the NVM to become available...
    *------------------------------------------------------------------------*/
   while( (NVM.STATUS & NVM_NVMBUSY_bm) == NVM_NVMBUSY_bm)
   {

   }

}/*End eepromWaitForNvm*/

static void eepromEnableMemMapping()
{

   /*------------------------------------------------------------------------*
    * Enable the eeprom memory mapped interface...
    *------------------------------------------------------------------------*/
   NVM.CTRLB |= NVM_EEMAPEN_bm;

}/*End eepromEnableMemMapping*/

static void eepromReadPage( uint8_t c_pageIndex,
                            uint8_t c_startingIndex,
                            uint8_t *pc_data,
                            uint8_t s_size)
{
   uint8_t c_index    = 0;

   eepromWaitForNvm();

   for( c_index = 0; c_index < s_size; c_index++)
   {
      pc_data[c_index] = gpc_eepromAddr[(int16_t)c_pageIndex*
      (int16_t)EEPROM_PAGESIZE + c_startingIndex + c_index];
   }/*End for( c_index = 0; c_index < s_size; c_index++)*/

}/*End eepromReadPage*/

static void eepromWritePage( uint8_t c_pageIndex,
                             uint8_t c_startingIndex,
                             uint8_t *pc_data,
                             uint8_t s_size)
{
   uint8_t c_index    = 0;
   uint16_t s_address = 0;

   eepromWaitForNvm();

   for( c_index = 0; c_index < s_size; c_index++)
   {
      gpc_eepromAddr[(int16_t)c_pageIndex*(int16_t)EEPROM_PAGESIZE + c_startingIndex + c_index] =
      pc_data[c_index];
   }/*End for( c_index = 0; c_index < s_size; c_index++)*/

   eepromWaitForNvm();

   /*------------------------------------------------------------------------*
    * Calculate page address.
    *------------------------------------------------------------------------*/
   s_address = (uint16_t)c_pageIndex*(uint16_t)EEPROM_PAGESIZE;

   /*------------------------------------------------------------------------*
    * Set the page address.
    *------------------------------------------------------------------------*/
   NVM.ADDR0 = s_address & 0xFF;
   NVM.ADDR1 = (s_address >> 8) & 0x1F;
   NVM.ADDR2 = 0x00;

   /*------------------------------------------------------------------------*
    * Issue the atomic write command.
    *------------------------------------------------------------------------*/
   NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;
   NVM_EXEC();

}/*End eepromWritePage*/

/*---------------------------------------------------------------------------*
 * This function is called every time a new circular eeprom is allocated on
 * the system.
 *---------------------------------------------------------------------------*/
static void eepromListInsert( t_eepromHandle **pt_new,
                              t_eepromHandle **pt_head,
                              t_eepromHandle **pt_tail)
{

   /*------------------------------------------------------------------------*
    * Are we adding the very first node in the list
    *------------------------------------------------------------------------*/
   if( ((*pt_head) == NULL) && ((*pt_tail) == NULL))
   {
      /*---------------------------------------------------------------------*
       * Establish circular link
       *---------------------------------------------------------------------*/
      (*pt_new)->pt_prev = (*pt_new);
      (*pt_new)->pt_next = (*pt_new);

      /*---------------------------------------------------------------------*
       * Move head and tail location
       *---------------------------------------------------------------------*/
      (*pt_head) = (*pt_new);
      (*pt_tail) = (*pt_new);

   }/*End if( ((*pt_head) == NULL) && ((*pt_tail) == NULL))*/
   else
   {

      (*pt_new)->pt_prev  = (*pt_tail);
      (*pt_new)->pt_next  = (*pt_head);
      (*pt_head)->pt_prev = (*pt_new);
      (*pt_tail)->pt_next = (*pt_new);

      /*---------------------------------------------------------------------*
       * Move tail location
       *---------------------------------------------------------------------*/
      (*pt_tail) = (*pt_new);

   }

}/*End eepromListInsert*/

static uint8_t hal_getFreePageCount( void)
{
   uint8_t c_freePages = 0;
   uint8_t c_index     = 0;

   for( c_index = 0; c_index < EEPROM_NUM_PAGES; c_index++)
   {
      if( gt_eepromList.ab_freePages[c_index] == true)
         c_freePages++;

   }/*End for( c_index = 0; c_index < EEPROM_NUM_PAGES; c_index++)*/

   return c_freePages;

}/*End hal_getFreePageCount*/

static t_eepromHandle *hal_findHandle( t_EEPROMHANDLE t_handle)
{
   t_eepromHandle *pt_curr   = gt_eepromList.pt_head;
   t_eepromHandle *pt_handle = (t_eepromHandle *)t_handle;

   /*------------------------------------------------------------------------*
    * Search the interrupt list for a specific handle address
    *------------------------------------------------------------------------*/
   if( pt_curr != NULL)
   {
      do
      {
         if( pt_curr == pt_handle)
         {
            return pt_curr;

         }

         pt_curr = pt_curr->pt_next;

      }while( pt_curr != gt_eepromList.pt_head); /*Until we loop full circle*/

   }/*End if( pt_curr != NULL)*/

   /*------------------------------------------------------------------------*
    * If we make it this far the handle has not been found in the open eeprom
    * list
    *------------------------------------------------------------------------*/
   return NULL;

}/*End hal_findHandle*/

static t_eepromHandle *hal_findHandleFromId( uint8_t c_userId)
{
   t_eepromHandle *pt_curr = gt_eepromList.pt_head;

   /*------------------------------------------------------------------------*
    * Search the interrupt list for a specific handle address
    *------------------------------------------------------------------------*/
   if( pt_curr != NULL)
   {
      do
      {
         if( c_userId == pt_curr->c_userId)
         {
            return pt_curr;
         }

         pt_curr = pt_curr->pt_next;

      }while( pt_curr != gt_eepromList.pt_head); /*Until we loop full circle*/

   }/*End if( pt_curr != NULL)*/

   /*------------------------------------------------------------------------*
    * If we make it this far the handle has not been found in the open eeprom
    * list
    *------------------------------------------------------------------------*/
   return NULL;

}/*End hal_findHandleFromId*/

static t_eepromHandle *eepromCreateListMember( uint8_t c_userId,
                                               int16_t s_sizeBytes,
                                               uint8_t c_startPageAddr)
{
   t_eepromHandle *pt_newBuff = NULL;

   /*------------------------------------------------------------------------*
    * Allocated memory for new eeprom type
    *------------------------------------------------------------------------*/
   arb_malloc( sizeof( t_eepromHandle),
               (void **)&pt_newBuff);

   //pt_newBuff = (t_eepromHandle *)malloc( sizeof( t_eepromHandle));

   if( pt_newBuff == NULL)
   {
      return pt_newBuff;
   }/*End if( pt_newBuff == NULL)*/

   /*------------------------------------------------------------------------*
    * Grab a random page address
    *------------------------------------------------------------------------*/
   pt_newBuff->c_startPageAddr = c_startPageAddr;

   /*------------------------------------------------------------------------*
    * Store the user ID....
    *------------------------------------------------------------------------*/
   pt_newBuff->c_userId = c_userId;

   /*------------------------------------------------------------------------*
    * The size of the eeprom in 8-bit words.
    *------------------------------------------------------------------------*/
   pt_newBuff->s_sizeBytes = s_sizeBytes;

   pt_newBuff->pt_next = NULL;
   pt_newBuff->pt_prev = NULL;

   /*------------------------------------------------------------------------*
    * Add the newly created channel handle to the drivers linked list of
    * active virtual channels on the particular ADC module.
    *------------------------------------------------------------------------*/
   eepromListInsert( &pt_newBuff,
                     &gt_eepromList.pt_head,
                     &gt_eepromList.pt_tail);

   gt_eepromList.c_numUsers++;
   gt_eepromList.s_listSizeBytes += sizeof( t_eepromHandle);

   return pt_newBuff;
}

static t_eepromError hal_eepromBuildList( void)
{
   uint16_t s_page       = 0;
   uint16_t s_header    = 0;
   uint8_t c_userId     = 0;
   uint16_t s_sizeBytes = 0;
   uint8_t c_numPages   = 0;
   uint8_t c_nextPage   = 0;
   t_eepromError t_err  = EEPROM_PASSED;

   /*------------------------------------------------------------------------*
    * Clear the memory region allocated for the linked-list.
    *------------------------------------------------------------------------*/
   memset( (void *)&gt_eepromList, 0, sizeof( gt_eepromList));

   /*------------------------------------------------------------------------*
    * We don't know if the EEPROM is being used, set all the pages to free..
    *------------------------------------------------------------------------*/
   memset( (void *)gt_eepromList.ab_freePages, true, EEPROM_NUM_PAGES);

   /*------------------------------------------------------------------------*
    * Search for all the eeprom pages that are currently in use. Each
    * allocated eeprom block is marked by a user ID and number of stored
    * elements in bytes.
    *------------------------------------------------------------------------*/
   eepromEnableMemMapping();
   eepromWaitForNvm();

   while( s_page < (EEPROM_NUM_PAGES*EEPROM_PAGESIZE))
   {
      s_header = ((uint16_t)gpc_eepromAddr[s_page+1] << 8) |
      (uint16_t)gpc_eepromAddr[s_page];

      if( s_header == EEPROM_HEADER) /*Found a new memory block*/
      {
         c_userId    = gpc_eepromAddr[s_page + 2];
         s_sizeBytes = ((uint16_t)gpc_eepromAddr[s_page + 4] << 8) |
         (uint16_t)gpc_eepromAddr[s_page + 3];
         c_numPages = eepromBytesToPages( s_sizeBytes);
         c_nextPage = gpc_eepromAddr[s_page + 5];

         if( eepromCreateListMember( c_userId,
                                     s_sizeBytes,
                                     (s_page/EEPROM_PAGESIZE)) < 0)
         {
            return EEPROM_OUT_OF_HEAP;

         }

         /*------------------------------------------------------------------*
          * Populate the list of active pages...
          *------------------------------------------------------------------*/
         gt_eepromList.ab_freePages[s_page/EEPROM_PAGESIZE] = false;
         c_numPages -= 1;
         while( c_numPages > 0)
         {
            gt_eepromList.ab_freePages[c_nextPage] = false;
            c_nextPage = gpc_eepromAddr[(int16_t)c_nextPage*EEPROM_PAGESIZE];
            c_numPages--;
         }

      }/*End if( s_header == EEPROM_HEADER)*/

      s_page += EEPROM_PAGESIZE;/*Next page*/

   }/*End while( s_page < (EEPROM_NUM_PAGES*EEPROM_PAGESIZE))*/

   return t_err;

}/*end hal_eepromBuildList*/

static void hal_eepromErasePage( uint8_t c_pageAdd)
{
   uint8_t c_index    = 0;
   uint16_t s_address = 0;

   eepromWaitForNvm();

   /*------------------------------------------------------------------------*
    * Calculate page address.
    *------------------------------------------------------------------------*/
   s_address = (uint16_t)c_pageAdd*(uint16_t)EEPROM_PAGESIZE;

   for( c_index = 0; c_index < EEPROM_PAGESIZE; c_index++)
   {
      gpc_eepromAddr[ s_address + c_index] = EEPROM_EEPROM_VALUE_AT_RESET;
   }/*End for( c_index = 0; c_index < EEPROM_PAGESIZE; c_index++)*/

   eepromWaitForNvm();

   /*------------------------------------------------------------------------*
    * Set the page address.
    *------------------------------------------------------------------------*/
   NVM.ADDR0 = s_address & 0xFF;
   NVM.ADDR1 = (s_address >> 8) & 0x1F;
   NVM.ADDR2 = 0x00;

   /*------------------------------------------------------------------------*
    * Issue the atomic write command.
    *------------------------------------------------------------------------*/
   NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;
   NVM_EXEC();

}/*End hal_eepromErasePage*/

static void hal_eepromResetBlock( uint8_t c_startPageAddr)
{
   uint8_t c_nextPageAddr = 0;
   uint8_t c_prevPageAddr = 0;

   eepromWaitForNvm();
   c_nextPageAddr = gpc_eepromAddr[(int16_t)c_startPageAddr*EEPROM_PAGESIZE
   + 5];

   hal_eepromErasePage( c_startPageAddr);

   /*------------------------------------------------------------------------*
    * Make sure the reset page is now available for use.
    *------------------------------------------------------------------------*/
   gt_eepromList.ab_freePages[c_startPageAddr] = true;

   while( c_nextPageAddr != c_startPageAddr)
   {
      eepromWaitForNvm();
      c_prevPageAddr = c_nextPageAddr;
      c_nextPageAddr = gpc_eepromAddr[(int16_t)c_prevPageAddr*EEPROM_PAGESIZE];
      /*---------------------------------------------------------------------*
       * Make sure the reset page is now available for use.
       *---------------------------------------------------------------------*/
      gt_eepromList.ab_freePages[c_prevPageAddr] = true;
      hal_eepromErasePage( c_prevPageAddr);

   }/*End while( c_nextPageAddr != c_startPageAddr)*/


}/*End hal_eepromResetBlock*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
t_EEPROMHANDLE hal_requestEepromBlock( uint8_t c_userId,
                                       int16_t s_sizeBytes)
{
   t_eepromHandle *pt_handle = NULL;
   t_eepromError t_err       = EEPROM_PASSED;
   uint8_t c_startPageAddr   = 0;
   uint8_t c_numPages        = 0;
   uint8_t ac_tempData[EEPROM_FIRST_PAGE_OVERHEAD];
   uint8_t c_nextPage        = 0;
   uint8_t c_prevPage        = 0;
   uint8_t c_checkSum        = 0;

   /*------------------------------------------------------------------------*
    * We are going to be adding an element to a shared list so enforce
    * mutual exclusion.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Has the processor been reset?
    *------------------------------------------------------------------------*/
   if( gb_eepromListPopulated == false)
   {
      gb_eepromListPopulated = true;

      /*---------------------------------------------------------------------*
       * Check the eeprom for stored information - the first two bytes of
       * a given page contain the user ID and size of the stored information
       * in bytes for a particular eeprom handle.
       *---------------------------------------------------------------------*/
      t_err = hal_eepromBuildList();
      if( t_err < 0)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return (t_EEPROMHANDLE)t_err;
      }

   }/*End if( gb_eepromListPopulated == false)*/

   /*------------------------------------------------------------------------*
    * See if the user ID is already mapped...
    *------------------------------------------------------------------------*/
   pt_handle = hal_findHandleFromId( c_userId);

   /*------------------------------------------------------------------------*
    * Check to see if the user ID is free and there is enough room on the
    * EEPROM. If so, create a new EEPROM 'block'...
    *------------------------------------------------------------------------*/
   if( pt_handle == NULL) /*Didn't find user ID create new memory block...*/
   {

      /*---------------------------------------------------------------------*
       * Calculate the number of pages needed...
       *---------------------------------------------------------------------*/
      c_numPages = eepromBytesToPages( s_sizeBytes);
      if( hal_getFreePageCount() >= c_numPages)
      {

         /*------------------------------------------------------------------*
          * Grab (randomly) the next available page address
          *------------------------------------------------------------------*/
         c_startPageAddr = eepromGetOpenPage();

         /*------------------------------------------------------------------*
          * Make sure the request page is no longer available.
          *------------------------------------------------------------------*/
         gt_eepromList.ab_freePages[c_startPageAddr] = false;

         if( c_numPages > 1)
         {
            c_nextPage = eepromGetOpenPage(); //Get the next random page...

         }/*End if( c_numPages > 1)*/
         else
            c_nextPage = c_startPageAddr; //Wrap back to the beginning

         /*------------------------------------------------------------------*
          * Make sure the request page is no longer available.
          *------------------------------------------------------------------*/
         gt_eepromList.ab_freePages[c_nextPage] = false;

         /*------------------------------------------------------------------*
          * Create EEPROM header...
          *------------------------------------------------------------------*/
         ac_tempData[0] = (uint8_t)(0x00FF & EEPROM_HEADER);
         ac_tempData[1] = (uint8_t)(0x00FF & (EEPROM_HEADER >> 8));
         ac_tempData[2] = c_userId;
         ac_tempData[3] = (uint8_t)(0x00FF & s_sizeBytes);
         ac_tempData[4] = (uint8_t)(0x00FF & (s_sizeBytes >> 8));
         ac_tempData[5] = c_nextPage;

         /*------------------------------------------------------------------*
          * Compute checksum using the first EEPROM_FIRST_PAGE_OVERHEAD - 1
          * bytes of the header and all user data bytes for this page- since
          * this is the first time the memory 'block' is being requested all
          * the user-data bytes are set to EEPROM_EEPROM_VALUE_AT_RESET so we
          * don't need a checksum on this part of the data.
          *------------------------------------------------------------------*/
         c_checkSum = eepromComputeChecksum( ac_tempData,
                                             EEPROM_FIRST_PAGE_OVERHEAD - 1);

         ac_tempData[6] = c_checkSum;

         /*------------------------------------------------------------------*
          * Write the EEPROM header block...
          *------------------------------------------------------------------*/
         eepromWritePage( c_startPageAddr,
                          0,
                          ac_tempData,
                          EEPROM_FIRST_PAGE_OVERHEAD);

         /*------------------------------------------------------------------*
          * Initialize the rest of the EEPROM pages making up this block.
          *------------------------------------------------------------------*/
         if( c_numPages > 1)
         {
            c_numPages -= 1; //Account for header page...
            c_prevPage = c_nextPage;
            while( c_numPages > 0)
            {

               c_nextPage = eepromGetOpenPage();
               c_numPages-=1;
               if( c_numPages == 0)
                  c_nextPage = c_startPageAddr; //Wrap back to the beginning

               /*------------------------------------------------------------*
                * Make sure the request page is no longer available.
                *------------------------------------------------------------*/
               gt_eepromList.ab_freePages[c_nextPage] = false;

               /*------------------------------------------------------------*
                * Again, at this time the user-data is set to
                * EEPROM_EEPROM_VALUE_AT_RESET so the checksum is identical
                * to the next page.
                *------------------------------------------------------------*/
               c_checkSum = c_nextPage;

               ac_tempData[0] = c_nextPage;
               ac_tempData[1] = c_checkSum;

               /*------------------------------------------------------------*
                * Write the next nominal EEPROM page
                *------------------------------------------------------------*/
               eepromWritePage( c_prevPage,
                                0,
                                ac_tempData,
                                2);

               c_prevPage = c_nextPage;
            }/*End while( c_numPages > 0)*/

         }/*End if( c_numPages > 1)*/

         /*------------------------------------------------------------------*
          * Allocated memory for new eeprom block
          *------------------------------------------------------------------*/
         pt_handle = eepromCreateListMember( c_userId,
                                             s_sizeBytes,
                                             c_startPageAddr);

      }/*End if( hal_getFreePageCount() >= c_numPages)*/

   }/*End if( pt_handle == NULL)*/
   else /*Handle already created for this ID*/
   {
      HAL_END_CRITICAL();//Enable interrupts

      return (t_EEPROMHANDLE)EEPROM_ALREADY_ACTIVE_USERID;
   }

   HAL_END_CRITICAL();//Enable interrupts

   return (t_EEPROMHANDLE)pt_handle;

}/*End hal_requestEepromBlock*/

t_EEPROMHANDLE hal_getEepromHandle( uint8_t c_userId)
{
   t_eepromHandle *pt_handle = NULL;
   t_eepromError t_err       = EEPROM_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to be adding an element to a shared list so enforce
    * mutual exclusion.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Has the processor been reset?
    *------------------------------------------------------------------------*/
   if( gb_eepromListPopulated == false)
   {
      gb_eepromListPopulated = true;

      /*---------------------------------------------------------------------*
       * Check the eeprom for stored information - the first two bytes of
       * a given page contain the user ID and size of the stored information
       * in bytes for a particular eeprom handle.
       *---------------------------------------------------------------------*/
      t_err = hal_eepromBuildList();
      if( t_err < 0)
      {
         HAL_END_CRITICAL();//Enable interrupts
         return (t_EEPROMHANDLE)t_err;
      }

   }/*End if( gb_eepromListPopulated == false)*/

   /*------------------------------------------------------------------------*
    * See if the user ID is already mapped...
    *------------------------------------------------------------------------*/
   pt_handle = hal_findHandleFromId( c_userId);

   if( pt_handle == NULL) /*Didn't find user ID...*/
   {

      HAL_END_CRITICAL();//Enable interrupts

      return (t_EEPROMHANDLE)EEPROM_ID_NOT_FOUND;

   }/*End if( pt_handle == NULL)*/

   HAL_END_CRITICAL();//Enable interrupts

   return (t_EEPROMHANDLE)pt_handle;

}/*End hal_getEepromHandle*/

t_eepromError hal_destroyEepromBlock( t_EEPROMHANDLE t_handle)
{
   t_eepromHandle *pt_handle      = NULL;
   static t_eepromHandle *pt_prev = NULL;
   static t_eepromHandle *pt_next = NULL;
   t_eepromError t_err            = EEPROM_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to be removing an element to a shared list so enforce
    * mutual exclusion.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   pt_handle = hal_findHandle( t_handle);

   /*------------------------------------------------------------------------*
    * If not NULL then this handle is mapped to an interrupt handler.
    *------------------------------------------------------------------------*/
   if( pt_handle != NULL)
   {

      /*---------------------------------------------------------------------*
       * Remove this interrupt handler from the device drivers linked list
       *---------------------------------------------------------------------*/
      pt_prev = pt_handle->pt_prev;
      pt_next = pt_handle->pt_next;

      /*---------------------------------------------------------------------*
       * Are there other interrupt handlers still on the list?
       *---------------------------------------------------------------------*/
      if( pt_handle != pt_next)
      {
         /*------------------------------------------------------------------*
          * Remove the node from this list.
          *------------------------------------------------------------------*/
         pt_prev->pt_next = pt_next;
         pt_next->pt_prev = pt_prev;

         if( pt_handle == gt_eepromList.pt_head)
            gt_eepromList.pt_head = pt_next;
         else if( pt_handle == gt_eepromList.pt_tail)
            gt_eepromList.pt_tail = pt_prev;

      }
      else /*Last node*/
      {
         gt_eepromList.pt_head = NULL;
         gt_eepromList.pt_tail = NULL;

      }

      /*---------------------------------------------------------------------*
       * Erase the eeprom pages allocated to this particular handle...
       *---------------------------------------------------------------------*/

      /*---------------------------------------------------------------------*
       * Remove this interrupt handler from memory
       *---------------------------------------------------------------------*/
      arb_free( (void **)&pt_handle);
      //free( (void *)pt_handle);

       /*---------------------------------------------------------------------*
       * One less eeprom open on the system
       *---------------------------------------------------------------------*/
      gt_eepromList.c_numUsers--;
      gt_eepromList.s_listSizeBytes -= sizeof(t_eepromHandle);

      /*---------------------------------------------------------------------*
       * Clear the old memory region.
       *---------------------------------------------------------------------*/
      memset( (void *)pt_handle, 0, sizeof( t_eepromHandle));

   }/*End if( pt_gpioIntHndl != NULL)*/
   else
      t_err = EEPROM_INVALID_HNDL;

   HAL_END_CRITICAL();//Enable interrupts

   return t_err;

}/*End hal_destroyEepromBlock*/

t_eepromError hal_writeEeprom( t_EEPROMHANDLE t_handle,
                               uint8_t *pc_data,
                               uint16_t s_sizeBytes)
{
   t_eepromHandle *pt_handle = NULL;
   t_eepromError t_err       = EEPROM_PASSED;
   uint8_t c_startPageAddr   = 0;
   uint8_t c_numPages        = 0;
   uint8_t ac_tempData[EEPROM_FIRST_PAGE_OVERHEAD];
   uint8_t c_nextPage        = 0;
   uint8_t c_prevPage        = 0;
   uint16_t s_currDataPtr    = 0;
   int16_t s_writeSize       = 0;
   uint8_t c_checkSum        = 0;

   /*------------------------------------------------------------------------*
    * We are going to be removing an element to a shared list so enforce
    * mutual exclusion.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   pt_handle = hal_findHandle( t_handle);

   /*------------------------------------------------------------------------*
    * If not NULL then this handle is mapped to an EEPROM handler.
    *------------------------------------------------------------------------*/
   if( pt_handle != NULL)
   {

      /*---------------------------------------------------------------------*
       * Calculate the number of pages needed...
       *---------------------------------------------------------------------*/
      c_numPages = eepromBytesToPages( s_sizeBytes);

      /*---------------------------------------------------------------------*
       * Make sure we are writing the amount that was originally requested
       * when the EEPROM block was allocated in the function
       * 'hal_requestEepromBlock'
       *---------------------------------------------------------------------*/
      if( c_numPages == eepromBytesToPages( pt_handle->s_sizeBytes))
      {

         /*------------------------------------------------------------------*
          * Reset the region allocated for the old EEPROM block...
          *------------------------------------------------------------------*/
         hal_eepromResetBlock( pt_handle->c_startPageAddr);

         /*------------------------------------------------------------------*
          * Grab (randomly) the next available page address
          *------------------------------------------------------------------*/
         c_startPageAddr = eepromGetOpenPage();

         /*------------------------------------------------------------------*
          * Store the new starting page...
          *------------------------------------------------------------------*/
         pt_handle->c_startPageAddr = c_startPageAddr;

         /*------------------------------------------------------------------*
          * Make sure the request page is no longer available.
          *------------------------------------------------------------------*/
         gt_eepromList.ab_freePages[c_startPageAddr] = false;

         if( c_numPages > 1)
         {
            c_nextPage = eepromGetOpenPage(); //Get the next random page...

         }/*End if( c_numPages > 1)*/
         else
            c_nextPage = c_startPageAddr; //Wrap back to the beginning

         /*------------------------------------------------------------------*
          * Make sure the request page is no longer available.
          *------------------------------------------------------------------*/
         gt_eepromList.ab_freePages[c_nextPage] = false;

         /*------------------------------------------------------------------*
          * Create EEPROM header...
          *------------------------------------------------------------------*/
         ac_tempData[0] = (uint8_t)(0x00FF & EEPROM_HEADER);
         ac_tempData[1] = (uint8_t)(0x00FF & (EEPROM_HEADER >> 8));
         ac_tempData[2] = pt_handle->c_userId;
         ac_tempData[3] = (uint8_t)(0x00FF & s_sizeBytes);
         ac_tempData[4] = (uint8_t)(0x00FF & (s_sizeBytes >> 8));
         ac_tempData[5] = c_nextPage;

         /*------------------------------------------------------------------*
          * Compute checksum using the first EEPROM_FIRST_PAGE_OVERHEAD - 1
          * bytes of the header.
          *------------------------------------------------------------------*/
         c_checkSum = eepromComputeChecksum( ac_tempData,
                                             EEPROM_FIRST_PAGE_OVERHEAD - 1);

         /*------------------------------------------------------------------*
          * Make sure we don't write too much data on the first page...
          *------------------------------------------------------------------*/
         if( s_sizeBytes < (EEPROM_PAGESIZE - EEPROM_FIRST_PAGE_OVERHEAD))
            s_writeSize = s_sizeBytes;
         else
            s_writeSize = (EEPROM_PAGESIZE - EEPROM_FIRST_PAGE_OVERHEAD);

         /*------------------------------------------------------------------*
          * Continue computing checksum using the first 's_writeSize' bytes of
          * the user-data.
          *------------------------------------------------------------------*/
         c_checkSum = ((uint16_t)c_checkSum + (uint16_t)eepromComputeChecksum
         ( pc_data,s_writeSize)) % 255;

         ac_tempData[6] = c_checkSum;

         /*------------------------------------------------------------------*
          * Write the EEPROM header block...
          *------------------------------------------------------------------*/
         eepromWritePage( c_startPageAddr,
                          0,
                          ac_tempData,
                          EEPROM_FIRST_PAGE_OVERHEAD);

         /*------------------------------------------------------------------*
          * Write the first page of user data...
          *------------------------------------------------------------------*/
         eepromWritePage( c_startPageAddr,
                          EEPROM_FIRST_PAGE_OVERHEAD,
                          pc_data,
                          s_writeSize);

         s_currDataPtr = s_writeSize;

         s_sizeBytes -= s_writeSize;
         c_numPages -= 1;

         /*------------------------------------------------------------------*
          * Initialize the rest of the EEPROM pages making up this block.
          *------------------------------------------------------------------*/
         c_prevPage = c_nextPage;
         while( s_sizeBytes > 0)
         {

            c_nextPage = eepromGetOpenPage();
            c_numPages -= 1;

            if( c_numPages == 0)
               c_nextPage = c_startPageAddr; //Wrap back to the beginning

            /*---------------------------------------------------------------*
             * Make sure the request page is no longer available.
             *---------------------------------------------------------------*/
            gt_eepromList.ab_freePages[c_prevPage] = false;

            /*---------------------------------------------------------------*
             * Make sure we don't write too much data on the nominal pages.
             *---------------------------------------------------------------*/
            if( s_sizeBytes < (EEPROM_PAGESIZE - EEPROM_NOMINAL_PAGE_OVERHEAD))
               s_writeSize = s_sizeBytes;
            else
               s_writeSize = (EEPROM_PAGESIZE - EEPROM_NOMINAL_PAGE_OVERHEAD);

            /*------------------------------------------------------------------*
             * Compute the checksum for this page using 's_writeSize' bytes of
             * the user-data along with the next page ptr.
             *------------------------------------------------------------------*/
            c_checkSum = eepromComputeChecksum( &pc_data[s_currDataPtr],
                                                s_writeSize);

            ac_tempData[0] = c_nextPage;
            ac_tempData[1] = ((uint16_t)c_checkSum + (uint16_t)c_nextPage)
            % 255;

            /*---------------------------------------------------------------*
             * Write the next nominal EEPROM page
             *---------------------------------------------------------------*/
            eepromWritePage( c_prevPage,
                             0,
                             ac_tempData,
                             2);

            /*---------------------------------------------------------------*
             * Write the first page of user data...
             *---------------------------------------------------------------*/
            eepromWritePage( c_prevPage,
                             EEPROM_NOMINAL_PAGE_OVERHEAD,
                             &pc_data[s_currDataPtr],
                             s_writeSize);

            s_currDataPtr += s_writeSize;

            s_sizeBytes -= s_writeSize;

            c_prevPage = c_nextPage;

         }/*End while( s_sizeBytes > 0)*/

      }/*End if( c_numPages <= eepromBytesToPages( pt_handle->s_sizeBytes))*/
      else
      {
         t_err = EEPROM_INVALID_WRITE_SIZE;
      }

   }/*End if( pt_gpioIntHndl != NULL)*/
   else
      t_err = EEPROM_INVALID_HNDL;

   HAL_END_CRITICAL();//Enable interrupts

   return t_err;

}/*End hal_writeEeprom*/

t_eepromError hal_readEeprom( t_EEPROMHANDLE t_handle,
                              uint8_t *pc_data,
                              uint16_t s_sizeBytes)
{
   t_eepromHandle *pt_handle = NULL;
   t_eepromError t_err       = EEPROM_PASSED;
   uint8_t ac_tempData[EEPROM_FIRST_PAGE_OVERHEAD];
   uint8_t c_currPage        = 0;
   int16_t s_readSize        = 0;
   int16_t s_readPtr         = 0;
   uint8_t c_checkSum        = 0;

   /*------------------------------------------------------------------------*
    * We are going to be removing an element to a shared list so enforce
    * mutual exclusion.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   pt_handle = hal_findHandle( t_handle);

   /*------------------------------------------------------------------------*
    * If not NULL then this handle is mapped to an EEPROM handler.
    *------------------------------------------------------------------------*/
   if( pt_handle != NULL)
   {

      /*---------------------------------------------------------------------*
       * Make sure we are not reading more than was originally requested
       *---------------------------------------------------------------------*/
      if( s_sizeBytes <= pt_handle->s_sizeBytes)
      {
         eepromWaitForNvm();
         c_currPage = pt_handle->c_startPageAddr;

         if( s_sizeBytes > (EEPROM_PAGESIZE - EEPROM_FIRST_PAGE_OVERHEAD))
         {
            s_readSize = (EEPROM_PAGESIZE - EEPROM_FIRST_PAGE_OVERHEAD);
         }/*End if( s_sizeBytes > (EEPROM_PAGESIZE -
         EEPROM_FIRST_PAGE_OVERHEAD))*/
         else
            s_readSize = s_sizeBytes;

         eepromReadPage( c_currPage,
                         0,
                         ac_tempData,
                         EEPROM_FIRST_PAGE_OVERHEAD);

         /*------------------------------------------------------------------*
          * Compute checksum using the first EEPROM_FIRST_PAGE_OVERHEAD - 1
          * bytes of the header.
          *------------------------------------------------------------------*/
         c_checkSum = eepromComputeChecksum( ac_tempData,
                                             EEPROM_FIRST_PAGE_OVERHEAD-1);

         eepromReadPage( c_currPage,
                         EEPROM_FIRST_PAGE_OVERHEAD,
                         pc_data,
                         s_readSize);

         /*------------------------------------------------------------------*
          * Continue computing the checksum using the user-data
          *------------------------------------------------------------------*/
         c_checkSum = (uint8_t)(((uint16_t)c_checkSum +
         (uint16_t)eepromComputeChecksum( pc_data, s_readSize)) % 255);

         /*------------------------------------------------------------------*
          * Does the calculate checksum match the stored checksum?
          *------------------------------------------------------------------*/
         if( c_checkSum != ac_tempData[6])
         {
            HAL_END_CRITICAL();//Enable interrupts
            return EEPROM_PAGE_ERROR;
         }/*End if( c_checkSum != ac_tempData[6])*/

         s_sizeBytes -= s_readSize;
         s_readPtr = s_readSize;

         c_currPage = ac_tempData[5];

         while( s_sizeBytes > 0)
         {
            if( s_sizeBytes > (EEPROM_PAGESIZE - EEPROM_NOMINAL_PAGE_OVERHEAD))
            {
               s_readSize = (EEPROM_PAGESIZE - EEPROM_NOMINAL_PAGE_OVERHEAD);
            }/*End if( s_sizeBytes > (EEPROM_PAGESIZE -
            EEPROM_FIRST_PAGE_OVERHEAD))*/
            else
               s_readSize = s_sizeBytes;

            /*---------------------------------------------------------------*
             * Get the next page address and the checksum.
             *---------------------------------------------------------------*/
            eepromReadPage( c_currPage,
                            0,
                            ac_tempData,
                            EEPROM_NOMINAL_PAGE_OVERHEAD);

            /*---------------------------------------------------------------*
             * Get the user data
             *---------------------------------------------------------------*/
            eepromReadPage( c_currPage,
                            EEPROM_NOMINAL_PAGE_OVERHEAD,
                            &pc_data[s_readPtr],
                            s_readSize);

            /*------------------------------------------------------------------*
             * Compute checksum using the user data and first byte of the
             * header...
             *------------------------------------------------------------------*/
            c_checkSum = (uint8_t)(((uint16_t)ac_tempData[0] +
            (uint16_t)eepromComputeChecksum( &pc_data[s_readPtr], s_readSize)) %
            255);

            /*------------------------------------------------------------------*
             * Does the calculate checksum match the stored checksum?
             *------------------------------------------------------------------*/
            if( c_checkSum != ac_tempData[1])
            {
               HAL_END_CRITICAL();//Enable interrupts
               return EEPROM_PAGE_ERROR;
            }/*End if( c_checkSum != ac_tempData[1])*/

            c_currPage = ac_tempData[0];
            s_readPtr += s_readSize;
            s_sizeBytes -= s_readSize;

         }/*End while( s_sizeBytes > 0)*/

      }/*End if( s_sizeBytes <= pt_handle->s_sizeBytes)*/

   }/*End if( pt_gpioIntHndl != NULL)*/
   else
      t_err = EEPROM_INVALID_HNDL;

   HAL_END_CRITICAL();//Enable interrupts

   return t_err;

}/*End hal_readEeprom*/
