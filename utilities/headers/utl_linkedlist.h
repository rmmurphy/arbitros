/*---------------------------------------------------------------------------*
 * Copyright (C) 2012 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
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
 * File Name   : utl_linkedList.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides an abstraction layer for the creation and
 *               control over linked-list objects.
 *
 * Last Update : June 11, 2012
 *---------------------------------------------------------------------------*/
#ifndef utl_linkedList_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define utl_linkedList_h
   #define LINKEDLIST_CHECKSUM (0xFADE)

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      LINKEDLIST_LINK_MAPPED  = -4, /*The link is already on a list*/
      LINKEDLIST_INVALID_CONT = -3, /*Not a valid container handle*/
      LINKEDLIST_INVALID_LINK = -2, /*Link not found on a particular list.*/
      LINKEDLIST_OUT_OF_HEAP  = -1, /*No more memory.*/
      LINKEDLIST_PASSED       = 0   /*Configuration valid.*/

   }t_linkedListError;

   /*------------------------------------------------------------------------*
    * This typedef is the primary building block for a linked-list. When
    * constructing a list, links are created by calling the function
    * 'utl_createLink' and inserting them into a particular container using
    * the function 'utl_insertLink'. A link is the individual piece that
    * encapsulates an area of memory that contains the data being stored on the
    * list by a particular user-space application, kernel object, or device
    * driver. The area of memory is known as an element.
    *------------------------------------------------------------------------*/
   typedef struct listLink
   {
      /*---------------------------------------------------------------------*
       * Pointer to the particular element encapsulated by this particular
       * 'link'. An element is the area of memory set aside for the actual
       * user-data being stored on the list.
       *---------------------------------------------------------------------*/
      void *pv_element;

      /*---------------------------------------------------------------------*
       * The size of the memory region pointed to by 'pv_element' in bytes
       *---------------------------------------------------------------------*/
      uint16_t s_elementSizeBytes;

      /*---------------------------------------------------------------------*
       * Address of the container that is associated with this link.
       *---------------------------------------------------------------------*/
      uint16_t s_contAddr;

      /*---------------------------------------------------------------------*
       * The size of the this 'link' in bytes
       *---------------------------------------------------------------------*/
      uint16_t s_linkSizeBytes;

      /*---------------------------------------------------------------------*
       * Pointer to the next 'link' on the list
       *---------------------------------------------------------------------*/
      struct listLink *pt_next;

      /*---------------------------------------------------------------------*
       * Pointer to the previous 'link' on the list
       *---------------------------------------------------------------------*/
      struct listLink *pt_prev;

   }t_listLink;

   /*------------------------------------------------------------------------*
    * This typedef defines collection of 'links' that define a particular list.
    * The first step in the creation of a list is to define the containing
    * 'object' using the function 'utl_createContainer' (for dynamic allocation)
    * or 'UTL_CREATE_CONTAINER' (for static allocation) from which 'links' can
    * be added using the function 'utl_insertLink'.
    *------------------------------------------------------------------------*/
   typedef struct
   {
      /*---------------------------------------------------------------------*
       * Variable used for checking whether or not a handle to a container is
       * valid.
       *---------------------------------------------------------------------*/
      uint16_t s_checkSum;

      /*---------------------------------------------------------------------*
       * The number of links associated with the list.
       *---------------------------------------------------------------------*/
      uint16_t s_numLinks;

      /*---------------------------------------------------------------------*
       * The size of this list in bytes (including the container).
       *---------------------------------------------------------------------*/
      uint16_t s_contSizeBytes;

      /*---------------------------------------------------------------------*
       * Pointer to the current 'link' being used on the list. This pointer
       * is not updated by this utility. It is up to the calling application
       * to make sure it is up-to-date.
       *---------------------------------------------------------------------*/
      t_listLink *pt_curr;

      /*---------------------------------------------------------------------*
       * Pointer to the first 'link' on the list.
       *---------------------------------------------------------------------*/
      t_listLink *pt_head;

      /*---------------------------------------------------------------------*
       * Pointer to the last 'link' on the list.
       *---------------------------------------------------------------------*/
      t_listLink *pt_tail;

   }t_listContainer;

   typedef volatile int16_t t_CONTHNDL; /*Handle to a 'list'*/

   typedef volatile int16_t t_LINKHNDL; /*Handle to a 'link' on a 'list'*/

   /*------------------------------------------------------------------------*
    * Global Macros
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * This macro creates a generic variable name so that global variables can
    * have unique identifiers
    *------------------------------------------------------------------------*/
   #define UTL_CREATENAME( name) (t_##name)

   /*------------------------------------------------------------------------*
    * This macro creates/initializes a container element and returns a handle.
    * A container is an object that holds a particular linked-list.
    *------------------------------------------------------------------------*/
   #define UTL_CREATE_CONTAINER( name)\
      t_listContainer UTL_CREATENAME(name) = {LINKEDLIST_CHECKSUM, 0, \
      sizeof(t_listContainer), NULL, NULL, NULL};\
      t_CONTHNDL name = (t_CONTHNDL)&UTL_CREATENAME(name);

    /*------------------------------------------------------------------------*
    * This macro sets the current link in a container.
    *------------------------------------------------------------------------*/
   #define UTL_SET_CURR_OF_CONT( t_contHndl, t_linkHndl) (((t_listContainer *)\
   t_contHndl)->pt_curr = (t_listLink *)t_linkHndl)

    /*------------------------------------------------------------------------*
    * This macro returns the current link in a container.
    *------------------------------------------------------------------------*/
   #define UTL_GET_CURR_OF_CONT( t_contHndl) (t_LINKHNDL)((t_listContainer *)\
   t_contHndl)->pt_curr

    /*------------------------------------------------------------------------*
    * This macro returns the head link in a container.
    *------------------------------------------------------------------------*/
   #define UTL_GET_HEAD_OF_CONT( t_contHndl) (t_LINKHNDL)((t_listContainer *)\
   t_contHndl)->pt_head

    /*------------------------------------------------------------------------*
    * This macro returns the tail link in a container.
    *------------------------------------------------------------------------*/
   #define UTL_GET_TAIL_OF_CONT( t_contHndl) (t_LINKHNDL)((t_listContainer *)\
   t_contHndl)->pt_tail

    /*------------------------------------------------------------------------*
    * This macro returns the next link on the list.
    *------------------------------------------------------------------------*/
   #define UTL_GET_NEXT_LINK( t_linkHndl) (t_LINKHNDL)((t_listLink *)\
   t_linkHndl)->pt_next

    /*------------------------------------------------------------------------*
    * This macro returns the previous link on the list.
    *------------------------------------------------------------------------*/
   #define UTL_GET_PREV_LINK( t_linkHndl) (t_LINKHNDL)((t_listLink *)\
   t_linkHndl)->pt_prev

    /*------------------------------------------------------------------------*
    * This macro returns the number of links attached to a given container.
    *------------------------------------------------------------------------*/
   #define UTL_GET_NUM_LINKS_CONT( t_contHndl) ((t_listContainer *)t_contHndl)\
   ->s_numLinks

   /*------------------------------------------------------------------------*
    * This macro returns a ptr to an individual record 'element' contained in
    * a given 'link'
    *------------------------------------------------------------------------*/
   #define UTL_GET_LINK_ELEMENT_PTR( t_linkHndl) (((t_listLink *)t_linkHndl)->\
   pv_element);

   /*------------------------------------------------------------------------*
    * This macro returns a ptr to an individual record 'element' contained in
    * the head of a given 'container'
    *------------------------------------------------------------------------*/
   #define UTL_GET_LINK_ELEMENT_PTR_CONT_HEAD( t_contHndl) (((t_listContainer\
    *)t_contHndl)->pt_head->pv_element);

    /*------------------------------------------------------------------------*
    * This macro returns a ptr to an individual record 'element' contained in
    * the tail of a given 'container'
    *------------------------------------------------------------------------*/
   #define UTL_GET_LINK_ELEMENT_PTR_CONT_TAIL( t_contHndl) (((t_listContainer\
    *)t_contHndl)->pt_tail->pv_element);

    /*------------------------------------------------------------------------*
    * This macro returns a ptr to an individual record 'element' contained in
    * the tail of a given 'container'
    *------------------------------------------------------------------------*/
   #define UTL_GET_LINK_ELEMENT_PTR_CONT_CURR( t_contHndl) (((t_listContainer\
    *)t_contHndl)->pt_curr->pv_element);

    /*------------------------------------------------------------------------*
    * This macro checks to see if a particular 'link' is contained on the
    * list given by 't_contHndl'.
    *------------------------------------------------------------------------*/
   #define UTL_IS_LINK_ON_LIST( t_linkHndl, t_contHndl)\
   (bool)(((t_listLink *)t_linkHndl)->s_contAddr == t_contHndl)

    /*------------------------------------------------------------------------*
    * This macro checks to see if the handle is a valid container
    *------------------------------------------------------------------------*/
   #define UTL_IS_CONT_VALID( t_contHndl)\
   (bool)(((t_listContainer *)t_contHndl)->s_checkSum == LINKEDLIST_CHECKSUM)

   /*------------------------------------------------------------------------*
    * This macro starts at the head of a list and returns each one of the
    * associated links in forward order - as long as there is a valid
    * 'container'...
    *------------------------------------------------------------------------*/
   #define UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, t_contHndl, s_count)\
      s_count = UTL_GET_NUM_LINKS_CONT( t_contHndl);\
      for( t_linkHndl = UTL_GET_HEAD_OF_CONT(t_contHndl); s_count > 0;\
      s_count--, t_linkHndl = UTL_GET_NEXT_LINK(t_linkHndl))

   /*------------------------------------------------------------------------*
    * This macro starts at the tail of a list and returns each one of the
    * associated links in reverse order- as long as there is a valid
    * 'container'...
    *------------------------------------------------------------------------*/
   #define UTL_TRAVERSE_CONTAINER_TAIL( t_linkHndl, t_contHndl, s_count)\
      s_count = UTL_GET_NUM_LINKS_CONT( t_contHndl);\
      for( t_linkHndl = UTL_GET_TAIL_OF_CONT(t_contHndl); s_count > 0;\
      s_count--, t_linkHndl = UTL_GET_PREV_LINK(t_linkHndl))

   /*------------------------------------------------------------------------*
    * This macro starts at the curr of a list and returns each one of the
    * associated links in forward order- as long as there is a valid
    * 'container'...
    *------------------------------------------------------------------------*/
   #define UTL_TRAVERSE_CONTAINER_CURR( t_linkHndl, t_contHndl, s_count)\
      s_count = UTL_GET_NUM_LINKS_CONT( t_contHndl);\
      for( t_linkHndl = UTL_GET_CURR_OF_CONT(t_contHndl); s_count > 0;\
      s_count--, t_linkHndl = UTL_GET_NEXT_LINK(t_linkHndl))

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_LINKHNDL utl_createLink( uint16_t s_elementSizeBytes);

   t_linkedListError utl_destroyLink( t_CONTHNDL t_contHndl,
                                      t_LINKHNDL t_linkHndl);

   t_linkedListError utl_insertLink( t_CONTHNDL t_contHndl,
                                     t_LINKHNDL t_linkHndl,
                                     bool b_tail);

   t_linkedListError utl_removeLink( t_CONTHNDL t_contHndl,
                                     t_LINKHNDL t_linkHndl);

   t_CONTHNDL utl_createContainer( void);

   t_linkedListError utl_destroyContainer( t_CONTHNDL t_contHndl);

   uint32_t utl_getDynListMemUsage( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef utl_linkedList_h*/
