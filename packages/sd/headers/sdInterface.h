/*---------------------------------------------------------------------------*
 * Copyright (C) 2013 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * File Name   : sdInterface.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides an interface for communicating between
 *               arbitros c object files and the sdfatlib written in C++.
 *
 * Last Update : June, 19, 2013
 *---------------------------------------------------------------------------*/
#ifndef sdInterface_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "arb_device.h"

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define sdInterface_h

   /*------------------------------------------------------------------------*
    * These flags MUST match the ones defined in SdFileBase.h
    *------------------------------------------------------------------------*/
   #define SD_FILE_READ   (0x01) /*Open a file as read only*/
   #define SD_FILE_WRITE  (0x02) /*Open as write only*/
   #define SD_FILE_APPEND (0X04) /*Set the file offset to end before each write*/
   #define SD_FILE_SYNC   (0X08) /*Synchronize writes with the hard drive*/
   #define SD_FILE_TRUNC  (0X10) /*Truncate the file to zero length*/
   #define SD_FILE_AT_END (0X20) /*Set the initial file position to end*/
   #define SD_FILE_CREAT  (0X40) /*Create a file if non existent*/

   #define SD_MAX_DIRECTORIES (10)

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   bool sd_begin( t_SPIHNDL t_spiHndl);

   void *sd_open(char *pc_filepath, uint8_t c_mode);

   bool sd_exists(char *pc_filepath);

   bool sd_mkdir(char *pc_filepath);

   bool sd_chdir(char *pc_filepath);

   bool sd_remove(char *pc_filepath);

   bool sd_rmdir(char *pc_filepath);

   bool sd_close( void *pv_fileHndle);

   int32_t sd_write( t_SPIHNDL t_spiHndl,
                     void *pv_fileHndle,
                     uint8_t *pc_data,
                     uint16_t s_size);

   int32_t sd_read( t_SPIHNDL t_spiHndl,
                    void *pv_fileHndle,
                    uint8_t *pc_data,
                    uint16_t s_size);

   char *sd_gcf( void);

   void sd_ls( void);

   void sd_rewind( void *pv_fileHndle);

   bool sd_truncate( void *pv_fileHndle,
                     uint32_t i_length);

   bool sd_rmDashR( void);

   bool sd_init( t_SPIHNDL t_spiHndl);

   int32_t sd_fileSize( void *pv_fileHndle);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef sdInterface_h*/