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
 * File Name   : sdInterface.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides an interface for communicating between
 *               arbitros c object files and the sdfatlib written in C++.
 *
 * Last Update : June, 19, 2013
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <SdFat.h>
#include <new.h>
#include "sdInterface.h"
#include "arb_memory.h"
#include "hal_pmic.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private variables
 *---------------------------------------------------------------------------*/
static SdFat sd;

/*---------------------------------------------------------------------------*
 * This variable points to the currently open file on the system
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public functions
 *---------------------------------------------------------------------------*/
bool sd_begin( t_SPIHNDL t_spiHndl)
{
   return sd.begin( t_spiHndl);
}/*End sd_begin*/

bool sd_init( t_SPIHNDL t_spiHndl)
{
   return sd.card()->init( t_spiHndl);
}/*End sd_init*/

void *sd_open( char *pc_filepath, uint8_t c_mode)
{
   SdFile *pt_file;
   bool b_status;
   char *pc_tok[SD_MAX_DIRECTORIES];
   int32_t i_tokCount = 0;
   int32_t i_index;
   SdBaseFile *pt_vol;
   static char ac_currDir[13];

   HAL_BEGIN_CRITICAL(); //Disable interrupts

   arb_malloc( sizeof(SdFile),
               (void **)&pt_file);

   /*------------------------------------------------------------------------*
    * Make sure the status of the file is initialized to 'FAT_FILE_TYPE_CLOSED'
    * otherwise the call to 'open' will return an error.
    *------------------------------------------------------------------------*/
   pt_file->close();

   if( pt_file != NULL)
   {
      /*---------------------------------------------------------------------*
       * Get the name of the current directory
       *---------------------------------------------------------------------*/
      pt_vol = sd.vwd();
      pt_vol->getFilename( ac_currDir);

      /*---------------------------------------------------------------------*
       * Find all the directories within the path
       *---------------------------------------------------------------------*/
      pc_tok[i_tokCount] = strtok( pc_filepath, "/");
      while( (pc_tok[i_tokCount] != NULL) &&
             (i_tokCount < SD_MAX_DIRECTORIES))
      {
         i_tokCount++;
         pc_tok[i_tokCount] = strtok( NULL, "/");
      }

      /*---------------------------------------------------------------------*
       * Create the directories if the aren't already available and a write
       * request has been issued - last token is the file name.
       *---------------------------------------------------------------------*/
      for( i_index = 0; i_index < (i_tokCount - 1); i_index++)
      {
         if( sd.chdir( pc_tok[i_index]) == false)
         {
            if( c_mode & SD_FILE_WRITE)
            {

               sd.mkdir( pc_tok[i_index]);
               sd.chdir( pc_tok[i_index]);

            }
         }
      }

      b_status = pt_file->open( pc_tok[(i_tokCount - 1)], c_mode);

      /*---------------------------------------------------------------------*
       * Change back to the initial directory.
       *---------------------------------------------------------------------*/
      sd.chdir( ac_currDir);

      HAL_END_CRITICAL(); //Disable interrupts

      if( b_status == true)
      {
         pt_vol = sd.vwd();
         pt_vol->getFilename( ac_currDir);
         return (void *)pt_file;
      }
      else
      {
         arb_free( (void **)&pt_file);
         return NULL;
      }

   }/*End if( pt_file != NULL)*/
   else
   {
      HAL_END_CRITICAL(); //Disable interrupts

      return NULL;
   }

}/*End sd_open*/

bool sd_close( void *pv_fileHndle)
{
   bool b_status;
   SdFile *pt_file = (SdFile *)pv_fileHndle;
   b_status = pt_file->close();
   if( b_status == true)
   {
      arb_free( (void **)&pt_file);
      pt_file = NULL;
   }

   return b_status;

}/*End sd_close*/

char *sd_gcf( void)
{
   char *pc_name = NULL;

   //myFile.getFilename( pc_name);

   return pc_name;

}/*End sd_gcf*/

void sd_ls( void)
{
   sd.ls(LS_DATE | LS_SIZE | LS_R);
}/*sd_ls*/

bool sd_chdir(char *pc_filepath)
{
   return sd.chdir( pc_filepath);
}/*sd_lsPrint*/

bool sd_mkdir(char *pc_filepath)
{
   if( sd.chdir( *pc_filepath) == false)
      return sd.mkdir( pc_filepath, true);

   return false;
}/*End sd_mkdir*/

bool sd_rmdir(char *pc_filepath)
{
   return sd.rmdir( pc_filepath);
}/*End sd_rmdir*/

bool sd_remove(char *pc_filepath)
{
   return sd.remove( pc_filepath);
}/*End sd_remove*/

bool sd_rmDashR( void)
{
   /*------------------------------------------------------------------------*
    * Remove all the files within, and the current working directory itself.
    *------------------------------------------------------------------------*/
   return sd.vwd()->rmRfStar();
}/*End sd_rmDashR*/

int32_t sd_write( t_SPIHNDL t_spiHndl,
                  void *pv_fileHndle,
                  uint8_t *pc_data,
                  uint16_t s_size)
{
   SdFile *pt_file = (SdFile *)pv_fileHndle;
   int32_t i_size;

   i_size = (int32_t)pt_file->write( (void *)pc_data, s_size);

   return i_size;

}/*End sd_write*/

int32_t sd_read( t_SPIHNDL t_spiHndl,
                 void *pv_fileHndle,
                 uint8_t *pc_data,
                 uint16_t s_size)
{
   SdFile *pt_file = (SdFile *)pv_fileHndle;
   int32_t i_size;

   i_size = (int32_t)pt_file->read( (void *)pc_data, s_size);

   return i_size;

}/*End sd_read*/

int32_t sd_fileSize( void *pv_fileHndle)
{
   SdFile *pt_file = (SdFile *)pv_fileHndle;

   return pt_file->fileSize();

}/*End sd_close*/

void sd_rewind( void *pv_fileHndle)
{
   SdFile *pt_file = (SdFile *)pv_fileHndle;
   pt_file->rewind();

}/*End sd_rewind*/

bool sd_truncate( void *pv_fileHndle,
                  uint32_t i_length)
{
   SdFile *pt_file = (SdFile *)pv_fileHndle;
   return pt_file->truncate( i_length);
}/*End sd_truncate*/
