/*---------------------------------------------------------------------------*
 * Copyright (C) 2014 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 * File Name   : usr_mathTest.c
 *
 * Project     : Watts-rock
 *               <http://code.google.com/p/watts-rock-motherboard/>
 *
 * Description : This file is responsible for testing functionality from
 *               utl_math.c.
 *
 * Last Update : Feb, 27, 2014
 *---------------------------------------------------------------------------*/
#ifndef usr_bms_h

   #ifdef __cplusplus
   extern "C" {
   #endif

	/*------------------------------------------------------------------------*
	 * Global Defines
	 *------------------------------------------------------------------------*/
	#define usr_bms_h

	/*------------------------------------------------------------------------*
	 * Include Files
	 *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "arb_error.h"

	/*------------------------------------------------------------------------*
	 * Global typedefs
	 *------------------------------------------------------------------------*/

	/*------------------------------------------------------------------------*
	 * Global Variables
	 *------------------------------------------------------------------------*/

	/*------------------------------------------------------------------------*
	 * Global Function Prototypes
	 *------------------------------------------------------------------------*/
   t_error usr_mathTestInit( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef usr_bms_h*/
