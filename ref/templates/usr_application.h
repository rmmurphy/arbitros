/*---------------------------------------------------------------------------*
 * Copyright (C) 2014 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 * File Name   : usr_application.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file contains all threads defined by the user-space
 *               application layer.
 *
 * Last Update : Feb, 27, 2014
 *---------------------------------------------------------------------------*/
#ifndef usr_application_h

   #ifdef __cplusplus
   extern "C" {
   #endif

	/*------------------------------------------------------------------------*
	 * Global Defines
	 *------------------------------------------------------------------------*/
	#define usr_application_h

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
   t_error usr_applicationInit( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef usr_application_h*/
