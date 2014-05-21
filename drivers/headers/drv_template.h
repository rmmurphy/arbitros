/*---------------------------------------------------------------------------*
 * File Name   : drv_template.h
 *
 * Description : This is a template for creating a device driver - replace
 *               the name 'template' with the name of the driver you are
 *               creating
 *
 * Programmer  : Ryan M Murphy
 *
 * Date        : Sept 22, 2011
 *---------------------------------------------------------------------------*/
#ifndef drv_template_h

   #ifdef __cplusplus
   extern "C" {
   #endif
	
	/*------------------------------------------------------------------------*
	 * Global Defines
	 *------------------------------------------------------------------------*/
	#define drv_template_h

	/*------------------------------------------------------------------------*
	 * Inlude Files
	 *------------------------------------------------------------------------*/
    #include "avr_compiler.h"

	/*------------------------------------------------------------------------*
	 * Global Typedefs
	 *------------------------------------------------------------------------*/
   typedef enum
   {
      TEMPLATE_INVALID_ARG = -5, /*Invalid ioctl argument*/
      TEMPLATE_INVALID_CMD = -4, /*Invalid ioctl command.*/
      TEMPLATE_NULL_PTR    = -3, /*Pointer is not mapped to a valid address.*/
      TEMPLATE_OUT_OF_HEAP = -1, /*No more memory.*/
      TEMPLATE_PASSED      = 0   /*Configuration good.*/

   }t_templateError; /*Possible error conditions returned by the device's 
                       ioctl routine*/

    typedef enum
    {
       ADD_YOUR_IOCTL_COMMANDS_HERE,
    }t_templateCmd;

	/*------------------------------------------------------------------------*
	 * Global Variables
	 *------------------------------------------------------------------------*/

	/*------------------------------------------------------------------------*
	 * Global Inline functions
	 *------------------------------------------------------------------------*/

	/*------------------------------------------------------------------------*
	 * Global Function Prototypes
	 *------------------------------------------------------------------------*/
   t_error templateInit( void);

   void templateExit( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif
	
#endif/*End #ifndef drv_template_h*/
