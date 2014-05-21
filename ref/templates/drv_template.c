/*---------------------------------------------------------------------------*
 * File Name   : drv_template.c
 *
 * Description : This is a template for creating a device driver - replace
 *               the name 'template' with the name of the driver you are
 *               creating
 *
 * Programmer  : Ryan M Murphy
 *
 * Date        : Sept 22, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Inlude Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "avr_compiler.h"
#include "arb_device.h"
#include "arb_semaphore.h"
#include "drv_template.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * This value represents a unique number assigned to this driver and has to be
 * different from all others registered with the kernel.
 *---------------------------------------------------------------------------*/
#define TEMPLATE_MAJOR_NUMBER (3)

/*---------------------------------------------------------------------------*
 * This number must reflect the total number of devices this driver
 * can control. For instance, if it were an ADC driver there would be ADCA
 * and ADCB and this value would be set to two.
 *---------------------------------------------------------------------------*/
#define TEMPLATE_MAX_MINORS (1)

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct
{

   /*------------------------------------------------------------------------*
    * Resources that can be shared amongst mutliple users (either a global
    * buffer or IO device) need to be protected against race conditions. We 
    * use this semaphore for just that purpose.
    *------------------------------------------------------------------------*/
   t_semHandle t_mutex;

   /*------------------------------------------------------------------------*
    * We are going to want to know how many 'handles' or users are attached to
    * this device driver.
    *------------------------------------------------------------------------*/
   uint8_t c_numUsers;

   /*------------------------------------------------------------------------*
    * Add your specific device configuration data here... For instance buffer 
    * declarations and/or io port configuration.
    *------------------------------------------------------------------------*/

}t_templateDev;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_error templateOpen( t_DEVHANDLE t_handle);

static int16_t templateRead( t_DEVHANDLE t_handle,
                             int8_t *pc_buff, 
                             uint16_t s_size);

static int16_t templateWrite( t_DEVHANDLE t_handle,
                              int8_t *pc_buff, 
                              uint16_t s_size);

static int32_t templateIoctl( t_DEVHANDLE t_handle,
                              uint16_t s_command,
                              int32_t  i_arguments);

static t_error templateClose( t_DEVHANDLE t_handle);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
t_deviceOperations gt_templateDevOps = 
{
    templateOpen,
    templateRead,
    templateWrite,
    templateIoctl,
    templateClose

};

/*---------------------------------------------------------------------------*
 * This is the device's shared memory, all actions on this variable must be
 * mutually exclusive.
 *---------------------------------------------------------------------------*/
static t_templateDev gat_templateDev[TEMPLATE_MAX_MINORS];

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static t_error templateOpen( t_DEVHANDLE t_handle)
{
   uint8_t c_minorNum        = 0;

   /*------------------------------------------------------------------------*
    * There may be mutiple device's this driver can access (for instance ADCA, 
    * and ADCB) which one is this user-space application trying to use?
    *------------------------------------------------------------------------*/
   c_minorNum = arb_getMinor( t_handle);

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gat_templateDev[c_minorNum].t_mutex,
             0);
                    
   /*------------------------------------------------------------------------*
    * Keep track of the number of user-space applications using the driver.
    *------------------------------------------------------------------------*/   
   gat_templateDev[c_minorNum].c_numUsers++;

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gat_templateDev[c_minorNum].t_mutex);

   return ARB_PASSED;

}/*End templateOpen*/

static int16_t templateRead( t_DEVHANDLE t_handle,
                             int8_t *pc_buff, 
                             uint16_t s_size)
{
   uint8_t c_minorNum        = 0;
   int16_t s_return          = (int16_t)ARB_PASSED;

   /*------------------------------------------------------------------------*
    * There may be mutiple device's this driver can access (for instance ADCA, 
    * and ADCB) which one is this user-space application trying to use?
    *------------------------------------------------------------------------*/
   c_minorNum = arb_getMinor( t_handle);

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gat_templateDev[c_minorNum].t_mutex,
             0);

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gat_templateDev[c_minorNum].t_mutex);

   return s_return;

}/*End templateRead*/

static int16_t templateWrite( t_DEVHANDLE t_handle,
                              int8_t *pc_buff, 
                              uint16_t s_size)
{
   uint8_t c_minorNum        = 0;
   int16_t s_return          = (int16_t)ARB_PASSED;

   /*------------------------------------------------------------------------*
    * There may be mutiple device's this driver can access (for instance ADCA, 
    * and ADCB) which one is this user-space application trying to use?
    *------------------------------------------------------------------------*/
   c_minorNum = arb_getMinor( t_handle);

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gat_templateDev[c_minorNum].t_mutex,
             0);

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gat_templateDev[c_minorNum].t_mutex);

   return s_return;

}/*End templateWrite*/

static int32_t templateIoctl( t_DEVHANDLE t_handle,
                              uint16_t s_command,
                              int32_t  i_arguments)
{
   int16_t s_return          = (int16_t)TEMPLATE_PASSED;
   uint8_t c_minorNum        = 0;

   /*------------------------------------------------------------------------*
    * There may be mutiple device's this driver can access (for instance ADCA, 
    * and ADCB) which one is this user-space application trying to use?
    *------------------------------------------------------------------------*/
   c_minorNum = arb_getMinor( t_handle);

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gat_templateDev[c_minorNum].t_mutex,
             0);

   switch( (t_templateCmd)s_command)
   {

      default:

         s_return = (int16_t)ARB_INVALID_CMD;

      break;

   }/*End switch( (t_templateCmd)s_command)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gat_templateDev[c_minorNum].t_mutex);

   return s_return;

}/*End templateIoctl*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
static t_error templateClose( t_DEVHANDLE t_handle)
{
   uint8_t c_minorNum        = 0;

   /*------------------------------------------------------------------------*
    * There may be mutiple device's this driver can access (for instance ADCA, 
    * and ADCB) which one is this user-space application trying to use?
    *------------------------------------------------------------------------*/
   c_minorNum = arb_getMinor( t_handle);

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gat_templateDev[c_minorNum].t_mutex,
             0);

   /*------------------------------------------------------------------------*
    * Keep track of the number of user-space applications using the driver.
    *------------------------------------------------------------------------*/ 
   gat_templateDev[c_minorNum].c_numUsers--;

   /*------------------------------------------------------------------------*
    * Remove any user-space specific generated memory here...
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gat_templateDev[c_minorNum].t_mutex);

   return ARB_PASSED;

}/*End templateClose*/

t_error templateInit( void)
{
   t_error t_err       = ARB_PASSED;
   uint8_t c_index     = 0;
   char ac_devName[MAX_DEVICE_NAME_BYTES];

   /*------------------------------------------------------------------------*
    * We are going to be removing items from memory that may be being used
    * by another process.
    *------------------------------------------------------------------------*/
   UTL_BEGIN_CRITICAL(); //Disable interrupts

   for( c_index = 0; c_index < TEMPLATE_MAX_MINORS; c_index++)
   {
      /*---------------------------------------------------------------------*
       * Make sure the kernel is aware that a new device has been loaded.
       *---------------------------------------------------------------------*/
      sprintf(ac_devName, "templateDevice%d", c_index);
      t_err = arb_registerDevice( ac_devName,
                                  arb_createDevId( TEMPLATE_MAJOR_NUMBER, 
                                  c_index),
                                  &gt_templateDevOps);

      if( t_err < 0)
      {
         UTL_END_CRITICAL(); //Enable interrupts
         return t_err;
      }

      /*---------------------------------------------------------------------*
       * Request a semaphore from the kernel. Since the signal port is a shared
       * resource we need to have all actions on it be mutually exclusive.
       *---------------------------------------------------------------------*/
      gat_templateDev[c_index].t_mutex = arb_semaphoreCreate( MUTEX);

      if( gat_templateDev[c_index].t_mutex < 0)
      {
         arb_destroyDevice( ac_devName);
         UTL_END_CRITICAL(); //Enable interrupts
         return gat_templateDev[c_index].t_mutex;

      }/*End if( gat_templateDev[c_index].t_mutex < 0)*/

      /*---------------------------------------------------------------------*
       * We dont have any users attached to this device
       *---------------------------------------------------------------------*/
      gat_templateDev[c_index].c_numUsers = 0;

   }/*End for( c_index = 0; c_index < TEMPLATE_MAX_MINORS; c_index++)*/

   UTL_END_CRITICAL(); //Enable interrupts

   return t_err;

}/*End templateInit*/

/*---------------------------------------------------------------------------*
 * Remove the device from the system
 *---------------------------------------------------------------------------*/
void templateExit( void)
{
   uint8_t c_index    = 0;
   char ac_devName[MAX_DEVICE_NAME_BYTES];

   /*------------------------------------------------------------------------*
    * We are going to be removing items from memory that may be being used
    * by another process.
    *------------------------------------------------------------------------*/
   UTL_BEGIN_CRITICAL(); //Disable interrupts

   for( c_index = 0; c_index < TEMPLATE_MAX_MINORS; c_index++)
   {
      if( gat_templateDev[c_index].t_mutex != 0) /*If created... destroy*/
      {
         sprintf(ac_devName, "templateDevice%d",c_index);
         arb_semaphoreDestroy( &gat_templateDev[c_index].t_mutex);
         arb_destroyDevice( ac_devName);

         /*------------------------------------------------------------------*
          * Remove any user-space specific generated memory here...
          *------------------------------------------------------------------*/
      }/*End if( gat_templateDev[c_index].t_mutex != 0)*/

   }/*End for( c_index = 0; c_index < TEMPLATE_MAX_MINORS; c_index++)*/

   UTL_END_CRITICAL(); //Enable interrupts

}/*End templateExit*/
