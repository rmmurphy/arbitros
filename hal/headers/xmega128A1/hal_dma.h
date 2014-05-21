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
 * File Name   : hal_dma.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides an abstraction layer for controlling a
 *               device specific DMA module. The functions herein are
 *               responsible for arbitrating access to the four XMEGA DMA
 *               channels. Channel requests are granted on a first-come
 *               first-serve basis, meaning a user requests access to a
 *               channel (using hal_requestDmaChannel) and is granted the
 *               next available resource - either channel 0,1,2,or 3. If
 *               successful (DMA_PASSED), a handle is returned
 *               (t_DMAHNDL) which is the primary mechanism for distinguishing
 *               one channel from another for all configuration changes
 *               (hal_configureDmaChannel) and
 *               interrupt requests (hal_requestDmaInterrupt).
 *
 * Last Update : Oct 23, 2011
 *---------------------------------------------------------------------------*/
#ifndef hal_dma_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define hal_dma_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      DMA_INVALID_TRIGGER_SOURCE  = -15, /*Invalid trigger source*/
      DMA_INVALID_TRANSFER_TYPE   = -14, /*Invalid transfer type config*/
      DMA_INVALID_BURST_MODE      = -13, /*Invalid burst mode config*/
      DMA_INVALID_BLOCK_SIZE      = -12, /*Invalid block size config*/
      DMA_INVALID_ADD_RELOAD_MODE = -11, /*Invalid address reload config*/
      DMA_INVALID_ADDRESS_MODE    = -10, /*Invalid address mode config*/
      DMA_INVALID_CHAN_PRIORITY   = -9,  /*Invalid channel priority config*/
      DMA_INVALID_BUFF_MODE       = -8,  /*Invalid buffering mode config*/
      DMA_NO_CHANNELS_OPEN        = -7,  /*No channels available*/
      DMA_INTERRUPT_OPEN          = -6,  /*Interrupt already open*/
      DMA_INVALID_INT_TYPE        = -5,  /*Request of invalid interrupt
                                           or one that is not open.*/
      DMA_INT_NOT_OPEN            = -4,  /*Interrupt hasn't been mapped*/
      DMA_NO_CONFIG               = -3,  /*DMA config hasn't been called*/
      DMA_INVALID_HANDLE          = -2,  /*Handle doesn't map to a DMA channel*/
      DMA_OUT_OF_HEAP             = -1,  /*No more memory.*/
      DMA_PASSED                  = 0    /*Configuration good.*/

   }t_dmaError;

   typedef enum
   {
      DMA_CHAN_0 = 0,
      DMA_CHAN_1 = 2,
      DMA_CHAN_2 = 4,
      DMA_CHAN_3 = 6

   }t_dmaChanId;

   typedef enum
   {
      DMA_TRANSFER_COMPLETE = 0,
      DMA_TRANSFER_ERROR

   }t_dmaIntType;

   typedef enum
   {
      DOUBLE_BUFF_DISABLED = 0,
      DOUBLE_BUFF_CHAN_0_1 = 4,
      DOUBLE_BUFF_CHAN_2_3 = 8,
      DOUBLE_BUFF_ALL_CHAN = 12

   }t_bufferingMode;

   typedef enum
   {
      ROUND_ROBIN = 0,
      CHAN_0_ROUND_REST,
      CHAN_0_1_ROUND_REST,
      CHAN_0_1_2_3,

   }t_channelPriority;

   typedef enum
   {
      FIXED     = 0,
      INCREMENT,
      DECREMENT

   }t_dmaAddressDirection;

   typedef enum
   {
      NO_RELOAD = 0,
      RELOAD_END_OF_BLOCK,
      RELOAD_END_OF_BURST,
      RELOAD_END_OF_TRANSACTION

   }t_dmaAddressReload;

   typedef enum
   {

      BLOCK = 0,
      SINGLE_SHOT,

   }t_dmaTransferType;

   typedef enum
   {
      ONE_BYTE = 0,
      TWO_BYTE,
      FOUR_BYTE,
      EIGHT_BYTE

   }t_dmaBurstMode;

   typedef enum
   {

      SOFTWARE                = (0x00<<0),
      EVENT_0                 = (0x01<<0),
      EVENT_1                 = (0x02<<0),
      EVENT_2                 = (0x03<<0),
      ADC1_CHAN_0             = (0x10<<0),
      ADC1_CHAN_1             = (0x11<<0),
      ADC1_CHAN_2             = (0x12<<0),
      ADC1_CHAN_3             = (0x13<<0),
      ADC1_COMBINED           = (0x14<<0),
      DAC1_CHAN_0             = (0x15<<0),
      DAC1_CHAN_1             = (0x16<<0),
      ADC2_CHAN_0             = (0x20<<0),
      ADC2_CHAN_1             = (0x21<<0),
      ADC2_CHAN_2             = (0x22<<0),
      ADC2_CHAN_3             = (0x23<<0),
      ADC2_COMBINED           = (0x24<<0),
      DAC2_CHAN_0             = (0x25<<0),
      DAC2_CHAN_1             = (0x26<<0),
      TIMER1_OVERFLOW         = (0x40<<0),
      TIMER1_ERROR            = (0x41<<0),
      TIMER1_COMPAREA         = (0x42<<0),
      TIMER1_COMPAREB         = (0x43<<0),
      TIMER1_COMPAREC         = (0x44<<0),
      TIMER1_COMPARED         = (0x45<<0),
      TIMER2_OVERFLOW         = (0x46<<0),
      TIMER2_ERROR            = (0x47<<0),
      TIMER2_COMPAREA         = (0x48<<0),
      TIMER2_COMPAREB         = (0x49<<0),
      SPI1_TRANSFER_COMPLETE  = (0x4A<<0),
      UART1_RX_COMPLETE       = (0x4B<<0),
      UART1_DATA_REG_EMPTY    = (0x4C<<0),
      UART2_RX_COMPLETE       = (0x4E<<0),
      UART2_DATA_REG_EMPTY    = (0x4F<<0),
      TIMER3_OVERFLOW         = (0x60<<0),
      TIMER3_ERROR            = (0x61<<0),
      TIMER3_COMPAREA         = (0x62<<0),
      TIMER3_COMPAREB         = (0x63<<0),
      TIMER3_COMPAREC         = (0x64<<0),
      TIMER3_COMPARED         = (0x65<<0),
      TIMER4_OVERFLOW         = (0x66<<0),
      TIMER4_ERROR            = (0x67<<0),
      TIMER4_COMPAREA         = (0x68<<0),
      TIMER4_COMPAREB         = (0x69<<0),
      SPI2_TRANSFER_COMPLETE  = (0x6A<<0),
      UART3_RX_COMPLETE       = (0x6B<<0),
      UART3_DATA_REG_EMPTY    = (0x6C<<0),
      UART4_RX_COMPLETE       = (0x6E<<0),
      UART4_DATA_REG_EMPTY    = (0x6F<<0),
      TIMER5_OVERFLOW         = (0x80<<0),
      TIMER5_ERROR            = (0x81<<0),
      TIMER5_COMPAREA         = (0x82<<0),
      TIMER5_COMPAREB         = (0x83<<0),
      TIMER5_COMPAREC         = (0x84<<0),
      TIMER5_COMPARED         = (0x85<<0),
      TIMER6_OVERFLOW         = (0x86<<0),
      TIMER6_ERROR            = (0x87<<0),
      TIMER6_COMPAREA         = (0x88<<0),
      TIMER6_COMPAREB         = (0x89<<0),
      SPI3_TRANSFER_COMPLETE  = (0x8A<<0),
      UART5_RX_COMPLETE       = (0x8B<<0),
      UART5_DATA_REG_EMPTY    = (0x8C<<0),
      UART6_RX_COMPLETE       = (0x8E<<0),
      UART6_DATA_REG_EMPTY    = (0x8F<<0),
      TIMER7_OVERFLOW         = (0xA0<<0),
      TIMER7_ERROR            = (0xA1<<0),
      TIMER7_COMPAREA         = (0xA2<<0),
      TIMER7_COMPAREB         = (0xA3<<0),
      TIMER7_COMPAREC         = (0xA4<<0),
      TIMER7_COMPARED         = (0xA5<<0),
      TIMER8_OVERFLOW         = (0xA6<<0),
      TIMER8_ERROR            = (0xA7<<0),
      TIMER8_COMPAREA         = (0xA8<<0),
      TIMER8_COMPAREB         = (0xA9<<0),
      SPI4_TRANSFER_COMPLETE  = (0xAA<<0),
      UART7_RX_COMPLETE       = (0xAB<<0),
      UART7_DATA_REG_EMPTY    = (0xAC<<0),
      UART8_RX_COMPLETE       = (0xAE<<0),
      UART8_DATA_REG_EMPTY    = (0xAF<<0)

   }t_dmaTriggerSource;

   typedef struct
   {
      /*---------------------------------------------------------------------*
       * The location where the DMA is grabbing data.
       *---------------------------------------------------------------------*/
      uint32_t              *pi_srcAddress;
      /*---------------------------------------------------------------------*
       * The location where the DMA is putting data.
       *---------------------------------------------------------------------*/
      uint32_t              *pi_destAddress;
      /*---------------------------------------------------------------------*
       * Controls how the DMA moves through adjacent address to
       * 'i_sourceAddress'
       *---------------------------------------------------------------------*/
      t_dmaAddressDirection t_srcAddDir;
      /*---------------------------------------------------------------------*
       * Controls how the DMA moves through adjacent address to
       * 't_destAddMode'
       *---------------------------------------------------------------------*/
      t_dmaAddressDirection t_destAddDir;
      /*---------------------------------------------------------------------*
       * Controls when the DMA starts over at the address defined by
       * 'i_sourceAddress'
       *---------------------------------------------------------------------*/
      t_dmaAddressReload    t_srcAddReload;
      /*---------------------------------------------------------------------*
       * Controls when the DMA starts over at the address defined by
       * 't_destAddReload'
       *---------------------------------------------------------------------*/
      t_dmaAddressReload    t_destAddReload;
      /*---------------------------------------------------------------------*
       * The size of the transfer in bytes.
       *---------------------------------------------------------------------*/
      uint16_t              s_blockSize;
      /*---------------------------------------------------------------------*
       * Controls how many bytes are read out of the location defined by
       * 'i_sourceAddress' each data transfer.
       *---------------------------------------------------------------------*/
      t_dmaBurstMode        t_burstMode;
      /*---------------------------------------------------------------------*
       * If 'SINGLE_SHOT' then each trigger causes a single data transfer of
       * 1, 2, 4, 8 bytes - which ever is defined by 't_burstMode'.
       * Otherwise, then dma will perform an entire block transfer of
       * 's_blockSize' bytes.
       *---------------------------------------------------------------------*/
      t_dmaTransferType     t_transferType;
      /*---------------------------------------------------------------------*
       * Configures the event that triggers a DMA transaction.
       *---------------------------------------------------------------------*/
      t_dmaTriggerSource    t_triggerSrc;
      /*---------------------------------------------------------------------*
       * Controls the number of blocks (as defined by 's_blockSize') the DMA
       * will transfer. If the value is 0 its disabled, if the valud < 0
       * then it repeats forever.
       *---------------------------------------------------------------------*/
      int8_t                c_repeatCount;

   }t_dmaChanConfig; /*Configuration for a particular DMA channel*/

   typedef struct
   {
      t_bufferingMode   t_buffMode;
      t_channelPriority t_chanPriority;

   }t_dmaCntrlConfig; /*Configuration for the DMA controller module*/

   typedef volatile int16_t t_DMAHNDL; /*Handle to a particular DMA channel*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * This function request access to the next available DMA channel.
    *------------------------------------------------------------------------*/
   t_DMAHNDL hal_requestDmaChannel( void);

   /*------------------------------------------------------------------------*
    * This function releases access over a particular DMA channel.
    *------------------------------------------------------------------------*/
   t_dmaError hal_releaseDmaChannel( t_DMAHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Configures the DMA transaction for the channel pointed to by
    * 't_handle'.
    *------------------------------------------------------------------------*/
   t_dmaError hal_configureDmaChannel( t_DMAHNDL t_handle,
                                       t_dmaChanConfig t_conf);

   /*------------------------------------------------------------------------*
    * Requests an interrupt mapping for the channel pointed to by 't_handle'.
    *------------------------------------------------------------------------*/
   t_dmaError hal_requestDmaInterrupt( t_DMAHNDL t_handle,
                                       t_dmaIntType  t_type,
                                       void (*pf_funPtr)( void));

   /*------------------------------------------------------------------------*
    * Releases the interrupt mapping for the channel pointed to by 't_handle'.
    *------------------------------------------------------------------------*/
   t_dmaError hal_releaseDmaInterrupt( t_DMAHNDL t_handle,
                                       t_dmaIntType  t_type);

   /*------------------------------------------------------------------------*
    * Manually triggers a DMA transfer
    *------------------------------------------------------------------------*/
   t_dmaError hal_dmaStartTransfer( t_DMAHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Enables the DMA on the channel pointed to by 't_handle'.
    *------------------------------------------------------------------------*/
   t_dmaError hal_dmaEnableChannel( t_DMAHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Disables the DMA on the channel pointed to by 't_handle'.
    *------------------------------------------------------------------------*/
   t_dmaError hal_dmaDisableChannel( t_DMAHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Configure the block transfer size for the DMA on the channel pointed to
    * by 't_handle'.
    *------------------------------------------------------------------------*/
   t_dmaError hal_setDmaBlockSize( t_DMAHNDL t_handle,
                                   uint16_t s_blockSize);

   /*------------------------------------------------------------------------*
    * Check for an onging transaction on the DMA channel pointed to by
    * 't_handle'.
    *------------------------------------------------------------------------*/
   int16_t hal_getDmaStatus( t_DMAHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Get the status of an interrupt on the DMA channel pointed to by
    * 't_handle'
    *------------------------------------------------------------------------*/
   int16_t hal_getDmaIntStatus( t_DMAHNDL t_handle,
                                t_dmaIntType t_int);

   /*------------------------------------------------------------------------*
    * Clear the status of an interrupt on the DMA channel pointed to by
    * 't_handle'
    *------------------------------------------------------------------------*/
   t_dmaError hal_clearDmaIntStatus( t_DMAHNDL t_handle,
                                     t_dmaIntType t_int);

   /*------------------------------------------------------------------------*
    * Check the amount of data transfered on the DMA channel pointed to by
    * 't_handle'.
    *------------------------------------------------------------------------*/
   uint16_t hal_getDmaTransferCount( t_DMAHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Set the source address on the DMA channel pointed to by 't_handle'.
    *------------------------------------------------------------------------*/
   t_dmaError hal_setDmaSourceAddress( t_DMAHNDL t_handle,
                                       uint32_t i_address);

   /*------------------------------------------------------------------------*
    * Set the destination address on the DMA channel pointed to by 't_handle'.
    *------------------------------------------------------------------------*/
   t_dmaError hal_setDmaDestAddress( t_DMAHNDL t_handle,
                                     uint32_t i_address);

   /*------------------------------------------------------------------------*
    * Returns the channel Id for a particular DMA pointed to by 't_handle'.
    *------------------------------------------------------------------------*/
   t_dmaChanId hal_getDmaChannelId( t_DMAHNDL t_handle);
   
   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef hal_dma_h*/
