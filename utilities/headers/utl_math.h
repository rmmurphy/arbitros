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
 * File Name   : utl_math.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides various optimized fixed-point and
 *               float-point vector/matrix operations.
 *
 * Last Update : September 26, 2012
 *---------------------------------------------------------------------------*/
#ifndef utl_math_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Public defines
    *------------------------------------------------------------------------*/
   #define utl_math_h
   #define UTL_MATH_FXDPNT_PI          (32767)
   #define UTL_MATH_FXDPNT_NEGATIVE_PI (-32768)
   #define UTL_MATH_FXDPNT_TWO_PI_WRAP (65536)
   #define UTL_MATH_SAT_CHECK          (0)

   /*------------------------------------------------------------------------*
    * Public typdefs
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * This typedef forces the compiler to allow 'type-punning' for the
    * functions that use optimized bit shifts.
    *------------------------------------------------------------------------*/
   typedef union
   {
      int32_t i_word32;
      int16_t as_word16[2];
   }t_typePunn;

   /*------------------------------------------------------------------------*
    * Public global variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Macro definitions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Inline functions
    *------------------------------------------------------------------------*/
   static inline int32_t __attribute__((always_inline)) \
   utl_abs32_32( int32_t i_y)
   {
      if( i_y < 0)
         i_y = -i_y;

      return i_y;

   }/*End utl_abs32_32*/

   static inline int32_t __attribute__((always_inline)) \
   utl_hardLimit32_32( int32_t i_x, int8_t c_n)
   {
      if( i_x > (((int32_t)1 << c_n) - 1))
       {
          return (((int32_t)1 << c_n) - 1);
      }
       else if( i_x < -((int32_t)1 << c_n))
       {
         return -((int32_t)1 << c_n);
      }

      return i_x;
   }

   /*-------------------------------------------------------------------------*
    *
    * Function:
    *	   utl_rShft32_16
    *
    * Description:
    *	   Converts a Qm.n 32-bit number into a Qm.n 16-bit number, where m is
    *    the number of integer bits and n represents the number of fractional
    *    bits.
    *
    * Parameters:
    *    i_x - Qm.n 32-bit number where m is the number of integer bits and n
    *          is the number of fractional bits
    *    c_n  - The number of fractional bits in the return value s_y
    *
    * Return:
    *    s_y - Qm.n 16-bit number where m is the number of integer bits and n
    *          is the number of fractional bits.
    *
    * References:
    *    http://en.wikipedia.org/wiki/Q_(number_format)
    *------------------------------------------------------------------------*/
   static inline int16_t __attribute__((always_inline)) \
   utl_rShft32_16( int32_t i_x, int8_t c_n)
   {
      t_typePunn t_tWord;
      int8_t c_sign = 1;

      if( i_x < 0)
      {
         i_x = -i_x;
         c_sign = -1;
      }

      /*---------------------------------------------------------------------*
       * Perform rounding...
       *---------------------------------------------------------------------*/
      //i_x += ((int32_t)1 << (c_n - 1));

      /*---------------------------------------------------------------------*
       * Normally a conversion from one Qm.n number to another requires a
       * shift right of 'x' number of bits- which can be a costly operation
       * in terms of processor cycles. This impact can be minimized by
       * analyzing the product for a pivot point that gives the least number
       * of shifts. Bits are shifted up or down from the surrounding words
       * depending on the move that gives the least number of operations. The
       * pivot point is then used as the starting location for the word that
       * gets returned. This can be easily understood with a simply example of
       * the mutliplication of two Q0.15 numbers;
       *    x1 = 0x2000, x2 = 0x2000
       *    y = (x1*x2) >> 15
       *      = (0x2000*0x2000) >> 15
       *      = (0x0400 0000) >> 15
       *      = 0x0800
       * This operation just moves the upper 15 bits of the 32-bit word into
       * the lower 16-bits and retains just 1-bit of the original lower
       * bits. This is the same thing as shifting up by 1-bit and returning the
       * upper 16-bits of the 32-bit word;
       *    y = UPPER[(0x2000*0x2000) << 1]
       *      = UPPER[(0x0400 0000) << 1]
       *      = UPPER[0x0800 0000]
       *      = 0x0800
       * In this example the pivot point is the second 16-bit word.
       *---------------------------------------------------------------------*/
      if( (c_n > 8) && (c_n <= 16)) /*Pivot-point = second 16-bit word...*/
      {
         /*------------------------------------------------------------------*
          * The least number of shifts occur if the first 16-bit word is
          * shifted up into the second 16-bit word...
          *------------------------------------------------------------------*/
         t_tWord.i_word32 = i_x << (16 - c_n);

         /*------------------------------------------------------------------*
          * Use ptr's in order to return a 16-bit word starting from the
          * location of the second 16-bit word.
          *------------------------------------------------------------------*/
         if( c_sign == -1)
              return -t_tWord.as_word16[1];
           else
            return t_tWord.as_word16[1];
      }

      i_x = i_x >> c_n;

      /*---------------------------------------------------------------------*
       * The least number of shifts occur if the 32-bit word is shifted down
       * and a 16-bit word is returned starting from the location of the first
       * 16-bit word.
       *---------------------------------------------------------------------*/
      if( c_sign == -1)
         return -(int16_t)i_x; /*Pivot-point = first 16-bit word...*/
       else
           return (int16_t)i_x;

   }/*End utl_rShft32_16*/

   /*------------------------------------------------------------------------*
    *
    * Function:
    *	   utl_linInterp32_32
    *
    * Description:
    *	   Finds the value y for a given x in between two sets of data points
    *    (x0,y0) and (x1,y1) using linear interpolation.
    *
    * Constraints: The maximum difference between ((x1-x0) and (y1-y0)) can be
    *              no larger than 32767/-32768.
    *
    * Parameters:
    *    i_x - Known Qm.n 32-bit x-axis position where m is the number of
    *          integer bits and n is the number of fractional bits
    *    i_x0 - Qm.n 32-bit x-axis data point immediately preceding i_x
    *    c_step - The lookup table 'x' axis step size in terms of shifts right.
    *    i_yo - Qm.n 32-bit y-axis data point immediately preceding i_y
    *    i_y1 - Qm.n 32-bit y-axis data point immediately following i_y
    *
    * Return:
    *    i_y - The resulting interpolated Qm.n 32-bit y-axis data point
    *          associated with i_x
    *------------------------------------------------------------------------*/
   static inline int32_t __attribute__((always_inline)) \
   utl_linInterp32_32( int32_t i_x,
                       int32_t i_x0,
                       int8_t c_step,
                       int32_t i_y0,
                       int32_t i_y1)
   {
      int32_t i_y     = 0;
      int32_t i_temp2 = 0;
      int16_t s_temp1 = 0;
      int16_t s_temp3 = 0;

      s_temp1 = (int16_t)(i_y1 - i_y0);
      s_temp3 = (int16_t)(i_x  - i_x0);

      i_temp2 = utl_rShft32_16((int32_t)s_temp1*(int32_t)s_temp3, c_step);
      i_y = i_y0 + i_temp2;

      return i_y;

   }/*End utl_linInterp32_32*/

   /*-------------------------------------------------------------------------*
    *
    * Function:
    *	   utl_quadInterp16_16
    *
    * Description:
    *	   Finds the value y for a given x in between three sets of data points
    *    (x0,y0), (x1,y1), and (x2,y2) using Newton's quadratic interpolation.
    *    See www.saylor.org/site/wp-content/uploads/2011/11/ME205-5.3-TEXT.pdf
    *    for further information.
    *
    * Parameters:
    *    s_x    - Known x-axis position
    *    s_x0   - Data point immediately preceding i_x
    *    s_x1   - Data point immediately following i_x
    *    c_step - The lookup table 'x' axis step size in terms of shifts right.
    *    s_y0   - 'Y' data point immediately preceding i_x
    *    s_y1   - 'Y' data point immediately following i_x
    *    s_y2   - 'Y' data point immediately following i_x0
    *
    * Return:
    *    s_y - The resulting interpolated y-axis data point associated with i_x
    *------------------------------------------------------------------------*/
   static inline int16_t __attribute__((always_inline)) \
   utl_quadInterp16_16( int16_t s_x,
                        int16_t s_x0,
                        int16_t s_x1,
                        int8_t c_step,
                        int16_t s_y0,
                        int16_t s_y1,
                        int16_t s_y2)
   {
      int16_t s_y;
      int16_t s_b1 = s_y1 - s_y0;
      int16_t s_b2 = (((s_y2 - s_y1) - (s_y1 - s_y0)) + (int16_t)1) >> 1;
      int16_t s_temp;
      int16_t s_temp2;
      s_temp = utl_rShft32_16((int32_t)s_b2*(int32_t)(s_x - s_x1), c_step);
      s_temp2 = utl_rShft32_16((s_x - s_x0)*(int32_t)(s_b1 + s_temp), c_step);
      s_y = s_y0 + s_temp2;

      return s_y;

   }/*End utl_quadInterp16_16*/

   /*------------------------------------------------------------------------*
    * Public function prototypes
    *------------------------------------------------------------------------*/
   int16_t utl_mult16x16_16( int16_t s_x1,
                             int16_t s_x2,
                             int8_t c_n);

   int32_t utl_mult16x16_32( int16_t s_x1,
                             int16_t s_x2,
                             int8_t c_n);

   int32_t utl_mult32x32_32( int32_t i_x1,
                             int32_t i_x2,
                             int8_t c_n);

   int16_t utl_div16x16_16( int16_t s_x1,
                            int16_t s_x2,
                            int8_t c_n);

   int32_t utl_mac16x16_32( int16_t s_x1,
                            int16_t s_x2,
                            int32_t i_y,
                            int8_t c_o);

   int32_t utl_vMult16x16_32( int16_t *ps_x1,
                              int16_t *ps_x2,
                              int32_t i_y,
                              int16_t s_length,
                              int8_t c_o,
                              int8_t c_n);

   int16_t utl_vMult16x16_16( int16_t *ps_x1,
                              int16_t *ps_x2,
                              int32_t i_y,
                              int16_t s_length,
                              int8_t c_o,
                              int8_t c_n);

   bool utl_matMult16x16_16( int16_t *ps_mat1,
                              int8_t c_rows1,
                              int8_t c_col1,
                              int16_t *ps_mat2,
                              int8_t c_rows2,
                              int8_t c_col2,
                              int16_t *ps_res,
                              int8_t c_co,
                              int8_t c_cn);

    void utl_matTrans16( int16_t *ps_mat,
                         int16_t *ps_trans,
                         int8_t c_rows,
                         int8_t c_col);

   float utl_macF( float f_x1,
                   float f_x2,
                   float f_y);

   float utl_divF( float f_x1,
                   float f_x2);

   float utl_multF( float f_x1,
                    float f_x2);

   int32_t utl_log10_32( uint32_t i_y);

   int32_t utl_log2_32( uint32_t i_y);

   uint32_t utl_alog10_32( int32_t i_log10);

   uint32_t utl_alog2_32( int32_t i_log2);

   int16_t utl_cos16_16( uint16_t s_phase);

   int16_t utl_sin16_16( uint16_t s_phase);

   uint32_t utl_sqrt32_32( uint32_t i_input,
                           int8_t c_n);

   int16_t utl_atan2_16( int16_t s_y, int16_t s_x);

   bool utl_matMultF( float *pf_mat1,
                      int32_t i_rows1,
                      int32_t i_col1,
                      float *pf_mat2,
                      int32_t i_rows2,
                      int32_t i_col2,
                      float *pf_res);

   bool utl_matInvF( float *pf_mat,
                     float *pf_inv,
                     int32_t i_numElements);

   void utl_matTransF( float *pf_mat,
                       float *pf_trans,
                       int32_t i_rows,
                       int32_t i_col);

   void utl_matEyeF( float *pf_mat,
                     int32_t i_size,
                     float f_value);

   bool utl_matEigsF( float *pf_mat,
                      int32_t i_size,
                      float *pf_eigValues,
                      float *pf_eigVectors);

   float utl_normF( float *pf_vect,
                    int32_t i_size);

   void utl_matScalerMultF( float *pf_mat,
                            int32_t i_rows,
                            int32_t i_col,
                            float f_value);

   void utl_matSubF( float *pf_mat1,
                     float *pf_mat2,
                     int32_t i_rows,
                     int32_t i_col,
                     float *pf_result);

   float utl_maxF( float *pf_vect,
                    int32_t i_size);

   float utl_minF( float *pf_vect,
                   int32_t i_size);

   float utl_absMaxF( float *pf_vect,
                      int32_t i_size);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif /*End #ifndef utl_math_h*/

