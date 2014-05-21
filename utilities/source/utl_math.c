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
 * File Name   : utl_math.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides various optimized fixed-point and
 *               float-point vector/matrix operations.
 *
 * Last Update : September 26, 2012
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include files
 *---------------------------------------------------------------------------*/
#include <string.h>
#include <math.h>
#include "utl_math.h"

/*---------------------------------------------------------------------------*
 * Private defines
 *---------------------------------------------------------------------------*/
#define UTL_MATH_MAX_MAT_COL_SZ          (18)
#define UTL_MATH_MAX_MAT_ROW_SZ          (9)
#define UTL_MATH_LOG10_SHFT              (10)
#define UTL_MATH_ALOG10_SHFT             (10)
#define UTL_MATH_SINCOS_SHFT             (12)
#define UTL_MATH_ATAN2_SHFT              (11)
#define UTL_MATH_SQRT_SHFT               (11)
#define UTL_MATH_LOG10_TBL_SZ            (17)
#define UTL_MATH_ALOG10_TBL_SZ           (11)
#define UTL_MATH_SINCOS_TBL_SZ           (16)
#define UTL_MATH_ATAN2_TBL_SZ            (18)
#define UTL_MATH_SQRT_TBL_SZ             (14)
#define UTL_MATH_ONEOVER_SQRT_OF_TWO     (23170)
#define UTL_MATH_SQRT_LKUP_TBL_MIN_INPUT (8192)
#define UTL_MATH_SQRT_LKUP_TBL_MAX_INPUT (34816)
#define UTL_MATH_ONEOVER_LOG10OF2_Q13    (27213) /* round((2^13/log10(2)))*/
#define UTL_MATH_LOG10OF2_Q15            (9864)  /* round((2^15*log10(2)))*/
#define POWER_METHOD_MAX_ERROR           (.0001f)

/*---------------------------------------------------------------------------*
 * Private function prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private variables
 *---------------------------------------------------------------------------*/

static int16_t gas_log10Tbl[UTL_MATH_LOG10_TBL_SZ] =
{
   -9864, -9001, -8188, -7418, -6688, -5994, -5332, -4700, -4094, -3513,
   -2955, -2418, -1900, -1401, -918, -452, 0
};

static uint16_t gas_alog10Tbl[UTL_MATH_ALOG10_TBL_SZ] =
{
   16384, 17606, 18919, 20331, 21848, 23478, 25229, 27112, 29134, 31308,
   33644
};

/*---------------------------------------------------------------------------*
 * Table size                     = 16
 * Input step size                = 4096 (4096 / 65536) = 0.0625 radians
 * Table input range              = 0 to 65536 (0 to 2PI radians)
 * Table output range             = 1 to 1
 *---------------------------------------------------------------------------*/
static int16_t gas_cosTbl[UTL_MATH_SINCOS_TBL_SZ+1] =
{
   32767, 30273, 23170, 12539, 0, -12539, -23170, -30273, -32768, -30273,
  -23170, -12539, 0, 12539, 23170, 30273, 32767
};

/*---------------------------------------------------------------------------*
 * Table size                     = 18
 * Input step size                = 2048 (2048 / 32768) = 0.0625
 * Table input range              = 0 to 32767 (0 to 1)
 * Table output range             = 0 to 46.73570459 degrees
 *---------------------------------------------------------------------------*/
static uint16_t gas_atan2Tbl[UTL_MATH_ATAN2_TBL_SZ] =
{
   0, 651, 1297, 1933, 2555, 3159, 3742, 4301, 4836, 5344, 5826, 6282, 6712,
   7117, 7497, 7855, 8192, 8508
};

/*---------------------------------------------------------------------------*
 * Table size                     = 14
 * Input step size                = 2048 (2048 / 32768) = 0.0625
 * Table input range              = 8192 to 34816 (.25 to
 *                                  1.0625)
 * Table output range             = .5 to 1.030776406 (16384 to 33775)
 *---------------------------------------------------------------------------*/
static uint16_t gas_sqrtTbl[UTL_MATH_SQRT_TBL_SZ] =
{
   16384, 18317, 20066, 21673, 23170, 24575, 25905, 27169, 28377, 29536, 30651,
   31727, 32767, 33775
};

/*---------------------------------------------------------------------------*
 * Each index 'n' into table represents a scale factor of [1/sqrt(2)]^(n-1)
 * where 'n' represents the number of shifts required to move the input
 * of utl_sqrt32_32 within the lower range of gas_sqrtTbl.
 * Q0.16
 *---------------------------------------------------------------------------*/
static uint16_t gas_sqrtUpNorm[UTL_MATH_SQRT_TBL_SZ] =
{
   23170, 16384, 11585, 8192, 5792, 4096, 2896, 2048, 1448, 1024, 724, 512,
   362, 256
};

/*---------------------------------------------------------------------------*
 * Each index 'n' into table represents a scale factor of [1/sqrt(2)]^(n-1)
 * where 'n' represents the number of shifts required to move the input
 * of utl_sqrt32_32 within the upper range of gas_sqrtTbl.
 * Q0.8
 *---------------------------------------------------------------------------*/
static uint16_t gas_sqrtDownNorm[UTL_MATH_SQRT_TBL_SZ] =
{
   362, 512, 724, 1024, 1448, 2048, 2896, 4096, 5793, 8192, 11585, 16384,
   23170, 32767
};

/*---------------------------------------------------------------------------*
 *
 * Function:
 *    utl_log10_32
 *
 * Description:
 *   This function performs a base 10 logarithm operation on the Qm.n 32-bit
 *   input variable i_y, where there are m = 17 integer bits and n = 15
 *   fractional bits. The function solves the equation log10(y) = M*log10(2)
 *   + log10(z) by looking at the input in matissa exponent form (y = (2^M)*z)
 *   and placing a restriction that z must lie in the range .5 <= z < 1.0.
 *
 * Parameters:
 *    i_y - Q17.15 32-bit number with a range of 0.000030517578125 to
 *          131071.999969482421875 (1 to 4,294,967,295)
 *
 * Return:
 *    i_log10 - Q16.15 32-bit number with a range of -4.515dB to 5.118dB
 *              (-147,958 to 167,688)
 *---------------------------------------------------------------------------*/
int32_t utl_log10_32( uint32_t i_y)
{

   uint32_t i_tempY  = 0;
   int32_t  i_log10  = 0;
   uint16_t s_z      = 0;
   int16_t  s_zIndex = 0;
   int16_t  s_M      = 0;
   int16_t  s_temp   = 0;

   /*------------------------------------------------------------------------*
    * Hard limit...
    *------------------------------------------------------------------------*/
   if( i_y <= 0)
      i_y = 1;

   i_tempY = i_y;

   /*------------------------------------------------------------------------*
    * Solve for M and z
    *------------------------------------------------------------------------*/
   if( i_y >= 32767) /*y >= 1.0*/
   {

      while( i_tempY > 32767)  /*while( y > 1)*/
      {
         s_M++;
         i_tempY = i_tempY >> 1;
      }

      s_z = (uint16_t)(i_y >> s_M); /* z = y / (2^M)*/

   }/*End if( i_y >= 32767)*/
   else if( i_y < 16384) /*y < .5*/
   {

      while( i_tempY < 16385) /*while( y < .5)*/
      {
         s_M++;
         i_tempY = i_tempY << 1;
      }

     s_z = (uint16_t)(i_y << s_M);
     s_M = -s_M;

   }
   else
   {
      s_M = 0;
      s_z = (int16_t)(i_y);
   }

   /*------------------------------------------------------------------------*
    * Subtract off the lower table bound (.5) and divide by the step size.
    *------------------------------------------------------------------------*/
   s_zIndex = (s_z - 16384) >> UTL_MATH_LOG10_SHFT;

   if( s_zIndex < (UTL_MATH_LOG10_TBL_SZ - 1))
   {
      s_temp = utl_linInterp32_32( s_z,
                                   (int32_t)(16384 + (s_zIndex*(1 <<
                                   UTL_MATH_LOG10_SHFT))),
                                   UTL_MATH_LOG10_SHFT,
                                   (int32_t)gas_log10Tbl[s_zIndex],
                                   (int32_t)gas_log10Tbl[s_zIndex + 1]);

      i_log10 = (int32_t)9864*(int32_t)s_M + s_temp;
   }
   else
      i_log10 = (int32_t)9864*(int32_t)s_M + gas_log10Tbl[s_zIndex];

   return i_log10;

}/*End utl_log10_32*/

/*---------------------------------------------------------------------------*
 * Function:
 *    utl_log2_32
 *
 * Description:
 *    This function performs a base 2 logarithm operation on the input
 *    variable i_y by first finding the log base 10 and dividing by
 *    log10(2).
 *
 * Parameters:
 *    i_y - Q17.15 32-bit number with a range of 0.000030517578125 to
 *          131071.999969482421875 (1 to 4,294,967,295)
 *
 * Return:
 *    i_log2 - Q16.15 32-bit number with a range of -15dB to
 *             16.9999999996641(-491,505 to 557,038)
 *---------------------------------------------------------------------------*/
int32_t utl_log2_32( uint32_t i_y)
{
   int32_t i_log10 = utl_log10_32( i_y);
   int8_t c_count = 0;

   while( i_log10 > 32767)
   {
      i_log10 = i_log10 >> 1;
      c_count++;
   }

   return (((int32_t)i_log10*(int32_t)UTL_MATH_ONEOVER_LOG10OF2_Q13) >>
   (13 - c_count));

}/*End utl_log2_32*/

/*---------------------------------------------------------------------------*
 * Function:
 *    utl_alog10_32
 *
 * Description:
 *    This function performs a base 10 anti logarithm operation on the input
 *    variable i_log10- which is in Q16.15 32-bit format.
 *
 * Parameters:
 *    i_log10 - Q16.15 32-bit number with a range of -4.515dB to 5.118dB
 *              (-147,958 to 167,688)
 *
 * Return:
 *    i_alog10 - Q17.15 32-bit number with a range of 0.000030517578125 to
 *               131071.999969482421875 (1 to 4,294,967,295)
 *---------------------------------------------------------------------------*/
uint32_t utl_alog10_32( int32_t i_log10)
{

   uint32_t i_alog10  = 0;
   int32_t  i_zLog10  = 0;
   uint16_t s_z       = 0;
   int16_t  s_zIndex  = 0;
   int16_t  s_M       = 0;
   uint32_t i_temp    = 1;
   int16_t  s_signOnM = 1;
   bool b_invert      = false;

   /*------------------------------------------------------------------------*
    * Solve M*log10(2) + log10(z) with log10(z) restricted to the range of
    * -0.3010dB to 0.0037dB.
    *------------------------------------------------------------------------*/
   if( i_log10 > 9864) /* if( log(y) > log(2)*/
   {

      s_M = i_log10 / 9864;

      if( (i_log10 - ((int32_t)s_M*(int32_t)9864) > 0))
         s_M++;

      i_zLog10 = i_log10 - (int32_t)s_M*(int32_t)9864;

      s_signOnM = 1;

   }/*End if( i_log10 > -gas_log10Tbl[0])*/
   else if( i_log10 < -9864) /* if( log(y) < -log(2)*/
   {

      s_M = (i_log10 / -9864);

      i_zLog10 = i_log10 + (int32_t)s_M*(int32_t)9864;

      s_signOnM = -1;

   }/*End else if( i_log10 < gas_log10Tbl[0])*/
   else /* -log(2) <= log(y) <= log(2) - the number fits within the table*/
   {
     if( i_log10 > 0)
     {
        i_log10 = -i_log10;
         b_invert = true;
      }
      i_zLog10  = i_log10;
      s_signOnM = 0; /*M = 0*/
   }

   /*------------------------------------------------------------------------*
    * Add the lower bound log(.5) and divide by the step size.
    *------------------------------------------------------------------------*/
   s_zIndex = (int16_t)((i_zLog10 + 9864) >> UTL_MATH_ALOG10_SHFT);

   if( s_zIndex < (UTL_MATH_ALOG10_TBL_SZ - 1))
   {

      s_z = utl_linInterp32_32( i_zLog10,
                                (int32_t)(-9864 + (s_zIndex*(1 <<
                                UTL_MATH_ALOG10_SHFT))),
                                UTL_MATH_ALOG10_SHFT,
                                (int32_t)gas_alog10Tbl[s_zIndex],
                                (int32_t)gas_alog10Tbl[s_zIndex + 1]);

   }
   else
   {
      s_z = utl_linInterp32_32( i_zLog10,
                                (int32_t)(-9864 + ((s_zIndex - 1)*(1 <<
                                UTL_MATH_ALOG10_SHFT))),
                                UTL_MATH_ALOG10_SHFT,
                                (int32_t)gas_alog10Tbl[s_zIndex - 1],
                                (int32_t)gas_alog10Tbl[s_zIndex]);
   }

   while( s_M > 0)
   {
      i_temp = (i_temp << 1);
      s_M--;
   }/*End while( s_M > 0)*/

   if( s_signOnM == 1)
   {

      if( (i_temp == 131072) && (s_z >= 32768))
         i_alog10 = 4294967295;
      else
         i_alog10 = i_temp*(uint32_t)s_z;
   }
   else if( s_signOnM == -1)
   {
      i_alog10 = (uint32_t)s_z / i_temp;
   }
   else
   {
      if( b_invert == true)
      {
         i_alog10 = ((uint32_t)32767*(uint32_t)32768) / s_z;
      }
      else
         i_alog10 = s_z;
   }

   return i_alog10;

}/*End utl_alog10_32*/

/*---------------------------------------------------------------------------*
 * Function:
 *    utl_alog2_32
 *
 * Description:
 *    This function performs a base 2 anti logarithm operation on the input
 *    variable i_log2 by converting it to base 10 and taking an alog10 on the
 *    result.
 *
 * Parameters:
 *    i_log2 - Q16.15 32-bit number with a range of -15dB to
 *             16.9999999996641(-491,505 to 557,038)
 *
 * Return:
 *    i_alog2 = Q17.15 32-bit number with a range of 0.000030517578125 to
 *              131071.999969482421875 (1 to 4,294,967,295)
 *---------------------------------------------------------------------------*/
uint32_t utl_alog2_32( int32_t i_log2)
{
   int8_t c_count = 0;

   while( i_log2 > 32767)
   {
      i_log2 = i_log2 >> 1;
      c_count++;
   }

   while( i_log2 < -32768)
   {
      i_log2 = i_log2 >> 1;
      c_count++;
   }

   return utl_alog10_32( (((int32_t)i_log2*(int32_t)UTL_MATH_LOG10OF2_Q15)
   >> (15 - c_count)));

}/*End utl_alog2_32*/

/*---------------------------------------------------------------------------*
 * Function:
 *    utl_cos16_16
 *
 * Description:
 *    Returns the cos of the fixed-point representation of a radian angle in
 *    Q1.15 16-bit format, where pi/-pi is represented by 32767/-32768. A
 *    maximum of ~2% error occurs at +/-90 deg when compared against a
 *    comparable floating-point cos function.
 *
 * Parameters:
 *    s_phase = Q0.16 number where 2*pi = 65,535
 *
 * Return:
 *    s_cos = Q1.15 number
 *---------------------------------------------------------------------------*/
int16_t utl_cos16_16( uint16_t s_phase)
{
   uint8_t c_index = 0;
   int16_t s_cos   = 0;

   c_index = (uint8_t)(s_phase >> UTL_MATH_SINCOS_SHFT);

   s_cos = utl_linInterp32_32( (int32_t)s_phase,
                               ((int32_t)c_index)*(1 << UTL_MATH_SINCOS_SHFT),
                               (int8_t)UTL_MATH_SINCOS_SHFT,
                               (int32_t)gas_cosTbl[c_index],
                               (int32_t)gas_cosTbl[c_index+1]);
   return s_cos;

}/*End utl_cos16_16*/

/*---------------------------------------------------------------------------*
 * Function:
 *    utl_sin16_16
 *
 * Description:
 *    Returns the sin of the fixed-point representation of a radian angle in
 *    Q1.15 16-bit format, where pi/-pi is represented by 32767/-32768. A
 *    maximum of ~2% error occurs at 0/180 deg when compared against a
 *    comparable floating-point sin function.
 *
 * Parameters:
 *    s_phase = Q0.16 number where 2*pi = 65,535
 *
 * Return:
 *    s_sin = Q1.15 number
 *---------------------------------------------------------------------------*/
int16_t utl_sin16_16( uint16_t s_phase)
{
   int32_t i_temp = 0;

   /*------------------------------------------------------------------------*
    * Rotate back 90 degrees
    *------------------------------------------------------------------------*/
   i_temp = (int32_t)s_phase - (UTL_MATH_FXDPNT_TWO_PI_WRAP >> 2);
   if( i_temp < 0)
      i_temp = i_temp + UTL_MATH_FXDPNT_TWO_PI_WRAP;

   return utl_cos16_16( (uint16_t)i_temp);

}/*End utl_sin16_16*/

/*---------------------------------------------------------------------------*
 * Function:
 *    utl_sqrt32_32
 *
 * Description:
 *    Performs a 32-bit square-root algorithm using a lookup table method.
 *    The table is based on Q1.15 numbers and can handle a range of input
 *    values between .25 and 1.0625 (8192 and 34816). In order to handle a
 *    wider range of values, the inputs are normalized so that they fall within
 *    the constraints of the table. The normalization factor is removed by
 *    scaling the resulting table value up of down by some factor of the
 *    sqrt(2) - depending on how many shifts were required during the
 *    normalization process. Additionally, scaling based on different Q values
 *    is handle using the aforementioned method.
 *
 * Parameters:
 *    i_input - Qm.n 32-bit number where m is the number of integer bits and n
 *              is the number of fractional bits. The algorithm can handle an
 *              input within the range of 0 to 536,870,912
 *    c_n - Number of fractional bits in the return value i_sqrt
 *
 * Return:
 *    i_sqrt - Qm.n 32-bit number where m is the number of integer bits and n
 *             is the number of fractional bits.
 *
 * Statistics:
 *    Accuracy: +/-.0008 for 1 <= i_input < 32767 (c_n = 15)
 *              +/-.005 for 32767 <= i_input < 3276700 (c_n = 15)
 *    Loading: 396 cycles for 8192 <= i_input < 34816 -03 optimization
 *---------------------------------------------------------------------------*/
uint32_t utl_sqrt32_32( uint32_t i_input, int8_t c_n)
{
   uint32_t i_sqrt;
   int32_t i_index;
   int8_t c_n1 = 0;
   uint16_t s_input;
   int8_t c_qChange = 15 - c_n;

   if( i_input == 0)
      return 0;

   /*------------------------------------------------------------------------*
    * Normalize the input so that the input value falls within .25 and 1- the
    * range of the sqrt lookup table. The number of shifts left or right
    * is calculated iteratively (c_n1) until the scaled input (i_input*(2^c_n1)
    * or i_input / (2^c_n1)) lands within the desired range. After
    * normalization, a table index is calculated and the subsequent sqrt is
    * obtained from the table. The value is then adjusted by 1/(sqrt(2)^c_n1)
    * or sqrt(2)^c_n1 depending on the direction of normalization. In order
    * to avoid a large number of multiplies, the multiplication factors are
    * stored in two separate lookup tables - one for up normalization and
    * the other for down normalization.
    *------------------------------------------------------------------------*/
   while( i_input <= UTL_MATH_SQRT_LKUP_TBL_MIN_INPUT)
   {
      i_input = i_input << 1;
      c_n1++;
   }/*End while( i_input <= UTL_MATH_SQRT_LKUP_TBL_MIN_INPUT)*/

   while( i_input >= UTL_MATH_SQRT_LKUP_TBL_MAX_INPUT)
   {
      i_input = i_input >> 1;
      c_n1--;
   }/*End while( i_input >= UTL_MATH_SQRT_LKUP_TBL_MAX_INPUT)*/

   s_input = (uint16_t)i_input;

   /*------------------------------------------------------------------------*
    * The sqrt table ranges from Q1.15 input values of 8192 to 34816
    * represented by index 0, and UTL_MATH_SQRT_TBL_SZ - 1,
    * respectively. Therefore the index to the table is given by:
    *    index = (s_input - 8192) / 2048
    *------------------------------------------------------------------------*/
   i_index = (int32_t)((s_input - UTL_MATH_SQRT_LKUP_TBL_MIN_INPUT) >>
   UTL_MATH_SQRT_SHFT);

   if( i_index < (UTL_MATH_SQRT_TBL_SZ - 1))
   {
      i_sqrt = utl_linInterp32_32( (int32_t)s_input,
                                   UTL_MATH_SQRT_LKUP_TBL_MIN_INPUT +
                                   (int32_t)(((i_index)*(1 <<
                                   UTL_MATH_SQRT_SHFT))),
                                   UTL_MATH_SQRT_SHFT,
                                   (int32_t)gas_sqrtTbl[i_index],
                                   (int32_t)gas_sqrtTbl[i_index+1]);

   }/*End if( i_index < (UTL_MATH_SQRT_TBL_SZ - 1))*/
   else
      i_sqrt = gas_sqrtTbl[(UTL_MATH_SQRT_TBL_SZ - 1)];

   /*------------------------------------------------------------------------*
    * Scale the result up or down depending on the direction of normalization
    * and the change in Q factor.
    *------------------------------------------------------------------------*/
   c_n1 = c_n1 + c_qChange;
   if( c_n1 > 0)
   {
      if( c_n1 > UTL_MATH_SQRT_TBL_SZ)
         c_n1 = UTL_MATH_SQRT_TBL_SZ;

      i_sqrt = ((int32_t)gas_sqrtUpNorm[c_n1 - 1]*(int32_t)i_sqrt) >> 15;
   }
   else if( c_n1 < 0)
   {
      c_n1 = -c_n1;

      if( c_n1 > UTL_MATH_SQRT_TBL_SZ)
         c_n1 = UTL_MATH_SQRT_TBL_SZ;

      i_sqrt = ((int32_t)i_sqrt*(int32_t)gas_sqrtDownNorm[c_n1 - 1]) >> 8;
   }

   return i_sqrt;

}/*End utl_sqrt32_32*/

/*---------------------------------------------------------------------------*
 * Function:
 *    utl_atan2_16
 *
 * Description:
 *    Performs a 16-bit Q15 arc tangent operation. See
 *    http://www.coranac.com/documents/arctangent/ for further information.
 *
 * Parameters:
 *
 *    s_y - Q1.15 16-bit number where m is the number of integer bits and n
 *          is the number of fractional bits
 *
 *    s_x - Q1.15 16-bit number where m is the number of integer bits and n
 *          is the number of fractional bits
 *
 * Return:
 *    s_atan2 - Q1.15 16-bit number where m is the number of integer bits and n
 *              is the number of fractional bits where +/-pi = 32767/-32768
 *              respectively.
 *---------------------------------------------------------------------------*/
int16_t utl_atan2_16( int16_t s_y, int16_t s_x)
{

   uint16_t s_div;
   int32_t i_index;
   int16_t s_angle;
   uint16_t s_absx;
   uint16_t s_absy;

   s_absx = (uint16_t)utl_abs32_32( (int32_t)s_x);
   s_absy = (uint16_t)utl_abs32_32( (int32_t)s_y);

   /*-------------------------------------------------------------------------*
    * Manipulate x and y in order to take obtain a value from a lookup table
    * encompassing the first 45 degrees of quadrant 1.
    *-------------------------------------------------------------------------*/
   if( s_absx >= s_absy)
   {
      s_div = (uint16_t)(((uint32_t)s_absy*(uint32_t)32767) / (s_absx + 1));
      i_index = s_div >> UTL_MATH_ATAN2_SHFT;

      s_angle = utl_linInterp32_32( (int32_t)s_div,
                                    (int32_t)(((i_index)*(1 <<
                                    UTL_MATH_ATAN2_SHFT))),
                                    UTL_MATH_ATAN2_SHFT,
                                    (int32_t)gas_atan2Tbl[i_index],
                                    (int32_t)gas_atan2Tbl[i_index+1]);

   }
   else
   {
      s_div = (uint16_t)(((uint32_t)s_absx*(uint32_t)32767) / (s_absy + 1));
      i_index = s_div >> UTL_MATH_ATAN2_SHFT;

      s_angle = utl_linInterp32_32( (int32_t)s_div,
                                    (int32_t)(((i_index)*(1 <<
                                    UTL_MATH_ATAN2_SHFT))),
                                    UTL_MATH_ATAN2_SHFT,
                                    (int32_t)gas_atan2Tbl[i_index],
                                    (int32_t)gas_atan2Tbl[i_index+1]);

     /*----------------------------------------------------------------------*
      * X and y were swapped in order to be able to use the 45 degree lookup
      * table. The 'true' angle lies above 45 degrees, therefore a rotation is
      * needed.
      *----------------------------------------------------------------------*/
      s_angle = (UTL_MATH_FXDPNT_PI >> 1) - s_angle;

   }

   /*------------------------------------------------------------------------*
    * Rotate the angle so that it falls back into its original quadrant
    *------------------------------------------------------------------------*/
   if( (s_x < 0) && ( s_y >= 0)) /*The angle should be in quadrant 2*/
   {
      s_angle = UTL_MATH_FXDPNT_PI - s_angle;
   }
   else if( (s_x <= 0) && (s_y <=0))  /*The angle should be in quadrant 3*/
   {
      s_angle = s_angle + UTL_MATH_FXDPNT_NEGATIVE_PI;

   }
   else if( (s_x >= 0) && (s_y <= 0)) /*The angle should be in quadrant 4*/
   {
      s_angle = -s_angle;
   }

   return s_angle;

}/*End utl_atan2_16*/

/*-------------------------------------------------------------------------*
 *
 * Function:
 *	   utl_mult16x16_16
 *
 * Description:
 *	   Signed multiply of two Qm.n 16-bit numbers returning a Qm.n 16-bit
 *    result. Where m is the number of integer bits and n represents the
 *    number of fractional bits.
 *
 * Parameters:
 *    s_x1 - Qm.n 16-bit number where m is the number of integer bits and n
 *           is the number of fractional bits
 *    s_x2 - Qm.n 16-bit number where m is the number of integer bits and n
 *           is the number of fractional bits
 *    c_n  - The number of fractional bits of the return value.
 *
 * Return:
 *    s_out - Qm.n 16-bit number where m is the number of integer bits and n
 *            is the number of fractional bits.
 *
 * References:
 *    http://en.wikipedia.org/wiki/Q_(number_format)
 *------------------------------------------------------------------------*/
int16_t utl_mult16x16_16( int16_t s_x1,
                          int16_t s_x2,
                          int8_t c_n)
{
   t_typePunn t_tWord;
   int8_t c_sign = 1;

   /*---------------------------------------------------------------------*
    * Check for negative saturation...
    *---------------------------------------------------------------------*/
#if UTL_MATH_SAT_CHECK
   if( (s_x1 == -(int16_t)1<<c_n) && (s_x2 == -(int16_t)1<<c_n))
      return (((int16_t)1 << c_n) - 1); /*Return max positive*/
#endif

   t_tWord.i_word32 = (int32_t)s_x1*(int32_t)s_x2;

   if( t_tWord.i_word32 < 0)
   {
      t_tWord.i_word32 = -t_tWord.i_word32;
       c_sign = -1;
   }

   /*---------------------------------------------------------------------*
    * Perform rounding...
    *---------------------------------------------------------------------*/
   t_tWord.i_word32 += ((int32_t)1 << (c_n - 1));

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
   if( (c_n > 8) && (c_n <= 16))/*Pivot-point = second 16-bit word...*/
   {
      /*------------------------------------------------------------------*
       * The least number of shifts occur if the first 16-bit word is
       * shifted up into the second 16-bit word...
       *------------------------------------------------------------------*/
      t_tWord.i_word32 = t_tWord.i_word32 << (16 - c_n);

      /*------------------------------------------------------------------*
       * Use ptr's in order to return a 16-bit word starting from the
       * location of the second 16-bit word.
       *------------------------------------------------------------------*/
      if( c_sign == -1)
         return -t_tWord.as_word16[1];
       else
          return t_tWord.as_word16[1];
   }

   t_tWord.i_word32 = t_tWord.i_word32 >> c_n;

   /*---------------------------------------------------------------------*
    * The least number of shifts occur if the 32-bit word is shifted down
    * and a 16-bit word is returned starting from the location of the first
    * 16-bit word.
    *---------------------------------------------------------------------*/
   if( c_sign == -1)
      return -(int16_t)t_tWord.i_word32; /*Pivot-point = first 16-bit word...*/
   else
      return (int16_t)t_tWord.i_word32;

}/*End utl_mult16x16_16*/

/*-------------------------------------------------------------------------*
 *
 * Function:
 *	   utl_mult16x16_32
 *
 * Description:
 *	   Signed multiply of two Qm.n 16-bit numbers returning a Qm.n 32-bit
 *    result. Where m is the number of integer bits and n represents the
 *    number of fractional bits.
 *
 * Parameters:
 *    s_x1 - Qm.n 16-bit number where m is the number of integer bits and n
 *           is the number of fractional bits
 *    s_x2 - Qm.n 16-bit number where m is the number of integer bits and n
 *           is the number of fractional bits
 *    c_n  - The number of fractional bits of the return value.
 *
 * Return:
 *    i_out - Qm.n 32-bit number where m is the number of integer bits and n
 *            is the number of fractional bits.
 *
 * References:
 *    http://en.wikipedia.org/wiki/Q_(number_format)
 *------------------------------------------------------------------------*/
int32_t utl_mult16x16_32( int16_t s_x1,
                          int16_t s_x2,
                          int8_t c_n)
{
   int32_t i_y = 0;

   i_y = (int32_t)s_x1*(int32_t)s_x2;

   if( i_y < 0)
     i_y = -((-i_y + ((int32_t)1 << (c_n - 1))) >> c_n);
   else
      i_y = (i_y + ((int32_t)1 << (c_n - 1))) >> c_n;

   return i_y;

}/*End utl_mult16x16_32*/

/*-------------------------------------------------------------------------*
 *
 * Function:
 *	   utl_mult32x32_32
 *
 * Description:
 *	   Signed multiply of two Qm.n 32-bit numbers returning a Qm.n 32-bit
 *    result. Where m is the number of integer bits and n represents the
 *    number of fractional bits.
 *
 * Parameters:
 *    i_x1 - Qm.n 32-bit number where m is the number of integer bits and n
 *           is the number of fractional bits
 *    i_x2 - Qm.n 32-bit number where m is the number of integer bits and n
 *           is the number of fractional bits
 *    c_n  - The number of fractional bits of the return value.
 *
 * Return:
 *    i_out - Qm.n 32-bit number where m is the number of integer bits and n
 *            is the number of fractional bits.
 *
 * References:
 *    http://en.wikipedia.org/wiki/Q_(number_format)
 *------------------------------------------------------------------------*/
int32_t utl_mult32x32_32( int32_t i_x1,
                          int32_t i_x2,
                          int8_t c_n)
{
   int64_t l_y;

   l_y = (int64_t)i_x1*(int64_t)i_x2;
   if( l_y < 0)
     l_y = -((-l_y + ((int64_t)1 << (c_n - 1))) >> c_n);
   else
      l_y = (l_y + ((int64_t)1 << (c_n - 1))) >> c_n;

   return (int32_t)l_y;

}/*End utl_mult32x32_32*/

/*-------------------------------------------------------------------------*
 *
 * Function:
 *	   utl_div16x16_16
 *
 * Description:
 *	   Signed divide of two Qm.n 16-bit numbers returning a Qm.n 16-bit
 *    result. Where m is the number of integer bits and n represents the
 *    number of fractional bits.
 *
 * Parameters:
 *    s_x1 - Qm.n 16-bit number where m is the number of integer bits and n
 *           is the number of fractional bits
 *    s_x2 - Qm.n 16-bit number where m is the number of integer bits and n
 *           is the number of fractional bits
 *    c_n  - The number of fractional bits of the return value.
 *
 * Return:
 *    s_out - Qm.n 16-bit number where m is the number of integer bits and n
 *            is the number of fractional bits.
 *
 * References:
 *    http://en.wikipedia.org/wiki/Q_(number_format)
 *------------------------------------------------------------------------*/
int16_t utl_div16x16_16( int16_t s_x1,
                         int16_t s_x2,
                         int8_t c_n)
{

#if UTL_MATH_SAT_CHECK
   if( s_x1 == s_x2)
      return (((int16_t)1 << c_n) - 1); /*Return max positive*/
   else if( (s_x1 == -32768) && (s_x2 == 32767))
      return ((int16_t)1 << c_n);
#endif

   return (int16_t)((((int32_t)s_x1*((int32_t)1<<c_n))) / s_x2);

}/*End utl_div16x16_16*/

/*------------------------------------------------------------------------*
 *
 * Function:
 *	   utl_mac16x16_32
 *
 * Description:
 *	   Signed multiply and accumulate of two Qm.n 16-bit vectors returning
 *    a Qm.n 32-bit result.
 *
 * Parameters:
 *    s_x1 - Pointer to a Qm.n 16-bit numbers where m is the number of
 *           integer bits and n is the number of fractional bits
 *    s_x2 - Pointer to a Qm.n 16-bit number where m is the number of
 *           integer bits and n is the number of fractional bits
 *    i_y - Qm.n accumulator initial value.
 *    c_o - Overflow prevention factor
 *
 * Return:
 *    i_y - Qm.n 32-bit number where m is the number of integer bits and n
 *          is the number of fractional bits
 *------------------------------------------------------------------------*/
int32_t utl_mac16x16_32( int16_t s_x1,
                         int16_t s_x2,
                         int32_t i_y,
                         int8_t c_o)
{
   int32_t i_tmp;

   i_tmp = (int32_t)s_x1*(int32_t)s_x2;
   if( i_tmp < 0)
      i_tmp = -((-i_tmp) >> c_o);
   else
       i_tmp = i_tmp >> c_o;

   i_y += i_tmp;

   return i_y;

}/*End utl_mac16x16_32*/

/*------------------------------------------------------------------------*
 *
 * Function:
 *	   utl_vMult16x16_32
 *
 * Description:
 *	   Signed multiply and accumulate of two Qm.n 16-bit vectors returning
 *    a Qm.n 32-bit result.
 *
 * Parameters:
 *    ps_x1 - Pointer to a vector of Qm.n 16-bit numbers where m is the
 *            number of integer bits and n is the number of fractional bits
 *    ps_x2 - Pointer to a vector of Qm.n 16-bit number where m is the
 *            number of integer bits and n is the number of fractional bits
 *    i_y - Qm.n accumulator initial value.
 *    s_length - The length of the vectors ps_x1 and ps_x2.
 *    c_o - Overflow prevention factor
 *    c_n - Number of fractional bits in the output.
 *
 * Return:
 *    i_y - Qm.n 32-bit number where m is the number of integer bits and n
 *          is the number of fractional bits
 *------------------------------------------------------------------------*/
int32_t utl_vMult16x16_32( int16_t *ps_x1,
                           int16_t *ps_x2,
                           int32_t i_y,
                           int16_t s_length,
                           int8_t c_o,
                           int8_t c_n)
{
   int16_t s_index;
   int32_t i_tmp;

   for( s_index = 0; s_index < s_length; s_index++)
   {
      i_tmp = (int32_t)ps_x1[s_index]*(int32_t)ps_x2[s_index];
      if( i_tmp < 0)
         i_tmp = -((-i_tmp) >> c_o);
      else
          i_tmp = i_tmp >> c_o;

      i_y += i_tmp;

   }/*End for( s_index = 0; s_index < s_length; s_index++)*/

   if( i_y < 0)
      i_y = -((-i_y + ((int32_t)1 << (c_n - 1))) >> c_n);
   else
       i_y = (i_y + ((int32_t)1 << (c_n - 1))) >> c_n;

   return i_y;

}/*End utl_mac16x16_32*/

/*------------------------------------------------------------------------*
 *
 * Function:
 *	   utl_vMult16x16_16
 *
 * Description:
 *	   Signed multiply and accumulate of two Qm.n 16-bit vectors returning
 *    a Qm.n 16-bit result.
 *
 * Parameters:
 *    ps_x1 - Pointer to a vector of Qm.n 16-bit numbers where m is the
 *            number of integer bits and n is the number of fractional bits
 *    ps_x2 - Pointer to a vector of Qm.n 16-bit number where m is the
 *            number of integer bits and n is the number of fractional bits
 *    i_y - Qm.n 32-bit accumulator initial value.
 *    s_length - The length of the vectors ps_x1 and ps_x2.
 *    c_o - Overflow prevention factor
 *    c_n - Number of fractional bits in the output.
 *
 * Return:
 *    s_y - Qm.n 16-bit number where m is the number of integer bits and n
 *          is the number of fractional bits
 *------------------------------------------------------------------------*/
int16_t utl_vMult16x16_16( int16_t *ps_x1,
                           int16_t *ps_x2,
                           int32_t i_y,
                           int16_t s_length,
                           int8_t c_o,
                           int8_t c_n)
{
   int16_t s_index;
   t_typePunn t_tWord;
   int32_t i_tmp;
   int8_t c_sign = 1;

   t_tWord.i_word32 = i_y;

   for( s_index = 0; s_index < s_length; s_index++)
   {
      /*------------------------------------------------------------------*
       * Check for negative saturation...
       *------------------------------------------------------------------*/
#if UTL_MATH_SAT_CHECK
      if( (ps_x1[s_index] == -32768) && (ps_x2[s_index] == -32768))
         t_tWord.i_word32 += ((((int32_t)1<<30) - 1) >> c_o);
      else
      {
          i_tmp = (int32_t)ps_x1[s_index]*(int32_t)ps_x2[s_index];
         if( i_tmp < 0)
            i_tmp = -((-i_tmp) >> c_o);
         else
             i_tmp = i_tmp >> c_o;

         t_tWord.i_word32 += i_tmp;
      }

#else
       i_tmp = (int32_t)ps_x1[s_index]*(int32_t)ps_x2[s_index];
      if( i_tmp < 0)
         i_tmp = -((-i_tmp) >> c_o);
      else
          i_tmp = i_tmp >> c_o;

      t_tWord.i_word32 += i_tmp;
#endif
   }/*End for( s_index = 0; s_index < s_length; s_index++)*/

   if( t_tWord.i_word32 < 0)
   {
      c_sign = -1;
       t_tWord.i_word32 = -t_tWord.i_word32;
   }

   t_tWord.i_word32 += ((int32_t)1 << (c_n - 1));

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
   if( (c_n > 8) && (c_n <= 16))/*Pivot-point = second 16-bit word...*/
   {
      /*------------------------------------------------------------------*
       * The least number of shifts occur if the first 16-bit word is
       * shifted up into the second 16-bit word...
       *------------------------------------------------------------------*/
      t_tWord.i_word32 = t_tWord.i_word32 << (16 - c_n);

      /*------------------------------------------------------------------*
       * Use ptr's in order to return a 16-bit word starting from the
       * location of the second 16-bit word.
       *------------------------------------------------------------------*/
      if( c_sign == -1)
         return -t_tWord.as_word16[1];
       else
          return t_tWord.as_word16[1];
   }

   t_tWord.i_word32 = t_tWord.i_word32 >> c_n;

   /*---------------------------------------------------------------------*
    * The least number of shifts occur if the 32-bit word is shifted down
    * and a 16-bit word is returned starting from the location of the first
    * 16-bit word.
    *---------------------------------------------------------------------*/
   if( c_sign == -1)
      return -(int16_t)t_tWord.i_word32; /*Pivot-point = first 16-bit word...*/
   else
      return (int16_t)t_tWord.i_word32;

}/*End utl_vMult16x16_16*/

bool utl_matMult16x16_16( int16_t *ps_mat1,
                          int8_t c_rows1,
                          int8_t c_col1,
                          int16_t *ps_mat2,
                          int8_t c_rows2,
                          int8_t c_col2,
                          int16_t *ps_res,
                          int8_t c_co,
                          int8_t c_cn)
{
   int8_t c_r1;
   int8_t c_c1;
   int8_t c_c2;
   int32_t i_sum;

   if( c_col1 == c_rows2)
   {
      for( c_r1 = 0; c_r1 < c_rows1; c_r1++)
      {
         for( c_c2 = 0; c_c2 < c_col2; c_c2++)
         {
            i_sum = 0;
            for( c_c1 = 0; c_c1 < c_col1; c_c1++)
            {
               i_sum = utl_mac16x16_32( (*(ps_mat1 + ((int16_t)c_r1*(int16_t)c_col1) + c_c1)),
                                        (*(ps_mat2 + ((int16_t)c_c1*(int16_t)c_col2) + c_c2)),
                                        i_sum,
                                        c_co);

            }/*End for( c_c1 = 0; c_c1 < c_col1; c_c1++)*/

            (*(ps_res + ((int16_t)c_r1*(int16_t)c_col2) + c_c2)) = utl_rShft32_16( i_sum, (c_cn - c_co));

         }/*End for( c_c2 = 0; c_c2 < c_col2; c_c2++)*/

      }/*End for( c_r1 = 0; c_r1 < c_rows1; c_r1++)*/
   }
   else
      return false;

   return true;

}/*End utl_matMult16x16_16*/

void utl_matTrans16( int16_t *ps_mat,
                     int16_t *ps_trans,
                     int8_t c_rows,
                     int8_t c_col)
{
   int8_t c_r1;
   int8_t c_c1;

   for( c_r1 = 0; c_r1 < c_rows; c_r1++)
   {
      for( c_c1 = 0; c_c1 < c_col; c_c1++)
      {
         (*(ps_trans + ((int16_t)c_c1*(int16_t)c_rows) + c_r1)) =
         (*(ps_mat + ((int16_t)c_r1*(int16_t)c_col) + c_c1));
      }
   }

}/*End utl_matTrans16*/

/*-------------------------------------------------------------------------*
 *
 * Function:
 *	   utl_macF
 *
 * Description:
 *	   Floating-point multiply and accumulate.
 *
 * Parameters:
 *    f_x1 - 32-bit floating-point multiplicand
 *    f_x2 - 32-bit floating-point multiplicand
 *    f_y - 32-bit floating-point accumulated result
 *
 * Return:
 *    f_y - 32-bit floating-point accumulation of f_x1*f_x2
 *------------------------------------------------------------------------*/
float utl_macF( float f_x1,
                float f_x2,
                float f_y)
{

   return (f_y + (f_x1*f_x2));

}/*End utl_macF*/

/*-------------------------------------------------------------------------*
 *
 * Function:
 *	   utl_divF
 *
 * Description:
 *	   Floating-point division
 *
 * Parameters:
 *    f_x1 - 32-bit floating-point numerator
 *    f_x2 - 32-bit floating-point divisor
 *    f_y - 32-bit floating-point result
 *
 * Return:
 *    f_y - 32-bit floating-point division of f_x1/f_x2
 *------------------------------------------------------------------------*/
float utl_divF( float f_x1,
                float f_x2)
{

   return (f_x1 / f_x2);

}/*End utl_divF*/

/*-------------------------------------------------------------------------*
 *
 * Function:
 *	   utl_multF
 *
 * Description:
 *	   Floating-point multiply.
 *
 * Parameters:
 *    f_x1 - 32-bit floating-point multiplicand
 *    f_x2 - 32-bit floating-point multiplicand
 *    f_y - 32-bit floating-point result
 *
 * Return:
 *    f_y - 32-bit floating-point multiplication of f_x1 and f_x2
 *------------------------------------------------------------------------*/
float utl_multF( float f_x1,
                 float f_x2)
{

   return f_x1*f_x2;

}/*End utl_multF*/

/*---------------------------------------------------------------------------*
 * Function utl_matMultF
 *
 * Description: Performs a matrix multiplication operation in floating point
 *
 * Input Parameters:
 *
 *              pf_mat1 = A i_rows1 by i_col1 matrix
 *
 *              i_rows1 = The number of rows in pi_mat1
 *
 *              i_col1  = The number of columns in pi_mat1
 *
 *              pf_mat2 = A i_rows2 by i_col2 matrix
 *
 *              i_rows2 = The number of rows in pi_mat2
 *
 *              i_col2  = The number of columns in pi_mat2
 *
 * Output Parameters:
 *
 *              pf_res = The resulting matrix multiplication
 *
 *              b_mult = true, if the matrix can be multiplied.
 *---------------------------------------------------------------------------*/
bool utl_matMultF( float *pf_mat1,
                   int32_t i_rows1,
                   int32_t i_col1,
                   float *pf_mat2,
                   int32_t i_rows2,
                   int32_t i_col2,
                   float *pf_res)
{
   int32_t i_r1 = 0;
   int32_t i_c1 = 0;
   int32_t i_c2 = 0;
   float f_sum = 0.0;

   if( i_col1 == i_rows2)
   {
      for( i_r1 = 0; i_r1 < i_rows1; i_r1++)
      {
         for( i_c2 = 0; i_c2 < i_col2; i_c2++)
         {
            f_sum = 0;
            for( i_c1 = 0; i_c1 < i_col1; i_c1++)
            {
               f_sum = utl_macF( (*(pf_mat1 + i_r1*i_col1 + i_c1)),
                                 (*(pf_mat2 + i_c1*i_col2 + i_c2)),
                                 f_sum);

            }/*End for( i_c1 = 0; i_c1 < i_col1; i_c1++)*/

            (*(pf_res + i_r1*i_col2 + i_c2)) = f_sum;

         }/*End for( i_c2 = 0; i_c2 < i_col2; i_c2++)*/

      }/*End for( i_r1 = 0; i_r1 < i_rows1; i_r1++)*/
   }
   else
      return false;

   return true;

}/*End utl_matMultF*/

/*---------------------------------------------------------------------------*
 * Function utl_matInvF
 *
 * Description: Performs a floating point matrix inversion operation using the
 *              Gauss-Jordan method.
 *
 * Input Parameters:
 *
 *              pf_mat = A i_numElements by i_numElements square matrix
 *
 *              i_numElements = The number of rows/columns
 *
 * Output Parameters:
 *
 *              pf_inv = The resulting inverted matrix
 *
 *              b_invertible = true, if the matrix is invertible.
 *---------------------------------------------------------------------------*/
bool utl_matInvF( float *pf_mat,
                  float *pf_inv,
                  int32_t i_numElements)
{
   int32_t i_r = 0;
   int32_t i_c = 0;
   float af_augMatr[UTL_MATH_MAX_MAT_ROW_SZ]\
   [UTL_MATH_MAX_MAT_COL_SZ];
   float af_tempRow[UTL_MATH_MAX_MAT_COL_SZ];
   int32_t i_currR = 0;
   int32_t i_nextR = 0;
   int32_t i_currC = 0;
   float f_temp = 0;

   if( (i_numElements*2) > UTL_MATH_MAX_MAT_COL_SZ)
      return false;
   else
   {

      /*---------------------------------------------------------------------*
       * Copy the elements of pi_mat into the augmented matrix...
       *---------------------------------------------------------------------*/
      for( i_r = 0; i_r < i_numElements; i_r++)
      {
         for( i_c = 0; i_c < i_numElements; i_c++)
         {
            af_augMatr[i_r][i_c] =
            *(pf_mat + i_r*i_numElements + i_c);
         }
      }

      /*---------------------------------------------------------------------*
       * Form identity matrix on left hand side of ai_augMatr...
       *---------------------------------------------------------------------*/
      for( i_r = 0; i_r < i_numElements; i_r++)
      {
         for( i_c = i_numElements; i_c < 2*i_numElements; i_c++)
         {
            if( i_r == (i_c - i_numElements))
               af_augMatr[i_r][i_c] = 1.0;
            else
               af_augMatr[i_r][i_c] = 0.0;
         }
      }

      i_currC = 0;

      for( i_currR = 0; i_currR < i_numElements; i_currR++)
      {
         if( i_currR == i_currC)
         {
            i_nextR = i_currR + 1;

            /*---------------------------------------------------------------*
             * Check to see if we need to swap rows if the current element
             * is zero - with margin for error...
             *---------------------------------------------------------------*/
            if( fabs(af_augMatr[i_currR][i_currC]) == 0)
            {
               if( (i_nextR != i_numElements) && (fabs(
               af_augMatr[i_nextR][i_currC]) > 0.0000001))
               {
                  for( i_c = 0; i_c < i_numElements*2; i_c++)
                  {
                     af_tempRow[i_c] = af_augMatr[i_currR][i_c];
                     af_augMatr[i_currR][i_c] = af_augMatr[i_nextR][i_c];
                     af_augMatr[i_nextR][i_c] = af_tempRow[i_c];
                  }
               }
               else
                  return false; /*Matrix is not invertible*/
            }

            /*---------------------------------------------------------------*
             * Make sure the diagonal element is a positive 1 by dividing the
             * entire row by its current value...
             *---------------------------------------------------------------*/
             f_temp = utl_divF( 1.0, af_augMatr[i_currR][i_currC]);
             for( i_c = i_currC; i_c < i_numElements*2; i_c++)
             {
                af_augMatr[i_currR][i_c] = utl_multF( af_augMatr[i_currR][i_c],
                f_temp);
             }
          }/*End if( i_currR == i_currC)*/

          /*-----------------------------------------------------------------*
           * Make sure all the rows above and below the diagonal element are
           * zero...
           *-----------------------------------------------------------------*/
          for( i_r = 0; i_r < i_currR; i_r++)
          {
             /*--------------------------------------------------------------*
              * Negate current value of the element we are going to zero...
              *--------------------------------------------------------------*/
             f_temp = -af_augMatr[i_r][i_currC];
             for( i_c = i_currC; i_c < i_numElements*2; i_c++)
             {

                /*-----------------------------------------------------------*
                 * Scale all elements in the row containing the diagnal by
                 * this value and add to the row we are trying to zero...
                 *-----------------------------------------------------------*/
                af_augMatr[i_r][i_c] = utl_macF( f_temp,
                af_augMatr[i_currR][i_c], af_augMatr[i_r][i_c]);

             }/*End for( i_c = 0; i_c < i_numElements*2; i_c++)*/
          }

          for( i_r = i_currR + 1; i_r < i_numElements; i_r++)
          {
             /*--------------------------------------------------------------*
              * Negate current value of the element we are going to zero...
              *--------------------------------------------------------------*/
             f_temp = -af_augMatr[i_r][i_currC];
             for( i_c = i_currC; i_c < i_numElements*2; i_c++)
             {
                /*-----------------------------------------------------------*
                 * Scale all elements in the row containing the diagnal by
                 * this value and add to the row we are trying to zero...
                 *-----------------------------------------------------------*/
                af_augMatr[i_r][i_c] = utl_macF( f_temp,
                af_augMatr[i_currR][i_c], af_augMatr[i_r][i_c]);

             }/*End for( i_c = 0; i_c < i_numElements*2; i_c++)*/
          }

          /*-----------------------------------------------------------------*
           * Increment to the next diagonal element...
           *-----------------------------------------------------------------*/
          i_currC++;

       }/*End while( i_currR != i_numElements)*/

       /*--------------------------------------------------------------------*
        * Copy the elements of pi_mat into the augmented matrix...
        *--------------------------------------------------------------------*/
       for( i_r = 0; i_r < i_numElements; i_r++)
       {
          for( i_c = 0; i_c < i_numElements; i_c++)
         {
             (*(pf_inv + i_r*i_numElements + i_c)) = af_augMatr[i_r][i_c +
             i_numElements];
          }
       }
   }

   return true; /*Matrix is invertible*/

}/*End utl_matInvF*/

/*---------------------------------------------------------------------------*
 * Function utl_matTransF
 *
 * Description: Performs the transpose of the input matrix
 *
 * Input Parameters:
 *
 *              pf_mat = A i_rows by i_col matrix
 *
 *              i_rows = The number of rows in pi_mat
 *
 *              i_col  = The number of columns in pi_mat
 *
 * Output Parameters:
 *
 *              pf_trans = The resulting transposed matrix
 *---------------------------------------------------------------------------*/
void utl_matTransF( float *pf_mat,
                    float *pf_trans,
                    int32_t i_rows,
                    int32_t i_col)
{
   int32_t i_r1 = 0;
   int32_t i_c1 = 0;

   for( i_r1 = 0; i_r1 < i_rows; i_r1++)
   {
      for( i_c1 = 0; i_c1 < i_col; i_c1++)
      {
         (*(pf_trans + i_c1*i_rows + i_r1)) = (*(pf_mat + i_r1*i_col + i_c1));
      }
   }

}/*End utl_matTransF*/

/*---------------------------------------------------------------------------*
 * Function utl_matEyeF
 *
 * Description: Fills in the diagonal elements of a square matrix with 1's and
 *              the off diagonal elements with 0's.
 *
 * Input Parameters:
 *
 *              pf_mat = i_sizexi_size matrix
 *
 *              i_size = number of rows and columns
 *
 *              f_value = the value to put on the diagonal elements
 *
 * Output Parameters:
 *
 *              pf_mat = the newly formed eye matrix
 *---------------------------------------------------------------------------*/
void utl_matEyeF( float *pf_mat,
                  int32_t i_size,
                  float f_value)
{
   int32_t i_r = 0;
   int32_t i_c = 0;

   for( i_r = 0; i_r < i_size; i_r++)
   {
      for( i_c = 0; i_c < i_size; i_c++)
      {
         if( i_c == i_r)
            (*(pf_mat + i_r*i_size + i_c)) = f_value;
         else
            (*(pf_mat + i_r*i_size + i_c)) = 0;
      }
   }

}/*End utl_matEyeF*/

/*---------------------------------------------------------------------------*
 * Function utl_matEigsF
 *
 * Description: Finds the absolute value of the eigenvalues and their
 *              corresponding eigenvectors of a square matrix using the
 *              power method with deflation. See http://www.miislita.com/
 *              information-retrieval-tutorial/matrix-tutorial-3-eigenvalues-
 *              eigenvectors.html and http://www.math.umn.edu/~olver/num_/lnqr.
 *              pdf for more details
 *
 * Input Parameters:
 *
 *              pf_mat = (i_size x i_size) matrix
 *
 *              i_size = number of rows and columns - maximum of 6 allowed
 *
 * Output Parameters:
 *
 *              pf_eigValues = column vector of positive eignvalues in
 *                             assending order.
 *
 *              pf_eigVectors = matrix of normalized eigenvectors of size
 *                             (i_size x i_size)
 *
 *              b_pass = true if the algorithm was able to run
 *---------------------------------------------------------------------------*/
bool utl_matEigsF( float *pf_mat,
                   int32_t i_size,
                   float *pf_eigValues,
                   float *pf_eigVectors)
{
   int32_t i_iterCount = 0;
   float f_error = 100.0;
   float f_lambdaNew = 0.0;
   float f_lambdaOld = 1.0;
   float af_x[6];
   float af_temp1[6*6];
   int32_t i_r = 0;
   int32_t i_c = 0;

   if( i_size > 6)
     return false;

   for( i_c = 0; i_c < i_size; i_c++)
   {

      for( i_r = 0; i_r < i_size; i_r++)
         af_x[i_r] = 1;

      f_lambdaOld = 1.0;
      f_error = 100.0;
      i_iterCount = 0;

      while( f_error > POWER_METHOD_MAX_ERROR)
      {

         utl_matMultF( pf_mat,
                       i_size,
                       i_size,
                       af_x,
                       i_size,
                       1,
                       af_temp1);

         /*------------------------------------------------------------------*
          * Find the new eigenvalue...
          *------------------------------------------------------------------*/
         f_lambdaNew = af_temp1[0];// / af_x[0];

         /*------------------------------------------------------------------*
          * Normalize the eigenvector
          *------------------------------------------------------------------*/
         //utl_normF( af_temp1, i_size);

         for( i_r = 0; i_r < i_size; i_r++)
            af_x[i_r] = af_temp1[i_r] / f_lambdaNew;

         /*------------------------------------------------------------------*
          * Calculate percentage error
          *------------------------------------------------------------------*/
         f_error = fabs((f_lambdaNew - f_lambdaOld)/f_lambdaNew)*100.0;

         f_lambdaOld = f_lambdaNew;
         i_iterCount++;

      }/*End while( f_error > POWER_METHOD_MAX_ERROR)*/

      /*---------------------------------------------------------------------*
       * Store the next largest eigenvalue and eigenvector in ascending order
       *---------------------------------------------------------------------*/
      pf_eigValues[(i_size - 1) - i_c] = f_lambdaNew;

      /*---------------------------------------------------------------------*
       * Normalize the eigenvector
       *---------------------------------------------------------------------*/
      utl_normF( af_x, i_size);

      for( i_r = 0; i_r < i_size; i_r++)
         (*(pf_eigVectors + (i_r*i_size) + (i_size - 1 - i_c))) = af_x[i_r];

      /*---------------------------------------------------------------------*
       * Perform Deflation where A* = A - c*X*X'
       *---------------------------------------------------------------------*/
      utl_matMultF( af_x,
                    i_size,
                    1,
                    af_x,
                    1,
                    i_size,
                    af_temp1);

      utl_matScalerMultF( af_temp1,
                          i_size,
                          i_size,
                          f_lambdaNew);

      utl_matSubF( pf_mat,
                   af_temp1,
                   i_size,
                   i_size,
                   pf_mat);

   }/*End for( i_c = 0; i_c < i_size; i_c++)*/

   return true;

}/*End utl_matEigsF*/

/*---------------------------------------------------------------------------*
 * Function utl_normF
 *
 * Description: Normalizes all the elements of pf_vect by 'norm' where norm =
 *              sqrt( pf_vect[0]*pf_vect[0] + pf_vect[1]*pf_vect[1] +...+
 *              pf_vect[i_size-1]*pf_vect[i_size-1])
 *
 * Input Parameters:
 *
 *              pf_vect = vector of size 'i_size'
 *
 *              i_size = number of elements
 *
 * Output Parameters:
 *
 *              pf_vect = normalized vector.
 *
 *              f_norm = normalization amount
 *---------------------------------------------------------------------------*/
float utl_normF( float *pf_vect,
                  int32_t i_size)
{
   float f_norm = 0.0;
   int32_t i_index = 0;

   /*------------------------------------------------------------------------*
    * Find the magnitude of the vector...
    *------------------------------------------------------------------------*/
   for( i_index = 0; i_index < i_size; i_index++)
   {
      f_norm += pf_vect[i_index]*pf_vect[i_index];
   }

   /*------------------------------------------------------------------------*
    * Find the envelope of the vector...
    *------------------------------------------------------------------------*/
   f_norm = sqrt(f_norm);

   for( i_index = 0; i_index < i_size; i_index++)
      pf_vect[i_index] = pf_vect[i_index] / f_norm;

   return f_norm;

}/*End utl_normF*/

/*---------------------------------------------------------------------------*
 * Function utl_matScalerMultF
 *
 * Description: Performs scalar matrix multiplication
 *
 * Input Parameters:
 *
 *              pf_mat = Matrix of size i_rows by i_col
 *
 *              i_rows = The number of rows
 *
 *              i_col  = The number of columns
 *
 *              f_value = The value pf_mat is being scaled by
 *
 * Output Parameters:
 *
 *              pf_mat = The scaled matrix.
 *---------------------------------------------------------------------------*/
void utl_matScalerMultF( float *pf_mat,
                         int32_t i_rows,
                         int32_t i_col,
                         float f_value)
{
   int32_t i_r = 0;
   int32_t i_c = 0;

   for( i_r = 0; i_r < i_rows; i_r++)
   {
      for( i_c = 0; i_c < i_col; i_c++)
      {
         (*(pf_mat + i_r*i_col + i_c)) =
         (*(pf_mat + i_r*i_col + i_c))*f_value;

      }/*End for( i_c = 0; i_c < i_col; i_c++)*/

   }/*End for( i_r = 0; i_r < i_rows; i_r++)*/

}/*End utl_matScalerMultF*/

/*---------------------------------------------------------------------------*
 * Function utl_matSubF
 *
 * Description: Subtracts pf_mat2 from pf_mat1
 *
 * Input Parameters:
 *
 *              pf_mat1 = Matrix of size i_rows by i_col
 *
 *              pf_mat2 = Matrix of size i_rows by i_col
 *
 *              i_rows = The number of rows
 *
 *              i_col  = The number of columns
 *
 * Output Parameters:
 *
 *              pf_result = The resulting matrix.
 *---------------------------------------------------------------------------*/
void utl_matSubF( float *pf_mat1,
                  float *pf_mat2,
                  int32_t i_rows,
                  int32_t i_col,
                  float *pf_result)
{
   int32_t i_r = 0;
   int32_t i_c = 0;

   for( i_r = 0; i_r < i_rows; i_r++)
   {
      for( i_c = 0; i_c < i_col; i_c++)
      {
         (*(pf_result + i_r*i_col+ i_c)) = (*(pf_mat1 + i_r*i_col+ i_c)) -
         (*(pf_mat2 + i_r*i_col+ i_c));

      }/*End for( i_c = 0; i_c < i_col; i_c++)*/

   }/*End for( i_r = 0; i_r < i_rows; i_r++)*/

}/*End utl_matSubF*/

/*---------------------------------------------------------------------------*
 * Function utl_maxF
 *
 * Description: Returns the max element of the vector pf_vect
 *
 * Input Parameters:
 *
 *              pf_vect = Vector of size 'i_size'
 *
 *              i_size = number of elements in the vector
 *
 * Output Parameters:
 *
 *              f_max = The max element
 *---------------------------------------------------------------------------*/
float utl_maxF( float *pf_vect,
                 int32_t i_size)
{
   int32_t i_r = 0;
   float f_max = pf_vect[0];

   for( i_r = 1; i_r < i_size; i_r++)
   {
      if( pf_vect[i_r] > f_max)
         f_max = pf_vect[i_r];

   }/*End for( i_r = 0; i_r < i_rows; i_r++)*/

   return f_max;

}/*End utl_maxF*/

/*---------------------------------------------------------------------------*
 * Function utl_absMaxF
 *
 * Description: Returns the max absolute value element of the vector pf_vect
 *
 * Input Parameters:
 *
 *              pf_vect = Vector of size 'i_size'
 *
 *              i_size = number of elements in the vector
 *
 * Output Parameters:
 *
 *              f_max = The max element
 *---------------------------------------------------------------------------*/
float utl_absMaxF( float *pf_vect,
                    int32_t i_size)
{
   int32_t i_r = 0;
   float f_max = fabs(pf_vect[0]);

   for( i_r = 1; i_r < i_size; i_r++)
   {
      if( fabs(pf_vect[i_r]) > f_max)
         f_max = fabs(pf_vect[i_r]);

   }/*End for( i_r = 0; i_r < i_rows; i_r++)*/

   return f_max;

}/*End utl_absMaxF*/

/*---------------------------------------------------------------------------*
 * Function utl_minF
 *
 * Description: Returns the min element of the vector pf_vect
 *
 * Input Parameters:
 *
 *              pf_vect = Vector of size 'i_size'
 *
 *              i_size = number of elements in the vector
 *
 * Output Parameters:
 *
 *              f_min = The min element
 *---------------------------------------------------------------------------*/
float utl_minF( float *pf_vect,
                 int32_t i_size)
{
   int32_t i_r = 0;
   float f_min = pf_vect[0];

   for( i_r = 1; i_r < i_size; i_r++)
   {
      if( pf_vect[i_r] < f_min)
         f_min = pf_vect[i_r];

   }/*End for( i_r = 0; i_r < i_size; i_r++)*/

   return f_min;

}/*End utl_minF*/

