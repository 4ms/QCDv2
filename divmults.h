/********************
 *  DIV/MULT VALUES *
 ********************/

/* 
Note: This file is not enabled by default.

The Div/Mult pot has 21-detents, but the top and bottom positions are not usable. 
Therefore we have 19 possible Division or Multiplication values. 
These are stored as P_1, P_2, through P_19, with P1 being the lowest value (counter-clockwise) and P19 being the greatest value (clockwise). P10 is the center value (typically will be equal to 1, which is "=".

Negative values represent multiplication (so -6 means x6)
Positive values represent division (so 4 means /4)
*/


#define DIVMULT_FACTORY
//#define DIVMULT_PRIME
//#define DIVMULT_CARNATIC
//#define DIVMULT_MULTONLY
//#define DIVMULT_FRACTIONALS



#if defined DIVMULT_PRIME
//Prime numbers
#define P_1 23
#define P_2 19
#define P_3 17
#define P_4 13
#define P_5 11
#define P_6 7
#define P_7 5
#define P_8 3
#define P_9 2
#define P_10 1
#define P_11 -2
#define P_12 -3
#define P_13 -5
#define P_14 -7
#define P_15 -11
#define P_16 -13
#define P_17 -17
#define P_18 -19
#define P_19 -23

#elif defined DIVMULT_CARNATIC
//Carnatic
#define P_1 11
#define P_2 9
#define P_3 8
#define P_4 7
#define P_5 6
#define P_6 5
#define P_7 4
#define P_8 3
#define P_9 2
#define P_10 1
#define P_11 -2
#define P_12 -3
#define P_13 -4
#define P_14 -5
#define P_15 -6
#define P_16 -7
#define P_17 -8
#define P_18 -9
#define P_19 -11

#elif defined DIVMULT_MULTONLY
//Multiply Only
#define P_1 1
#define P_2 -2
#define P_3 -3
#define P_4 -4
#define P_5 -5
#define P_6 -6
#define P_7 -7
#define P_8 -8
#define P_9 -9
#define P_10 -10
#define P_11 -11
#define P_12 -12
#define P_13 -13
#define P_14 -14
#define P_15 -15
#define P_16 -16
#define P_17 -17
#define P_18 -18
#define P_19 -19

#elif defined DIVMULT_FRACTIONALS
//Use fractions, so each setting gets a numerator N_# and a denominator D_#
#define N_1 1
#define D_1 32
#define N_2 1
#define D_2 16
#define N_3 1
#define D_3 8
#define N_4 1
#define D_4 7
#define N_5 1
#define D_5 6
#define N_6 1
#define D_6 5
#define N_7 1
#define D_7 4
#define N_8 1
#define D_8 3
#define N_9 1
#define D_9 2
#define N_10 1
#define D_10 1
#define N_11 2
#define D_11 1
#define N_12 3
#define D_12 1
#define N_13 4
#define D_13 1
#define N_14 5
#define D_14 1
#define N_15 6
#define D_15 1
#define N_16 7
#define D_16 1
#define N_17 8
#define D_17 1
#define N_18 12
#define D_18 1
#define N_19 16
#define D_19 1


#else
//Factory Default:
#define P_1 32
#define P_2 16
#define P_3 8
#define P_4 7
#define P_5 6
#define P_6 5
#define P_7 4
#define P_8 3
#define P_9 2
#define P_10 1
#define P_11 -2
#define P_12 -3
#define P_13 -4
#define P_14 -5
#define P_15 -6
#define P_16 -7
#define P_17 -8
#define P_18 -12
#define P_19 -16

#endif

