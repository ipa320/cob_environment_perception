/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file utils.h
 *
 * Utils
 *  \version 1.0
 *  \author Julio Sagardoy
 */

#define true 1
#define false 0

/** READS the specified bit b inside a c byte/char 
*/
#define BITREAD(c,b) (c & (1<<b))

/** SETS the specified bit b inside a c byte/char 
*/
#define BITSET(c,b) (c |= (1<<b))

/** CLEARS the specified bit b inside a c byte/char.
(Dspl. bit, then one complement everything then AND with char)
*/
#define BITCLR(c,b) (c &= ~(1<<b))

/** TOGGLES bit value. (XOR)
*/
#define BITTOG(c,b) (c ^= (1<<b))

