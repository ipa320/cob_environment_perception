/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file servo.c
 *
 * Code for servo pwm setup.
 *  \version 1.0
 *  \author Julio Sagardoy
 */

/// AVR includes
#include <avr/io.h>
#include <util/delay.h>

/// Own includes
#include "servo.h"
#include "utils.h"

/// look-up table for unsigned to pwm register value conversion
static const uint16_t pwmt[] = { 
2205,
2199,
2192,
2186,
2179,
2173,
2166,
2160,
2154,
2147,
2141,
2134,
2128,
2121,
2115,
2108,
2102,
2095,
2089,
2082,
2076,
2069,
2063,
2056,
2050,
2043,
2037,
2030,
2024,
2017,
2011,
2004,
1998,
1992,
1985,
1979,
1972,
1966,
1959,
1953,
1946,
1940,
1933,
1927,
1920,
1914,
1907,
1901,
1894,
1888,
1881,
1875,
1868,
1862,
1855,
1849,
1842,
1836,
1830,
1823,
1817,
1810,
1804,
1797,
1791,
1784,
1778,
1771,
1765,
1758,
1752,
1745,
1739,
1732,
1726,
1719,
1713,
1706,
1700,
1693,
1687,
1680,
1674,
1668,
1661,
1655,
1648,
1642,
1635,
1629,
1622,
1616,
1609,
1603,
1596,
1590,
1583,
1577,
1570,
1564,
1557,
1551,
1544,
1538,
1531,
1525,
1518,
1512,
1506,
1499,
1493,
1486,
1480,
1473,
1467,
1460,
1454,
1447,
1441,
1434,
1428,
1421,
1415,
1408,
1402,
1395,
1389,
1382,
1376,
1369,
1363,
1356,
1350,
1344,
1337,
1331,
1324,
1318,
1311,
1305,
1298,
1292,
1285,
1279,
1272,
1266,
1259,
1253,
1246,
1240,
1233,
1227,
1220,
1214,
1207,
1201,
1194,
1188,
1182,
1175,
1169,
1162,
1156,
1149,
1143,
1136,
1130,
1123,
1117,
1110,
1104,
1097,
1091,
1084,
1078,
1071,
1065,
1058,
1052,
1045,
1039,
1032,
1026,
1020,
1013,
1007,
1000,
994,
987,
981,
974,
968,
961,
955,
948,
942,
935,
929,
922,
916,
909,
903,
896,
890,
883,
877,
870,
864,
858,
851,
845,
838,
832,
825,
819,
812,
806,
799,
793,
786,
780,
773,
767,
760,
754,
747,
741,
734,
728,
721,
715,
708,
702,
696,
689,
683,
676,
670,
663,
657,
650,
644,
637,
631,
624,
618,
611,
605,
598,
592,
585,
579,
572,
566,
559,
553};

static int8_t old_val;		// static variable last commanded value
static int8_t direction;
static int16_t rem_steps;
static uint8_t latency;

/** Returns the current value of the servo position
*/
char servo_read()
{
	return old_val;
}

void servo_set_direction( const uint8_t dir )
{
	if(dir>=1)
		direction = 1;
	else
		direction = 0;
}
void servo_advance()
{
	if( direction == 1)
	{
		old_val ++;
		OCR1A = pwmt[old_val+128];
	}
	else 
	{
		old_val --;
		OCR1A = pwmt[old_val+128];
	}
}
void servo_set_rem_steps( const uint8_t steps)
{
	rem_steps = steps;
}
int16_t servo_get_rem_steps()
{
	return rem_steps;
}
void servo_set_latency( const uint8_t lat)
{
	latency = lat;
}
uint8_t servo_get_latency()
{
	return latency;
}

/** Sets servo to the desired value, with the desired delay between steps (latency).
*/
void servo_set_val( const int8_t target_val, const int8_t lat )
{
	uint8_t val;
	
	if(target_val > old_val)	// compute direction
	{
		for(val=old_val+128; val < target_val+128; val++)	// +127 converts to unsigned char
		{
			OCR1A = pwmt[val];
			_delay_ms(lat);
			//_delay_ms(1);
		}
	}
	if(target_val < old_val)
	{
		for(val=old_val+128; val > target_val+128; val--)	
		{
			OCR1A = pwmt[val];
			_delay_ms(lat);
			//_delay_ms(1);
		}
	}
	old_val = target_val;	// update final position value
}

/** Sets servo to its neutral value, and updates the position register
*/
void servo_set_neutral()
{
	OCR1A = pwmt[127];
	old_val = 0;
}

/** Sets servo to its max value, and updates the position register
*/
void servo_set_max()
{
	OCR1A = pwmt[255];
	old_val = 127;
}

/** Sets servo to its min value, and updates the position register
*/
void servo_set_min()
{
	OCR1A = pwmt[0];
	old_val = -128;
}

/** Inits servo pwm module.
*/
void servo_init()
{
	BITSET(DDRB,PB5);	// set PORTB pin 5 as output for OC output (mandatory by datasheet)
	TCCR1A = 0x82;		// phase correct PWM, 8-bit, TOP=ICR1, set when up-count/clear when down-count
	TCCR1B = 0x12;  	// prescaler=8

	ICR1 = 18432;		// after 18432 timer counts (which is 10ms), slope goes down. Because of double-slope feature of Phase-Correct PWM, final period will thus be of 20ms, which is the required by the servo
}
