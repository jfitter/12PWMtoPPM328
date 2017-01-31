#ifndef _12PWMtoPPM328_H_
#define _12PWMtoPPM328_H_

/***********************************************************************************************//**
 *  \brief      12 Channel PWM to 1 channel PPM converter for RC receivers - Header File.
 *  \file       12PWMtoPPM328.h
 *  \author     John Fitter <jfitter@eagleairaust.com.au>
 *  \version    1.0
 *  \date       January 2017
 *  \copyright  Copyright (C) 2017 J. F. Fitter <jfitter@eagleairaust.com.au>
 *  \par
 *  \par        License
 *              This program is free software; you can redistribute it and/or modify it under
 *              the terms of the GNU Lesser General Public License as published by the Free
 *              Software Foundation; either version 2.1 of the License, or (at your option)
 *              any later version.
 *  \par
 *              This Program is distributed in the hope that it will be useful, but WITHOUT ANY
 *              WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *              PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details
 *              at http://www.gnu.org/copyleft/gpl.html
 *  \par
 *              You should have received a copy of the GNU Lesser General Public License along
 *              with this library; if not, write to the Free Software Foundation, Inc.,
 *              51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *//***********************************************************************************************/

#include <avr/io.h>

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

/** SERVO INPUT FILTERS 
 *  \par
 *  Using both filters is not recommended and may reduce servo input resolution \n
 *  #define _AVERAGE_FILTER_     // Average filter to smooth servo input capture jitter \n
 *  #define _JITTER_FILTER_      // Cut filter to remove 0,5us servo input capture jitter
*/

#ifndef F_CPU
#define F_CPU   16000000UL                              /**< CPU speed Hz */
#endif

#ifndef true
#define true    1
#endif

#ifndef false               
#define false   0
#endif

#define PB2 PORTB2                  // 328 does not define PBX but defines an equivalent as PORTBX, 
#define PB1 PORTB1                  // comment these lines out if you already have a PB2 defined. 
#define PB0 PORTB0                  

#define TTS (F_CPU/8/1000/1000)                         /**< Timer1 ticks per microsecond */

/** Servo limit values */
#define PRE_PULSE 400                                   /**< PPM pre pulse in uS */
#define MAX_SERVO 2100                                  /**< Servo maximum in uS */
#define MIN_SERVO 900                                   /**< Servo minimum in uS */
#define NTL_SERVO 1500                                  /**< Servo neutral in uS */
#define TDF_SERVO 1100                                  /**< Throttle power-up default in uS */

#define PPM_PRE_PULSE (TTS*PRE_PULSE)                   /**< 400us PPM pre pulse */
#define PPM_SERVO_MIN (TTS*MIN_SERVO-PPM_PRE_PULSE)     /**< Servo minimum position */
#define PPM_SERVO_CTR (TTS*NTL_SERVO-PPM_PRE_PULSE)     /**< Servo center position */
#define PPM_SERVO_MAX (TTS*MAX_SERVO-PPM_PRE_PULSE)     /**< Servo maximum position */
#define PPM_THR_DEF (TTS*TDF_SERVO-PPM_PRE_PULSE)       /**< Throttle default at power on */
#define PPM_THR_FS (TTS*MIN_SERVO-PPM_PRE_PULSE)        /**< Throttle during failsafe */
#define SERVO_CHANNELS 12                               /**< Number of servo input channels */
#define PPM_ARRAY_MAX (SERVO_CHANNELS*2+2)              /**< Size of ppm[..] data array */
                                                        /**< (servo channels * 2 + 2) */
/** PPM period 18.5ms - 26.5ms (54hz - 37Hz) */
#define PPM_PERIOD (TTS*((SERVO_CHANNELS+1)*(MAX_SERVO+PRE_PULSE)-(SERVO_CHANNELS*NTL_SERVO)))  

/** Data array for storing ppm (12 channels) pulse widths. */
volatile uint16_t ppm[PPM_ARRAY_MAX] = {
    PPM_PRE_PULSE,
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 1  */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 2 */
    PPM_THR_DEF,   PPM_PRE_PULSE,                       /**< Channel 3 (throttle) */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 4 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 5 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 6 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 7 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 8 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 9 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 10 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 11 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 12 */
    PPM_PERIOD
};

/** Servo fail-safe values */
const uint16_t failsafe_ppm[ PPM_ARRAY_MAX ] = {
    PPM_PRE_PULSE,
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 1 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 2 */
    PPM_THR_FS,    PPM_PRE_PULSE,                       /**< Channel 3 (throttle) */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 4 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 5 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 6 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 7 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 8 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 9 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 10 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 11 */
    PPM_SERVO_CTR, PPM_PRE_PULSE,                       /**< Channel 12 */
    PPM_PERIOD
};

/** AVR parameters for ArduPilot MEGA v1.4 PPM Encoder (ATmega328P) */
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)

#define SERVO_TIMER_CNT         TCNT1
#define PPM_DDR                 DDRB
#define PPM_PORT                PORTB                   /**< Port for PPM output */
#define PPM_OUT_PIN             PB2                     /**< Port pin for PPM output */
#define PPM_COMPARE             OCR1B

#define SERVO_INPUT             ((((uint16_t)(PINC&0b00111111))<<6)|(uint16_t)(PIND>>2))

#define SET_SERVO_INPUTS()      {DDRD   &= 0b00000011; DDRC &= 0b11000000;}
#define SET_SERVO_PULLUPS()     {PORTD  |= 0b11111100; PORTC |= 0b00111111;}
#define SET_SERVO_INT_MASK()    {PCMSK2 |= 0b11111100; PCMSK1 |= 0b00111111;}
#define SERVO_INT_CLEAR()       {PCIFR  |= (1<<PCIF2)|(1<<PCIF1);}
#define SERVO_INT_ENABLE()      {PCICR  |= (1<<PCIE2)|(1<<PCIE1);}

#define SET_PPM_OUTPUT_DIR()    {DDRB   |= (1<<PPM_OUT_PIN);}
#define RESET_TMR1()            {TCCR1A  = 0; TCCR1B = 0;}
#define ENABLE_CCP1()           {TIMSK1 |= (1<<OCIE1B);}
#define DISABLE_CCP1()          {TIMSK1 &= ~(1<<OCIE1B);}
#define SET_TMR1_PSC8()         {TCCR1B  = (1<<CS11);}
#define SET_CCP1_TCC()          {TCCR1A  = (1<<COM1B0);}
#define RESET_PPM_OUTPUT()      {PORTB  &= ~(1<<PPM_OUT_PIN);}

#else
#error NO SUPPORTED DEVICE FOUND! (ATmega328/p)
#endif

volatile bool servo_input_missing  = true;              /**< indicate missing SERVO input signals */
volatile bool ppm_generator_active = false;             /**< indicate if PPM generator is active */
volatile bool brownout_reset       = false;             /**< indicate a brownout restart */
                                                         
#endif // _12PWMtoPPM328_H_                              
                                                         
                                                         