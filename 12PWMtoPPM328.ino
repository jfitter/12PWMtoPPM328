/***********************************************************************************************//**
 * \brief       12 Channel PWM to 1 channel PPM converter for RC receivers - Source File.
 * \file        12PWMtoPPM328.ino
 * \author      John Fitter <jfitter@eagleairaust.com.au>
 * \version     1.0
 * \date        January 2017
 * \copyright   Copyright (C) 2017 J. F. Fitter <jfitter@eagleairaust.com.au>
 * \par         Details
 *              Firmware for Arduino Nano v3 to measure 12 PWM R/C servo signals and output
 *              a PPM pulse train in addition to serial packets. 
 * \par         
 *              This firmware is based on ArduPPM Version v0.9.87 from 
 *              http://code.google.com/p/ardupilot-mega/source/browse/Tools/ArduPPM/
 *              and has been substantially modified to support only Atmel328 devices. It is 
 *              based on an idea from David/Buzz Sept 3rd 2012 (I am not responsible for the goto's)
 * \par         
 *              Input:  Up to 12 PWM signals. PW = 800 to 2200uS, PRF = 50 to 300Hz
 *              Output: Standard PPM 12ch pulse train
 *                      ASCII Serial packets at 115200,n,8,1 [[PWuS ]...]crlf
 * \par         
 *              Compile-time option to either "hold last good PPM value" or 
 *              "hold default value/s" in the case of no actual input signal for each channel.
 *              See FAILHOLD and FAILCENTRE in .h file
 * \par         
 * \par         License
 *              This program is free software; you can redistribute it and/or modify it under
 *              the terms of the GNU Lesser General Public License as published by the Free
 *              Software Foundation; either version 2.1 of the License, or (at your option)
 *              any later version.
 * \par         
 *              This Program is distributed in the hope that it will be useful, but WITHOUT ANY
 *              WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *              PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details
 *              at http://www.gnu.org/copyleft/gpl.html
 * \par         
 *              You should have received a copy of the GNU Lesser General Public License along
 *              with this library; if not, write to the Free Software Foundation, Inc.,
 *              51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 * \par         
 * \par         USAGE DETAILS 
 * \par         Purpose:
 *                  A diagnostic tool for setting up flight controllers.\n
 * \par         Start with:
 *                  An Arduino Nano v3 with a Atmega328p chip on it and 16MHz clock.\n
 *                  An IDE to program the chip with this code (Arduino, Visual Studio, etc).
 * \par         How:?
 *                  Use the Arduino IDE to program this firmware onto the Arduino chip.
 * \par        
 *                  Connect up to 12 RC PWM input signals so that the wires go to:
 * \li                  red (red)      =>> 5v (see TIPS below)
 * \li                  black (brown)  =>> GND or 0V pin on Arduino
 * \li                  white (yellow) =>> PWM signal pins, these connect to D2..D7 and A0..A5
 * \par        
 *                  Connect the PPM output so that the wires go to:
 * \li                  red (red)      =>> 5v
 * \li                  black (brown)  =>> GND or 0V
 * \li                  white (yellow) =>> D10 
 * \par
 *                  Connect USB to PC and run the 12 Channel RC Monitor application (Windows tm) 
 * \par         Done! 
 * \par         Why?
 *              Some flight controller's setup software, eg. CleanFlight for CCD3, does not display 
 *              its output in real time. The output data is displayed at the same rate as if it were
 *              received over the telemetry link. For setting up the controller and experimenting 
 *              with mixer settings it is convenient to have a fast responding servo signal monitor 
 *              rather than having to connect real servos.
 * \par         Tips:
 *              Any channel that you don't connect a PWM input on, will emit a default value (which 
 *              is 1500 for all channels except throttle, which is 1100). Disconnecting any channel 
 *              after booting will cause the system to use the last-known-position of the input, 
 *              until a good signal returns.
 * \par
 *              Generally the USB power is sufficient to power the R/C receiver. If powering the
 *              receiver separately and monitoring the output via USB it is recommended to 
 *              disconnect all of the power wires from the servo inputs, ie. allow the Arduino to be
 *              powered only by USB power.
 * \par
 *              It is not a good idea to run the R/C receiver at more than 5V.
 * \par
 * \par         Legals:     
 *              <b>*** WARNING - WARNING - WARNING ***</b>\n
 *              DO NOT use this circuit in a real model\n
 *              It is for bench testing only. Use in a model at your own risk.
 * \par
 *              Be familiar with local regulations relating to air safety and 
 *              safety of persons on the ground.
 * \par
 *              Neither the code nor the hardware is redundant nor error resistant. The only 
 *              concession is the watchdog timer which is set to 250mS. 
 * \par
 *              This is not a "fail-safe" unit and has no fail-safe functionality.
 * 
 *//***********************************************************************************************/
 
#include "Arduino.h"
#include "printf.h"
#include "12PWMtoPPM328.h"

/***********************************************************************************************//**
 * PROGRAM SETUP
 *//***********************************************************************************************/

void setup() {
    
    Serial.begin(115200);
    printf_begin();
    Serial.println(F("12ch PWM to PPM Test Program\r\n"));

    if(MCUSR & 1) MCUSR = 0;                            // Power-on Reset - Clear MCU Status reg
    else if(MCUSR & 2) MCUSR = 0;                       // External Reset - Clear MCU Status reg
    else if(MCUSR & 4) {                                // Brown-Out Reset
        MCUSR = 0;                                      // Clear MCU Status register
        brownout_reset = true;                  
    }else MCUSR = 0;                                    // Watchdog Reset - Clear MCU Status reg
                                                
    ppm_encoder_init();                                 // Servo input and PPM generator init
    SET_PPM_OUTPUT_DIR();                               // Set PPM pin (PPM_OUT_PIN,OC1B) to output
    sei();                                              // Enable Global interrupt flag
}

/***********************************************************************************************//**
 * PROGRAM MAIN LOOP
 *//***********************************************************************************************/

void loop() {
    static uint8_t servo_ch = 1;
    uint16_t ppm_val;

    while(true) {
        _delay_us(1900);                                // Slow down while loop
        ppm_val = ppm_read_channel(servo_ch);
        printf_P(PSTR("%4u "), ppm_val);
        if (++servo_ch > SERVO_CHANNELS) {
            servo_ch = 1;
            Serial.println("");
        }
    }
}

/***********************************************************************************************//**
 * PPM READING HELPER - interrupt safe and non blocking function (servo_ch is 1 based)
 * \param[in] servo_ch  Servo channel (1..12)                             
 *//***********************************************************************************************/

uint16_t ppm_read_channel( uint8_t servo_ch ) {

    uint8_t _ch = servo_ch;                             // Limit channel to valid value
    if(_ch == 0) _ch = 1;
    if(_ch > SERVO_CHANNELS) _ch = SERVO_CHANNELS;
    uint8_t ppm_ndx = (_ch << 1) - 1;                   // Calculate ppm[..] position
    uint16_t ppm_tmp = ppm[ppm_ndx];                    // Read ppm[..] non blocking interrupt safe
    while(ppm_tmp != ppm[ppm_ndx])
    ppm_tmp = ppm[ppm_ndx];
    
    return (ppm_tmp + PPM_PRE_PULSE) / ONE_US;          // Return as normal servo value
}

/***********************************************************************************************//**
 *  PPM GENERATOR START - TOGGLE ON COMPARE INTERRUPT ENABLE
 *  \par
 *  This starts OUTGOING PPM stream on PPM_PORT (PORTB, Arduino D8-D13) at 
 *  PPM_OUT_PIN (PB2, arduino pin D10) 
 *//***********************************************************************************************/

void ppm_start(void) {

    if (ppm_generator_active) return;                   // Prevent re-enabling active PPM generator
    uint8_t SREG_tmp = SREG;                            // Store interrupt status and reg flags
    cli();                                              // Stop interrupts
    RESET_PPM_OUTPUT();                                 // Make sure initial output state is low
    /*_delay_us(1);*/                                   // Wait for output pin to settle
                                                
    SERVO_TIMER_CNT = 0;                                // Set initial compare toggle to max (32ms)
    PPM_COMPARE = 0xFFFF;                               // allow system time to start
                                                
    SET_CCP1_TCC();                                     // Set toggle on compare output
    SET_TMR1_PSC8();                                    // Set TIMER1 8x pre-scaler
    ENABLE_CCP1();                                      // Enable output compare interrupt
    ppm_generator_active = true;                        // Indicate that PPM generator is active
    SREG = SREG_tmp;                                    // Restore interrupt status and reg flags
}

/***********************************************************************************************//**
 * PPM GENERATOR STOP - TOGGLE ON COMPARE INTERRUPT DISABLE
 *//***********************************************************************************************/

void ppm_stop(void) {

    uint8_t SREG_tmp = SREG;                            // Store interrupt status and reg flags
    cli();                                              // Stop interrupts
    DISABLE_CCP1();                                     // Disable output compare interrupt
    RESET_TMR1();                                       // Reset TIMER1 registers
    ppm_generator_active = false;                       // Indicate that PPM generator not active
    SREG = SREG_tmp;                                    // Restore interrupt status and reg flags
}

/***********************************************************************************************//**
 *  WATCHDOG INTERRUPT (interrupt only mode, no reset)
 *  \par
 *  If watchdog is triggered then enable missing signal flag and copy power on or failsafe 
 *  positions use failsafe values if PPM generator is active or if chip has been reset from
 *  a brown-out
 *//***********************************************************************************************/

ISR(WDT_vect) {

    if (ppm_generator_active || brownout_reset) {
        for (uint8_t i=0; i<PPM_ARRAY_MAX; i++) {       // Copy fail-safe values to ppm[..]
            ppm[i] = failsafe_ppm[i];
        }
    }
    servo_input_missing = true;                         // Set missing receiver signal flag
    /*servo_input_errors = 0;*/                         // Reset servo input error flag
}

/***********************************************************************************************//**
 * SERVO/PPM INPUT - PIN CHANGE INTERRUPT, for pins D2..D7 and pins C0..C5
 *//***********************************************************************************************/

ISR(PCINT2_vect) {

    // Servo pulse start timing
    static uint16_t servo_start[SERVO_CHANNELS] = 
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    static uint8_t throttle_timeout = 0;                // Missing throttle signal fail-safe
    static uint16_t servo_pins_old = 0;                 // Servo input pin storage 
    uint16_t servo_pins;                                // Used to store current servo input pins
    uint16_t servo_time = SERVO_TIMER_CNT;              // Read current servo pulse change time
                                                
CHECK_PINS_START:                                       // Start of servo input check
    servo_pins = SERVO_INPUT;                           // Store current servo input pins
                                                
    uint16_t servo_change = servo_pins^servo_pins_old;  // Calculate servo input pin change mask
    uint16_t servo_pin = 1;                             // Set initial servo pin and channel
    uint8_t  servo_channel = 0;                 
                                                
CHECK_PINS_LOOP:                                        // Input servo pin check loop
                                                
    if(servo_change & servo_pin) {                      // Check for pin change on current servo ch
        if(servo_pins & servo_pin)                      // High (raising edge)
            servo_start[servo_channel] = servo_time;        
        else {
            uint16_t servo_width = servo_time-          // Get servo pulse width
                servo_start[servo_channel]-PPM_PRE_PULSE;
            uint8_t _ppm_channel=(servo_channel<<1)+1;  // Calculate servo ch position in ppm[..]
            if (servo_width > PPM_SERVO_MAX)            // Check that servo pulse signal is valid 
                goto CHECK_PINS_ERROR;                  // before sending to ppm encoder
            if (servo_width < PPM_SERVO_MIN) 
                goto CHECK_PINS_ERROR;
            goto CHECK_PINS_NOERROR;

        CHECK_PINS_ERROR:                               // on width input error, use default/
                                                        // fail-safe value, OR previous value    
            #define FAILHOLD 1                          // choose the error handling type here!
            #ifdef FAILCENTRE                   
            servo_width = failsafe_ppm[_ppm_channel];   // failsafe defaults, most channels 
            #endif                                      // centred, throttle lowered. 
                                                
            #ifdef FAILHOLD                     
            servo_width = ppm[_ppm_channel];            // all channels hold their
            #endif                                      // previous position! 
                                                
        CHECK_PINS_NOERROR:                     
            if(_ppm_channel == 5) throttle_timeout = 0; // Reset throttle failsafe timeout
                           
            #ifdef _AVERAGE_FILTER_             
            servo_width += ppm[_ppm_channel];           // Average filter to smooth input jitter
            servo_width >>= 1;                  
            #endif                              
                                                
            #ifdef _JITTER_FILTER_                      // 0.5us cut filter to remove input jitter
            int16_t ppm_tmp = ppm[_ppm_channel]-servo_width;
            if(ppm_tmp == 1)  goto CHECK_PINS_NEXT;
            if(ppm_tmp == -1) goto CHECK_PINS_NEXT;
            #endif

            ppm[_ppm_channel] = servo_width;            // Update ppm[..]
        }                                           
    }                                               
                                                    
CHECK_PINS_NEXT:                                    
    servo_pin <<= 1;                                    // Select next servo pin
    servo_channel++;                                    // Select next servo channel
    if (servo_channel < SERVO_CHANNELS)                 // Check channel and process if needed
        goto CHECK_PINS_LOOP;                       
    goto CHECK_PINS_DONE;                           
                                                    
CHECK_PINS_DONE:                                        // All servo input pins now been processed
    wdt_reset();                                        // Reset Watchdog Timer
    servo_input_missing = false;                        // Reset servo input missing flag to show
                                                        // that we have received servo input signals
    servo_pins_old = servo_pins;                        // Save servo input pins for next check
    if(ppm_generator_active == false) ppm_start();      // Start PPM generator if not ready running
    if(throttle_timeout++ >= 128) {                     // Throttle fail-safe
        throttle_timeout = 0;                           // Reset throttle time-out
        ppm[5] = PPM_THR_FS;                            // Set throttle fail-safe value
    }                                               
    if(servo_pins != SERVO_INPUT) goto CHECK_PINS_START;// Has servo input changed while processing
                                                        // pins, if so we need to re-check pins
    SERVO_INT_CLEAR();                                  // Clear interrupt event from already 
}                                                       // processed pin changes
ISR(PCINT1_vect, ISR_ALIASOF(PCINT2_vect));

/***********************************************************************************************//**
 * PPM OUTPUT - TIMER1 COMPARE INTERRUPT
 *//***********************************************************************************************/

ISR(TIMER1_COMPB_vect) {

    static uint8_t ppm_channel = PPM_ARRAY_MAX - 1;     // Current active ppm channel
    PPM_COMPARE += ppm[ppm_channel];                    // Update timing for next compare toggle
    if(++ppm_channel >= PPM_ARRAY_MAX) ppm_channel = 0; // Select the next ppm channel
}

/***********************************************************************************************//**
 * PPM ENCODER INIT
 *//***********************************************************************************************/

void ppm_encoder_init(void) {

    // SERVO/PPM INPUT PINS
    SET_SERVO_INPUTS();                                 // Set servo pins to inputs
    SET_SERVO_PULLUPS();                                // Activate pull-ups on servo input pins
                                                
    // SERVO/PPM INPUT - PIN CHANGE INTERRUPT   
    SERVO_INT_ENABLE();                                 // Enable servo input interrupt
    SET_SERVO_INT_MASK();                               // Set the pin mask for servo interrupts
    SET_PPM_OUTPUT_DIR();                               // Set PPM pin to output
                                                
    // ENABLE WATCHDOG INTERRUPT MODE           
    wdt_disable();                                      // Disable watchdog
    wdt_reset();                                        // Reset watchdog timer    
    WDTCSR |= (1 << WDCE) | (1 << WDE);                 // Start timed watchdog set-up sequence
    WDTCSR  = (1 << WDIE) | (1 << WDP2);                // Set 250 ms watchdog time-out
}                                                       // and enable interrupt

