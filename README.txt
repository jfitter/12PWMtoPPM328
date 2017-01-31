**********************************************************************************************
* John's 12 channel PWM to PPM/Serial converter for R/C use.                                 *
**********************************************************************************************

# Purpose:
* A diagnostic tool for setting up flight controllers.

# Start with:

* An Arduino Nano v3 with a Atmega328p chip on it and 16MHz clock. 
* An IDE to program the chip with this code (Arduino, Visual Studio, etc).

# How:?

* Use the Arduino IDE to program this firmware onto the Arduino chip.

* Connect up to 12 RC PWM input signals so that the wires go to:
     red (red)      =>> 5v (see TIPS below)
     black (brown)  =>> GND or 0V pin on Arduino
     white (yellow) =>> PWM signal pins, these connect to D2,D3,D4,D5,D6,D7,A0,A1,A2,A3,A4,A5

* Connect the PPM output so that the wires go to:
     red (red)      =>> 5v
     black (brown)  =>> GND or 0V
     white (yellow) =>> D10 

* Connect USB to PC and run the 12 Channel RC Monitor application (Windows tm) 

# Done! 

TIPS:  
* Any channel that you don't connect a PWM input on, will emit a default value 
* (which is 1500 for all channels except throttle, which is 1100).
* Disconnecting any channel after booting will cause the system to use the last-known-position 
* of the input, until a good signal returns.
* Generally the USB power is sufficient to power the R/C receiver.
* If powering the receiver separately and monitoring the output via USB it is recommended to
* disconnect all of the power wires from the servo inputs, ie. allow the Arduino to be powered
* only by USB power. It is not a good idea to run the R/C receiver at more than 5V.

*** WARNING - WARNING - WARNING - DO NOT use this circuit in a real model ***
* It is for bench testing only. Use in a model at your own risk.
* Be familiar with local regulations relating to air safety and safety of persons on the ground.
* Neither the code nor the hardware is redundant nor error resistant. The only concession is the
* watchdog timer which is set to 250mS. 
* This is not a "fail-safe" unit and has no fail-safe functionality.
