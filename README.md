# ATtiny85-Servo-Driver-x8
Command up to 8 PPM servo motors with an ATtiny85 and a SIPO shift register!

Originally written by me in summer of 2021, I'm now officially creating a repo in order to share it with others. Using only 2 of the 85's IO pins hooked up to the latch and data lines of a serial in parallel out (SIPO) shift register, you can control 8 servo motors with an update rate of ~50Hz. It should be able to set all 8 servos independently with anywhere from 500-2000us pulse widths.

Basic design is: each output of the shift register (SR) sends a pulse-period modulated (PPM) signal to one servo. The SR is updated and latched whenever one of the servo signals needs to change state. Signal start times are typically simultaneous, but are also automatically adjusted to coordinate pulses with similar durations, and prevent state changes from being closer than the time needed to update the SR. In other words, it takes appx ~19us to update and latch the SR, so if two signals need to come low within 38us of each other then their start times need to be staggered. 

I wrote this software with I2C in mind to create a servo driver accessory for another microcontroller, presently it does not have that capability but it should be rather simple to implement. Back in '21 I had been troubleshooting using an oscilloscope and an elaborate debug method involving viewing debug signals on one of the 85's pins, if I ever get around to throwing that setup together again I'll add I2C comms. For now I wouldn't want to just blindly include a library since it's liable to break something, but if anyone else wants to try their hand at it by all means go right ahead!
