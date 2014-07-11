SoundMotions
============

 Author: Matt Pinner @mpinenr
 
 Intent : 
 Smooth Sound and Motion Reactivity for Wearable LEDs
 
 Wearing LEDs can be dumb. mostly they're not respectful of the surrounding environment. 
 
 This code uses LOTS of data to infer the energy of the wearer and environment to turn down and slow the reactivity of LEDs.
 
  
 Libraries were used from the following sources:
 - I2Cdev for MPU9150 originally by Jeff Rowberg <jeff@rowberg.net> 
 at https://github.com/jrowberg/i2cdevlib (modified by Aaron Weiss <aaron@sparkfun.com>)
 - Adafruit's neopixel library : https://github.com/adafruit/Adafruit_NeoPixel
 - Msgeq7 for determining frequency bands and accelerometer readings : https://github.com/justinb26/MSGEQ7-library
 - EWMA for smoothing : https://github.com/CBMalloch/Arduino/tree/master/libraries/EWMA

