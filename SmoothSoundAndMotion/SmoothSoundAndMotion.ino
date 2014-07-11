/* 
 
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


 */

#undef int()
#include <stdio.h>
#include <math.h>
#include <EWMA.h>

// number of sound frequency bands supported
#define BANDS 7

// we'll be tracking an Exponentially Weighted Moving Average (EWMA) for each band of sound 
EWMA myEwmaBand[BANDS];

// we'll also track an EWMA for our accelleration and gryo 
EWMA myEwmaAccel, myEwmaGyro;

// defines how many measurements the ewma for each sensor is calculated over.
// more measures will increase the smoothing. 
//fewer will make the impact of the most recent measurement greater and therefore more immediate
#define EWMA_SOUND_BUCKETS 10  // 10 - very reactive
#define EWMA_ACCEL_BUCKETS 400  // 400 - very slow to react
#define EWMA_GRYO_BUCKETS 100   // 100 - slow affect


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

#include <MSGEQ7.h>
#include <Adafruit_NeoPixel.h>


#define LED_PIN 13

#define STRIP_PIN 2
#define NECK_PIN 2

// define MSGEQ7 : Output, Strobe, and Reset Pins
#define MSGEQ7_OUTPUT_PIN A3
#define MSGEQ7_STROBE_PIN 22
#define MSGEQ7_RESET_PIN 23

// define geometry of the jacket
#define SPINE_LEDS 24
#define NECK_LEDS 8

// create objects for our leds
Adafruit_NeoPixel strip = Adafruit_NeoPixel(SPINE_LEDS, STRIP_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel neck = Adafruit_NeoPixel(NECK_LEDS, NECK_PIN, NEO_GRB + NEO_KHZ800);


// create an object for our sound sensor's hardware FFT
MSGEQ7 msgeq7;


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

// lots of data from our motion sensor can be stored here for caching and globe access

// stores raw accel/gyro/mag measurements from device
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

// stores previous and max accelleration values for delta calculation and self tuning
int16_t lastax, lastay, lastaz;
int16_t ewmaAccelX, ewmaAccelY, ewmaAccelZ;

// stores previous and max gyro values for delta calculation and self tuning
int16_t lastgx, lastgy, lastgz;
int16_t ewmaGyroX, ewmaGyroY, ewmaGyroZ;


// stores previous and max sound values for delta calculation and self tuning
int16_t lastBand[BANDS];
int16_t emBand[BANDS];


bool blinkState = false;

void setup() {
  delay(100);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  neck.begin();
  neck.show(); // Initialize all pixels to 'off'

  // Initialize Output, Strobe, and Reset Pins    
  msgeq7.init(MSGEQ7_OUTPUT_PIN, MSGEQ7_STROBE_PIN, MSGEQ7_RESET_PIN); 

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);


  // initialize our sensor averages
  myEwmaAccel.init(myEwmaAccel.periods(EWMA_ACCEL_BUCKETS)); 
  myEwmaGyro.init(myEwmaGyro.periods(EWMA_GRYO_BUCKETS)); 

  for (int i = 0; i < BANDS; i++) {
    myEwmaBand[i].init(myEwmaBand[i].periods(EWMA_SOUND_BUCKETS));
  }

  return; // done with setup
}


// this happens as fast as posssible over and over
// NO DELAYS
// 
// standard senor loop:
// 0. read sensors
// 1. derive meaning
// 2. update outputs
void loop() {


  // read sound bands
  // - results are stored for globe access)
  readEq();

  // read motion sensors 
  // - results are stored for globe access)
  readMotion();


  // calculate a brightness from our Averages and scale to 1-255 for led display use
  // our Y direction is the most impactfull as it is alligned with the human body up and down
  // the Y accelleration implies up and down motion -> walking?dancing
  // the Y gryo is a rotation (or more accurately a change in rotation) as in turning around or rotation at the hips or shoulders
  int brightness = map (ewmaAccelY + ewmaGyroY, 0, 200000, 1, 255); 


  // update our leds
  // our color/birghtness math is kind of sad here
  // i choose very specific/simple colors for each band and to always match each other.
  for (int i = 0; i < SPINE_LEDS; i++ ) {
    strip.setPixelColor(i, 0, 0, 0);
    neck.setPixelColor(i, 0, 0, 0);


    // we only care about the 4 lower bands
    // the highs are always full of noise.
    // the mid are great for human voice detection
    // the lows imply some music 

    // purple for a middle band
    // starts could go to the top
    if (emBand[3]/40 > (i-15)) {
      strip.setPixelColor(SPINE_LEDS-i, brightness/2, 0, brightness);
      neck.setPixelColor(8-i/3, brightness/2, 0, brightness);
    }

    // blue for a lower middle band
    // starts nearish the bottom
    if (emBand[2]/50 > (i-12)) {
      strip.setPixelColor(SPINE_LEDS-i, 0, 0, brightness);
      neck.setPixelColor(8-i/3, 0, 0, brightness);
    }

    // pink for the lower band
    // starts near the bottom
    if (emBand[1]/60 > (i-6)) {
      strip.setPixelColor(SPINE_LEDS-i, brightness, 0, brightness);
      neck.setPixelColor(8-i/3, brightness, 0, brightness);
    }

    // tourquise for the lowest band
    // starts from the bottom
    if (emBand[0]/70 > i) {
      strip.setPixelColor(SPINE_LEDS-i, 0, brightness, brightness);
      neck.setPixelColor(8-i/3, 0, brightness, brightness);
    }


    // end for each led loop
  }


  // push new led colors to the hardware
  strip.show();
  neck.show();


  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);


  return; // done with main loop (do it again)
}




// reads sound data
// stores results in global variables
// updates EWMAs
void readEq() {

  msgeq7.poll(); // Update values from MSGEQ7  
  showEqValues();  // Print the current EQ values to Serial

  for (int i = 0; i < BANDS; i++) {
    // record the difference in readings from last reading, so lots of zeros
    emBand[i] = msgeq7.getValue(i);
    myEwmaBand[i].record(abs(lastBand[i] - emBand[i]));
    lastBand[i] = emBand[i];
  }

  return;
}

// reads motion data
// stores results in global variables
// updates EWMAs
void readMotion() {

  // read raw accel/gyro measurements from device
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  ewmaAccelY = myEwmaAccel.record(abs(lastay - ay));
  lastay = abs(ay);
  ewmaGyroY = myEwmaGyro.record(abs(lastgy - gy));
  lastgy = abs(gy);

  showMotionValues();

  return;
}





// prints sound bands to serial out 
// for debugging
void showEqValues()
{
  Serial.print("Current values - ");

  Serial.print("63HZ: ");
  Serial.print(msgeq7.getValue(0));
  Serial.print(" 160HZ: ");
  Serial.print(msgeq7.getValue(1));
  Serial.print(" 400HZ: ");
  Serial.print(msgeq7.getValue(2));
  Serial.print(" 1000HZ: ");
  Serial.print(msgeq7.getValue(3));
  Serial.print(" 4500HZ: ");
  Serial.print(msgeq7.getValue(4));
  Serial.print(" 6250HZ: ");
  Serial.print(msgeq7.getValue(5));
  Serial.print(" 16000HZ: ");
  Serial.print(msgeq7.getValue(6));
  Serial.println(); 

  return;
}


// prints motion data to serial out 
// for debugging
void showMotionValues() {

  // display tab-separated raw accel/gyro/mag values
  Serial.print("a/g/m:\t");
  Serial.print(ax); 
  Serial.print("\t");
  Serial.print(ay); 
  Serial.print("\t");
  Serial.print(az); 
  Serial.print("\t");
  Serial.print(gx); 
  Serial.print("\t");
  Serial.print(gy); 
  Serial.print("\t");
  Serial.print(gz); 
  Serial.print("\t");
  Serial.print(mx); 
  Serial.print("\t");
  Serial.print(my); 
  Serial.print("\t");
  Serial.print(mz);

  // display calculated ewma values
  Serial.print("\tew: ");
  Serial.print(ewmaAccelY); 
  Serial.print("\t");
  Serial.print(ewmaGyroY); 
  Serial.print("\t");
  Serial.println("\t");

  return;
}












