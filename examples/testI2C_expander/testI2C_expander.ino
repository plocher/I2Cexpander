/*
 *  Test harness code for I2C extender
 *
 *  Circuit:  A standard Arduino with
 *  2x I2C-8574 cards https://spcoast.github.io/pages/I2C-8574.html   and
 *  4x I2C-7311 cards https://spcoast.github.io/pages/I2C-7311.html
 *  for a total of 112 I/O points that can be controlled
 *
 *  Copyright (c) 2014 John Plocher, released under the terms of the MIT License (MIT)
 */

#include <Wire.h>
#include <I2Cexpander.h>

#define Dc 40   // Delay for cylon...
#define Dl 1000 // Delay for ladder...

#define NUMPORTS 12 // 0..(NUMPORTS - 1)
I2Cexpander m[NUMPORTS];

void turnAllOff() {
    for (int x = 0; x < NUMPORTS; x++) {
        m[x].write(0xFFFFFFFF);
    }
}
void turnAllOn() {
    for (int x = 0; x < NUMPORTS; x++) {
        m[x].write(0x00000000);
    }
}

// walk the bits in the ports 0..max bits 0..7
void cylon(int iterations) {
    for (int iter = 0; iter < iterations; iter++) {
        for (int b = 0; b <= 7; b++) {
            for (int x = 0; x < NUMPORTS; x++) {
                m[x].put( ~(1 << b));
            }
            delay(Dc);
        }
        for (int b = 7; b >= 0; b--) {
            for (int x = 0; x < NUMPORTS; x++) {
                m[x].put( ~(1 << b));
            }
            delay(Dc);
        }
        for (int x = 0; x < NUMPORTS; x++) {
          m[x].put(0xFFFF);
        }
        delay(Dc);
    }
}

// walk the ports 
void ladder(int iterations) {
    for (int iter = 0; iter < iterations; iter++) {
        int lastport = -1;
        for (int x = 0; x < NUMPORTS; x++) {
            if (lastport != -1) { m[lastport].put(0xFFFF); }
            m[x].put(0x0000);
            lastport = x;
            delay(Dl);
        }
        m[lastport].put(0xFFFF);
        delay(Dl);
        for (int x = NUMPORTS - 1; x >= 0; x--) {
            if (lastport == x)  { lastport = -1; }
            if (lastport != -1) { m[lastport].put(0xFFFF); }
            m[x].put(0x0000);
            lastport = x;
            delay(Dl);
        }
        m[lastport].put(0xFFFF);
        delay(Dl);
    }
}

#define CONFIG_TURTLE   B0111  // Turtle Turnout Control - 1 out (Motor), 3 in (Occupied, Normal, Diverging)
void setup()
{
    for (int x = 0; x < NUMPORTS; x++) {
        m[x]       = I2Cexpander();
    }
    Wire.begin();
    // MAX731x shares I2C address range with the 8475's
    // MAX731x #8-12 have the same addresses as PCF8574 #0-7 and
    // MAX731x #20-27 overlap with the PCA8574A's
    m[11].init(3, I2Cexpander::MAX731x,  B0);         // 16 bits of output
    m[10].init(2, I2Cexpander::MAX731x,  0x0000);     // 16 bits of output
    m[ 9].init(1, I2Cexpander::MAX731x,  B1111111111111111);  // 16 bits of inputs
    m[ 8].init(0, I2Cexpander::MAX731x,  0xFFFF);     // also 16 inputs

    // 8475's come in two flavors, each with its own I2C address range
    // so each can use the same sequence range of 0..7
    // each I2C-8574 card has one of each, so init them in pairs:
    m[ 7].init(1, I2Cexpander::PCF8574A, B11111111);  // 8x inputs
    m[ 6].init(1, I2Cexpander::PCF8574,  (CONFIG_TURTLE << 4) | CONFIG_TURTLE);  // doesn't need to be hardcoded

    m[ 5].init(0, I2Cexpander::PCF8574A, B0);         // 8-bits output
    m[ 4].init(0, I2Cexpander::PCF8574,  B0);         // 8-bits output

    m[ 3].init(0, I2Cexpander::ARDIO_D,  B0);
    m[ 2].init(0, I2Cexpander::ARDIO_C,  B0);
    m[ 1].init(0, I2Cexpander::ARDIO_B,  B0);
    m[ 0].init(0, I2Cexpander::ARDIO_A,  B0);
}

void loop()
{
  turnAllOn();   delay(1000); 
  turnAllOff();  delay(1000); 
  cylon(5);      delay(300);
  ladder(1);     delay(200); 
  turnAllOff();  delay(300);
}

