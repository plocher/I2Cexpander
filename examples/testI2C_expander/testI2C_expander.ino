/*
 *  Test harness code for I2C extender
 *
 *  Copyright (c) 2014 John Plocher, released under the terms of the MIT License (MIT)
 */

#include <Wire.h>
#include <I2Cexpander.h>

#define Dc 40   // Delay for cylon...
#define Dl 1000 // Delay for ladder...

#define NUMPORTS 16+4
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

void setup()
{
    for (int x = 0; x < NUMPORTS; x++) {
        m[x]       = I2Cexpander();
    }
    Wire.begin();
    switch (NUMPORTS) {
        case 10: m[19].init(7, I2Cexpander::PCF8574,  B0); // 0 = output bit, 1=input
        case 19: m[18].init(6, I2Cexpander::PCF8574,  B0);
        case 18: m[17].init(7, I2Cexpander::PCF8574A, B0);
        case 17: m[16].init(6, I2Cexpander::PCF8574A, B0);
        
        case 16: m[15].init(5, I2Cexpander::PCF8574,  B0);
        case 15: m[14].init(4, I2Cexpander::PCF8574,  B0);
        case 14: m[13].init(5, I2Cexpander::PCF8574A, B0);
        case 13: m[12].init(4, I2Cexpander::PCF8574A, B0);
        
        case 12: m[11].init(3, I2Cexpander::PCF8574,  B0);
        case 11: m[10].init(2, I2Cexpander::PCF8574,  B0);
        case 10: m[ 9].init(3, I2Cexpander::PCF8574A, B0);
        case 9:  m[ 8].init(2, I2Cexpander::PCF8574A, B0);
        
        case 8:  m[ 7].init(1, I2Cexpander::PCF8574A, B0);
        case 7:  m[ 6].init(1, I2Cexpander::PCF8574,  B0);
        case 6:  m[ 5].init(0, I2Cexpander::PCF8574A, B0);
        case 5:  m[ 4].init(0, I2Cexpander::PCF8574,  B0);

        case 4:  m[ 3].init(0, I2Cexpander::ARDIO_D,  B0);
        case 3:  m[ 2].init(0, I2Cexpander::ARDIO_C,  B0);
        case 2:  m[ 1].init(0, I2Cexpander::ARDIO_B,  B0);
        case 1:  m[ 0].init(0, I2Cexpander::ARDIO_A,  B0);
    }
}

void loop()
{
  turnAllOn();   delay(1000); 
  turnAllOff();  delay(1000); 
  cylon(5);      delay(300);
  ladder(1);     delay(200); 
  turnAllOff();  delay(300);
}

