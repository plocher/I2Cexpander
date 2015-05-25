
/*
 * I2C AtoD, DtoA test code
 *
 * 2014 John Plocher  SPCoast
 * 
 * PCF8591 I2C AD/DA interface
 * http://www.spcoast.com/wiki/index.php/I2C-AD4DA#tab=1_1
 *
 * Slider potentiometer board for I/O
 * http://www.spcoast.com/wiki/index.php/LightUI
 *
 */

#include <Wire.h>
#include <I2Cextender.h>
#include <elapsedMillis.h>

#define NUMPORTS 8  // 2x ADC boards...   See 
I2Cextender m[NUMPORTS];

void setup()
{
    Serial.begin(19200);
    
    Wire.begin();
    for (int x = 0; x < NUMPORTS; x++) {
        m[x]       = I2Cextender();
    }
    // .init(addr, type, channel)
    m[ 7].init(1, I2Cextender::PCF8591,  0x03); // R  Background/horizon
    m[ 6].init(1, I2Cextender::PCF8591,  0x02); // G
    m[ 5].init(1, I2Cextender::PCF8591,  0x01); // B
    m[ 4].init(1, I2Cextender::PCF8591,  0x00); // W
    
    m[ 3].init(0, I2Cextender::PCF8591,  0x03); // R  Forground/overhead
    m[ 2].init(0, I2Cextender::PCF8591,  0x02); // G
    m[ 1].init(0, I2Cextender::PCF8591,  0x01); // B
    m[ 0].init(0, I2Cextender::PCF8591,  0x00); // W
}

void loop() {
    boolean changed = false;
    for (int x = 0; x < NUMPORTS; x++) {
        m[x].get();
        changed |= m[x].changed();
    }
    if (changed) {
        //  display:
        // 0:(255,128, 16)  24
        // 1:(  0, 32, 99)  24
        
        Serial.print("0:(");  Serial.print(m[3].current()); Serial.print(','); //R
                              Serial.print(m[2].current()); Serial.print(','); //G
                              Serial.print(m[1].current()); Serial.print(')'); //B
                              Serial.print(" ");
                              Serial.println(m[0].current());                  //W
                              
        Serial.print("1:(");  Serial.print(m[7].current()); Serial.print(',');
                              Serial.print(m[6].current()); Serial.print(',');
                              Serial.print(m[5].current()); Serial.print(')');
                              Serial.print(" ");
                              Serial.println(m[4].current());
    }
}
 
