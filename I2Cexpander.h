/*
 * I2Cexpander.h - Library for interfacing to various IO Expander chips
 * Copyright (c) 20011...2015 John Plocher, released under the terms of the MIT License (MIT)
 *
 * Support for
 *  Onboard pins on
 *      Arduino AVR '328 (DUEMILANOVE, Pro-Mini...)
 *      Particle Photon
 *      DigiStump OAK
 *
 *  16 bit Expanders
 *      PCA9555
 *      MAX7313
 *      MCP23016
 *
 *   8 bit expanders
 *      PCF8574
 *      PCF8574A
 *      PCF8591  DAC/ADC
 *
 *  See example programs for usage information
 */

#ifndef I2Cexpander_h
#define I2Cexpander_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include <Wire.h>

#elif defined(SPARK_CORE)

#include "application.h"

#ifndef bitRead
#define bitRead(value, bit)            (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)             ((value) |= (1UL << (bit)))
#define bitClear(value, bit)           ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#endif

#endif

class I2Cexpander {
public:
    I2Cexpander(void);
    void     init(uint16_t address, uint16_t chip, uint16_t config, boolean debounce=false);

	void     digitalWrite(uint8_t dataPin, uint8_t val);
	uint8_t  digitalRead(uint8_t dataPin);

    uint16_t read(void) ;
	void     write(uint16_t data);
    void     write(void)        { I2Cexpander::write(next);   }
    uint16_t get(void)          { return I2Cexpander::read(); }
    void     put(uint16_t data) { next = data; I2Cexpander::write(data); }
    void     put(void)          { I2Cexpander::write(next);   }
    
    uint16_t getSize(void)      { return _size; };  
    uint16_t current()          { return I2Cexpander::_current; };
    uint16_t last()             { return I2Cexpander::_last;    };
	uint8_t  i2caddr()          { return (_i2c_address); };
    boolean  changed()          { return ((I2Cexpander::_current & I2Cexpander::_config) != (I2Cexpander::_last & I2Cexpander::_config)); };
    uint16_t next;
	byte     debugflag;
    enum IOSize {
      B_UNKNOWN =  0,
      B2        =  2,
      B4        =  4,
      B6        =  6,
      B8        =  8,
      B16       = 16,
      T_UNKNOWN =  0
    };
    enum ExpanderType {
      IGNORE    =  0,
      I2CLCD    =  0,
      PCA9555   =  1,       // Bits 0  1  2  3  4  5  6  7  8   9  10  11  12  13  14  15  16
      MCP23016,             // Bits 0  1  2  3  4  5  6  7  8   9  10  11  12  13  14  15  16
      PCF8574,              // Bits 0  1  2  3  4  5  6  7  8
      PCF8574A,             // Bits 0  1  2  3  4  5  6  7  8
    // DAC/ADC chip
      PCF8591,              // 4 A-D converters, 1 D-A
    // 16 bit MAX 3713
      MAX7313,              // Bits 0  1  2  3  4  5  6  7  8   9  10  11  12  13  14  15  16
#if defined(ARDUINO_AVR_DUEMILANOVE)
	// built-in Arduino ports, skipping RX/TX, Lnet RX/TX and I2C pins
      ARDIO_A,              // Bits D2   D3  D4   D5 - low digital
      ARDIO_B,              //      D6   D9  D10  D11  - high digital
      ARDIO_C,              //      D12  D13 A0   A1  - mixed, digital and analog
      ARDIO_D,              //      A2   A3  A6  A7   - analog (A6 & A7 are input only)
	// undocumented - special obsolete board with custom pinout
      ARDIO12_A,            //      D0  D1  D5  D6   D2  D9 D10  D11  // ArdioShield LNET 1.2
      ARDIO12_B,            //      D12 D13 A0  A1   A2  A3 D3   D4   // ArdioShield LNET 1.2
	// undocumented - special obsolete board with custom pinout
      ARDIO13_A,            //      D2  D3  D4  D5   D6  D9 D10  D11  // ArdioShield LNET 1.3
      ARDIO13_B,            //      D12 D13 A0  A1   A2  A3 A6   A7   // ArdioShield LNET 1.3 (A6,A7 analog IN only!)
#endif
#if defined(SPARK_CORE)    // Built in Photon Ports
      PHOTON_A,             //      D2, D3, D4,  D5,  -- -- -- --
      PHOTON_B,             //      D6, D7, A0,  A1,  -- -- -- --
      PHOTON_C,             //      A2, A3, DAC, WKP, -- -- -- --
#endif
#if defined(ARDUINO_ESP8266_OAK)
	  OAK_A,	            // 		GPIO 13, 12, 14, 16     Pins 7,8,9,10
	  OAK_B,	            //		GPIO  5,  4, TX, RX     Pins 1,5,4,3
	  OAK_C,	            //		GPIO 15, 17     	    Pins 6,11       // no pullups allowed on pin 6/GPIO15
#endif
	  
    };
    enum PCA9555Registers {
      REGISTER_9555INPUT  =  0,
      REGISTER_9555OUTPUT =  2,
      REGISTER_9555INVERT =  4,
      REGISTER_9555CONFIG =  6
    };

    enum MAX7313Resisters {
    };

    enum BaseAddress {
      base7313     = 0x10,  // has 2x contiguous address ranges: 0x10-0x2F and 0x50-0x6F, for 64x chips...
      base9555     = 0x20,
      base23016    = 0x20,
      base8574A    = 0x38,
      base8574     = 0x20,
      base8591     = 0x48
    };
    
 private:
    uint8_t  _size;
    uint8_t  _chip;
    uint8_t  _address;
    uint8_t  _i2c_address;
    uint16_t _config;   
    uint16_t _current;
	uint16_t _last, _lastw;
	boolean _debounce;
    uint16_t _read(void) ;
    
    void        init8      (uint8_t i2caddr, uint16_t dir);
    void        write8     (uint16_t data);
    uint16_t    read8      (void);

    void        init9555     (uint8_t i2caddr, uint16_t dir);
    void        write9555    (uint16_t data);
    uint16_t    read9555     (void);

    void        init7313     (uint8_t i2caddr, uint16_t dir);
    void        write7313    (uint16_t data);
    uint16_t    read7313     (void);

    void        init8591    (uint8_t i2caddr, uint16_t dir);
    void        write8591   (uint16_t data);
    uint16_t    read8591    (void);
	void        writeif    (uint8_t port,    uint16_t data, uint8_t bit);

#if defined(ARDUINO_AVR_DUEMILANOVE)
    void        initArduino(void);
    uint16_t    readArduino(void);
    void        writeArduino(uint16_t data);
#endif
#if defined(SPARK_CORE)    // Built in Photon Ports
    void        initPhoton (void);
    uint16_t    readPhoton (void);
    void        writePhoton(uint16_t data);
#endif
#if defined(ARDUINO_ESP8266_OAK)
    void        initOAK (void);
    uint16_t    readOAK (void);
    void        writeOAK(uint16_t data);
#endif

};

#endif // I2Cexpander_h

