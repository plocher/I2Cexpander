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
 *      MAX731x
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
#define bitRead(value, bit)            (((uint16_t)(value) >> (bit)) & 0x01)
#define bitSet(value, bit)             ((value) |= (uint16_t)(1UL << (bit)))
#define bitClear(value, bit)           ((value) &=(uint16_t)( ~(1UL << (bit))))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#endif

#endif

class I2Cexpander {
public:
    I2Cexpander(void);
    void     init(uint16_t address, uint16_t chip, uint16_t config, boolean debounce=false);

    void     digitalWrite(uint8_t dataPin, uint8_t val);
    uint8_t  digitalRead(uint8_t dataPin);

    uint32_t read(void) ;
    void     write(uint32_t data);
    void     write(void)        { I2Cexpander::write(next);   }
    uint32_t get(void)          { return I2Cexpander::read(); }
    void     put(uint32_t data) { next = data; I2Cexpander::write(data); }
    void     put(void)          { I2Cexpander::write(next);   }
    
    uint16_t getSize(void)      { return I2Cexpander::_size; };
    uint32_t current()          { return I2Cexpander::_current; };
    uint32_t last()             { return I2Cexpander::_last;    };
    uint16_t config()           { return I2Cexpander::_config; };
    uint16_t chip()             { return I2Cexpander::_chip; };
    uint8_t  i2caddr()          { return I2Cexpander::_i2c_address; };
    boolean  changed()          { if (_firsttime) { _firsttime = 0; _last = ~_current; } // force changed() to trigger
                                  if ((_chip == PCF8591) || (_chip == PCA9685)) {
									  return ((I2Cexpander::_current) != (I2Cexpander::_last));
								  }
								  return ((I2Cexpander::_current & I2Cexpander::_config) != (I2Cexpander::_last & I2Cexpander::_config));
                                };
    uint16_t next;
    byte     debugflag;
    enum IOSize {
      B_UNKNOWN =  0,
      B2        =  2,
      B4        =  4,
      B6        =  6,
      B8        =  8,
      B16       = 16,
      B32       = 32,
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
      MAX731x,              // Bits 0  1  2  3  4  5  6  7  8   9  10  11  12  13  14  15  16
	  MAX7311 = MAX731x,
	  MAX7312 = MAX731x,
	  MAX7313 = MAX731x,
	// LED PWM Controller
	  PCA9685,
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
#if defined(SPARK_CORE)     // Built in Photon Ports
      PHOTON_A,                    //      D2, D3, D4,  D5,  -- -- -- --
      PHOTON_B,                    //      D6, D7, A0,  A1,  -- -- -- --
      PHOTON_C,                    //      A2, A3, DAC, WKP, -- -- -- --
#endif
#if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
      WEMOS_A,                    //      GPIO   4,  0,  2, 14    Pins D2 D3 D4 D5
      WEMOS_B,                    //      GPIO  12, 13,  3,  1    Pins D6 D7 RX TX
      WEMOS_MATRIX,               //      GPIO   4,  2, 14, 12    Pins D3 D4 D5 D6
      WEMOS,                      //      GPIO  16, 13,  3,  1    Pins D0 D7 RX TX
#endif
      
    };
    enum PCA9555Registers {
		PCA9555_INPUT  =  0,
		PCA9555_OUTPUT =  2,
		PCA9555_INVERT =  4,
		PCA9555_CONFIG =  6
    };

    enum MAX731xRegisters {
    };
	
    enum PCA8591Registers {
		PCA8591_Channel1 = 0x00,
		PCA8591_Channel2,
		PCA8591_Channel3,
		PCA8591_Channel4
    };
	
	
	enum PCA9685Registers {
		PCA9685_MODE1           = 0x00,
		PCA9685_MODE2           = 0x01,
		PCA9685_BASE_LED0       = 0x06,  // 4 bytes, 12 bits of LED ON: +0, +1, 12 bits of LED OFF: +2 and +3

		// Mode 1 bits
		PCA9685_MODE1_RESTART   = 0x80,
		PCA9685_MODE1_EXTCLK    = 0x40,
		PCA9685_MODE1_AUTOINC   = 0x20,
		PCA9685_MODE1_SLEEP     = 0x10,
		PCA9685_MODE1_SUBADR1   = 0x08,
		PCA9685_MODE1_SUBADR2   = 0x04,
		PCA9685_MODE1_SUBADR3   = 0x02,
		PCA9685_MODE1_ALLCALL   = 0x01,

		// Mode 2 bits
		PCA9685_MODE2_INVERT    = 0x10,
		PCA9685_MODE2_ONACK     = 0x08,
		PCA9685_MODE2_TOTEM     = 0x04,
		PCA9685_MODE2_OEHIZ     = 0x02,
		PCA9685_MODE2_OEDRV     = 0x01,
		PCA9685_MODE2_OEOFF     = 0x00,
		
		PCA9685_LED0			= 0x00,	// 12 bit values
		PCA9685_LED1,
		PCA9685_LED2,
		PCA9685_LED3,
		PCA9685_LED4,
		PCA9685_LED5,
		PCA9685_LED6,
		PCA9685_LED7,
		PCA9685_LED8,
		PCA9685_LED9,
		PCA9685_LED10,
		PCA9685_LED11,
		PCA9685_LED12,
		PCA9685_LED13,
		PCA9685_LED14,
		PCA9685_LED15
	};

    enum BaseAddress {
      base731x     = 0x10,  // has 2x contiguous address ranges: 0x10-0x2F and 0x50-0x6F, for 64x chips...
      base9555     = 0x20,
      base23016    = 0x20,
      base8574A    = 0x38,
      base8574     = 0x20,
      base8591     = 0x48,
	  base9685     = 0x40
    };
    
 //private:
    uint8_t  _size;
    uint8_t  _chip;
    uint8_t  _address;
    uint8_t  _i2c_address;
    uint16_t _config;   
    uint32_t _current;
    uint32_t _last, _lastw;
    bool     _firsttime;
    boolean _debounce;
	
	void printData(uint32_t data);
	void printString(const char *tag);
	
    uint32_t _read(void) ;
    
    void        init8      (uint8_t i2caddr, uint16_t dir);
    void        write8     (uint32_t data);
    uint32_t    read8      (void);

    void        init9555     (uint8_t i2caddr, uint16_t dir);
    void        write9555    (uint32_t data);
    uint32_t    read9555     (void);

    void        init9685     (uint8_t i2caddr, uint16_t dir);
    void        write9685    (uint32_t data);
    uint32_t    read9685     (void);

    void        init731x     (uint8_t i2caddr, uint16_t dir);
    void        write731x    (uint32_t data);
    uint32_t    read731x     (void);

    void        init8591    (uint8_t i2caddr, uint16_t dir);
    void        write8591   (uint32_t data);
    uint32_t    read8591    (void);
    uint32_t    Xread8591    (void);
    void        writeif    (uint8_t port,    uint32_t data, uint8_t bit);

#if defined(ARDUINO_AVR_DUEMILANOVE)
    void        initArduino(void);
    uint32_t    readArduino(void);
    void        writeArduino(uint32_t data);
#endif
#if defined(SPARK_CORE)    // Built in Photon Ports
    void        initPhoton (void);
    uint32_t    readPhoton (void);
    void        writePhoton(uint32_t data);
#endif
#if defined(ARDUINO_ESP8266_OAK)
    void        initOAK (void);
    uint32_t    readOAK (void);
    void        writeOAK(uint32_t data);
#endif
#if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
    void        initWemos (void);
    uint32_t    readWemos (void);
    void        writeWemos(uint32_t data);
#endif

};

#endif // I2Cexpander_h

