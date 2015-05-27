
/*
 * I2Cexpander.h - Library for interfacing to various IO Expander chips
 * Copyright (c) 2011 John Plocher, Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
 */

#ifndef I2Cexpander_h
#define I2Cexpander_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include <Wire.h>

#elif defined(SPARK_CORE)

#include "application.h"
#include <stdint.h>

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
    void     init(uint16_t address, uint16_t type, uint16_t config, boolean debounce=false);
    uint16_t read(void) ;
    void     write(uint16_t data);
    
    uint16_t get(void)          { return I2Cexpander::read(); }
    void     put(uint16_t data) { I2Cexpander::write(data);  next = data; }
    void     put(void)          { I2Cexpander::write(next);   }
    
    uint16_t getSize(void)      { return _size; };  
    uint16_t current()          { return I2Cexpander::_current; };
    uint16_t last()             { return I2Cexpander::_last;    };
    uint16_t i2caddr()          { return (_i2c_address); };
    boolean  changed()          { return ((I2Cexpander::_current & I2Cexpander::_config) != (I2Cexpander::_last & I2Cexpander::_config)); };
    uint16_t next;

    enum IOSize {
      B_UNKNOWN =  0,
      B4        =  4,
      B6        =  6,
      B8        =  8,
      B16       = 16,
      B32       = 32,
      T_UNKNOWN =  0
    };

    enum ExpanderType {
      PCA9555   =  1,
      MCP23016  =  2,
      PCF8574   =  3,
      PCF8574A  =  4,
      PCF8591   =  5,	// 4 A-D converters, 1 D-A

	// built-in Arduino ports, 
      ARDIO_A   =  10,   // Bits D0  D1  D2  D3  D4  D5   D6  D7 - low digital
      ARDIO_B   =  11,   //      D8  D9  D10 D11 D12 D13 --  --  - high digital
      ARDIO_C   =  12,   //      A0  A1  A2  A3  A4  A5  A6  A7  - analog
	// undocumented - special obsolete board with custom pinout
      ARDIO12_A =  13,   //      D0  D1  D5  D6   D2  D9 D10  D11  // ArdioShield LNET 1.2
      ARDIO12_B =  14,   //      D12 D13 A0  A1   A2  A3 D3   D4   // ArdioShield LNET 1.2
	// undocumented - special obsolete board with custom pinout
      ARDIO13_A =  15,   //      D2  D3  D4  D5   D6  D9 D10  D11  // ArdioShield LNET 1.3
      ARDIO13_B =  16,   //      D12 D13 A0  A1   A2  A3 A6   A7   // ArdioShield LNET 1.3 (A6,A7 analog IN only!) 
        // Built in Photon Ports
      PHOTON_A  =  20,   //      D2, D3, D4,  D5,  -- -- -- --
      PHOTON_B  =  21,   //      D6, D7, A0,  A1,  -- -- -- --
      PHOTON_C  =  22,   //      A2, A3, DAC, WKP, -- -- -- --
    };
    enum PCA9555Registers {
      REGISTER_INPUT  =  0,
      REGISTER_OUTPUT =  2,
      REGISTER_INVERT =  4,
      REGISTER_CONFIG =  6
    };
    enum BaseAddress {
      base9555     = 0x20,
      base23016    = 0x20,
      base8574A    = 0x38,
      base8574     = 0x20,
      base8591     = 0x48
    };
    
 private:
    uint16_t _size;
    uint16_t _chip;
    uint16_t _address;
    uint16_t _i2c_address;
    uint16_t _config;   
    uint16_t _current;
    uint16_t _last;
    boolean _debounce;
    uint16_t _read(void) ;
    
    void        init8     (uint16_t i2caddr, uint16_t dir);
    void        init16    (uint16_t i2caddr, uint16_t dir);
    void        initA     (uint16_t dir);
    void        initB     (uint16_t dir);
    void        initC     (uint16_t dir);
    void        init12A   (uint16_t dir);
    void        init12B   (uint16_t dir);
    void        init13A   (uint16_t dir);
    void        init13B   (uint16_t dir);
    void        initPhoton(uint16_t dir, uint16_t chip);
    
    void        write16   (uint16_t i2caddr, uint16_t data);
    void        write8    (uint16_t i2caddr, uint16_t data);
    void        writeDAC  (uint16_t i2caddr, uint16_t data);
    void        writeif   (uint16_t port,    uint16_t data, uint16_t bit);
    uint16_t    readADC   (uint16_t i2caddr);
    uint16_t    read8     (uint16_t i2caddr);
    uint16_t    read16    (uint16_t i2caddr);
};

#endif // I2Cexpander_h


