// IO Expander wrapper library
// Copyright (c) 2011 John Plocher, Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)

#include "I2Cexpander.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include <Wire.h>
#elif defined(SPARK)
#include "application.h"
#endif

// #define I2C_EXTENDER_DEBUG
// #define I2C_EXTENDER_INVERTLOCAL  - writing a "1" puts port ON (@5v) rather than OFF @0v

I2Cexpander::I2Cexpander() {
    _address = -1;
    _chip = -1;
    _config = -1;   
    _last = -1;
    _current = -2;
    _size = B_UNKNOWN;
}

void I2Cexpander::init(uint16_t address, uint16_t chip, uint16_t config, boolean debounce /* == false */ ) {
#ifdef I2C_EXTENDER_DEBUG
    Serial.print("I2C:init(");
    Serial.print(address, DEC); Serial.print(", ");
    Serial.print(chip, DEC); Serial.print(", ");
    Serial.print(config, DEC);
    Serial.print(")\n");
#endif  
    _address = address;
    _chip = chip;
    _config = config;
    _i2c_address = -1; // default
    _debounce = debounce;
    switch (_chip) {
	case I2Cexpander::PCA9555:    _size = B16; init16(base9555  + _address, _config);  break;
	case I2Cexpander::MCP23016:   _size = B16; init16(base23016 + _address, _config);  break;
	case I2Cexpander::PCF8574A:   _size = B8;  init8(base8574A  + _address, _config);  break;
	case I2Cexpander::PCF8574:    _size = B8;  init8(base8574   + _address, _config);  break;
	case I2Cexpander::PCF8591:    _size = B8;  break;  // config is which ADC ...

	case I2Cexpander::ARDIO_A:    _size = B8;  initA(_config);                         break;
	case I2Cexpander::ARDIO_B:    _size = B6;  initB(_config);                         break;
	case I2Cexpander::ARDIO_C:    _size = B8;  _config |= 0x03; initC(_config);   break;
	
	case I2Cexpander::ARDIO12_A:  _size = B8;  init12A(_config);                       break;
	case I2Cexpander::ARDIO12_B:  _size = B8;  init12B(_config);                       break;
	case I2Cexpander::ARDIO13_A:  _size = B8;  init13A(_config);                       break;
	case I2Cexpander::ARDIO13_B:  _size = B8;  _config |= 0xC0; init13B(_config); break;
	case I2Cexpander::PHOTON_A:
	case I2Cexpander::PHOTON_B:
	case I2Cexpander::PHOTON_C:   _size = B4;  initPhoton(_config, _chip); break;
	default:
	    _size=0; break;
    }
}

uint16_t I2Cexpander::read(void) {  
    uint16_t v1 = _read();
    if (!_debounce) return v1;
    uint16_t v2 = v1;
    do {
	v1 = v2;
	v2 = _read();
    } while (v2 != v1);
    return v1;
}
	
uint16_t I2Cexpander::_read() {  
    uint16_t data = 0;
    int error = 0;
#ifdef I2C_EXTENDER_DEBUG
    Serial.print("I2C:read(a=0x"); Serial.print(_address, HEX); 
    Serial.print(", chip=");       Serial.print(_chip,    DEC); 
    Serial.print(", conf=B");      Serial.print(_config,  BIN);
    Serial.print(") "); 
#endif
    switch (_chip) {
        case I2Cexpander::PCA9555:        data = read16(base9555  + _address);   break;
        case I2Cexpander::MCP23016:       data = read16(base23016 + _address);   break;
        case I2Cexpander::PCF8574A:       data = read8(base8574A  + _address);   break;
        case I2Cexpander::PCF8574:        data = read8(base8574   + _address);   break;
        case I2Cexpander::PCF8591:        data = readADC(base8591 + _address);   break;

        case I2Cexpander::ARDIO_A:        
                                          bitWrite(data,0, digitalRead(0));
                                          bitWrite(data,1, digitalRead(1));
#if !defined(__AVR_ATmega32U4__)        // don't mess with I2C pins!
                                          bitWrite(data,2, digitalRead(2));
                                          bitWrite(data,3, digitalRead(3));
#endif
                                          bitWrite(data,4, digitalRead(4));
                                          bitWrite(data,5, digitalRead(5));
                                          bitWrite(data,6, digitalRead(6));
                                          bitWrite(data,7, digitalRead(7));
					break;
        case I2Cexpander::ARDIO_B:        
                                          bitWrite(data,0, digitalRead(8));
                                          bitWrite(data,1, digitalRead(9));
                                          bitWrite(data,2, digitalRead(10));
                                          bitWrite(data,3, digitalRead(11));
                                          bitWrite(data,4, digitalRead(12));
                                          bitWrite(data,5, digitalRead(13));
					break;
        case I2Cexpander::ARDIO_C:        
                                          bitWrite(data,0, digitalRead(A0));
                                          bitWrite(data,1, digitalRead(A1));
                                          bitWrite(data,2, digitalRead(A2));
                                          bitWrite(data,3, digitalRead(A3));
#if defined(__AVR_ATmega32U4__)         // don't mess with I2C pins!
                                          bitWrite(data,4, digitalRead(A4));
                                          bitWrite(data,5, digitalRead(A5));
#else
#if defined(A6)                        // A6 and A7 are analog only inputs, treat as digital...
                                          bitWrite(data,4, (analogRead(A6) > 100) ? 1 : 0);
					  bitWrite(data,5, (analogRead(A7) > 100) ? 1 : 0);
#endif
#endif
					break;
        
        case I2Cexpander::ARDIO12_A:        
                                          bitWrite(data,0, digitalRead(0));
                                          bitWrite(data,1, digitalRead(1));
                                          bitWrite(data,2, digitalRead(5));
                                          bitWrite(data,3, digitalRead(6));
#if !defined(__AVR_ATmega32U4__)        // don't mess with I2C pins!
                                          bitWrite(data,4, digitalRead(2));
#else
                                          bitWrite(data,4, digitalRead(A4));
#endif
                                          bitWrite(data,5, digitalRead(9));
                                          bitWrite(data,6, digitalRead(10));
                                          bitWrite(data,7, digitalRead(11));

					break;
        case I2Cexpander::ARDIO12_B:        
                                          bitWrite(data,0, digitalRead(12));
                                          bitWrite(data,1, digitalRead(13));
                                          bitWrite(data,2, digitalRead(A0));
                                          bitWrite(data,3, digitalRead(A1));

                                          bitWrite(data,4, digitalRead(A2));
                                          bitWrite(data,5, digitalRead(A3));
#if !defined(__AVR_ATmega32U4__)        // don't mess with I2C pins!
                                          bitWrite(data,6, digitalRead(3));
#else
                                          bitWrite(data,6, digitalRead(A5));
#endif
                                          bitWrite(data,7, digitalRead(4));

					break;
        case I2Cexpander::ARDIO13_A:        
                                          bitWrite(data,0, digitalRead(2));
                                          bitWrite(data,1, digitalRead(3));
                                          bitWrite(data,2, digitalRead(4));
                                          bitWrite(data,3, digitalRead(5));

                                          bitWrite(data,4, digitalRead(6));
                                          bitWrite(data,5, digitalRead(9));
                                          bitWrite(data,6, digitalRead(10));
                                          bitWrite(data,7, digitalRead(11));

					break;
        case I2Cexpander::ARDIO13_B:        
                                          bitWrite(data,0, digitalRead(12));
                                          bitWrite(data,1, digitalRead(13));
                                          bitWrite(data,2, digitalRead(A0));
                                          bitWrite(data,3, digitalRead(A1));
                                        
                                          bitWrite(data,4, digitalRead(A2));
                                          bitWrite(data,5, digitalRead(A3));
                                          bitWrite(data,6, (analogRead(A6) > 100) ? 1 : 0);
                                          bitWrite(data,7, (analogRead(A7) > 100) ? 1 : 0);

					break;
        case I2Cexpander::PHOTON_A:        
                                          bitWrite(data,0, digitalRead(2));
                                          bitWrite(data,1, digitalRead(3));
                                          bitWrite(data,2, digitalRead(4));
                                          bitWrite(data,3, digitalRead(5));
					break;
        case I2Cexpander::PHOTON_B:        
                                          bitWrite(data,0, digitalRead(6));
                                          bitWrite(data,1, digitalRead(7));
                                          bitWrite(data,2, digitalRead(A0));
                                          bitWrite(data,3, digitalRead(A1));
					break;
        case I2Cexpander::PHOTON_C:        
                                          bitWrite(data,0, digitalRead(A2));
                                          bitWrite(data,1, digitalRead(A3));
                                          bitWrite(data,2, digitalRead(A6));
                                          bitWrite(data,3, digitalRead(A7));
					break;

        default:                          error = 1;                            break;
    } 
    if (!error) {      
        I2Cexpander::_last = I2Cexpander::_current;
        I2Cexpander::_current = data;
    } 
#ifdef I2C_EXTENDER_DEBUG
    Serial.print(" => "); 
    if (error)              Serial.print("Error"); 
    else if (_size == B8)   Serial.print((byte)data, BIN); 
    else if (_size == B16)  Serial.print((uint16_t)data, BIN); 
    else                   {Serial.print("unknown data size: "); Serial.print(data, BIN);}
    Serial.print("\n");  
#endif
    return data;
}
                       
// only write a bit if the config regiaster allows writing to it
void I2Cexpander::writeif(uint16_t port, uint16_t data, uint16_t bit) {
    boolean mybit = bitRead(data, bit);
#ifdef I2C_EXTENDER_INVERTLOCAL
    mybit = !mybit;
#endif
#ifdef I2C_EXTENDER_DEBUG
    Serial.print("    I2C:writeif(p=");
    Serial.print(port, DEC); 
    Serial.print(", data=");
    if (_size == B8)          Serial.print((byte)data,     BIN); 
	else if (_size == B8)     Serial.print((byte)data,     BIN); 
    else if (_size == B16)    Serial.print((uint16_t)data, BIN); 
    else                   {  Serial.print(data, BIN); Serial.print("?");}
    Serial.print(", bit=");
    Serial.print(bit, DEC);
    Serial.print(") => ");
    Serial.print(mybit, BIN);
    Serial.print("\n");
    digitalWrite(port, mybit);
#else   
    if (bitRead(_config, bit) == 0) digitalWrite(port, mybit);
#endif
}
void I2Cexpander::write(uint16_t data) {
#ifdef I2C_EXTENDER_DEBUG
    Serial.print("I2C:write("); 
    Serial.print(   "a=0x");   Serial.print(_address,   HEX); 
    Serial.print(", chip=");   Serial.print(_chip,      DEC); 
    Serial.print(", conf=");   Serial.print(_config,    DEC);
    Serial.print(") => ");  
    if (_size == B8)          Serial.print((byte)data,     BIN); 
    else if (_size == B8)     Serial.print((byte)data,     BIN); 
    else if (_size == B16)    Serial.print((uint16_t)data, BIN); 
    else                   {  Serial.print(data, BIN); Serial.print("?");}
    Serial.print("\n");
#endif
    
    switch (_chip) {
    case I2Cexpander::PCA9555:        write16(base9555  + _address, data | _config); break; 
    case I2Cexpander::MCP23016:       write16(base23016 + _address, data | _config); break; 
    case I2Cexpander::PCF8574A:       write8 (base8574A + _address, data | _config); break;
    case I2Cexpander::PCF8574:        write8 (base8574  + _address, data | _config); break;
    case I2Cexpander::PCF8591:        writeDAC(base8591 + _address, data);           break;

    case I2Cexpander::ARDIO_A:        
                                      writeif( 0, data, 0);
                                      writeif( 1, data, 1);
#if !defined(__AVR_ATmega32U4__)    // don't mess with I2C pins!
                                      writeif( 2, data, 2);
                                      writeif( 3, data, 3);
#endif
                                      writeif( 4, data, 4);
                                      writeif( 5, data, 5);
                                      writeif( 6, data, 6);
                                      writeif( 7, data, 7);
                                                                                    break;
    case I2Cexpander::ARDIO_B:        
                                      writeif( 8, data, 0);
                                      writeif( 9, data, 1);
                                      writeif(10, data, 2);
                                      writeif(11, data, 3);
                                      writeif(12, data, 4);
                                      writeif(13, data, 5);
                                                                                    break;
    case I2Cexpander::ARDIO_C:        
                                      writeif(A0, data, 0);
                                      writeif(A1, data, 1);
                                      writeif(A2, data, 2);
                                      writeif(A3, data, 3);
#if defined(__AVR_ATmega32U4__)     // don't mess with I2C pins!
                                      writeif(A4, data, 4);
                                      writeif(A5, data, 5);
#endif
				      // Can not write to A6 and A7
                                                                                    break;

    case I2Cexpander::ARDIO12_A:        
                                      writeif( 0, data, 0);
                                      writeif( 1, data, 1);
                                      writeif( 5, data, 2);
                                      writeif( 6, data, 3);

#if !defined(__AVR_ATmega32U4__)    // don't mess with I2C pins!
                                      writeif( 2, data, 4);
#else
                                      writeif(A4, data, 4);
#endif
                                      writeif( 9, data, 5);
                                      writeif(10, data, 6);
                                      writeif(11, data, 7);
                                                                                    break;

    case I2Cexpander::ARDIO12_B:        
                                      writeif(12, data, 0);
                                      writeif(13, data, 1);
                                      writeif(A0, data, 2);
                                      writeif(A1, data, 3);

                                      writeif(A2, data, 4);
                                      writeif(A3, data, 5);
#if !defined(__AVR_ATmega32U4__)    // don't mess with I2C pins!
                                      writeif( 3, data, 6);
#else
                                      writeif(A5, data, 6);
#endif
                                      writeif( 4, data, 7);
                                                                                    break;
    case I2Cexpander::ARDIO13_A:        
                                      writeif( 2, data, 0);
                                      writeif( 3, data, 1);
                                      writeif( 4, data, 2);
                                      writeif( 5, data, 3);

                                      writeif( 6, data, 4);
                                      writeif( 9, data, 5);
                                      writeif(10, data, 6);
                                      writeif(11, data, 7);
                                                                                    break;

    case I2Cexpander::ARDIO13_B:        
                                      writeif(12, data, 0);
                                      writeif(13, data, 1);
                                      writeif(A0, data, 2);
                                      writeif(A1, data, 3);
                                    
                                      writeif(A2, data, 4);
                                      writeif(A3, data, 5);
                                      // A6 and A7 are input only analog pins
																					break;
    case I2Cexpander::PHOTON_A:        
                                      writeif( 2, data, 0);
                                      writeif( 3, data, 1);
                                      writeif( 4, data, 2);
                                      writeif( 5, data, 3);
                                                                                    break;
    case I2Cexpander::PHOTON_B:        
                                      writeif( 6, data, 0);
                                      writeif( 7, data, 1);
                                      writeif(A0, data, 2);
                                      writeif(A1, data, 3);
                                                                                    break;
    case I2Cexpander::PHOTON_C:        
                                      writeif(A2, data, 0);
                                      writeif(A3, data, 1);
                                      writeif(A6, data, 2);
                                      writeif(A7, data, 3);
                                                                                    break;

    default:                                                                        break;
    }
    //I2Cexpander::_last = I2Cexpander::_current;                                   
    //I2Cexpander::_current = data | _config;
}



// ------------------------------------------

void I2Cexpander::init8(uint16_t i2caddr, uint16_t dir) {
	_i2c_address = i2caddr;
	write8(i2caddr, dir);
}

uint16_t I2Cexpander::read8(uint16_t i2caddr) {
    uint16_t _data = -1;
    // Wire.beginTransmission(i2caddr);
    Wire.requestFrom(i2caddr, 1);
    if(Wire.available()) {
      _data = Wire.read();
    }
    // Wire.endTransmission();  
    return _data;
}

void I2Cexpander::write8(uint16_t i2caddr, uint16_t data)
{ 
    Wire.beginTransmission(i2caddr);
    Wire.write(0xff & data);
    Wire.endTransmission();  
}


// ------------------------------------------

uint16_t I2Cexpander::read16(uint16_t i2caddr) {
    uint16_t data = 0;
    Wire.beginTransmission(i2caddr);
    Wire.write(REGISTER_INPUT);
    Wire.endTransmission();  
    // Wire.beginTransmission(i2caddr);
    Wire.requestFrom(i2caddr, 2);
    if(Wire.available()) {
        data = Wire.read();
    }
    if(Wire.available()) {
        data |= (Wire.read() << 8);  
    }
    // Wire.endTransmission();  
    return data;
}


void I2Cexpander::write16(uint16_t i2caddr, uint16_t data) {
    Wire.beginTransmission(i2caddr);
    Wire.write(REGISTER_OUTPUT);
    Wire.write(0xff & data);  //  low byte
    Wire.write(data >> 8);    //  high byte
    Wire.endTransmission();  
}

// ------------------------------------------

uint16_t I2Cexpander::readADC(uint16_t i2caddr) {
#ifdef I2C_EXTENDER_DEBUG
    Serial.print("readADC(a=0x");	Serial.print(i2caddr, HEX);
    Serial.print(", Channel=0x");  	Serial.print(_config, HEX);
    Serial.print(") => ");
#endif
    int _d1 = -1;
    int _d2 = -1;
    Wire.beginTransmission(i2caddr);
    Wire.write(_config);
    Wire.endTransmission();
    Wire.requestFrom(i2caddr, 2);
    // Why two bytes? The PCF8591 returns the previously measured value first – then the current byte.
    _d1 = Wire.read();
    _d2 = Wire.read();  // ignore the first byte received (see above)
    // Wire.endTransmission();  
#ifdef I2C_EXTENDER_DEBUG
    Serial.print("old: ");;
    Serial.print(_d1, DEC);
    Serial.print(", new: ");
    Serial.print(_d2, DEC);Serial.print(" ");
#endif
    return (uint16_t)_d2;
}

void I2Cexpander::writeDAC(uint16_t i2caddr, uint16_t data)
{ 
    Wire.beginTransmission(i2caddr);
    Wire.write(0x40);
    Wire.write(0xff & data);
    Wire.endTransmission();  
}


void I2Cexpander::init16(uint16_t i2caddr, uint16_t dir) {
	_i2c_address = i2caddr;
    Wire.beginTransmission(i2caddr);
    Wire.write(REGISTER_CONFIG);
    Wire.write(0xff & dir);  // low byte
    Wire.write(dir >> 8);    // high byte
    Wire.endTransmission();  
}

// set Arduino I/O pin direction (1=OUTPUT, 0-INPUT)
void I2Cexpander::initA(uint16_t dir) {
    pinMode(0,  bitRead(dir, 0) ? INPUT_PULLUP : OUTPUT);
    pinMode(1,  bitRead(dir, 1) ? INPUT_PULLUP : OUTPUT);
#if !defined(__AVR_ATmega32U4__)    // don't mess with I2C pins on UNO!
    pinMode(2,  bitRead(dir, 2) ? INPUT_PULLUP : OUTPUT);
    pinMode(3,  bitRead(dir, 3) ? INPUT_PULLUP : OUTPUT);
#endif
    pinMode(4,  bitRead(dir, 4) ? INPUT_PULLUP : OUTPUT);
    pinMode(5,  bitRead(dir, 5) ? INPUT_PULLUP : OUTPUT);
    pinMode(6,  bitRead(dir, 6) ? INPUT_PULLUP : OUTPUT);
    pinMode(7,  bitRead(dir, 7) ? INPUT_PULLUP : OUTPUT);
}
void I2Cexpander::initB(uint16_t dir) {
    pinMode(8,  bitRead(dir, 0) ? INPUT_PULLUP : OUTPUT);
    pinMode(9,  bitRead(dir, 1) ? INPUT_PULLUP : OUTPUT);
    pinMode(10, bitRead(dir, 2) ? INPUT_PULLUP : OUTPUT);
    pinMode(11, bitRead(dir, 3) ? INPUT_PULLUP : OUTPUT);

    pinMode(12, bitRead(dir, 4) ? INPUT_PULLUP : OUTPUT);
    pinMode(13, bitRead(dir, 5) ? INPUT_PULLUP : OUTPUT);
}
void I2Cexpander::initC(uint16_t dir) {
    pinMode(A0, bitRead(dir, 0) ? INPUT_PULLUP : OUTPUT);
    pinMode(A1, bitRead(dir, 1) ? INPUT_PULLUP : OUTPUT);
    pinMode(A2, bitRead(dir, 2) ? INPUT_PULLUP : OUTPUT);
    pinMode(A3, bitRead(dir, 3) ? INPUT_PULLUP : OUTPUT);
#if defined(__AVR_ATmega32U4__)     // don't mess with I2C pins on LEO!
    pinMode(A4, bitRead(dir, 4) ? INPUT_PULLUP : OUTPUT);
    pinMode(A5, bitRead(dir, 5) ? INPUT_PULLUP : OUTPUT);
#endif
}

// set Arduino I/O pin direction (1=OUTPUT, 0-INPUT)
void I2Cexpander::init12A(uint16_t dir) {
    pinMode(0,  bitRead(dir, 0) ? INPUT : OUTPUT);
    pinMode(1,  bitRead(dir, 1) ? INPUT : OUTPUT);
    pinMode(5,  bitRead(dir, 2) ? INPUT : OUTPUT);
    pinMode(6,  bitRead(dir, 3) ? INPUT : OUTPUT);
    
#if !defined(__AVR_ATmega32U4__)    // don't mess with I2C pins!
    pinMode(2,  bitRead(dir, 4) ? INPUT : OUTPUT);
#else
    pinMode(A4, bitRead(dir, 4) ? INPUT : OUTPUT);
#endif
    pinMode(9,  bitRead(dir, 5) ? INPUT : OUTPUT);
    pinMode(10, bitRead(dir, 6) ? INPUT : OUTPUT);
    pinMode(11, bitRead(dir, 7) ? INPUT : OUTPUT);
}
void I2Cexpander::init12B(uint16_t dir) {
    pinMode(12, bitRead(dir, 0) ? INPUT : OUTPUT);
    pinMode(13, bitRead(dir, 1) ? INPUT : OUTPUT);
    pinMode(A0, bitRead(dir, 2) ? INPUT : OUTPUT);
    pinMode(A1, bitRead(dir, 3) ? INPUT : OUTPUT);

    pinMode(A2, bitRead(dir, 4) ? INPUT : OUTPUT);
    pinMode(A3, bitRead(dir, 5) ? INPUT : OUTPUT);

#if !defined(__AVR_ATmega32U4__)    // don't mess with I2C pins!
    pinMode(3,  bitRead(dir, 6) ? INPUT : OUTPUT);
#else
    pinMode(A5, bitRead(dir, 6) ? INPUT : OUTPUT);
#endif
    pinMode(4,  bitRead(dir, 7) ? INPUT : OUTPUT);
}
// set pro-MINI Arduino I/O pin direction (1=OUTPUT, 0-INPUT)
void I2Cexpander::init13A(uint16_t dir) {
    pinMode(2,  bitRead(dir, 0) ? INPUT : OUTPUT);
    pinMode(3,  bitRead(dir, 1) ? INPUT : OUTPUT);
    pinMode(4,  bitRead(dir, 2) ? INPUT : OUTPUT);
    pinMode(5,  bitRead(dir, 3) ? INPUT : OUTPUT);

    pinMode(6,  bitRead(dir, 4) ? INPUT : OUTPUT);
    pinMode(9,  bitRead(dir, 5) ? INPUT : OUTPUT);
    pinMode(10, bitRead(dir, 6) ? INPUT : OUTPUT);
    pinMode(11, bitRead(dir, 7) ? INPUT : OUTPUT);
}
void I2Cexpander::init13B(uint16_t dir) {
    pinMode(12, bitRead(dir, 0) ? INPUT : OUTPUT);
    pinMode(13, bitRead(dir, 1) ? INPUT : OUTPUT);
    pinMode(A0, bitRead(dir, 2) ? INPUT : OUTPUT);
    pinMode(A1, bitRead(dir, 3) ? INPUT : OUTPUT);

    pinMode(A2, bitRead(dir, 4) ? INPUT : OUTPUT);
    pinMode(A3, bitRead(dir, 5) ? INPUT : OUTPUT);
    // A6 and A7 are Analog IN, pinmode doesn't work with them
}

void I2Cexpander::initPhoton(uint16_t dir, uint16_t portset) {
    switch (portset) {
        case I2Cexpander::PHOTON_A:
            pinMode(2,  bitRead(dir, 0) ? INPUT : OUTPUT);
            pinMode(3,  bitRead(dir, 1) ? INPUT : OUTPUT);
            pinMode(4,  bitRead(dir, 2) ? INPUT : OUTPUT);
            pinMode(5,  bitRead(dir, 3) ? INPUT : OUTPUT);
            break;
        case I2Cexpander::PHOTON_B:
            pinMode(6,  bitRead(dir, 0) ? INPUT : OUTPUT);
            pinMode(7,  bitRead(dir, 1) ? INPUT : OUTPUT);
            pinMode(A0, bitRead(dir, 2) ? INPUT : OUTPUT);
            pinMode(A1, bitRead(dir, 3) ? INPUT : OUTPUT);
            break;
        case I2Cexpander::PHOTON_C:
            pinMode(A2,  bitRead(dir, 0) ? INPUT : OUTPUT);
            pinMode(A3,  bitRead(dir, 1) ? INPUT : OUTPUT);
            pinMode(A6, bitRead(dir, 2) ? INPUT : OUTPUT);
            pinMode(A7, bitRead(dir, 3) ? INPUT : OUTPUT);
            break;
    }
}


