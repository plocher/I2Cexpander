// IO Expander wrapper library
// interfaces with a variety of I2C expanders, ADC/DAC devices and Arduino/Photon local pins
// Copyright (c) 20011...2015 John Plocher, released under the terms of the MIT License (MIT)
//
//  The expander names ARDIO* are convenience functions for accessing the onboard Arduino IO pins.
//  Ditto for the PHOTON* ones
//
//  The digitalWrite/Read functions are convenience interfaces - 
//     NOTE:  You should use the Arduino provided ones for onboard pins if you need performance.

#include "I2Cexpander.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include <Wire.h>
#elif defined(SPARK_CORE)
#include "application.h"
#endif

// #define I2C_EXTENDER_ONBOARD_DEBUG
// #define I2C_EXTENDER_DEBUG
// #define I2C_EXTENDER_INVERTLOCAL  - writing a "1" puts port ON (@5v) rather than OFF @0v

// Arduino 12, 13L ARDUINO_AVR_DUEMILANOVE
// OAK:  	ARDUINO_ESP8266_OAK
// Photon	SPARK_CORE

I2Cexpander::I2Cexpander() {
    _address = -1;
    _chip    = -1;
    _config  = -1;   
    _last    = -1;
    _current = -2;
    _size    = B_UNKNOWN;
    _i2c_address = -1; // default
	next     = 0;
	debugflag = 0;
}

void I2Cexpander::init(uint16_t address, uint16_t chip, uint16_t config, boolean debounce /* == false */ ) {
#ifdef XXI2C_EXTENDER_DEBUG
    Serial.print("I2Cexpander:init(");
    Serial.print(address, DEC); Serial.print(", ");
    Serial.print(chip, DEC);    Serial.print(", ");
    Serial.print(config, DEC);  Serial.print(", ");
    Serial.print(debounce ? "true (debounce)" : "false (raw)");
    Serial.print(")\n");
#endif  
    _address     = address;
    _chip        = chip;
    _config      = config;
    _i2c_address = -1; // default
    _debounce    = debounce;

	switch (_chip) {
		case I2Cexpander::MAX7313:    _size = B16; init7313(            _address, _config);  break;
		case I2Cexpander::PCA9555:    _size = B16; init9555(base9555  + _address, _config);  break;
		case I2Cexpander::MCP23016:   _size = B16; init9555(base23016 + _address, _config);  break;
		case I2Cexpander::PCF8574A:   _size = B8;  init8(base8574A    + _address, _config);  break;
		case I2Cexpander::PCF8574:    _size = B8;  init8(base8574     + _address, _config);  break;

		case I2Cexpander::PCF8591:    _size = B8;  init8591(base8591  + _address, _config);  break;  // _config is used to determine which ADC ...
#if defined(ARDUINO_AVR_DUEMILANOVE)
		case I2Cexpander::ARDIO_A:
		case I2Cexpander::ARDIO_B:
		case I2Cexpander::ARDIO_C:
		case I2Cexpander::ARDIO_D:
		case I2Cexpander::ARDIO12_A:
		case I2Cexpander::ARDIO12_B:
		case I2Cexpander::ARDIO13_A:
		case I2Cexpander::ARDIO13_B:  initArduino(); break;
#endif
#if defined(SPARK_CORE)
		case I2Cexpander::PHOTON_A:
		case I2Cexpander::PHOTON_B:
		case I2Cexpander::PHOTON_C:   initPhoton(); break;
#endif
#if defined(ARDUINO_ESP8266_OAK)
		case I2Cexpander::OAK_A:
		case I2Cexpander::OAK_B:
		case I2Cexpander::OAK_C:      initOAK(); break;
#endif
		
		case IGNORE:
		default:
		    _size=0; break;
    }
}

void I2Cexpander::digitalWrite(uint8_t dataPin, uint8_t val) {
	bitWrite(next, dataPin, val); I2Cexpander::write(next);
}

uint8_t  I2Cexpander::digitalRead(uint8_t dataPin) {
	get();
	return bitRead(_current, dataPin) ? HIGH : LOW; 
}

// Software Debounce
uint16_t I2Cexpander::read(void) {  
	uint16_t	v1 = _read();
	if (!_debounce)
		return v1;
	uint16_t	v2 = v1;
	do {
		v1 = v2;
		v2 = _read();
	} while (v2 != v1);
	return v1;
}

// Raw read
uint16_t I2Cexpander::_read() {  
    uint16_t data = 0;
    int error = 0;
#ifdef I2C_EXTENDER_DEBUG
    if (debugflag && (_config != 0)) {
	    Serial.print("I2C:read(a=0x"); Serial.print(_address, HEX); 
	    Serial.print(", chip=");       Serial.print(_chip,    DEC); 
	    Serial.print(", conf=B");      Serial.print(_config,  BIN);
	    Serial.print(") "); 
	}
#endif
    switch (_chip) {
        case I2Cexpander::MAX7313:        data = read9555();   break; // 7313 is same as 9555
        case I2Cexpander::PCA9555:        data = read9555();   break;
        case I2Cexpander::MCP23016:       data = read9555();   break;
        case I2Cexpander::PCF8574A:       data = read8();   break;
        case I2Cexpander::PCF8574:        data = read8();   break;
        case I2Cexpander::PCF8591:        data = read8591();   break;

#if defined(ARDUINO_AVR_DUEMILANOVE)
		case I2Cexpander::ARDIO_A:    
		case I2Cexpander::ARDIO_B:  
		case I2Cexpander::ARDIO_C:
		case I2Cexpander::ARDIO_D:
		case I2Cexpander::ARDIO12_A:
		case I2Cexpander::ARDIO12_B:
		case I2Cexpander::ARDIO13_A:
		case I2Cexpander::ARDIO13_B:      data = readArduino(); break;
#endif
#if defined(SPARK_CORE)
        case I2Cexpander::PHOTON_A: 
        case I2Cexpander::PHOTON_B:  
		case I2Cexpander::PHOTON_C: 	  data = readPhoton();  break;
#endif
#if defined(ARDUINO_ESP8266_OAK)
		case I2Cexpander::OAK_A:
		case I2Cexpander::OAK_B:
		case I2Cexpander::OAK_C:          data = readOAK(); break;
#endif

		default:
			error = 1;
		case IGNORE:
	    	break;
    } 
    if (!error) {      
        I2Cexpander::_last = I2Cexpander::_current;
        I2Cexpander::_current = data;
    } 
#ifdef I2C_EXTENDER_DEBUG
    if (debugflag && (_config != 0)) {
	    Serial.print(" => "); 
	    if (error)              Serial.print("Error"); 
	    else if (_size == B8)   Serial.print((byte)data, BIN); 
	    else if (_size == B16)  Serial.print((uint16_t)data, BIN); 
	    else                   {Serial.print("unknown data size: "); Serial.print(data, BIN);}
	    Serial.print("\n");  
	}
#endif
    return data;
}

void I2Cexpander::write(uint16_t data) {
#ifdef I2C_EXTENDER_DEBUG
    if (debugflag && (_lastw != data)) {
	    Serial.print("I2C:write("); 
	    Serial.print(   "a=0x");   Serial.print(_address,   HEX); 
	    Serial.print(", chip=");   Serial.print(_chip,      DEC); 
	    Serial.print(", conf=");   Serial.print(_config,    DEC);
	    Serial.print(") data(");  
	    if (_size == B8)          Serial.print((byte)data,     BIN); 
	    else if (_size == B8)     Serial.print((byte)data,     BIN); 
	    else if (_size == B16)    Serial.print((uint16_t)data, BIN); 
	    else                   {  Serial.print(data, BIN); Serial.print("?");}
	    Serial.print(")\n");
	}
#endif

    switch (_chip) {
    case I2Cexpander::MAX7313:         write9555(data); break;  // 7313 is same as 9555
    case I2Cexpander::PCA9555:         write9555(data); break;
    case I2Cexpander::MCP23016:        write9555(data); break;
    case I2Cexpander::PCF8574A:        write8(data); break;
    case I2Cexpander::PCF8574:         write8(data); break;
    case I2Cexpander::PCF8591:         write8591(data);           break;

#if defined(ARDUINO_AVR_DUEMILANOVE)
    case I2Cexpander::ARDIO_A:
    case I2Cexpander::ARDIO_B:
    case I2Cexpander::ARDIO_C:
    case I2Cexpander::ARDIO_D:
    case I2Cexpander::ARDIO12_A:
    case I2Cexpander::ARDIO12_B:
    case I2Cexpander::ARDIO13_A:
	case I2Cexpander::ARDIO13_B: 	  writeArduino(data); break;
#endif
#if defined(SPARK_CORE)
    case I2Cexpander::PHOTON_A:        
    case I2Cexpander::PHOTON_B:        
    case I2Cexpander::PHOTON_C:	      writePhoton(data); break;
#endif
#if defined(ARDUINO_ESP8266_OAK)
		case I2Cexpander::OAK_A:
		case I2Cexpander::OAK_B:
		case I2Cexpander::OAK_C:      writeOAK(data); break;
#endif

    case IGNORE:
    default:  break;
    }
	_lastw = data;
}

/*
***************************************************************************
**                                  8  b i t                             **
***************************************************************************
 */

void I2Cexpander::init8(uint8_t i2caddr, uint16_t dir) {
	_i2c_address = i2caddr;
	write8(dir);
}

uint16_t I2Cexpander::read8() {
    uint16_t _data = -1;
    // Wire.beginTransmission(_i2c_address);
    Wire.requestFrom(_i2c_address, (uint8_t)1);
    if(Wire.available()) {
      _data = Wire.read();
    }
    // Wire.endTransmission();  
    return _data;
}

void I2Cexpander::write8(uint16_t data) {
    data = data | _config;
    Wire.beginTransmission(_i2c_address);
    Wire.write(0xff & data);
    Wire.endTransmission();  
}


/*
***************************************************************************
**                                  16 b i t  9555                       **
***************************************************************************
 */

void I2Cexpander::init9555(uint8_t i2caddr, uint16_t dir) {
	_i2c_address = i2caddr;
    Wire.beginTransmission(i2caddr);
    Wire.write(REGISTER_9555CONFIG);
    Wire.write(0xff & dir);  // low byte
    Wire.write(dir >> 8);    // high byte
    Wire.endTransmission();  
}

uint16_t I2Cexpander::read9555() {
    uint16_t data = 0;
    Wire.beginTransmission(_i2c_address);
    Wire.write(REGISTER_9555INPUT);
    Wire.endTransmission();  
    // Wire.beginTransmission(_i2c_address);
    Wire.requestFrom(_i2c_address, (uint8_t)2);
    if(Wire.available()) {
        data = Wire.read();
    }
    if(Wire.available()) {
        data |= (Wire.read() << 8);  
    }
    // Wire.endTransmission();  
    return data;
}

void I2Cexpander::write9555(uint16_t data) {
    data = data | _config;
    Wire.beginTransmission(_i2c_address);
    Wire.write(REGISTER_9555OUTPUT);
    Wire.write(0xff & data);  //  low byte
    Wire.write(data >> 8);    //  high byte
    Wire.endTransmission();  
}


/*
***************************************************************************
**                                  16 b i t  7313                       **
***************************************************************************
**
** Same as 9555 except for extended address range
 */

void I2Cexpander::init7313(uint8_t i2caddr, uint16_t dir) {
	uint8_t a = (i2caddr < 0x20) ? base7313 + i2caddr : base7313 + 0x30 + i2caddr;
    I2Cexpander::init9555(a, dir);
	
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x0E);  // O16 config, master brightness
    Wire.write(0xFF);  
    Wire.endTransmission();  
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x0F);  // Config
    Wire.write(0x08);  //  No Global Brightness
    Wire.endTransmission();  
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x02);  // Blink Phase 0 -  specify each output’s logic level during the PWM on-time
    Wire.write(0xFF);  //  
    Wire.write(0xFF);  //  
    Wire.endTransmission();  
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x10);  // P0/P1 dimmer
    Wire.write(0xFF);  // P0/P1 = 00001111
    Wire.write(0xFF);  // P2/P3 = 11111111
    Wire.write(0xFF);  // P2/P3 = 11111111
    Wire.write(0xFF);  // P2/P3 = 11111111
    Wire.write(0xFF);  // P2/P3 = 11111111
    Wire.write(0xFF);  // P2/P3 = 11111111
    Wire.write(0xFF);  // P2/P3 = 11111111
    Wire.write(0xFF);  // P2/P3 = 11111111
    Wire.endTransmission();  
}


/*
***************************************************************************
**                                  ADC / DAC                            **
***************************************************************************
 */

void I2Cexpander::init8591(uint8_t i2caddr, uint16_t dir) {
	_i2c_address = i2caddr;
}

uint16_t I2Cexpander::read8591() {
#ifdef I2C_EXTENDER_DEBUG
    if (debugflag) {
	    Serial.print("readADC(a=0x");	Serial.print(_i2c_address, HEX);
	    Serial.print(", Channel=0x");  	Serial.print(_config, HEX);
	    Serial.print(") => ");
	}
#endif
    int _d1 = -1;
    int _d2 = -1;
    Wire.beginTransmission(_i2c_address);
    Wire.write(_config);
    Wire.endTransmission();
    Wire.requestFrom(_i2c_address, (uint8_t)2);
    // Why two bytes? The PCF8591 returns the previously measured value first – then the current byte.
    _d1 = Wire.read();
    _d2 = Wire.read();  // ignore the first byte received (see above)
    // Wire.endTransmission();  
#ifdef I2C_EXTENDER_DEBUG
    if (debugflag) {
	    Serial.print("old: ");;
	    Serial.print(_d1, DEC);
	    Serial.print(", new: ");
	    Serial.print(_d2, DEC);Serial.print(" ");
	}
#endif
    return (uint16_t)_d2;
}

void I2Cexpander::write8591(uint16_t data) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x40);
    Wire.write(0xff & data);
    Wire.endTransmission();  
}

/*
***************************************************************************
 */

// only write a bit to an Arduino or Photon port
// if the config register allows writing to it
//                                                                 WRITEIF
void I2Cexpander::writeif(uint8_t port, uint16_t data, uint8_t bit) {
    boolean mybit = bitRead(data, bit);
#ifdef I2C_EXTENDER_INVERTLOCAL
    mybit = !mybit;
#endif
#ifdef I2C_EXTENDER_DEBUG
    if (debugflag) {
	    Serial.print("                       I2C:writeif(pin=");
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
	}
#endif
    if (bitRead(_config, bit) == 0) ::digitalWrite(port, mybit);
}

#if defined(ARDUINO_AVR_DUEMILANOVE)
/*
***************************************************************************
**                                 A R D U I N O                         **
***************************************************************************
 */

void I2Cexpander::initArduino(void) {	//                             INIT
   switch (_chip) {
// set Arduino I/O pin direction (1=OUTPUT, 0-INPUT)
	case I2Cexpander::ARDIO_A:  _size = B4;  
	    pinMode(2,  bitRead(_config, 0) ? INPUT : OUTPUT);
	    pinMode(3,  bitRead(_config, 1) ? INPUT : OUTPUT);
	    pinMode(4,  bitRead(_config, 2) ? INPUT : OUTPUT);
	    pinMode(5,  bitRead(_config, 3) ? INPUT : OUTPUT);
	break;
	case I2Cexpander::ARDIO_B:  _size = B4;  
	    pinMode(6,  bitRead(_config, 0) ? INPUT : OUTPUT);
	    pinMode(9,  bitRead(_config, 1) ? INPUT : OUTPUT);
	    pinMode(10, bitRead(_config, 2) ? INPUT : OUTPUT);
	    pinMode(11, bitRead(_config, 3) ? INPUT : OUTPUT);
	break;
	case I2Cexpander::ARDIO_C:  _size = B4;  
		pinMode(12, bitRead(_config, 0) ? INPUT : OUTPUT);
	    pinMode(13, bitRead(_config, 1) ? INPUT : OUTPUT);
	    pinMode(A0, bitRead(_config, 2) ? INPUT : OUTPUT);
	    pinMode(A1, bitRead(_config, 3) ? INPUT : OUTPUT);
	break;
	case I2Cexpander::ARDIO_D:  _size = B4;  _config |= 0b1100; 
	    pinMode(A2, bitRead(_config, 0) ? INPUT : OUTPUT);
	    pinMode(A3, bitRead(_config, 1) ? INPUT : OUTPUT);
	    //      A6 and 
		//      A7 are Analog IN, pinmode doesn't work with them
	break;

// set Arduino I/O pin direction (1=OUTPUT, 0-INPUT)	
	case I2Cexpander::ARDIO12_A:  _size = B8;
	    pinMode(0,  bitRead(_config, 0) ? INPUT : OUTPUT);
	    pinMode(1,  bitRead(_config, 1) ? INPUT : OUTPUT);
	    pinMode(5,  bitRead(_config, 2) ? INPUT : OUTPUT);
	    pinMode(6,  bitRead(_config, 3) ? INPUT : OUTPUT);

#if !defined(__AVR_ATmega32U4__)    // don't mess with I2C pins!
	    pinMode(2,  bitRead(_config, 4) ? INPUT : OUTPUT);
#else
	    pinMode(A4, bitRead(_config, 4) ? INPUT : OUTPUT);
#endif
	    pinMode(9,  bitRead(_config, 5) ? INPUT : OUTPUT);
	    pinMode(10, bitRead(_config, 6) ? INPUT : OUTPUT);
	    pinMode(11, bitRead(_config, 7) ? INPUT : OUTPUT);
	break;
	case I2Cexpander::ARDIO12_B:  _size = B8;
	    pinMode(12, bitRead(_config, 0) ? INPUT : OUTPUT);
	    pinMode(13, bitRead(_config, 1) ? INPUT : OUTPUT);
	    pinMode(A0, bitRead(_config, 2) ? INPUT : OUTPUT);
	    pinMode(A1, bitRead(_config, 3) ? INPUT : OUTPUT);

	    pinMode(A2, bitRead(_config, 4) ? INPUT : OUTPUT);
	    pinMode(A3, bitRead(_config, 5) ? INPUT : OUTPUT);

#if !defined(__AVR_ATmega32U4__)    // don't mess with I2C pins!
	    pinMode(3,  bitRead(_config, 6) ? INPUT : OUTPUT);
#else
	    pinMode(A5, bitRead(_config, 6) ? INPUT : OUTPUT);
#endif
	    pinMode(4,  bitRead(_config, 7) ? INPUT : OUTPUT);
	break;

	// set pro-MINI Arduino I/O pin direction (1=OUTPUT, 0-INPUT)
	case I2Cexpander::ARDIO13_A:  _size = B8;  
	    pinMode(2,  bitRead(_config, 0) ? INPUT : OUTPUT);
	    pinMode(3,  bitRead(_config, 1) ? INPUT : OUTPUT);
	    pinMode(4,  bitRead(_config, 2) ? INPUT : OUTPUT);
	    pinMode(5,  bitRead(_config, 3) ? INPUT : OUTPUT);

	    pinMode(6,  bitRead(_config, 4) ? INPUT : OUTPUT);
	    pinMode(9,  bitRead(_config, 5) ? INPUT : OUTPUT);
	    pinMode(10, bitRead(_config, 6) ? INPUT : OUTPUT);
	    pinMode(11, bitRead(_config, 7) ? INPUT : OUTPUT);
	break;
	case I2Cexpander::ARDIO13_B:  _size = B8;  _config |= 0xC0; 
		pinMode(12, bitRead(_config, 0) ? INPUT : OUTPUT);
	    pinMode(13, bitRead(_config, 1) ? INPUT : OUTPUT);
	    pinMode(A0, bitRead(_config, 2) ? INPUT : OUTPUT);
	    pinMode(A1, bitRead(_config, 3) ? INPUT : OUTPUT);

	    pinMode(A2, bitRead(_config, 4) ? INPUT : OUTPUT);
	    pinMode(A3, bitRead(_config, 5) ? INPUT : OUTPUT);
	    // A6 and A7 are Analog IN, pinmode doesn't work with them
	break;
    }
}

uint16_t I2Cexpander::readArduino(void) {	//                         READ
	uint16_t data = 0;
    switch (_chip) {

    case I2Cexpander::ARDIO_A:        
		bitWrite(data,0,::digitalRead(2));
		bitWrite(data,1,::digitalRead(3));
		bitWrite(data,2,::digitalRead(4));
		bitWrite(data,3,::digitalRead(5));
	break;
    case I2Cexpander::ARDIO_B:        
		bitWrite(data,0,::digitalRead(6));
		bitWrite(data,1,::digitalRead(9));
		bitWrite(data,2,::digitalRead(10));
		bitWrite(data,3,::digitalRead(11));
	break;
    case I2Cexpander::ARDIO_C:        
		bitWrite(data,0,::digitalRead(12));
		bitWrite(data,1,::digitalRead(13));
		bitWrite(data,2,::digitalRead(A0));
		bitWrite(data,3,::digitalRead(A1));
	break;
	case I2Cexpander::ARDIO_D:        
		bitWrite(data,0,::digitalRead(A2));
		bitWrite(data,1,::digitalRead(A3));
		bitWrite(data,2, (analogRead(A6) > 100) ? 1 : 0);
		bitWrite(data,3, (analogRead(A7) > 100) ? 1 : 0);
	break;

    case I2Cexpander::ARDIO12_A:        
          bitWrite(data,0,::digitalRead(0));
          bitWrite(data,1,::digitalRead(1));
          bitWrite(data,2,::digitalRead(5));
          bitWrite(data,3,::digitalRead(6));
#if !defined(__AVR_ATmega32U4__)        // don't mess with I2C pins!
          bitWrite(data,4,::digitalRead(2));
#else
          bitWrite(data,4,::digitalRead(A4));
#endif
          bitWrite(data,5,::digitalRead(9));
          bitWrite(data,6,::digitalRead(10));
          bitWrite(data,7,::digitalRead(11));

	break;
    case I2Cexpander::ARDIO12_B:        
         bitWrite(data,0,::digitalRead(12));
         bitWrite(data,1,::digitalRead(13));
         bitWrite(data,2,::digitalRead(A0));
         bitWrite(data,3,::digitalRead(A1));

         bitWrite(data,4,::digitalRead(A2));
         bitWrite(data,5,::digitalRead(A3));
#if !defined(__AVR_ATmega32U4__)        // don't mess with I2C pins!
         bitWrite(data,6,::digitalRead(3));
#else
         bitWrite(data,6,::digitalRead(A5));
#endif
         bitWrite(data,7,::digitalRead(4));

	break;
    case I2Cexpander::ARDIO13_A:        
		bitWrite(data,0,::digitalRead(2));
		bitWrite(data,1,::digitalRead(3));
		bitWrite(data,2,::digitalRead(4));
		bitWrite(data,3,::digitalRead(5));

		bitWrite(data,4,::digitalRead(6));
		bitWrite(data,5,::digitalRead(9));
		bitWrite(data,6,::digitalRead(10));
		bitWrite(data,7,::digitalRead(11));

	break;
    case I2Cexpander::ARDIO13_B:        
		bitWrite(data,0,::digitalRead(12));
		bitWrite(data,1,::digitalRead(13));
		bitWrite(data,2,::digitalRead(A0));
		bitWrite(data,3,::digitalRead(A1));

		bitWrite(data,4,::digitalRead(A2));
		bitWrite(data,5,::digitalRead(A3));
		bitWrite(data,6, (analogRead(A6) > 100) ? 1 : 0);
		bitWrite(data,7, (analogRead(A7) > 100) ? 1 : 0);

	break;
	default: break;
	}
return data;
}

void I2Cexpander::writeArduino(uint16_t data) {	//             WRITE
#ifdef I2C_EXTENDER_ONBOARD_DEBUG
    if (debugflag) {
		static uint16_t last = 0x1234;
		if (data != last) {
			Serial.print("I2C:writeArduino("); 
			Serial.print("port=");   Serial.print(_chip, DEC);Serial.print(", ");
			const __FlashStringHelper *s;
		    switch(_chip) {
			  case ARDIO_A:   s=F("ARDIO_A");   break;
		      case ARDIO_B:   s=F("ARDIO_B");   break;
		      case ARDIO_C:   s=F("ARDIO_C");   break;
		      case ARDIO_D:   s=F("ARDIO_D");   break;
		      case ARDIO12_A: s=F("ARDIO12_A"); break;
		      case ARDIO12_B: s=F("ARDIO12_B"); break;
		      case ARDIO13_A: s=F("ARDIO13_A"); break;
		      case ARDIO13_B: s=F("ARDIO13_B"); break;
		      case PHOTON_A:  s=F("PHOTON_A");  break;
		      case PHOTON_B:  s=F("PHOTON_B");  break;
		      case PHOTON_C:  s=F("PHOTON_C");  break;
			  default:        s=F("UNKNOWN");   break;
		    }
		    Serial.print(s);
		    Serial.print(", conf=0x");   Serial.print(_config,    HEX);
		    Serial.print(") data(");  
		    if (_size == B4)          Serial.print((byte)data & 0x0F,       BIN); 
		    else if (_size == B6)     Serial.print((byte)data & 0x3F,       BIN); 
		    else if (_size == B8)     Serial.print((byte)data & 0xFF,       BIN); 
		    else if (_size == B16)    Serial.print((uint16_t)data & 0xFFFF, BIN); 
		    else                   {  Serial.print(data, BIN); Serial.print("?");}
		    Serial.print(")\n");
			last=data;
		}
	}
#endif
	switch (_chip) {
	case I2Cexpander::ARDIO_A:        
	     writeif( 2, data, 0);
	     writeif( 3, data, 1);
	     writeif( 4, data, 2);
	     writeif( 5, data, 3);
	break;
	case I2Cexpander::ARDIO_B:        
	     writeif( 6, data, 0);
	     writeif( 9, data, 1);
	     writeif(10, data, 2);
	     writeif(11, data, 3);
	break;
	case I2Cexpander::ARDIO_C:        
	     writeif(12, data, 0);
	     writeif(13, data, 1);
	     writeif(A0, data, 2);
	     writeif(A1, data, 3);
	break;
	case I2Cexpander::ARDIO_D:        
	     writeif(A2, data, 0);
	     writeif(A3, data, 1);
	     //      A6 and 
		 //      A7 are input-only analog pins
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
	default: break;
	}
}
#endif

#if defined(SPARK_CORE)
/*
***************************************************************************
**                                   P H O T O N                         **
***************************************************************************
 */
void I2Cexpander::initPhoton(void) {	//                             INIT
	_size = B4;  
	switch (_chip) {
	case I2Cexpander::PHOTON_A:
		pinMode(2, bitRead(_config, 0) ? INPUT : OUTPUT);
		pinMode(3, bitRead(_config, 1) ? INPUT : OUTPUT);
		pinMode(4, bitRead(_config, 2) ? INPUT : OUTPUT);
		pinMode(5, bitRead(_config, 3) ? INPUT : OUTPUT);
	break;
	case I2Cexpander::PHOTON_B:
		pinMode(6, bitRead(_config, 0) ? INPUT : OUTPUT);
		pinMode(7, bitRead(_config, 1) ? INPUT : OUTPUT);
		pinMode(A0, bitRead(_config, 2) ? INPUT : OUTPUT);
		pinMode(A1, bitRead(_config, 3) ? INPUT : OUTPUT);
	break;
	case I2Cexpander::PHOTON_C:
		pinMode(A2, bitRead(_config, 0) ? INPUT : OUTPUT);
		pinMode(A3, bitRead(_config, 1) ? INPUT : OUTPUT);
		pinMode(A6, bitRead(_config, 2) ? INPUT : OUTPUT);
		pinMode(A7, bitRead(_config, 3) ? INPUT : OUTPUT);
	break;
	default:
	break;
	}
}

uint16_t	I2Cexpander::readPhoton(void) {  //                        READ
	uint16_t data = 0;
	switch (_chip) {
	case I2Cexpander::PHOTON_A:
		bitWrite(data, 0,::digitalRead(2));
		bitWrite(data, 1,::digitalRead(3));
		bitWrite(data, 2,::digitalRead(4));
		bitWrite(data, 3,::digitalRead(5));
	break;
	case I2Cexpander::PHOTON_B:
		bitWrite(data, 0,::digitalRead(6));
		bitWrite(data, 1,::digitalRead(7));
		bitWrite(data, 2,::digitalRead(A0));
		bitWrite(data, 3,::digitalRead(A1));
	break;
	case I2Cexpander::PHOTON_C:
		bitWrite(data, 0,::digitalRead(A2));
		bitWrite(data, 1,::digitalRead(A3));
		bitWrite(data, 2,::digitalRead(A6));
		bitWrite(data, 3,::digitalRead(A7));
	break;
	default:
	break;
	}
	return data;
}

void I2Cexpander::writePhoton(uint16_t data) { //	                 WRITE
	switch (_chip) {
	case I2Cexpander::PHOTON_A:
		writeif(2, data, 0);
		writeif(3, data, 1);
		writeif(4, data, 2);
		writeif(5, data, 3);
	break;
	case I2Cexpander::PHOTON_B:
		writeif(6, data, 0);
		writeif(7, data, 1);
		writeif(A0, data, 2);
		writeif(A1, data, 3);
	break;
	case I2Cexpander::PHOTON_C:
		writeif(A2, data, 0);
		writeif(A3, data, 1);
		writeif(A6, data, 2);
		writeif(A7, data, 3);
	break;

	default:
	break;

	}
}
#endif

#if defined(ARDUINO_ESP8266_OAK)
/*
***************************************************************************
**                      D i g i s t u m p   O A K                        **
***************************************************************************
 */
void I2Cexpander::initOAK(void) {	//                             INIT
	_size = B4;  
	switch (_chip) {
	case I2Cexpander::OAK_A:
		pinMode( 7, bitRead(_config, 0) ? INPUT : OUTPUT);
		pinMode( 8, bitRead(_config, 1) ? INPUT : OUTPUT);
		pinMode( 9, bitRead(_config, 2) ? INPUT : OUTPUT);
		pinMode(10, bitRead(_config, 3) ? INPUT : OUTPUT);
	break;
	case I2Cexpander::OAK_B:
		pinMode( 1, bitRead(_config, 0) ? INPUT : OUTPUT);
		pinMode( 5, bitRead(_config, 1) ? INPUT : OUTPUT);
		pinMode( 4, bitRead(_config, 2) ? INPUT : OUTPUT);
		pinMode( 3, bitRead(_config, 3) ? INPUT : OUTPUT);
	break;
	case I2Cexpander::OAK_C:
		pinMode( 6, bitRead(_config, 0) ? INPUT : OUTPUT);
		pinMode(A0, bitRead(_config, 1) ? INPUT : OUTPUT);
        _size = B2;
	break;
	default:
	break;
	}
}

uint16_t	I2Cexpander::readOAK(void) {  //                        READ
	uint16_t data = 0;
	switch (_chip) {
	case I2Cexpander::OAK_A:
		bitWrite(data, 0,::digitalRead( 7));
		bitWrite(data, 1,::digitalRead( 8));
		bitWrite(data, 2,::digitalRead( 9));
		bitWrite(data, 3,::digitalRead(10));
	break;
	case I2Cexpander::OAK_B:
		bitWrite(data, 0,::digitalRead( 1));
		bitWrite(data, 1,::digitalRead( 5));
		bitWrite(data, 2,::digitalRead( 4));
		bitWrite(data, 3,::digitalRead( 3));
	break;
	case I2Cexpander::OAK_C:
		bitWrite(data, 0,::digitalRead( 6));
		bitWrite(data, 1,::digitalRead(A0));
	break;
	default:
	break;
	}
	return data;
}

void I2Cexpander::writeOAK(uint16_t data) { //	                 WRITE
	switch (_chip) {
	case I2Cexpander::OAK_A:
		writeif( 7, data, 0);
		writeif( 8, data, 1);
		writeif( 9, data, 2);
		writeif(10, data, 3);
	break;
	case I2Cexpander::OAK_B:
		writeif( 1, data, 0);
		writeif( 5, data, 1);
		writeif( 4, data, 2);
		writeif( 3, data, 3);
	break;
	case I2Cexpander::OAK_C:
		writeif( 6, data, 0);
		writeif(A0, data, 1);
	break;

	default:
	break;

	}
}
#endif


