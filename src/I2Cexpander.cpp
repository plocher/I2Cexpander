/*!
   @file I2Cexpander.cpp

   @mainpage I2C IO Expander wrapper library

   @section intro_sec Introduction

   This library provides a common interface to a variety of I2C expanders,
   ADC/DAC devices and Arduino/Photon/Wemos/ESP platform-local pins.

   The design philosophy is to abstract the setup and initialization of the various chipsets
   into an "init" function, and then provide high level "read" and "write" calls that do the right thing.
   It is built on top of the base Wire infrastructure, and coexists (but does not inter-operate) with
   other I2C device libraries.

   This version is limited to a single I2C bus; it does not know how to manage/route through I2C muxes or
   switch between different MCU I2C appliances.

   The digitalWrite/Read functions are convenience interfaces, but not very performant -
   You should use the Arduino provided ones for onboard pins if you need performance.

   The support for PHOTON is rudimentary - their emulation of the Arduino environment is problematic.


    It is based on an array of devices that can be read and written as desired.
    Instead of extending the digitalRead()/digitalWrite abstraction, I chose to read and write
    in units of 4,6, 8 or 16 bits, depending on the device in question.

    In the model railroad community, this is slightly reminiscent of the Chubb CMRI system's design

    To put this in context, this is part of a code-generated control system for a model railroad
    layout where there are many microcontrollers in use, one for every place on the layout where
    there are things to control.

    My default program flow is

    <pre>
    define each layout device (signal heads, turnout controllers, occupancy detectors,...) along with the particular bits are used to talk to it.
    loop() {
        read the layout state
        walk thru every device and ask it to update itself
        if anything changed, handle the side effects (i.e., track becomes occupied, signal needs to turn red...)
        if needed, update outputs (i.e., write new values)
    }
    </pre>

    The list of supported I2C expanders is
    <pre>

          PCA9555       // Bits 0  1  2  3  4  5  6  7  8   9  10  11  12  13  14  15  16
          MCP23016      // Bits 0  1  2  3  4  5  6  7  8   9  10  11  12  13  14  15  16
          PCF8574       // Bits 0  1  2  3  4  5  6  7  8
          PCF8574A      // Bits 0  1  2  3  4  5  6  7  8
          PCF8591       // 4 8-bit A/D converters, 1 8-bit D/A
          MAX731x       // Bits 0  1  2  3  4  5  6  7  8   9  10  11  12  13  14  15  16
          PCA9685

        Pseudo-expanders - can read and write built in pins (as digital I/O) as well:

        // if defined(ARDUINO_AVR_DUEMILANOVE)
          // built-in Arduino ports, skipping RX/TX, Lnet RX/TX and I2C pins
          ARDIO_A       // Bits D2   D3  D4   D5 - low digital
          ARDIO_B       //      D6   D9  D10  D11  - high digital
          ARDIO_C       //      D12  D13 A0   A1  - mixed, digital and analog
          ARDIO_D       //      A2   A3  A6  A7   - analog (A6 & A7 are input only)
        // endif
        // if defined(SPARK_CORE) // Built in Photon Ports
          PHOTON_A      //      D2, D3, D4,  D5,  -- -- -- --
          PHOTON_B      //      D6, D7, A0,  A1,  -- -- -- --
          PHOTON_C      //      A2, A3, DAC, WKP, -- -- -- --
        // endif
        // if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
          WEMOS_A       //      GPIO   4,  0,  2, 14    Pins D2 D3 D4 D5
          WEMOS_B       //      GPIO  12, 13,  3,  1    Pins D6 D7 RX TX
          WEMOS_C       //      GPIO  16, 13,  3,  1    Pins D0 D7 RX TX
        // endif
    </pre>

   @section dependencies Dependencies

   Arduino wire I2C implementation

   @section author Author

   Written by John Plocher

   @section license License

   Released under the terms of the MIT License (MIT)
 */

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

/**
 * Library version
 */
const char *I2Cexpander::version = "2.0.0";

I2Cexpander::I2Cexpander() {
    _address     = -1;
    _chip        = -1;
    _config      = -1;   
    _last        = -1;
    _firsttime   = 1;
    _current     = -2;
    _size        = B_UNKNOWN;
    _i2c_address = -1; // default
	_debounce    = 0;
    next         = 0;
    debugflag    = 0;
}
void I2Cexpander::init(uint16_t address, uint16_t device_type, uint16_t config, boolean debounce /* == false */ ) {
#ifdef I2C_EXTENDER_DEBUG
    Serial.print("I2Cexpander:init(");
    Serial.print(address, DEC); Serial.print(", ");
    Serial.print(device_type, DEC);    Serial.print(", ");
    Serial.print(config, DEC);  Serial.print(", ");
    Serial.print("debounce "); Serial.print(_debounce ? "on" : "off");
    Serial.print(")\n");
#endif  
    _address     = address;
    _chip        = device_type;
    _config      = config;
    _i2c_address = -1; // default
    _debounce    = debounce;

    Wire.setClock(400000UL);

    switch (_chip) {
        case I2Cexpander::MAX731x:    _size = B16; init731x(            _address, _config);  break;
        case I2Cexpander::PCA9555:    _size = B16; init9555(base9555  + _address, _config);  break;
        case I2Cexpander::MCP23016:   _size = B16; init9555(base23016 + _address, _config);  break;
        case I2Cexpander::PCF8574A:   _size = B8;  init8(base8574A    + _address, _config);  break;
        case I2Cexpander::PCF8574:    _size = B8;  init8(base8574     + _address, _config);  break;

        case I2Cexpander::PCF8591:    _size = B32; init8591(base8591  + _address, _config);  break;  // _config is unused
        case I2Cexpander::PCA9685:    _size = B16; init9685(base9685  + _address, _config);  break;  // _config is used to determine which LED channel ...
#if defined(ARDUINO_AVR_DUEMILANOVE)
        case I2Cexpander::ARDIO_A:
        case I2Cexpander::ARDIO_B:
        case I2Cexpander::ARDIO_C:
        case I2Cexpander::ARDIO_D:    initArduino(); break;
#endif
#if defined(SPARK_CORE)
        case I2Cexpander::PHOTON_A:
        case I2Cexpander::PHOTON_B:
        case I2Cexpander::PHOTON_C:     initPhoton(); break;
#endif
#if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
        case I2Cexpander::WEMOS_A:
        case I2Cexpander::WEMOS_B:
        case I2Cexpander::WEMOS_MATRIX:
        case I2Cexpander::WEMOS:        initWemos(); break;
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

bool  I2Cexpander::changed() {
    if (I2Cexpander::_firsttime) {
        I2Cexpander::_firsttime = 0;
        I2Cexpander::_last = ~I2Cexpander::_current;  // force a true response the first time thru...
    }
    if ((I2Cexpander::_chip == I2Cexpander::PCF8591) || (I2Cexpander::_chip == I2Cexpander::PCA9685)) {
        return ((I2Cexpander::_current) != (I2Cexpander::_last));  // no I/O direction mask
    }
    return ((I2Cexpander::_current & I2Cexpander::_config) != (I2Cexpander::_last & I2Cexpander::_config));
};


void I2Cexpander::printData(uint32_t data) {
    if (_size == B4)        {
							  Serial.print((byte)(data >>  0) & 0x0F, BIN);
						    }
    else if (_size == B8)        {
							  Serial.print((byte)(data >>  0) & 0xFF, BIN);
						    }
    else if (_size == B16)  { 
							  Serial.print((byte)(data >>  8) & 0xFF, BIN); Serial.print("_");
							  Serial.print((byte)(data >>  0) & 0xFF, BIN); 
						  	}
    else if (_size == B32)  { 
							  Serial.print((byte)(data >> 24) & 0xFF, BIN); Serial.print("_");
							  Serial.print((byte)(data >> 16) & 0xFF, BIN); Serial.print("_");
							  Serial.print((byte)(data >>  8) & 0xFF, BIN); Serial.print("_");
							  Serial.print((byte)(data >>  0) & 0xFF, BIN); 
						  	}
    else                    { Serial.print("unknown data size: "); Serial.print(_size, DEC); Serial.print(", data: "); Serial.print(data, BIN);}
}

void I2Cexpander::printString(const char *tag) {
	Serial.print(tag);
    Serial.print(" addr=0x");         Serial.print(_address,   HEX);
    Serial.print(" i2c_address=0x");  Serial.print(_i2c_address,   HEX);
    Serial.print(", chip=");          Serial.print(_chip,      DEC);
    Serial.print(", conf=");          Serial.print(_config,    DEC);
    Serial.print(", data size=");     Serial.print(_size,      DEC);	
}

// Software Debounce
uint32_t I2Cexpander::read(void) {  
    uint32_t    v1 = _read();
    if (!_debounce)
        return v1;
    uint32_t    v2 = v1;
    do {
        v1 = v2;
        v2 = _read();
    } while (v2 != v1);
    return v1;
}

// Raw read
uint32_t I2Cexpander::_read() {  
    uint32_t data = 0;
    int error = 0;
#ifdef I2C_EXTENDER_DEBUG
    if (debugflag) {
        Serial.print("I2C:read(a=0x"); Serial.print(_address, HEX); 
        Serial.print(", chip=");       Serial.print(_chip,    DEC); 
        Serial.print(", conf=0b");      Serial.print(_config,  BIN);
        Serial.print(") "); 
    }
#endif
    switch (_chip) {
        case I2Cexpander::MAX731x:        data = read9555();   break; // 731x is same as 9555
        case I2Cexpander::PCA9555:        data = read9555();   break;
        case I2Cexpander::MCP23016:       data = read9555();   break;
        case I2Cexpander::PCF8574A:       data = read8();      break;
        case I2Cexpander::PCF8574:        data = read8();      break;
        case I2Cexpander::PCF8591:        data = read8591();   break;
        case I2Cexpander::PCA9685:        data = read9685();   break;

#if defined(ARDUINO_AVR_DUEMILANOVE)
        case I2Cexpander::ARDIO_A:    
        case I2Cexpander::ARDIO_B:  
        case I2Cexpander::ARDIO_C:
        case I2Cexpander::ARDIO_D:          data = readArduino(); break;
#endif
#if defined(SPARK_CORE)
        case I2Cexpander::PHOTON_A:
        case I2Cexpander::PHOTON_B:
        case I2Cexpander::PHOTON_C:         data = readPhoton();  break;
#endif
#if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
        case I2Cexpander::WEMOS_MATRIX:
        case I2Cexpander::WEMOS:
        case I2Cexpander::WEMOS_A:
        case I2Cexpander::WEMOS_B:      data = readWemos(); break;
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
    if (debugflag) {
        Serial.print(" => "); 
        if (error)              { Serial.print("Error"); }
        else if (_size == B8)   { 
								  Serial.print((byte)(data >>  0) & 0xFF, BIN);
							    }
        else if (_size == B16)  { 
								  Serial.print((byte)(data >>  8) & 0xFF, BIN); Serial.print("_");
								  Serial.print((byte)(data >>  0) & 0xFF, BIN); 
							  	}
        else if (_size == B32)  { 
								  Serial.print((byte)(data >> 24) & 0xFF, BIN); Serial.print("_");
								  Serial.print((byte)(data >> 16) & 0xFF, BIN); Serial.print("_");
								  Serial.print((byte)(data >>  8) & 0xFF, BIN); Serial.print("_");
								  Serial.print((byte)(data >>  0) & 0xFF, BIN); 
							  	}
        else                    { Serial.print("unknown data size: "); Serial.print(data, BIN);}
        Serial.print("\n");  
    }
#endif
    return data;
}

void I2Cexpander::write(uint32_t data) {
#ifdef I2C_EXTENDER_DEBUG
    if (debugflag && (_lastw != data)) {
		printString("I2Cexpander::write([");
		Serial.print("] data=");  
		printData(data);
		Serial.println(")");
    }
#endif

    switch (_chip) {
    case I2Cexpander::MAX731x:         write9555(data); break;  // 731x is same as 9555
    case I2Cexpander::PCA9555:         write9555(data); break;
    case I2Cexpander::MCP23016:        write9555(data); break;
    case I2Cexpander::PCF8574A:        write8(data);    break;
    case I2Cexpander::PCF8574:         write8(data);    break;
    case I2Cexpander::PCF8591:         write8591(data); break;
    case I2Cexpander::PCA9685:         write9685(data); break;

#if defined(ARDUINO_AVR_DUEMILANOVE)
    case I2Cexpander::ARDIO_A:
    case I2Cexpander::ARDIO_B:
    case I2Cexpander::ARDIO_C:
    case I2Cexpander::ARDIO_D:        writeArduino(data); break;
#endif
#if defined(SPARK_CORE)
    case I2Cexpander::PHOTON_A:
    case I2Cexpander::PHOTON_B:
    case I2Cexpander::PHOTON_C:         writePhoton(data); break;
#endif
#if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
    case I2Cexpander::WEMOS_MATRIX:
    case I2Cexpander::WEMOS:
    case I2Cexpander::WEMOS_A:
    case I2Cexpander::WEMOS_B:          writeWemos(data); break;
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

void I2Cexpander::init8(uint8_t i2caddr, uint16_t config) {
    _i2c_address = i2caddr;
    write8(config);
}

uint32_t I2Cexpander::read8() {
    uint32_t _data = -1;
    // Wire.beginTransmission(_i2c_address);
    Wire.requestFrom(_i2c_address, (uint8_t)1);
    if(Wire.available()) {
      _data = Wire.read();
    }
    // Wire.endTransmission();  
    return _data;
}

void I2Cexpander::write8(uint32_t data) {
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
    Wire.write(PCA9555_CONFIG);
    Wire.write(0xff & dir);  // low byte
    Wire.write(dir >> 8);    // high byte
    Wire.endTransmission();  
}

uint32_t I2Cexpander::read9555() {
    uint32_t data = 0;
    Wire.beginTransmission(_i2c_address);
    Wire.write(PCA9555_INPUT);
    int n = Wire.endTransmission(false);  
    if (! ((n == 0) || (n ==7)) ) {
		return (_last);
    }
    Wire.requestFrom(_i2c_address, (uint8_t)2, (uint8_t)1);
    data = Wire.read();
    data |= (Wire.read() << 8);  
    return data;
}

void I2Cexpander::write9555(uint32_t data) {
    data = data | _config;
    Wire.beginTransmission(_i2c_address);
    Wire.write(PCA9555_OUTPUT);
    Wire.write(0xff & data);  //  low byte
    Wire.write(data >> 8);    //  high byte
    Wire.endTransmission();  
}


/*
***************************************************************************
**                                  16 b i t  731x                       **
***************************************************************************
**
** Same as 9555 except for extended address range
 */

void I2Cexpander::init731x(uint8_t i2caddr, uint16_t dir) {
    uint8_t a = (i2caddr < 0x20) ? base731x + i2caddr : base731x + 0x30 + i2caddr;
    I2Cexpander::init9555(a, dir);
    
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x0F);  // Config
    Wire.write(0x08);  //  No Global Brightness
    Wire.endTransmission();  
	/*
	 // alternative for PWM stuff...
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x0E);  // O16 config, master brightness
	Wire.write(0xFF);  
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
	*/
}


/*
***************************************************************************
**                        16 b i t  9685 LED PWM driver                  **
***************************************************************************
 */

void I2Cexpander::init9685(uint8_t i2caddr, uint16_t dir) {
    _i2c_address = i2caddr;
	Wire.beginTransmission(i2caddr);
    Wire.write(PCA9685_MODE1);
    Wire.write(PCA9685_MODE1_RESTART | PCA9685_MODE1_AUTOINC | PCA9685_MODE1_ALLCALL);
    Wire.endTransmission();  
    delay(1);
    Wire.beginTransmission(i2caddr);
    Wire.write(PCA9685_MODE2);
    Wire.write(PCA9685_MODE2_TOTEM | PCA9685_MODE2_OEOFF);
    Wire.endTransmission();  
    delay(1);
}

// read uses the config value to distinguish which LED to read/write

uint32_t I2Cexpander::read9685() {
    uint32_t data = 0;
    uint16_t startdata = 0;
    uint16_t stopdata = 0;
    Wire.beginTransmission(_i2c_address);
    Wire.write(PCA9685_BASE_LED0 + (_config * 4));
    int n = Wire.endTransmission(false);  
    if (! ((n == 0) || (n ==7)) ) {
		return (_last);
    }
    Wire.requestFrom(_i2c_address, (uint8_t)4, (uint8_t)1);
    startdata = Wire.read();
    startdata |= (Wire.read() << 8);  
    stopdata = Wire.read();
    stopdata |= (Wire.read() << 8);  
	if (stopdata == startdata)     data = 0;
	else if (stopdata < startdata) data = startdata + stopdata & 0x0FFF;
	else                           data = stopdata - startdata;
    return data;
}

void I2Cexpander::write9685(uint32_t data) {
    data = data & 0x0FFF;		// 12 bits
    int b1 = (data     ) & 0x00FF; // low
    int b2 = (data >> 8) & 0x00FF; // and high bits
	// TODO: Stagger starting phase to ensure each string is independent, to reduce power supply spiking
    Wire.beginTransmission(_i2c_address);
    Wire.write(PCA9685_BASE_LED0 + (_config * 4));
    Wire.write(0x00);  
    Wire.write(0x00);  
    Wire.write(b1);  
    Wire.write(b2);  
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

uint32_t I2Cexpander::read8591() {
	uint32_t result;
	byte result1;
	byte result2;
	byte result3;
	byte result4;
	
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x04);
    Wire.endTransmission();
    Wire.requestFrom(_i2c_address, (uint8_t)5);

    Wire.read(); // ignore the Analog Output value

    result1 = Wire.read();
    result2 = Wire.read();
    result3 = Wire.read();
    result4 = Wire.read(); 

    result = ((result4 & 0xFF) << 24) | ((result3 & 0xFF) << 16) | ((result2 & 0xFF) << 8) | ((result1 & 0xFF) << 0);
    return result;
}
	
uint32_t I2Cexpander::Xread8591() {
	//#ifdef I2C_EXTENDER_DEBUG
    if (debugflag) {
        Serial.print("readADC(a=0x");   Serial.print(_i2c_address, HEX);
        Serial.print(", Channel=");   Serial.print(_config, DEC);
        Serial.print(") => ");
    }
	//#endif
    uint32_t _d1 = -1;
    uint32_t _d2 = -1;
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x04 | (_config & 0x03));		// 0000-00XX   WHERE XX IS THE A/D TO READ (00, 01, 10 OR 11 FOR 0, 1, 2 AND 3)
    int n = Wire.endTransmission();  
    if (! ((n == 0) || (n == 7)) ) {
#ifdef I2C_EXTENDER_DEBUG
	    if (debugflag) {
	        Serial.print(" ERROR: Wire.endTransmission() -> 0x");     Serial.print((uint8_t) n, HEX);
	    }
#endif
		return (_last);
    }
    Wire.requestFrom(_i2c_address, (uint8_t)2);
    // Why two bytes? The PCF8591 returns the previously measured value first – then the current byte.
    _d1 = Wire.read();
    _d2 = Wire.read();  // ignore the first byte received (see above)
	//#ifdef I2C_EXTENDER_DEBUG
    if (debugflag) {
        Serial.print("old: 0x");     Serial.print((uint32_t) _d1, HEX);
        Serial.print(", new: 0x");   Serial.print((uint32_t) _d2, HEX);
		Serial.println();
    }
	//#endif
    return (uint32_t)_d2;
}

void I2Cexpander::write8591(uint32_t data) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x40);
    Wire.write(0xff & data);
    Wire.endTransmission();  
}

/*
***************************************************************************
 */

// only write a bit to a MCU port using digitalWrite()
// if the config register allows writing to it
//                                                                 WRITEIF
void I2Cexpander::writeif(uint8_t port, uint32_t data, uint8_t bit) {
    boolean mybit = bitRead(data, bit);
#ifdef I2C_EXTENDER_INVERTLOCAL
    mybit = !mybit;
#endif
#ifdef I2C_EXTENDER_DEBUG
    if (debugflag) {
        Serial.print("                       I2C:writeif(pin=");
        Serial.print(port, DEC); 
        Serial.print(", data=");
		printData(data);

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

void I2Cexpander::initArduino(void) {   //                             INIT
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
    }
}

uint32_t I2Cexpander::readArduino(void) {   //                         READ
    uint32_t data = 0;
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

    default: break;
    }
return data;
}

void I2Cexpander::writeArduino(uint32_t data) { //             WRITE
#ifdef I2C_EXTENDER_ONBOARD_DEBUG
    if (debugflag) {
        static uint32_t last = 0x1234;
        if (data != last) {
            Serial.print("I2C:writeArduino("); 
            Serial.print("port=");   Serial.print(_chip, DEC);Serial.print(", ");
            const __FlashStringHelper *s;
            switch(_chip) {
              case ARDIO_A:   s=F("ARDIO_A");   break;
              case ARDIO_B:   s=F("ARDIO_B");   break;
              case ARDIO_C:   s=F("ARDIO_C");   break;
              case ARDIO_D:   s=F("ARDIO_D");   break;
              case A:  s=F("A");  break;
              case B:  s=F("B");  break;
              case C:  s=F("C");  break;
              default:        s=F("UNKNOWN");   break;
            }
            Serial.print(s);
            Serial.print(", conf=0x");   Serial.print(_config,    HEX);
            Serial.print(") data(");  
            printData(data);
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
void I2Cexpander::initPhoton(void) {    //                             INIT
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

uint32_t    I2Cexpander::readPhoton(void) {  //                        READ
    uint32_t data = 0;
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

void I2Cexpander::writePhoton(uint32_t data) { //                    WRITE
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

#if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
/*
***************************************************************************
**                      W e m o s  D 1  R 2                              **
***************************************************************************
      WEMOS_A,              //      GPIO   4,  0,  2, 14    Pins D2 D3 D4 D5
      WEMOS_B,              //      GPIO  12, 13,  3,  1    Pins D6 D7 RX TX
      WEMOS_C,              //      GPIO  16, 13,  3,  1    Pins D0 D7 RX TX
      WEMOS_MATRIX,         //      GPIO   4,  2, 14, 12    Pins D3 [D4 D5 D6] used by LEDCONTROL
 */
void I2Cexpander::initWemos(void) {   //                             INIT
    _size = B4;
    switch (_chip) {
    case I2Cexpander::WEMOS_A:
        pinMode(D2, bitRead(_config, 0) ? INPUT : OUTPUT);
        pinMode(D3, bitRead(_config, 1) ? INPUT : OUTPUT);
        pinMode(D4, bitRead(_config, 2) ? INPUT : OUTPUT);
        pinMode(D5, bitRead(_config, 3) ? INPUT : OUTPUT);
    break;
    case I2Cexpander::WEMOS_B:
        pinMode(D6, bitRead(_config, 0) ? INPUT : OUTPUT);
        pinMode(D7, bitRead(_config, 1) ? INPUT : OUTPUT);
        pinMode(RX, bitRead(_config, 2) ? INPUT : OUTPUT);
        pinMode(TX, bitRead(_config, 3) ? INPUT : OUTPUT);
    break;
    case I2Cexpander::WEMOS_C:
        pinMode(D0, bitRead(_config, 0) ? INPUT : OUTPUT);
        pinMode(D7, bitRead(_config, 1) ? INPUT : OUTPUT);
        pinMode(RX, bitRead(_config, 2) ? INPUT : OUTPUT);
        pinMode(TX, bitRead(_config, 3) ? INPUT : OUTPUT);
    break;
    case I2Cexpander::WEMOS_MATRIX:
        pinMode(D3, bitRead(_config, 0) ? INPUT : OUTPUT);
        pinMode(D4, bitRead(_config, 1) ? INPUT : OUTPUT);
        pinMode(D5, bitRead(_config, 2) ? INPUT : OUTPUT);
        pinMode(D6, bitRead(_config, 3) ? INPUT : OUTPUT);
    break;

    default: break;
    }
}

uint32_t    I2Cexpander::readWemos(void) {  //                        READ
    uint32_t data = 0;
    switch (_chip) {
    case I2Cexpander::WEMOS_A:
        bitWrite(data, 0,::digitalRead(D2));
        bitWrite(data, 1,::digitalRead(D3));
        bitWrite(data, 2,::digitalRead(D4));
        bitWrite(data, 3,::digitalRead(D5));
    break;
    case I2Cexpander::WEMOS_B:
        bitWrite(data, 0,::digitalRead(D6));
        bitWrite(data, 1,::digitalRead(D7));
        bitWrite(data, 2,::digitalRead(RX));
        bitWrite(data, 3,::digitalRead(TX));
    break;
    case I2Cexpander::WEMOS_C:
        bitWrite(data, 0,::digitalRead(D0));
        bitWrite(data, 1,::digitalRead(D7));
        bitWrite(data, 2,::digitalRead(RX));
        bitWrite(data, 3,::digitalRead(TX));
    break;
    case I2Cexpander::WEMOS_MATRIX:
        bitWrite(data, 0,::digitalRead(D3));
        bitWrite(data, 1,::digitalRead(D4));
        bitWrite(data, 2,::digitalRead(D5));
        bitWrite(data, 3,::digitalRead(D6));
    break;

    default: break;
    }
    return data;
}

void I2Cexpander::writeWemos(uint32_t data) { //                   WRITE
    switch (_chip) {
    case I2Cexpander::WEMOS_A:
        writeif(D2, data, 0);
        writeif(D3, data, 1);
        writeif(D4, data, 2);
        writeif(D5, data, 3);
    break;
    case I2Cexpander::WEMOS_B:
        writeif(D6, data, 0);
        writeif(D7, data, 1);
        writeif(RX, data, 2);
        writeif(TX, data, 3);
    break;
    case I2Cexpander::WEMOS_C:
        writeif(D0, data, 0);
        writeif(D7, data, 1);
        writeif(RX, data, 2);
        writeif(TX, data, 3);
    break;
    case I2Cexpander::WEMOS_MATRIX:
        writeif(D3, data, 0);
        //writeif(D4, data, 1);
        //writeif(D5, data, 2);
        //writeif(D6, data, 3);
    break;

    default: break;
    }
}
#endif



