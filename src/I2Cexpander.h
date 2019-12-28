/*!
 * @file I2Cexpander.h
 *
 * A Library for interfacing to various IO Expander chips
 *
 * Written by John Plocher, 2011-2019
 *
 * released under the terms of the MIT License (MIT)
 *
 * Support for
 *  Onboard pins on
 *      Arduino AVR '328 (DUEMILANOVE, Pro-Mini...)
 *      Particle Photon
 *      Wemos D1-mini ESP8266
 *
 *  16 bit Expanders, 8 bit expanders. D/A and A/D converters and LED/Servo controllers
 *
 *  See example programs for usage information
 *  I2Cexpander is an abstraction library that manages various I2C devices as IO points you can use.
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

/**
 * A collection of I2C expanders with a simple API:
 *    init()
 *    read() / get()
 *    write() / put()
 */
class I2Cexpander {
public:
    static const char *version;
    /*!
        @brief  I2Cexpander class Constructor.
                No arguments so that it can be either statically initialized OR dynamic.
    */
    I2Cexpander(void);

    /*!
        @brief  Initialize the I2C expander device.
                Usually called in the setup() routine for a one-time initialization.
        @param    address
                  The zero-based chip sequence number.
                  Instead of remembering the address ranges used by the various I2C devices,
                  the library does it for you, and will calculate the I2C address based on this value.
                  For example, an 8-bit 8574 starts at I2C address 0x20, bit the -A version
                  starts at 0x38.  In either case, here you would simply pass [0,1,2,3,4,5,6,7]
                  to the init function and it will translate depending on the device type.
        @param    device_type
                  The manufacturer's name for the device [MCP23016, PCF8574, MAX7311, ...]
                  or a virtual name for the onboard MCU pins [ARDIO_A, WEMOS_C, ...]
                  The special type "IGNORE" can be used to document I2C addresses used elsewhere.
        @param    config
                  Usually, the Input-vs-Output pin direction settings, used on a device-by-device basis
                  for device specific configuration.
        @param    debounce
                  For bit-I/O, ensure that 2x readings are the same before noting a pin change.
    */
    void     init(uint16_t address, uint16_t device_type, uint16_t config, boolean debounce=false);

    /*!
        @brief  Arduino compatibility routine.
                Write a bit to an expander.  Updates current cached state and writes data to the device.
        @param    dataPin
                  which bit in the I/O device's control [0..7 or 0..15, etc].
        @param    val
                  HIGH, 1, LOW, 0
    */
    void     digitalWrite(uint8_t dataPin, uint8_t val);
    /*!
        @brief  Arduino compatibility routine.
                Read from an expander, update the cached state and return the bit value
        @param    dataPin
                  which bit in the I/O device's control [0..7 or 0..15, etc].
        @return  the value of the bit - HIGH, 1 or LOW, 0
    */
    uint8_t  digitalRead(uint8_t dataPin);

    /*!
        @brief  Read data from an expander, update the cached state and return the data
        @return  the data from the device (1,4,8, 16 or 32 bits, per the device type)
    */
    uint32_t read(void);

    /*!
        @brief  wrapper for read().
        @return  the data from the device (1,4,8, 16 or 32 bits, per the device type)
    */    uint32_t get(void)          { return I2Cexpander::read(); }

    /*!
        @brief  Write data to an expander
        @param data
                (1,4,8, 16 or 32 bits, per the device type)
    */
    void     write(uint32_t data);
    /*!
        @brief  wrapper for write(data).
        @param data
                (1,4,8, 16 or 32 bits, per the device type)
    */
    void     put(uint32_t data) { next = data; I2Cexpander::write(next); }
    /*!
        @brief  wrapper for write(this->next).
    */
    void     put(void)          { I2Cexpander::write(next);   }
    /*!
        @brief  wrapper for write(this->next).
    */    void     write(void)        { I2Cexpander::write(next);   }

    /*!
        @brief  How many bits does this expander read/write?
        @return (1,4,8, 16 or 32 bits, per the device type)
    */
    uint16_t getSize(void)      { return I2Cexpander::_size; };

    /*!
        @brief  Cached data - the last read from the device
        @return the same value as the last "read()"
    */
    uint32_t current()          { return I2Cexpander::_current; };

    /*!
        @brief  Cached data - the last write to the device
        @return the previous "write()" value.
    */
    uint32_t last()             { return I2Cexpander::_last;    };

    /*!
        @brief  Configuration initialization info
        @return the "config" value sent to the "init()" function.
    */
    uint16_t config()           { return I2Cexpander::_config; };
    /*!
        @brief  Device Type
        @return the "device_type" value sent to the "init()" function.
    */
    uint16_t chip()             { return I2Cexpander::_chip; };

    /*!
        @brief  Real I2C Address
        @return the device's I2C address (the sequence "address" + device_address_base).
    */
    uint8_t  i2caddr()          { return I2Cexpander::_i2c_address; };

    /*!
        @brief  Have any INPUT bits changed since the last "read()"?
        @return TRUE if something changed.
    */
    bool  changed();
    /**
     * collection point for bits to-be-written
     */
    uint32_t next;

    /**
     * enable debugging if compiled in...
     */
    byte     debugflag;

    /** The number of bits managed by an expander device. */
    enum IOSize {
      B_UNKNOWN =  0,   ///< Usually an error...
      B4        =  4,   ///< 4-bit - Virtual expanders (aka MCU pins)
      B6        =  6,   ///< 6-bit - potentially used for some virtual expanders
      B8        =  8,   ///< 8-bit values
      B16       = 16,   ///< 16-bit
      B32       = 32    ///< 32 bit
    };

    /** The devices understood by this library. */
    enum ExpanderType {
      IGNORE    =  0,       ///< device is managed outside of this library...
      I2CLCD    =  0,       ///< ... handled elsewhere
      PCA9555   =  1,       ///< Bits 0  1  2  3  4  5  6  7  8   9  10  11  12  13  14  15  16
      MCP23016,             ///< Bits 0  1  2  3  4  5  6  7  8   9  10  11  12  13  14  15  16
      PCF8574,              ///< Bits 0  1  2  3  4  5  6  7  8
      PCF8574A,             ///< Bits 0  1  2  3  4  5  6  7  8
    // DAC/ADC chip
      PCF8591,              ///< 4 A-D converters, 1 D-A
    // 16 bit MAX 3713
      MAX731x,              ///< Bits 0  1  2  3  4  5  6  7  8   9  10  11  12  13  14  15  16
	  MAX7311 = MAX731x,
	  MAX7312 = MAX731x,
	  MAX7313 = MAX731x,
	// LED PWM Controller
	  PCA9685,              ///< 16 bit PWM controller

	  // Virtual Expanders - expose the pins on the MCU
	  // These tend to be 4-bit "devices" to match the IO4 Architecture used
	  // by the rest of the SPCoast interface boards.
// ARDUINO_AVR_DUEMILANOVE - built-in Arduino ports, skipping RX/TX, Lnet RX/TX and I2C pins
      ARDIO_A,              ///< 4x Bits D2   D3  D4   D5 - low digital
      ARDIO_B,              ///< 4x Bits D6   D9  D10  D11  - high digital
      ARDIO_C,              ///< 4x Bits D12  D13 A0   A1  - mixed, digital and analog
      ARDIO_D,              ///< 4x Bits A2   A3  A6  A7   - analog (A6 & A7 are input only)
// SPARK_CORE - Built in Photon Ports
      PHOTON_A,             ///< 4x Bits D2, D3, D4,  D5,  -- -- -- --
      PHOTON_B,             ///< 4x Bits D6, D7, A0,  A1,  -- -- -- --
      PHOTON_C,             ///< 4x Bits A2, A3, DAC, WKP, -- -- -- --
// ARDUINO_ESP8266_WEMOS_D1MINI - Builtin Wemos ports
      WEMOS_A,              ///< 4x Bits GPIO   4,  0,  2, 14    Pins D2 D3 D4 D5
      WEMOS_B,              ///< 4x Bits GPIO  12, 13,  3,  1    Pins D6 D7 RX TX
      WEMOS_C,              ///< 4x Bits GPIO  16, 13,  3,  1    Pins D0 D7 RX TX
      WEMOS_MATRIX,         ///< 4x Bits GPIO   4,  2, 14, 12    Pins D3 [D4 D5 D6] used by LEDCONTROL

    };

 private:
    uint8_t  _size;         ///< How many bits?
    uint8_t  _chip;         ///< device_type
    uint8_t  _address;      ///< Sequential address
    uint8_t  _i2c_address;  ///< Real I2C address
    uint16_t _config;       ///< per-device-type configuration info
    uint32_t _current;      ///< current "read" cache
    uint32_t _last;         ///< last "read"
    uint32_t _lastw;        ///< last "write"
    bool     _firsttime;    ///< private flag for changed() to force an update on first check
    boolean _debounce;      ///< should read() ensure noise-free inputs?



    /// Many I2C devices are register compatible with the 9555...
    enum PCA9555Registers {
		PCA9555_INPUT  =  0,
		PCA9555_OUTPUT =  2,
		PCA9555_INVERT =  4,
		PCA9555_CONFIG =  6
    };

    /**
        The PCF8591 is a single-chip, single-supply low-power 8-bit CMOS data acquisition
        device with four analog inputs, one analog output and a serial I2C-bus interface. Three
        address pins A0, A1 and A2 are used for programming the hardware address, allowing
        the use of up to eight devices connected to the I2C-bus without additional hardware.
        Address, control and data to and from the device are transferred serially via the two-line
        bidirectional I2C-bus.
        The functions of the device include analog input multiplexing, on-chip track and hold
        function, 8-bit analog-to-digital conversion and an 8-bit digital-to-analog conversion. The
        maximum conversion rate is given by the maximum speed of the I2C-bus.
     */
    enum PCA8591Registers {
		PCA8591_Channel1 = 0x00,
		PCA8591_Channel2,
		PCA8591_Channel3,
		PCA8591_Channel4
    };

    /**
        The PCA9685 is an I²C-bus controlled 16-channel LED controller optimized for Red/Green/Blue/Amber
        (RGBA) color backlighting applications. Each LED output has its own 12-bit resolution (4096 steps)
        fixed frequency individual PWM controller that operates at a programmable frequency from a typical
        of 24 Hz to 1526 Hz with a duty cycle that is adjustable from 0 % to 100 % to allow the LED to be
        set to a specific brightness value. All outputs are set to the same PWM frequency.

        The PCA9685 allows staggered LED output on and off times to minimize current surges.
        The on and off time delay is independently programmable for each of the 16 channels.
        It has 4096 steps (12-bit PWM) of individual LED brightness control.
        When multiple LED controllers are incorporated in a system, the PCA9685 has a programmable
        prescaler to adjust the PWM pulse widths of multiple devices.
        It has an external clock input pin that will accept user-supplied clock (50 MHz max.)
        in place of the internal 25 MHz oscillators. This feature allows synchronization of multiple devices.
        The PCA9685 has a built-in oscillator for the PWM control; the frequency is adjustable from about
        24 Hz to 1526 Hz. This allows the use of PCA9685 with external power supply controllers.
        All bits are set at the same frequency.
     */
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

	/// I2C base addresses for each chip family
    enum BaseAddress {
      base731x     = 0x10,  // has 2x contiguous address ranges: 0x10-0x2F and 0x50-0x6F, for 64x chips...
      base9555     = 0x20,
      base23016    = 0x20,
      base8574A    = 0x38,
      base8574     = 0x20,
      base8591     = 0x48,
	  base9685     = 0x40
    };


    /**
     * Print an item in binary: xxxxxxxx_xxxxxxxx
     * @param data
     */
	void printData(uint32_t data);

	/**
	 * Internal debugging helper - print info about device along with some tag string...
	 * @param tag
	 */
	void printString(const char *tag);

	/**
	 * underlying dispatch routine for reading an expander
	 * @return data from device
	 */
    uint32_t _read(void) ;
    /**
     * only write a bit to an Arduino or Photon port if the config register allows writing to it
     * @param port
     * @param data
     * @param bit
     */
    void        writeif    (uint8_t port,    uint32_t data, uint8_t bit);

    /// Implementation details for each device type


    /// 8-bit devices, such as the 8574 and 8574A
    /**
     * initialize 8-bit expanders
     *
     * @param i2caddr
     * @param config
     */
    void        init8      (uint8_t i2caddr, uint16_t config);
    /**
     * write 8-bits
     * @param data to be written
     */
    void        write8     (uint32_t data);
    /**
     * read 8-bits
     * @return  data read from device
     */
    uint32_t    read8      (void);


    /// 16-bit devices based on the 9555

    /**
     * initialize 16-bit expanders
     * @param i2caddr
     * @param config
     */
    void        init9555     (uint8_t i2caddr, uint16_t config);
    /**
     * Write 16 bits
     * @param data
     */
    void        write9555    (uint32_t data);
    /**
     * read 16 bits
     * @return data read from device
     */
    uint32_t    read9555     (void);

    /// LED controller

    /**
     * initialize controller
     * @param i2caddr
     * @param config
     */
    void        init9685     (uint8_t i2caddr, uint16_t config);
    /**
     * write data
     * @param data
     */
    void        write9685    (uint32_t data);
    /**
     * read data
     * @return data read from device
     */
    uint32_t    read9685     (void);

    /**
     * initialize 731x series - up to 64 devices...
     * @param i2caddr
     * @param config
     */
    void        init731x     (uint8_t i2caddr, uint16_t config);
    /**
     * write 16 bits
     * @param data
     */
    void        write731x    (uint32_t data);
    /**
     * read 16 bits
     * @return data read from device
     */
    uint32_t    read731x     (void);

    /// ADC controller
    /**
     * initialize ADC
     * @param i2caddr
     * @param config
     */
    void        init8591    (uint8_t i2caddr, uint16_t config);
    /**
     * write data to the D/A converter
     * @param data
     */
    void        write8591   (uint32_t data);
    /**
     * read data from the A/D converters
     * @return data read from device
     */
    uint32_t    read8591    (void);
    /**
     * read data from the A/D converters (debug/test version
     * @return data read from device
     */
    uint32_t    Xread8591    (void);

#if defined(ARDUINO_AVR_DUEMILANOVE)
    void        initArduino(void);  ///< virtual expanders ARDIO_A, ARDIO_B, ARDIO_C
    uint32_t    readArduino(void);
    void        writeArduino(uint32_t data);
#endif
#if defined(SPARK_CORE)    // Built in Photon Ports
    void        initPhoton (void);  ///< virtual expanders PHOTON_A, PHOTON_B, PHOTON_C
    uint32_t    readPhoton (void);
    void        writePhoton(uint32_t data);
#endif
#if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
    void        initWemos (void);  ///< virtual expanders WEMOS_A, WEMOS_B, WEMOS_C
    uint32_t    readWemos (void);
    void        writeWemos(uint32_t data);
#endif

};

#endif // I2Cexpander_h

