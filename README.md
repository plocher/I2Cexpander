# I2Cexpander - I2C I/O device abstraction
[![Build Status](https://api.travis-ci.org/plocher/I2Cexpander.svg?branch=master)](https://travis-ci.org/plocher/I2Cexpander)
  -  [Library Documentation](https://plocher.github.io/I2Cexpander/html/class_i2_cexpander.html)


I2Cexpander is an abstraction library that manages various I2C devices as IO points you can use.

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

    #if defined(ARDUINO_AVR_DUEMILANOVE)
      // built-in Arduino ports, skipping RX/TX, Lnet RX/TX and I2C pins
      ARDIO_A       // Bits D2   D3  D4   D5 - low digital
      ARDIO_B       //      D6   D9  D10  D11  - high digital
      ARDIO_C       //      D12  D13 A0   A1  - mixed, digital and analog
      ARDIO_D       //      A2   A3  A6  A7   - analog (A6 & A7 are input only)
    #endif
    #if defined(SPARK_CORE) // Built in Photon Ports
      PHOTON_A      //      D2, D3, D4,  D5,  -- -- -- --
      PHOTON_B      //      D6, D7, A0,  A1,  -- -- -- --
      PHOTON_C      //      A2, A3, DAC, WKP, -- -- -- --
    #endif
    #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
      WEMOS_A       //      GPIO   4,  0,  2, 14    Pins D2 D3 D4 D5
      WEMOS_B       //      GPIO  12, 13,  3,  1    Pins D6 D7 RX TX
      WEMOS_C       //      GPIO  16, 13,  3,  1    Pins D0 D7 RX TX
    #endif
</pre>

== Circuit ==

Connect I2C expanders to the I2C bus, set their address jumpers...
This test code presumes you have LEDs and dropping resistors connected to your I2C expander I/O pins
See [I2C-7311](https://spcoast.github.io/pages/I2C-7311.html) and
[I2C-8574](https://spcoast.github.io/pages/I2C-8574.html) for examples

== Getting Started:==

Here's an example of driving 2 I2C devices and the onboard pins in a cylon pattern
(using common anode LEDs that light when driving cathode to ground thru the I2C device pin...)

<pre>
#include "I2Cexpander.h"
I2Cexpander m[3+3];	// 3 I2C extenders plus 3 built in Arduino "ports"

void setup() {
    for (int x = 0; x &lt; sizeof(m) / sizeof(class I2Cexpander); x++) {
        m[x]       = I2Cexpander();
    }
    Wire.begin();

    // initialize the I2C IO expanders
    // .init(i2c slave address, device type, IOmask
    //     IOmask:  defines which pins are inputs or outputs
    //              Inputs are "1" bits,  0xFFFF is all inputs 
    //              Outputs are "0" bits, 0x0000 is all outputs
    // .put(value)  value is masked against IOmask before being written to device,
    // .get()       return is masked against IOmask so only Input bits have meaning

    // can manage regular I2C devices, all pins "Outputs"
    m[ 5].init(1, I2Cexpander::PCA9555,  0); // 16-bits...
    m[ 4].init(0, I2Cexpander::PCF8574A, 0); //  8-bits, in a high I2C address range
    m[ 3].init(0, I2Cexpander::PCF8574,  0); //  8-bits, in a low I2C address range

    // ... as well as built in ports
    // (address is ignored for non-I2C pseudo devices...)
    m[ 2].init(0, I2Cexpander::ARDIO_C, 0); //  4-bits: D12  D13 A0   A1  - mixed, digital and analog
    m[ 1].init(0, I2Cexpander::ARDIO_B, 0); //  4-bits: D6   D9  D10  D11 - high digital
    m[ 0].init(0, I2Cexpander::ARDIO_A, 0); //  4-bits: D2   D3  D4   D5  - low digital
}

#define DELAYCOUNT 25	// Delay for so many mS between transitions
void loop() {
    int numports = sizeof(m) / sizeof(I2Cexpander);

    for (int b = 0; b &lt;= 15; b++) {
        for (int x = 0; x &lt; numports; x++) {
            if (m[x].getSize() &gt;= b) {
                m[x].put( ~(1 &lt;&lt; b));
            }
        }
        delay(DELAYCOUNT);
    }
    for (int b = 15 b &gt;= 0; b--) {
        for (int x = 0; x &lt; numports; x++) {
            if (m[x].getSize() &gt;= b) {
                m[x].put( ~(1 &lt;&lt; b));
            }
        }
        delay(DELAYCOUNT);
    }
    for (int x = 0; x &lt; numports; x++) {
        m[x].put(0xFFFF); // clean up...
    }
    delay(DELAYCOUNT);
}
</pre>



