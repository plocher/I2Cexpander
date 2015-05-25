I2Cexpander is an abstraction library that manages various I2C devices as IO points you can use.

It is based on an array of devices that can be read and written as desired.  
Instead of extending the digitalRead()/digitalWrite abstraction, I chose to read and write in units of 4,6, 8 or 16 bits, depending on the device in question.

To put this in context, this is part of a code-generated control system for a model railroad layout where there are many microcontrollers in use, one for every place on the layout where there are things to control.

My default program flow is

<code>
	define each layout device (signal heads, turnout controllers, occupancy detectors,...) along with which bits are used to talk to it.
	loop() {
	    read the layout state
	    walk thru every device and ask it to update itself
	    if anything changed, handle the side effects (i.e., track becomes occupied, signal needs to turn red...)
	    if needed, update outputs (i.e., write new values)
	}
</code>

== Circuit ==

Connect I2C expanders to the I2C bus, set their address jumpers...
This test code presumes you have LEDs and dropping resistors connected to your I2C expander I/O pins
See http://www.spcoast.com/wiki/index.php/IO16-8574 and http://www.spcoast.com/wiki/index.php/IO16-9555 for examples

== Getting Started:==

Here's an example of driving 2 I2C devices and the onboard pins in a cylon pattern (presuming common anode LEDs that light when driving cathode to ground thru the I2C device pin...)

<code>
#include "I2Cexpander.h"
I2Cexpander m[3+3];	// 3 I2C extenders plus 3 built in Photon "ports"

void setup() {
    Wire.stretchClock(true);
    for (int x = 0; x < NUMPORTS; x++) {
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

    // can manage regular I2C devices...
    m[ 5].init(1, I2Cexpander::PCA9555,  0); // 16-bits...
    m[ 4].init(0, I2Cexpander::PCF8574A, 0); //  8-bits...
    m[ 3].init(0, I2Cexpander::PCF8574,  0); //  8-bits...

    // ... as well as built in ports
    // (address is ignored for non-I2C pseudo devices...)
    m[ 2].init(0, I2Cexpander::PHOTON_C, 0); //  4-bits: A3, A3, A6 (DAC1), A7 (WKP)
    m[ 1].init(0, I2Cexpander::PHOTON_B, 0); //  4-bits: D6, D7, A0, A1
    m[ 0].init(0, I2Cexpander::PHOTON_A, 0); //  4-bits: D2, D3, D4, D5
}

#define DELAYCOUNT 25	// Delay for so many mS
void loop() {
    int numports = sizeof(m) / sizeof(I2Cexpander);

    for (int b = 0; b <= 15; b++) {
        for (int x = 0; x < numports; x++) {
            if (m[x].getSize() >= b) {
                m[x].put( ~(1 << b));
            }
        }
        delay(DELAYCOUNT);
    }
    for (int b = 15 b >= 0; b--) {
        for (int x = 0; x < numports; x++) {
            if (m[x].getSize() >= b) {
                m[x].put( ~(1 << b));
            }
        }
        delay(DELAYCOUNT);
    }
    for (int x = 0; x < numports; x++) {
        m[x].put(0xFFFF); // clean up...
    }
    delay(DELAYCOUNT);
}
</code>



