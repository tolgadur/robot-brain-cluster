#ifndef __FAST_IO_H
#define __FAST_IO_H

#include "mbed.h"

// Super-fast DigitalOut-like class for mbed
//   by Igor Skochinsky

// includes FastOut, FastPortOut and MaskedPortOut classes
// usage:
//   FastOut<LED2> led2;
//   FastPortOut<Port0, LED_MASK> ledport2;
//   MaskedPortOut<Port0, LED_MASK> ledport3;
//   led2 = 1;
//   ledport2 = LED_MASK;
//   ledport3 = LED_MASK;
// MaskedPortOut works the same way as FastPortOut, but it pre-sets the pin mask so that write can be done in one operation
// this makes things faster but you can't control other pins of the port, even with other classes


// pin definitions in PinNames.h start from LPC_GPIO0_BASE for P0_0 and there are 32 pins to a port (0 to 31)
// Thus:
// pin = LPC_GPIO0_BASE + port * 32 + bit
// port = (pin - LPC_GPIO0_BASE) / 32
// bit  = (pin - LPC_GPIO0_BASE) % 32

#define PORTNO(pin) (((pin) - P0_0)/32)
#define BITNO(pin)  (((pin) - P0_0)%32)

// calculate the GPIO port definition for the pin
// we rely on the fact that port structs are 0x20 bytes apart
#define PORTDEF(pin) ((LPC_GPIO_TypeDef*)(LPC_GPIO0_BASE + PORTNO(pin)*0x20))

#define PORTDEFPORT(port) ((LPC_GPIO_TypeDef*)(LPC_GPIO0_BASE + port*0x20))

// calculate the mask for the pin's bit in the port
#define PINMASK(pin) (1UL << BITNO(pin))

// each port takes two PINSEL registers (8 bytes or 64 bits)
// so there are 16 pins per PINSEL
#define PINSELREG(pin)  (*(volatile uint32_t*)(LPC_PINCON_BASE + 4*(((pin) - P0_0)/16)))
#define PINSELMASK(pin, v) (v << (((pin - P0_0)%16)*2) )

// usage: FastOut<LED2> led2;
// then use the same assignment operators as with DigitalOut
template <PinName pin> class FastOut
{
public:
    FastOut()
    {
        // set PINSEL bits to 0b00 (GPIO)
        PINSELREG(pin) &= ~PINSELMASK(pin, 3);
        // set FIODIR bit to 1 (output)
        PORTDEF(pin)->FIODIR |= PINMASK(pin);
    }
    void write(int value)
    { 
        if ( value )
            PORTDEF(pin)->FIOSET = PINMASK(pin);
        else
            PORTDEF(pin)->FIOCLR = PINMASK(pin);
    } 
    int read()
    {
        return PORTDEF(pin)->FIOPIN & PINMASK(pin) != 0;
    }
    FastOut& operator= (int value) { write(value); return *this; };
    FastOut& operator= (FastOut& rhs) { return write(rhs.read()); };
    operator int() { return read(); };
};

#define PINSELREG(pin)  (*(volatile uint32_t*)(LPC_PINCON_BASE + 4*(((pin) - P0_0)/16)))
#define PINSELMASK(pin, v) (v << (((pin - P0_0)%16)*2) )

// usage: FastPortOut<Port0, mask> led2;
// then use the same assignment operators as with DigitalOut
/*template <enum PortName port, uint32_t mask = 0xFFFFFFFF> class FastPortOut
{
public:
    FastPortOut()
    {
        // init pins selected by the mask
        uint32_t pin = LPC_GPIO0_BASE + port * 32;
        for ( uint32_t pinmask = mask; pinmask !=0; pinmask >>= 1 )
        {
            if ( pinmask & 1 )
            {
                // set PINSEL bits to 0b00 (GPIO)
                PINSELREG(pin) &= ~PINSELMASK(pin, 3);
                // set FIODIR bit to 1 (output)
                PORTDEF(pin)->FIODIR |= PINMASK(pin);
            }
            pin++;
        }
    }
    void write(int value)
    {
        PORTDEFPORT(port)->FIOSET = value & mask;
        PORTDEFPORT(port)->FIOCLR = ~value & mask;
    } 
    int read()
    {
        return PORTDEFPORT(port)->FIOPIN & mask;
    }
    FastPortOut& operator= (int value) { write(value); return *this; };
    FastPortOut& operator= (FastPortOut& rhs) { return write(rhs.read()); };
    operator int() { return read(); };
};

// usage: MaskedPortOut<Port0, mask> led2;
// then use the same assignment operators as with DigitalOut
template <enum PortName port, uint32_t mask = 0xFFFFFFFF> class MaskedPortOut
{
public:
    MaskedPortOut()
    {
        // init pins selected by the mask
        uint32_t pin = LPC_GPIO0_BASE + port * 32;
        for ( uint32_t pinmask = mask; pinmask !=0; pinmask >>= 1 )
        {
            if ( pinmask & 1 )
            {
                // set PINSEL bits to 0b00 (GPIO)
                PINSELREG(pin) &= ~PINSELMASK(pin, 3);
                // set FIODIR bit to 1 (output)
                PORTDEF(pin)->FIODIR |= PINMASK(pin);
            }
            pin++;
        }
        // set mask
        PORTDEFPORT(port)->FIOMASK = mask;
    }
    void write(int value)
    {
        PORTDEFPORT(port)->FIOPIN = value;
    } 
    int read()
    {
        return PORTDEFPORT(port)->FIOPIN;
    }
    MaskedPortOut& operator= (int value) { write(value); return *this; };
    MaskedPortOut& operator= (MaskedPortOut& rhs) { return write(rhs.read()); };
    operator int() { return read(); };

};
*/
#endif