#ifndef	Id12la_h
#define Id12la_h


#include <Arduino.h>
#include <inttypes.h>

#define USE_SOFTWARE_SERIAL

#ifdef USE_SOFTWARE_SERIAL
#include <SoftwareSerial.h>
#endif

class Id12la
{
	public:
#ifdef USE_SOFTWARE_SERIAL
		Id12la(uint8_t rx_pin, uint8_t tx_pin, uint8_t reset_pin, uint8_t tir_pin);
#else
		Id12la(uint8_t reset_pin, uint8_t tir_pin);
#endif
        String read();
        void begin();

    private:
#ifdef USE_SOFTWARE_SERIAL
		uint8_t _rxPin;
		uint8_t _txPin;
#endif
       uint8_t _resetPin;	// out - forces the reader to take a read
	   uint8_t _tirPin;		// in  - tag-in-range pin
       void reset();
	   bool tagInRange();
};

#endif
