#include <Arduino.h>
#include <inttypes.h>
#include "Id12la.h"
#include "Ascii.h"

//
// This class drives a Id12la RFID reader from ID-Innovations
// The datasheet can be found at Sparkfun (SEN-11827): https://www.sparkfun.com/products/11827
// We used a Sparkfun breakout board (SEN-13030):      https://www.sparkfun.com/products/13030
//
// The connections to the breakboard are: (nc == not-connected)
//          +--v-----------v--+
//      5V  |o VCC       GND o| GND
//      nc  |o READ      RES o| 20 (SDA)
// 19(RX1)  |o D0        ANT o| nc
//      nc  |o D1        ANT o| nc
//     GND  |o FORM       CP o| nc
//      nc  |o TIR            |
//          +--^-----------^--+
//
// The Arduino MEGA:
//  - reads the RFID output on reader
//  - forces the RFID to re-read by resetting it via pin 20
//

const int TagPayloadBytes = 10;
const int TagTransmissionBytes = TagPayloadBytes + 6; // [STX][Payload(10)][CHK(2)][CR][NL][ETX]
//extern void blink(int, int);

Ascii ascii;
byte buf[TagTransmissionBytes];
//const int io_blink_on = 20, io_blink_off = 20;

extern void debug(String);
extern void debugln(String);

#ifdef USE_SOFTWARE_SERIAL

SoftwareSerial *reader;

Id12la::Id12la(uint8_t rxPin, uint8_t txPin, uint8_t resetPin, uint8_t tirPin) {
	this->_rxPin = rxPin;
	this->_txPin = txPin;
	this->_resetPin = resetPin;
	this->_tirPin = tirPin;

	pinMode(this->_resetPin, OUTPUT);
	//digitalWrite(this->_resetPin, HIGH);
  digitalWrite(this->_resetPin, LOW);

	pinMode(this->_tirPin, INPUT);

	reader = new SoftwareSerial(this->_rxPin, this->_txPin);
}
#else

Id12la::Id12la(uint8_t resetPin, uint8_t tirPin) {
	this->_resetPin = resetPin;
	this->_tirPin = tirPin;

	pinMode(this->_resetPin, OUTPUT);
	//digitalWrite(this->_resetPin, HIGH);
  digitalWrite(this->_resetPin, LOW);

	pinMode(this->_tirPin, INPUT);
}
#endif

//
// Initialize the reader port
//
void Id12la::begin() {
#ifdef USE_SOFTWARE_SERIAL
	reader->begin(9600);
	while (!reader)
		;
#else
	Serial1.begin(9600);
	while (!Serial1)
		;
#endif
	ascii = Ascii();
}

void Id12la::resetChip() {
	digitalWrite(this->_resetPin, LOW);
}

void Id12la::unresetChip() {
  digitalWrite(this->_resetPin, LOW);
  digitalWrite(this->_resetPin, HIGH);
  delay(250);
}

bool Id12la::tagInRange() {
	return digitalRead(this->_tirPin) == HIGH;
}

void clearBufs() {
	uint8_t *p;

	for (p = buf; (unsigned char)(p - buf) < sizeof(buf); p++)
		*p = 0;
}

#ifdef USE_SOFTWARE_SERIAL
byte readByte() { return reader->read(); }
int availableBytes() { return reader->available(); }
#else
byte readByte() { return Serial1.read(); }
int availableBytes() { return Serial1.available(); }
#endif

String dataArrivedSafely(int nbytes) {
	int computedSum, sentSum;
	uint8_t *p;

	// check we got enough bytes
	if (nbytes != TagTransmissionBytes)
		return String("error:Too short");

	// check we received the special bytes
	if (buf[0] != ascii.STX || buf[13] != ascii.CR || buf[14] != ascii.NL || buf[15] != ascii.ETX)
		return String("error:Bad special characters");

	// compute CRC
	for (computedSum = 0, p = buf + 1; p - (buf + 1) < TagPayloadBytes; p += 2)
		computedSum ^= ascii.bcd(p);
	sentSum = ascii.bcd(p);

	if (computedSum != sentSum)
		return String("error:Bad checksum");
	return String("Ok");
}

//
// Read a TAG
//
String Id12la::read() {
	byte *p = buf;
	unsigned long endOfTime;
	String status;

  unresetChip();
  
	if (!tagInRange()){
    resetChip();
		return String("error:no-tag");
	}

	clearBufs();                            // clear read buffer
	endOfTime = micros() + 1000;

	while (availableBytes() < TagTransmissionBytes + 1)
		if ((long)(micros() - endOfTime) >= 0){
      resetChip();
			return String("error:timeout");
		}

	while ((*p = readByte()) != Ascii::STX) // skip noise up to Ascii::STX
   ;
	p++;
	while ((*p++ = readByte()) != Ascii::ETX)
   ;
	*p = 0;									// NULL terminate the buffer

	for (int i = availableBytes(); i; i--) {// flush incoming data
		readByte();
	}

	status = dataArrivedSafely(p - buf);
	if (status.startsWith("error:")){
    resetChip();
		return status;
	}

	char *tag = (char*)buf + 1;
	*(tag + TagPayloadBytes) = 0;			// NULL-terminate tag

  resetChip();
	return String(tag);
}
