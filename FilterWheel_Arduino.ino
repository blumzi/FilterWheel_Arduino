#include <Stepper.h>
#include "Id12la.h"
#include "Ascii.h"
#include "Debug.h"

//
// Glossary:
//  The following terms are used:
//
//	position:
//		- there are 8 positions on the filter wheel, distanced 200
//			steps (full motor revolution) one from the other.
//		- on the 8-filter wheel there are filters (and RFID tags) at every position, on
//			the 4-filter wheel only one in two positions has a filter (and RFID tag).
//		- at each position the optical detector gate-pin goes LOW
//
// tag:
//	- RFID tags are located on the filter wheel, to identify the filter positions.
//	- the RFID reader will attempt to read the tags ONLY when the optical slit is detected.
//	- an RFID tag has 10 HEX characters, e.g. "7F0007F75E".
//	- an empty RFID tag indicates the reader could not read a tag.  This can occur if:
//		- there is no tag, e.g. in odd positions on a 4-filter wheel
//		- the existing tag is faulty
//
// revolution:
//		- 1600 steps
//
//	speed:
//		- the speed at which the stepper will rotate (STEPPER_NORMAL_SPEED)
//
//	direction:
//		- the stepper direction: CW (positive number of steps) or CCW (negative number of steps)
//
//  Transport layer:
//	  packet format:[STX][PAYLOAD][CRC 2][ETX]
//		- Ascii::STX - one byte, ASCII:2 (start-of-text)
//		- Ascii::ETX - one byte, ASCII:3 (end-of-text)
//		- PAYLOAD - Even number of bytes (padded with NULL, if odd), ASCII printable characters ONLY
//		- CRC - two bytes
//
//  Data layer:	command[:argument]
//		- Commands may have optional arguments, separated by a colon (:)
//
//	Commands (always initiated by the PC):
//
//	1. Get position
//		command: get-tag, argument: none
//		reply:   tag:<tag> or tag:no-tag
//		operation:
//			- If optical slot is NOT detected, search for nearest position (up to half motor revolution, first CW then CCW)
//			- When optical slot is detected, read RFID tag.
//			- If a <tag> was read, reply: tag:<tag>
//			- If no tag could be read, reply: tag:no-tag
//
//	2. Move CW or CCW
//		command: move-cw:N or move-ccw:N
//		reply:   tag:<tag> or tag:no-tag
//		operation:
//			- The arduino will rotate the motor N (default: 1) full revolution(s), in the specified direction
//			- The arduino will perform exactly the same procedure as for the 'get-tag' command
//


const int STEPPER_STEPS_PER_REV = 1600;
const int STEPPER_NORMAL_SPEED = 37;

const int STEPPER_PHASE1_PIN = 22;
const int STEPPER_PHASE2_PIN = 24;

const int PHOTO_GATE_PIN = 32;
const int PHOTO_VCC_PIN = 30;

const int RFID_RESET_PIN = 48;
const int RFID_TIR_PIN = 50;
const int RFID_RX_PIN = 52;
const int RFID_TX_PIN = 53;

#define CW(steps) (steps)				// clock-wise direction
#define CCW(steps) (-(steps))			// counter-clock-wise direction

#define ON  true
#define OFF false

Stepper stepper(STEPPER_STEPS_PER_REV, STEPPER_PHASE1_PIN, STEPPER_PHASE2_PIN);

const int blink_led_on = 500, blink_led_off = 500;
unsigned long lookAliveInterval = 5000;
unsigned long nextLookAlive = millis() + lookAliveInterval;

char hostInBuf[256], hostOutBuf[256];

#ifdef USE_SOFTWARE_SERIAL
Id12la tagReader(RFID_RX_PIN, RFID_TX_PIN, RFID_RESET_PIN, RFID_TIR_PIN);
#else
Id12la tagReader(RFID_RESET_PIN, RFID_TIR_PIN);
#endif

//
// reads a packet from the PC, on the Serial port
// blocks until a good packet is received (bad packets are silently discarded)
//
// returns: number of characters read
//
int readPacketFromHost() {
	char *p;

	if (!Serial.available())
		return 0;

	// clean the hostInBuf
	for (p = hostInBuf; (unsigned)(p - hostInBuf) < sizeof(hostInBuf); p++)
		*p = 0;

	for (p = hostInBuf; (unsigned)(p - hostInBuf) < sizeof(hostInBuf); p++) {
		while (!Serial.available())
			;
		*p = Serial.read();
		if (*p == Ascii::CR || *p == Ascii::NL) {
			*p = 0;
			break;
		}
	}

	// TODO: checksum;
	return p - hostInBuf;
}

//
// Sends a packet to the PC
//
void sendPacketToHost(String payload) {
	debug("reply: ");
	debugln(payload);
	Serial.println(payload);
}

//
// Search for the optical slit.
//  - maxSteps:  max number of steps to move in each direction
//  - stepsPerMove: how much to move each time
//
bool lookForSlit(int maxSteps = STEPPER_STEPS_PER_REV / 2, int stepsPerMove = 1) {
	int count, slitWidth;
	bool found = false;

	photoLED(ON);
	if (slitDetected()) {
		found = true;
		goto out;
	}

	// Search CW
	for (count = 1; count < maxSteps; count++) {
		if (slitDetected()) {
			for (slitWidth = 0; slitDetected(); slitWidth++)    // measure slit width
				stepper.step(CW(1));
			stepper.step(CCW(slitWidth / 2));                   // go back half slit width
			found = true;
			goto out;
		}
		stepper.step(CW(stepsPerMove));
	}

	stepper.step(CCW(maxSteps));	// go back

	// Search CCW
	for (count = 1; count < maxSteps; count++) {
		if (slitDetected()) {
			for (slitWidth = 0; slitDetected(); slitWidth++)    // measure slit width
				stepper.step(CCW(1));
			stepper.step(CW(slitWidth / 2));                    // go back half slit width
			found = true;
			goto out;
		}
		stepper.step(CCW(stepsPerMove));
	}

out:
	photoLED(OFF);
	return found;
}

void blink() {
	digitalWrite(LED_BUILTIN, HIGH);
	delay(blink_led_on);
	digitalWrite(LED_BUILTIN, LOW);
	delay(blink_led_off);
}

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	lookAlive();

	Serial.begin(57600);	// opens serial port, sets data rate to 9600 bps
	while (!Serial)
		;
	tagReader.begin();

	stepper.setSpeed(STEPPER_NORMAL_SPEED);//define  the stepper speed

	// optical-gate pins
	pinMode(PHOTO_VCC_PIN, OUTPUT);         // LED power
	digitalWrite(PHOTO_VCC_PIN, HIGH);      // turn LED on

	pinMode(PHOTO_GATE_PIN, INPUT_PULLUP);  // HIGH: no slit, LOW: slit

	debugln("filter wheel ready ...");

	// Make sure we're positioned at a filter slot
	lookForSlit();
}

//
// The photo detector gate-pin drops to ground when
//  it finds the positioning slit, and according to Ezra it
//  is accurate, i.e. drops to ground if and only if it detected
//  the slit.
//
bool slitDetected() {
	bool detected = digitalRead(PHOTO_GATE_PIN) == LOW;

	debug(detected ? 'X' : 'x');
	return detected;
}

void photoLED(bool onOff) {
	digitalWrite(PHOTO_VCC_PIN, onOff ? HIGH : LOW);
	if (onOff == true)
		delay(10);
}

//
// Tries to read the slot tag.
// 1. Searches for the optical slit
// 2. Reads the tag
//
String doGetTag() {
	String tag;

	if (!lookForSlit())
		return String("error:Cannot find slit");

	tag = tagReader.read();
	if (tag.startsWith("error:"))
		return tag;

	return String("tag:") + tag;
}

//
// Uses the stepper to rotate the wheel either CW or CCW (positions < 0).
// The positions are spaced one full stepper revolution one from the other.
//
String doMove(int positions) {
	int steps = positions * STEPPER_STEPS_PER_REV;

	debug("moving: ");
	debug(steps);
	debugln(" steps");

	stepper.step(steps);
	return doGetTag();
}

void doStep(int steps) {
	stepper.step(steps);
}

void lookAlive() {
	if ((long)(millis() - nextLookAlive) >= 0) {
		blink();
		blink();
		nextLookAlive = millis() + lookAliveInterval;
	}
}

void loop() {
	String command, reply;
	const int maxPositionsToMove = 7;
	int positions, nBytesRead;

	lookAlive();

	if ((nBytesRead = readPacketFromHost()) > 0) {
		command = String(hostInBuf);
		debug("command: ");
		debugln(command);

		if (command.startsWith("get-tag", 0)) {
			reply = doGetTag();
		}
		else if (command.startsWith("move-cw:", 0)) {
			positions = command.substring(strlen("move-cw:")).toInt();
			if (positions < 1 || positions > maxPositionsToMove)
				reply = "error:bad-param";
			else
				reply = doMove(positions);
		}
		else if (command.startsWith("move-ccw:", 0)) {
			positions = command.substring(strlen("move-ccw:")).toInt();
			if (positions < 1 || positions > maxPositionsToMove)
				reply = "error:bad-param";
			else
				reply = doMove(-positions);
		}
		else if (command.startsWith("step:", 0)) {
			int nSteps = command.substring(strlen("step:")).toInt();
			doStep(nSteps);
		}
		else if (command.startsWith("search:", 0)) {
			int nSteps = command.substring(strlen("search:")).toInt();
			lookForSlit(nSteps);
		}
		else if (command.startsWith("help", 0)) {
			reply = String("commands:\n") +
				String("  get-tag    - reads the RFID tag\n") +
				String("  move-cw:N  - moves N slots clock-wise\n") +
				String("  move-ccw:N - moves N slots counter-clock-wise\n") +
				String("  step:N     - moves N steps clock-wise (steps < 0 => counter-clock-wise)\n") +
				String("  search:N   - searches for the slit up-to N steps clock-wise, then counter-clock-wise");
		}
		if (reply.length() > 0)
			sendPacketToHost(reply);
	}
}
