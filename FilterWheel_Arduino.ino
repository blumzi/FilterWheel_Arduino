#include <elapsedMillis.h>

#include <Stepper.h>
#include "Id12la.h"
#include "Ascii.h"

//
// Glossary:
//  The following terms are used:
//
//  position:
//    - there are 8 positions on the filter wheel, distanced 200
//      steps (full motor revolution) one from the other.
//    - on the 8-filter wheel there are filters (and RFID tags) at every position, on
//      the 4-filter wheel only one in two positions has a filter (and RFID tag).
//    - at each position the optical detector gate-pin goes LOW
//
// tag:
//  - RFID tags are located on the filter wheel, to identify the filter positions.
//  - the RFID reader will attempt to read the tags ONLY when the optical slit is detected.
//  - an RFID tag has 10 HEX characters, e.g. "7F0007F75E".
//  - an empty RFID tag indicates the reader could not read a tag.  This can occur if:
//    - there is no tag, e.g. in odd positions on a 4-filter wheel
//    - the existing tag is faulty
//
// revolution:
//    - 1600 steps
//
//  speed:
//    - the speed at which the stepper will rotate (STEPPER_NORMAL_SPEED)
//
//  direction:
//    - the stepper direction: CW (positive number of steps) or CCW (negative number of steps)
//
//  Transport layer:
//    packet format:[STX][PAYLOAD][CRC 2][ETX]
//    - Ascii::STX - one byte, ASCII:2 (start-of-text)
//    - Ascii::ETX - one byte, ASCII:3 (end-of-text)
//    - PAYLOAD - Even number of bytes (padded with NULL, if odd), ASCII printable characters ONLY
//    - CRC - two bytes
//
//  Data layer: command[:argument]
//    - Commands may have optional arguments, separated by a colon (:)
//
//  Commands (always initiated by the PC):
//
//  1. Get position
//    command: get-tag, argument: none
//    reply:   tag:<tag> or tag:no-tag
//    operation:
//      - If optical slot is NOT detected, search for nearest position (up to half motor revolution, first CW then CCW)
//      - When optical slot is detected, read RFID tag.
//      - If a <tag> was read, reply: tag:<tag>
//      - If no tag could be read, reply: tag:no-tag
//
//  2. Move CW or CCW
//    command: move-cw:N or move-ccw:N
//    reply:   tag:<tag> or tag:no-tag
//    operation:
//      - The arduino will rotate the motor N (default: 1) full revolution(s), in the specified direction
//      - The arduino will perform exactly the same procedure as for the 'get-tag' command
//

const int DEBUG_PIN = 26;

const int STEPPER_STEPS_PER_REV = 1600;
const int STEPPER_NORMAL_SPEED = 37;

const int STEPPER_PHASE1_PIN = 34;
const int STEPPER_PHASE2_PIN = 36;

const int SLIT_DETECTOR_GATE_PIN = 32;
const int SLIT_DETECTOR_VCC_PIN = 30;

const int CD2_PIN = 41;  // when LOW, detects connector #2 
const int CD3_PIN = 43;  // when LOW, detects connector #3
const int CD4_PIN = 45;  // when LOW, detects connector #4

const int RFID_RESET_PIN = 48;
const int RFID_TIR_PIN = 50;
const int RFID_RX_PIN = 52;
const int RFID_TX_PIN = 53;

const int LOOKALIVE_LED_PIN = 46;

#define CW(steps) (steps)       // clock-wise direction
#define CCW(steps) (-(steps))     // counter-clock-wise direction

#define ON  true
#define OFF false

Stepper stepper(STEPPER_STEPS_PER_REV, STEPPER_PHASE1_PIN, STEPPER_PHASE2_PIN);

const int blink_led_on = 500, blink_led_off = 500;
unsigned long lookAliveInterval = 5000;
elapsedMillis timeFromLastlookAlive = 0;

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
  //char *p, c;

//  if (!Serial.available())
//    return 0;

  memset(hostInBuf, 0, sizeof(hostInBuf));
//  if (debugging()) {
//    for (p = hostInBuf; ; p++) {
//      while (!Serial.available())
//        delay(10);
//      *p = Serial.read();
//      if (*p == Ascii::NL || *p == Ascii::CR)
//        break;
//    }
//  } else {
//    bool inPayload = false;
//    
//    for (p = hostInBuf; ; p++) {
//      c = Serial.read();
//
//      if (c ==  Ascii::STX) {
//        inPayload = true;
//        continue;
//      } else if (c == Ascii::ETX)
//          return;
//      else if (inPayload)
//        *p++ = c;
//    }
    
    //Serial.readStringUntil(Ascii::STX);
    //Serial.readBytesUntil(Ascii::ETX, hostInBuf, sizeof(hostInBuf));
    int nRead;
    
    if ((nRead = Serial.readBytesUntil('\r', hostInBuf, sizeof(hostInBuf))) > 0) {
      hostInBuf[nRead] = 0;
      //debugln("got ==" + String(hostInBuf) + "==");
      return nRead;
    }
//  }
//  if (((p = strchr(hostInBuf, Ascii::NL)) != NULL) || ((p = strchr(hostInBuf, Ascii::CR)) != NULL))
//    *p = 0;

  return 0;
}

//
// Sends a packet to the PC
//
void sendPacketToHost(String payload) {
  Serial.println(payload);
}

//
// Search for the optical slit.
//  - maxSteps:  max number of steps to move in each direction
//  - stepsPerMove: how much to move each time
//
bool lookForSlit(int maxSteps = (STEPPER_STEPS_PER_REV / 2) + 20, int stepsPerMove = 1) {
  int count, slitWidth;
  bool found = false;

  SlitDetectorLed(ON);
  debugln("Led ON");
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

  stepper.step(CCW(maxSteps));  // go back

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
  SlitDetectorLed(OFF);
  debugln(String("\r\n") + "Led Off");
  return found;
}

/*
 * This is s synchronous blink, i.e. the program continues only after the 
 *  blinking is done.
 */
void blink(int nblinks, int onMillis = blink_led_on, int offMillis = blink_led_off) {
  for (; nblinks; nblinks--) {
    digitalWrite(LOOKALIVE_LED_PIN, HIGH);
    delay(onMillis);
    digitalWrite(LOOKALIVE_LED_PIN, LOW);
    delay(offMillis);
  }
}

void setup() {
  pinMode(DEBUG_PIN, INPUT_PULLUP);

  // cable detect pins
  pinMode(CD2_PIN, INPUT_PULLUP);
  pinMode(CD3_PIN, INPUT_PULLUP);
  pinMode(CD4_PIN, INPUT_PULLUP);

  pinMode(LOOKALIVE_LED_PIN, OUTPUT);

  Serial.begin(57600);  // opens serial port, sets data rate to 57600 bps
  while (!Serial)
    ;
  tagReader.begin();   

  stepper.setSpeed(STEPPER_NORMAL_SPEED);    //define  the stepper speed

  // slit detector pins
  pinMode(SLIT_DETECTOR_VCC_PIN, OUTPUT);         // IR LED power
  SlitDetectorLed(OFF);
  pinMode(SLIT_DETECTOR_GATE_PIN, INPUT_PULLUP);  // HIGH: no slit, LOW: slit

  // Make sure we're positioned at a filter slot
  //lookForSlit();  // TBD: scoot a bit and look again, for 4 slot wheels

  String tag = doGetTag(); 
  if (tag.startsWith("error:no-tag"))
    doMove(1);
    
  blink(3);
}

//
// The photo detector gate-pin drops to ground when
//  it finds the positioning slit, and according to Ezra it
//  is accurate, i.e. drops to ground if and only if it detected
//  the slit.
//
bool slitDetected() {
  for (int i = 0; i < 10; i++)
    if (digitalRead(SLIT_DETECTOR_GATE_PIN) == LOW) {
      debug(String('X'));
      return true;
    }
  debug(String('x'));
  return false;
}

void SlitDetectorLed(bool onOff) {
  digitalWrite(SLIT_DETECTOR_VCC_PIN, onOff ? HIGH : LOW);
  if (onOff == true)
    delay(100);
}

String checkConnectors() {
  bool c2 = digitalRead(CD2_PIN) == HIGH;  // HIGH == not-connected
  bool c3 = digitalRead(CD3_PIN) == HIGH;
  bool c4 = digitalRead(CD4_PIN) == HIGH;

  if (c2 || c3 || c4) {
    String msg = String("error:connector:");
    
    if (c2) msg += String("2 ");
    if (c3) msg += String("3 ");
    if (c4) msg += String("4 ");

    msg.trim();
    return msg;
  } else
    return "ok";
}

/*
 * This is for debugging ONLY
 */
String getConnectorDetectPins() {
  String msg = String("");

  msg += String("CD2_PIN: ")   + String(digitalRead(CD2_PIN) == HIGH ? "not-connected" : "connected");
  msg += String(", CD3_PIN: ") + String(digitalRead(CD3_PIN) == HIGH ? "not-connected" : "connected");
  msg += String(", CD4_PIN: ") + String(digitalRead(CD4_PIN) == HIGH ? "not-connected" : "connected");
  msg += String("\r\n");
  msg += checkConnectors() + String("\r\n");

  return msg;
}

//
// Tries to read the slot tag.
// 1. Searches for the optical slit
// 2. Reads the tag
//
String doGetTag() {
  String reply;

  if ((reply = checkConnectors()) != "ok")
    return reply;
  
  if (!lookForSlit())
    return String("error:no-slit");

  reply = tagReader.read();
  if (reply.startsWith("error:no-tag")){
    /*
     * We found a slit but not a tag.  This may be a
     *  four-position wheel.  We'll move one position
     *  clock-wise and look for a tag.
     */
    reply = doMove(1);
    if (reply.startsWith("error:no-tag"))
      return reply;
  }
  return String("tag:") + reply;
}

//
// Uses the stepper to rotate the wheel either CW or CCW (positions < 0).
// The positions are spaced one full stepper revolution one from the other.
//
String doMove(int positions) {
  stepper.step(positions * STEPPER_STEPS_PER_REV);
  return doGetTag();
}

/*
 * Performs a number of steps
 */
void doStep(int steps) {
  stepper.step(steps);
}

/*
 * This is an asynchronous blink.  It is called from the main loop.
 */
void lookAlive() {
  static long unsigned int lookAlive_delay = 100;
  static long unsigned int intervals[] = {
    lookAliveInterval + 10 * lookAlive_delay,	// 0: go high
    lookAliveInterval +  1 * lookAlive_delay,	// 1: go low
    lookAliveInterval +  2 * lookAlive_delay,	// 2: go high
    lookAliveInterval +  3 * lookAlive_delay,	// 3: go low and reset
  };
  static int counter = 0;
  static int state = LOW;

  if (timeFromLastlookAlive > intervals[counter]) {
    state ^= 1;
    digitalWrite(LOOKALIVE_LED_PIN, state);
    counter++;
    if (counter == 4) {
      counter = 0;
      timeFromLastlookAlive = 0;
    }
  }
}

String toggleSlitDetectorIRLed() {
  int val = digitalRead(SLIT_DETECTOR_VCC_PIN);

  if (val == HIGH)
    SlitDetectorLed(OFF);
  else
    SlitDetectorLed(ON);

  return digitalRead(SLIT_DETECTOR_VCC_PIN) == HIGH ? "ON" : "OFF";    
}

// Main loop
void loop() {
  String command, reply;
  const int maxPositionsToMove = 7;
  int positions, nBytesRead;

  lookAlive();

  if ((nBytesRead = readPacketFromHost()) > 0) {
    command = String(hostInBuf);
    //debugln("command: " + command);

    if (command.startsWith("get-tag", 0) || command.startsWith("tag", 0)) {
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
    else if (command.startsWith("cd")) {
      reply = getConnectorDetectPins();
    }
    else if (command.startsWith("l")) {
      reply = toggleSlitDetectorIRLed();
    }
    else if (command.startsWith("help", 0) || command.startsWith("?", 0)) {
      reply = String("commands:\r\n") +
        String("  get-tag    - reads the RFID tag\r\n") +
        String("  move-cw:N  - moves N slots clock-wise\r\n") +
        String("  move-ccw:N - moves N slots counter-clock-wise\r\n") +
        String("  cdpins     - show the status of the connector-detect pins\r\n") +
        String("  step:N     - moves N steps clock-wise (steps < 0 => counter-clock-wise)\r\n") +
        String("  l          - toggles slit detector Infra Red led\r\n") +
        String("  d          - detects the slit\r\n") +
        String("  search:N   - searches for the slit up-to N steps clock-wise, then counter-clock-wise\r\n");
    }
    else if (command.startsWith("d", 0)) {
      reply = slitDetected() ? String("SLIT") : String("NO-SLIT");
    }
    if (reply.length() > 0)
      sendPacketToHost(reply);
  }
}

bool debugging() {
  return digitalRead(DEBUG_PIN) == LOW;
}

void debug(String message) {
  if (!debugging())
    return;

  Serial.print(message);
}

void debugln(String message) {
  debug(message + "\r\n");
}
