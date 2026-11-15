// Macro Motor Controller v12
// For controlling a stepper motor-driven focus-stacking camera rig for macro photography.
// Can also be used as just an intervalometer. Serial + Bluetooth control, WiFi firmware update.
// David M. Helton - ESP32 Edition
//
// Changes from v11:
//  - The last frame of a stack now gets the same settle delay as every other frame
//  - Step timing is done in microseconds (no more whole-millisecond truncation),
//    and the step pulse has a guaranteed minimum width
//  - Commands are read a whole line at a time and range-checked, so typos can't
//    start a run, zero a delay or hang the controller. Settings are locked during a run
//  - Position tracking: mark a start (a) and end (b) point, return to start (g),
//    relative jog (j), optional backlash take-up (k)
//  - Step size can be given in microns (e)
//  - Settings are saved to flash and restored at power-up
//  - WiFi update: connect timeout, long passwords, PIN-protected page, no internet
//    needed for the page, and 'p' leaves upload mode again
//  - Nothing in the main loop blocks any more (shutter pulse, waits and motor moves
//    are all timed), so stop is always immediate

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <Preferences.h>
#include "BluetoothSerial.h"

// ---------------------------------------------------------------- Hardware

//Change this to fit the number of steps per revolution for your motor
const int stepsPerRevolution = 2038;
//Linear travel of the rail per motor step: 4mm per revolution of the knob
const float mmPerStep = 4.0f / stepsPerRevolution;
//Fastest speed the motor can be asked to run at
const float MAX_RPM = 15.0f;

//Pin that controls the motor direction (LOW = clockwise)
const int directionPin = 27;
//Pin that controls the motor steps
const int stepPin = 26;
//Pin that enables motor power (LOW = enabled)
const int enablePin = 25;
//Camera shutter output
const int shutterPin = 21;
//Onboard LED
const int ledPin = 2;
//Pushbutton to jog clockwise
const int ccbuttonPin = 19;
//Pushbutton to jog counterclockwise
const int ccwbuttonPin = 18;
//Pushbutton to fire the shutter manually ("Trig" on the PCB)
const int shutterbuttonPin = 16;

//Keep the motor powered between moves. Leave false for a geared motor on a
//leadscrew (cooler, no vibration). Set true if you use microstepping, because
//a microstepping driver lets the rotor relax to a full step when disabled.
const bool HOLD_TORQUE = false;

//Width of the step pulse and direction setup time, microseconds
const unsigned int STEP_PULSE_US = 5;
//Time between enabling the driver and the first step, microseconds
const unsigned long ENABLE_SETTLE_US = 2000;
//How long the shutter output is held, milliseconds
const unsigned long SHUTTER_PULSE_MS = 100;
//A command with no line ending is accepted after this much silence, milliseconds
const unsigned long INPUT_IDLE_MS = 300;

// ---------------------------------------------------------------- Settings (saved to flash)

float motorRPM = 8;         // r
long runLimit = 1;          // m - number of moves; frames taken = moves + 1; 0 = run until stopped
long shotdelayTime = 900;   // y - ms to wait after each shot (cover the exposure + card write)
long delayTime = 1000;      // t - ms to wait after each move for vibration to die down
long numSteps = stepsPerRevolution; // q - motor steps per move; 0 = intervalometer only
int direction = 0;          // d - 1 = clockwise, 0 = counterclockwise
long backlashSteps = 0;     // k - steps of gear slack to take up when reversing; 0 = off

// ---------------------------------------------------------------- Types and state

enum SeqState {
  SEQ_IDLE,
  SEQ_PREP,             // taking up backlash before the first frame
  SEQ_SHOOTING,         // shutter output is high
  SEQ_WAIT_AFTER_SHOT,
  SEQ_MOVING,
  SEQ_SETTLING
};

struct LineReader {
  Stream* port;
  char buf[48];
  uint8_t len;
  unsigned long lastCharMs;
};

BluetoothSerial SerialBT;
WebServer server(80);
Preferences prefs;

//Everything printed to "out" goes to both USB serial and Bluetooth
class Tee : public Print {
public:
  size_t write(uint8_t c) override {
    Serial.write(c);
    SerialBT.write(c);
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    Serial.write(b, n);
    SerialBT.write(b, n);
    return n;
  }
};
Tee out;

LineReader usbReader = { &Serial, "", 0, 0 };
LineReader btReader = { &SerialBT, "", 0, 0 };

//Sequence
SeqState seqState = SEQ_IDLE;
unsigned long seqTimerMs = 0;
long framesTaken = 0;
int seqDir = 1;

//Motor. Position is in steps from wherever the rail was at power-up, clockwise positive.
long position = 0;
int lastMoveDir = 0;        // direction of the most recent step, +1 / -1, 0 = unknown
bool moveActive = false;
long moveLegs[2] = { 0, 0 };
uint8_t moveLegIdx = 0;
unsigned long nextStepUs = 0;
bool jogging = false;

//Marked start/end points
bool hasA = false, hasB = false;
long pointA = 0, pointB = 0;

//Shutter
bool shutterActive = false;
unsigned long shutterOnMs = 0;
bool shutterButtonWasDown = false;
unsigned long shutterButtonMs = 0;

//Bluetooth / WiFi update
bool btConnected = false;
bool otaMode = false;
bool otaAuthed = false;
bool otaUploadOk = false;
const char* otaHost = "macro";
const char* otaUser = "macro";
String otaPin;

// ---------------------------------------------------------------- Prototypes

void printMenu();
void printStatus();
void printDuration(long seconds);
void saveSettings();
void loadSettings();
void driver(bool on);
void stepOnce(int dir);
unsigned long stepIntervalUs();
bool stepDue();
void startMove(long leg1, long leg2);
void serviceMotor();
void serviceButtons();
void fireShutter();
void serviceShutter();
void startSequence();
void stopAll(bool announce);
void takeFrame();
void finishSequence();
void serviceSequence();
void pollInput(LineReader& r);
void handleLine(char* line, Stream& src);
bool parseLong(const char* s, long& v);
bool parseFloatArg(const char* s, float& v);
bool setLong(const char* arg, long lo, long hi, long& dst, const char* name);
bool readLineBlocking(Stream& port, String& line, unsigned long timeoutMs);
void startWifiUpload(Stream& src);
void stopWifiUpload();
void setupUpdateServer();
void checkBluetoothConnection();

// ---------------------------------------------------------------- Update page

const char updatePage[] =
  "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>Macro Stacker Update</title></head>"
  "<body style='font-family:sans-serif;max-width:28em;margin:2em auto;padding:0 1em'>"
  "<h3>Macro Stacker firmware update</h3>"
  "<p><input type='file' id='f' accept='.bin'></p>"
  "<p><button id='b'>Upload</button></p><p id='s'></p>"
  "<script>"
  "document.getElementById('b').onclick=function(){"
  "var f=document.getElementById('f').files[0],s=document.getElementById('s');"
  "if(!f){s.textContent='Choose a .bin file first';return;}"
  "var d=new FormData();d.append('update',f);"
  "var x=new XMLHttpRequest();"
  "x.upload.onprogress=function(e){if(e.lengthComputable)"
  "s.textContent='Uploading '+Math.round(e.loaded/e.total*100)+'%';};"
  "x.onload=function(){s.textContent=x.status==200?"
  "'Done - the controller is rebooting':'Update failed: '+x.responseText;};"
  "x.onerror=function(){s.textContent='Connection lost';};"
  "x.open('POST','/update');x.send(d);};"
  "</script></body></html>";

// ---------------------------------------------------------------- Setup / loop

void setup() {
  //Put the outputs in a safe state before anything else
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);
  pinMode(shutterPin, OUTPUT);
  digitalWrite(shutterPin, LOW);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  pinMode(ccbuttonPin, INPUT_PULLUP);
  pinMode(ccwbuttonPin, INPUT_PULLUP);
  pinMode(shutterbuttonPin, INPUT_PULLUP);

  Serial.begin(115200);
  SerialBT.begin("Macro ESP32"); //Bluetooth device name

  loadSettings();
  setupUpdateServer();
  printMenu();
}

void loop() {
  checkBluetoothConnection();
  pollInput(usbReader);
  pollInput(btReader);
  serviceButtons();
  serviceShutter();
  serviceMotor();
  serviceSequence();

  if (otaMode) {
    server.handleClient();
  }
  //Nothing time-critical going on: give the radio tasks some room
  if (seqState == SEQ_IDLE && !moveActive && !jogging) {
    delay(1);
  }
}

// ---------------------------------------------------------------- Menu / status

void printMenu() {
  out.println();
  out.println("Macro Motor Controller v12 - David M. Helton");
  out.println("Send one command per line, e.g. m50");
  out.println("s - start stack      p - stop (also leaves wifi upload mode)");
  out.print("m - number of moves (currently= ");
  out.print(runLimit);
  if (runLimit > 0) {
    out.print(" = ");
    out.print(runLimit + 1);
    out.println(" frames)");
  } else {
    out.println(" = run until stopped)");
  }
  out.print("q - motor steps per move (currently= ");
  out.print(numSteps);
  out.print(" = ");
  out.print(mmPerStep * numSteps, 4);
  out.println("mm), 0 = intervalometer");
  out.print("e - step size in microns instead of steps (1 step = ");
  out.print(mmPerStep * 1000.0f, 2);
  out.println("um)");
  out.print("y - delay after shot (currently= ");
  out.print(shotdelayTime);
  out.println("ms)");
  out.print("t - delay after moving motor (currently= ");
  out.print(delayTime);
  out.println("ms)");
  out.print("d - motor direction 1=cw 0=ccw (currently= ");
  out.print(direction);
  out.println(direction == 1 ? " cw)" : " ccw)");
  out.print("r - motor speed in RPM (currently= ");
  out.print(motorRPM);
  out.print(", max=");
  out.print(MAX_RPM, 0);
  out.println(")");
  out.print("k - backlash take-up in steps (currently= ");
  out.print(backlashSteps);
  out.println(", 0=off)");
  out.println("a - mark start here   b - mark end here (sets direction and moves)");
  out.println("g - go to marked start   j - jog by steps, e.g. j200 or j-200");
  out.println("w - show position and status");
  out.println("i - quick snap   u - upload bin file via wifi   x - reset");
  out.println("h - display this menu again");
}

void printStatus() {
  out.print("Position: ");
  out.print(position);
  out.print(" steps (");
  out.print(position * mmPerStep, 3);
  out.println("mm from power-up position)");
  out.print("Start mark: ");
  if (hasA) out.println(pointA); else out.println("not set");
  out.print("End mark: ");
  if (hasB) out.println(pointB); else out.println("not set");
  out.print("State: ");
  if (seqState != SEQ_IDLE) {
    out.print("running, ");
    out.print(framesTaken);
    out.print(" frames taken");
    if (runLimit > 0) {
      out.print(" of ");
      out.print(runLimit + 1);
    }
    out.println();
  } else if (moveActive) {
    out.println("moving");
  } else if (otaMode) {
    out.println("wifi upload mode");
  } else {
    out.println("idle");
  }
}

void printDuration(long s) {
  if (s < 60) {
    out.print(s);
    out.println(" (S)");
    return;
  }
  long h = s / 3600, m = (s % 3600) / 60, sec = s % 60;
  if (h > 0) {
    out.print(h);
    out.print(":");
    if (m < 10) out.print("0");
  }
  out.print(m);
  out.print(":");
  if (sec < 10) out.print("0");
  out.print(sec);
  out.println(h > 0 ? " (H:M:S)" : " (M:S)");
}

// ---------------------------------------------------------------- Saved settings

void saveSettings() {
  prefs.begin("mmc", false);
  prefs.putFloat("rpm", motorRPM);
  prefs.putLong("moves", runLimit);
  prefs.putLong("shotdelay", shotdelayTime);
  prefs.putLong("movedelay", delayTime);
  prefs.putLong("steps", numSteps);
  prefs.putInt("dir", direction);
  prefs.putLong("backlash", backlashSteps);
  prefs.end();
}

void loadSettings() {
  prefs.begin("mmc", true);
  motorRPM = prefs.getFloat("rpm", motorRPM);
  runLimit = prefs.getLong("moves", runLimit);
  shotdelayTime = prefs.getLong("shotdelay", shotdelayTime);
  delayTime = prefs.getLong("movedelay", delayTime);
  numSteps = prefs.getLong("steps", numSteps);
  direction = prefs.getInt("dir", direction);
  backlashSteps = prefs.getLong("backlash", backlashSteps);
  prefs.end();
  //Never trust what comes back from flash
  if (!(motorRPM >= 0.1f && motorRPM <= MAX_RPM)) motorRPM = 8;
  runLimit = constrain(runLimit, 0L, 100000L);
  shotdelayTime = constrain(shotdelayTime, 0L, 3600000L);
  delayTime = constrain(delayTime, 0L, 3600000L);
  numSteps = constrain(numSteps, 0L, 1000000L);
  direction = direction == 1 ? 1 : 0;
  backlashSteps = constrain(backlashSteps, 0L, 5000L);
}

// ---------------------------------------------------------------- Motor

void driver(bool on) {
  digitalWrite(enablePin, on ? LOW : HIGH);
}

unsigned long stepIntervalUs() {
  return (unsigned long)(60000000.0f / motorRPM / stepsPerRevolution);
}

//One step. dir is +1 (clockwise) or -1 (counterclockwise)
void stepOnce(int dir) {
  digitalWrite(directionPin, dir > 0 ? LOW : HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(stepPin, LOW);
  position += dir;
  lastMoveDir = dir;
}

//True when it is time for the next step, and schedules the one after
bool stepDue() {
  unsigned long now = micros();
  if ((long)(now - nextStepUs) < 0) return false;
  unsigned long interval = stepIntervalUs();
  //If we fell behind (a slow print, say) don't sprint to catch up
  if ((unsigned long)(now - nextStepUs) > interval) nextStepUs = now;
  nextStepUs += interval;
  return true;
}

//Start a move of leg1 steps followed by leg2 steps (signed, clockwise positive)
void startMove(long leg1, long leg2) {
  moveLegs[0] = leg1;
  moveLegs[1] = leg2;
  moveLegIdx = 0;
  if (leg1 == 0 && leg2 == 0) return;
  jogging = false;
  moveActive = true;
  driver(true);
  nextStepUs = micros() + ENABLE_SETTLE_US;
}

void serviceMotor() {
  if (!moveActive) return;
  while (moveLegIdx < 2 && moveLegs[moveLegIdx] == 0) moveLegIdx++;
  if (moveLegIdx >= 2) {
    moveActive = false;
    if (!HOLD_TORQUE || seqState == SEQ_IDLE) driver(false);
    return;
  }
  if (!stepDue()) return;
  int dir = moveLegs[moveLegIdx] > 0 ? 1 : -1;
  stepOnce(dir);
  moveLegs[moveLegIdx] -= dir;
}

// ---------------------------------------------------------------- Buttons and shutter

void serviceButtons() {
  //Manual shutter button: one shot per press
  bool down = digitalRead(shutterbuttonPin) == LOW;
  if (down != shutterButtonWasDown && millis() - shutterButtonMs > 50) {
    shutterButtonWasDown = down;
    shutterButtonMs = millis();
    if (down && seqState == SEQ_IDLE) fireShutter();
  }

  //Jog buttons only work when nothing else is driving the motor
  bool idle = seqState == SEQ_IDLE && !moveActive;
  bool cw = digitalRead(ccbuttonPin) == LOW;
  bool ccw = digitalRead(ccwbuttonPin) == LOW;
  if (idle && cw != ccw) {
    if (!jogging) {
      jogging = true;
      driver(true);
      nextStepUs = micros() + ENABLE_SETTLE_US;
    }
    if (stepDue()) stepOnce(cw ? 1 : -1);
  } else if (jogging) {
    jogging = false;
    if (idle) driver(false);
  }
}

void fireShutter() {
  digitalWrite(shutterPin, HIGH);
  digitalWrite(ledPin, HIGH);
  shutterActive = true;
  shutterOnMs = millis();
}

void serviceShutter() {
  if (shutterActive && millis() - shutterOnMs >= SHUTTER_PULSE_MS) {
    digitalWrite(shutterPin, LOW);
    digitalWrite(ledPin, otaMode ? HIGH : LOW);
    shutterActive = false;
  }
}

// ---------------------------------------------------------------- Stack sequence
// frame -> wait (y) -> move -> settle (t) -> frame -> ... -> last frame -> wait (y) -> done
// Every frame, including the last, is taken after a full settle delay.

void startSequence() {
  if (seqState != SEQ_IDLE) {
    out.println("Already running");
    return;
  }
  if (moveActive) {
    out.println("Motor is still moving, try again in a moment");
    return;
  }
  if (otaMode) {
    out.println("In wifi upload mode, send p to leave it first");
    return;
  }
  framesTaken = 0;
  seqDir = direction == 1 ? 1 : -1;
  out.println("Started");
  if (hasA && hasB && position != pointA) {
    out.println("Note: not at the marked start (send p, then g, to go there first)");
  }
  if (HOLD_TORQUE) driver(true);
  //If the last thing the motor did was turn the other way, take up the slack
  //now so the first move travels the same distance as all the others
  if (backlashSteps > 0 && numSteps > 0 && lastMoveDir != seqDir) {
    out.println("Taking up backlash");
    startMove(-seqDir * backlashSteps, seqDir * backlashSteps);
    seqState = SEQ_PREP;
  } else {
    takeFrame();
  }
}

void stopAll(bool announce) {
  bool wasBusy = seqState != SEQ_IDLE || moveActive;
  seqState = SEQ_IDLE;
  moveActive = false;
  jogging = false;
  driver(false);
  if (announce) {
    if (wasBusy) {
      out.print("Stopped at position ");
      out.println(position);
    } else if (!otaMode) {
      out.println("Stopped");
    }
  }
  if (otaMode) stopWifiUpload();
}

void takeFrame() {
  fireShutter();
  framesTaken++;
  seqState = SEQ_SHOOTING;
  out.print("Frame ");
  out.print(framesTaken);
  if (runLimit > 0) {
    long total = runLimit + 1;
    long left = total - framesTaken;
    out.print(" of ");
    out.print(total);
    out.println(" taken");
    if (left > 0) {
      float moveMs = (float)stepIntervalUs() * numSteps / 1000.0f;
      float perFrameMs = moveMs + delayTime + SHUTTER_PULSE_MS + shotdelayTime;
      out.print("Estimated time left: ");
      printDuration((long)((left * perFrameMs + shotdelayTime) / 1000.0f));
    }
  } else {
    out.println(" taken");
  }
}

void finishSequence() {
  seqState = SEQ_IDLE;
  driver(false);
  out.print("Stack complete: ");
  out.print(framesTaken);
  out.print(" frames over ");
  out.print((framesTaken - 1) * numSteps * mmPerStep, 3);
  out.println("mm");
  if (hasA) out.println("Send g to return to the start mark");
}

void serviceSequence() {
  switch (seqState) {
    case SEQ_IDLE:
      break;

    case SEQ_PREP:
      if (!moveActive) {
        seqState = SEQ_SETTLING;
        seqTimerMs = millis();
      }
      break;

    case SEQ_SHOOTING:
      if (!shutterActive) {
        seqState = SEQ_WAIT_AFTER_SHOT;
        seqTimerMs = millis();
      }
      break;

    case SEQ_WAIT_AFTER_SHOT:
      if (millis() - seqTimerMs < (unsigned long)shotdelayTime) break;
      if (runLimit > 0 && framesTaken > runLimit) {
        finishSequence();
      } else if (numSteps == 0) {
        //Intervalometer: no movement, just the two delays between frames
        seqState = SEQ_SETTLING;
        seqTimerMs = millis();
      } else {
        startMove(seqDir * numSteps, 0);
        seqState = SEQ_MOVING;
      }
      break;

    case SEQ_MOVING:
      if (!moveActive) {
        out.print("Motor moved");
        if (runLimit > 0) {
          out.print(", distance left: ");
          out.print((runLimit - framesTaken) * numSteps * mmPerStep, 3);
          out.print("mm");
        }
        out.println();
        seqState = SEQ_SETTLING;
        seqTimerMs = millis();
      }
      break;

    case SEQ_SETTLING:
      if (millis() - seqTimerMs >= (unsigned long)delayTime) takeFrame();
      break;
  }
}

// ---------------------------------------------------------------- Command input

//Collects characters into a line. A line ends at CR/LF, or after a short
//silence so terminals that send no line ending still work.
void pollInput(LineReader& r) {
  while (r.port->available() > 0) {
    char c = (char)r.port->read();
    if (c == '\r' || c == '\n') {
      if (r.len > 0) {
        r.buf[r.len] = 0;
        r.len = 0;
        handleLine(r.buf, *r.port);
      }
    } else if (r.len < sizeof(r.buf) - 1) {
      r.buf[r.len++] = c;
      r.lastCharMs = millis();
    }
  }
  if (r.len > 0 && millis() - r.lastCharMs > INPUT_IDLE_MS) {
    r.buf[r.len] = 0;
    r.len = 0;
    handleLine(r.buf, *r.port);
  }
}

bool parseLong(const char* s, long& v) {
  char* end;
  long parsed = strtol(s, &end, 10);
  if (end == s) return false;
  while (*end == ' ') end++;
  if (*end != 0) return false;
  v = parsed;
  return true;
}

bool parseFloatArg(const char* s, float& v) {
  char* end;
  float parsed = strtof(s, &end);
  if (end == s) return false;
  while (*end == ' ') end++;
  if (*end != 0) return false;
  v = parsed;
  return true;
}

//Parse and range-check a whole-number setting. Leaves dst alone on any error.
bool setLong(const char* arg, long lo, long hi, long& dst, const char* name) {
  long v;
  if (!parseLong(arg, v) || v < lo || v > hi) {
    out.print(name);
    out.print(" needs a number from ");
    out.print(lo);
    out.print(" to ");
    out.print(hi);
    out.print(" (currently ");
    out.print(dst);
    out.println(")");
    return false;
  }
  dst = v;
  saveSettings();
  return true;
}

void handleLine(char* line, Stream& src) {
  //Trim leading and trailing spaces
  while (*line == ' ') line++;
  int n = strlen(line);
  while (n > 0 && line[n - 1] == ' ') line[--n] = 0;
  if (n == 0) return;

  char cmd = tolower(line[0]);
  const char* arg = line + 1;
  while (*arg == ' ') arg++;
  bool hasArg = *arg != 0;
  bool running = seqState != SEQ_IDLE;

  //Commands that take no number must be sent on their own, so a typed word
  //like "stop" is rejected instead of being read as "s"
  if (strchr("sphwixuabg", cmd) && hasArg) {
    out.print("Unknown command '");
    out.print(line);
    out.println("' - send h for help");
    return;
  }

  //These always work
  switch (cmd) {
    case 'p': stopAll(true); return;
    case 'h': printMenu(); return;
    case 'w': printStatus(); return;
    case 'x':
      stopAll(false);
      out.println("Restarting...");
      delay(200);
      ESP.restart();
      return;
  }

  //Everything else is refused during a stack so a run can't be changed half way
  if (running) {
    out.println("Stack is running - send p to stop it first");
    return;
  }

  switch (cmd) {
    case 's':
      startSequence();
      break;

    case 'i':
      fireShutter();
      out.println("Snap");
      break;

    case 'm':
      if (setLong(arg, 0, 100000, runLimit, "m (moves)")) {
        out.print("Moves set to: ");
        out.print(runLimit);
        if (runLimit > 0) {
          out.print(" (");
          out.print(runLimit + 1);
          out.println(" frames)");
        } else {
          out.println(" (runs until stopped)");
        }
      }
      break;

    case 'y':
      if (setLong(arg, 0, 3600000, shotdelayTime, "y (delay after shot, ms)")) {
        out.print("Delay after shot set to: ");
        out.println(shotdelayTime);
      }
      break;

    case 't':
      if (setLong(arg, 0, 3600000, delayTime, "t (delay after move, ms)")) {
        out.print("Delay after motor set to: ");
        out.println(delayTime);
      }
      break;

    case 'q':
      if (setLong(arg, 0, 1000000, numSteps, "q (steps per move)")) {
        out.print("Steps per move set to: ");
        out.print(numSteps);
        out.print(" = ");
        out.print(mmPerStep * numSteps, 4);
        out.println("mm");
      }
      break;

    case 'e': {
      float um;
      if (!parseFloatArg(arg, um) || !(um > 0) || um > 100000) {
        out.println("e (step size, microns) needs a number above 0, e.g. e50");
        break;
      }
      long steps = lround(um / (mmPerStep * 1000.0f));
      if (steps < 1) steps = 1;
      numSteps = steps;
      saveSettings();
      out.print("Steps per move set to: ");
      out.print(numSteps);
      out.print(" = ");
      out.print(mmPerStep * numSteps * 1000.0f, 1);
      out.println("um");
      break;
    }

    case 'd': {
      long v = direction;
      if (setLong(arg, 0, 1, v, "d (direction, 1=cw 0=ccw)")) {
        direction = (int)v;
        saveSettings();
        out.println(direction == 1 ? "Direction set to clockwise" : "Direction set to counterclockwise");
      }
      break;
    }

    case 'r': {
      float v;
      if (!parseFloatArg(arg, v) || !(v >= 0.1f) || v > MAX_RPM) {
        out.print("r (RPM) needs a number from 0.1 to ");
        out.print(MAX_RPM, 0);
        out.print(" (currently ");
        out.print(motorRPM);
        out.println(")");
        break;
      }
      motorRPM = v;
      saveSettings();
      out.print("RPM set to: ");
      out.println(motorRPM);
      break;
    }

    case 'k':
      if (setLong(arg, 0, 5000, backlashSteps, "k (backlash steps)")) {
        out.print("Backlash take-up set to: ");
        out.println(backlashSteps);
      }
      break;

    case 'j': {
      long v;
      if (!parseLong(arg, v) || v == 0 || v < -2000000 || v > 2000000) {
        out.println("j (jog) needs a number of steps, e.g. j200 or j-200");
        break;
      }
      if (moveActive) {
        out.println("Motor is still moving");
        break;
      }
      out.print("Jogging ");
      out.print(v);
      out.println(" steps");
      startMove(v, 0);
      break;
    }

    case 'a':
      pointA = position;
      hasA = true;
      hasB = false;
      out.print("Start marked at position ");
      out.println(pointA);
      out.println("Jog to the far end of the subject and send b");
      break;

    case 'b': {
      if (!hasA) {
        out.println("Mark the start first with a");
        break;
      }
      long dist = position - pointA;
      if (dist == 0) {
        out.println("End is the same as the start - jog to the far end first");
        break;
      }
      if (numSteps == 0) {
        out.println("Steps per move is 0 - set a step size with q or e first");
        break;
      }
      long travel = dist > 0 ? dist : -dist;
      pointB = position;
      hasB = true;
      direction = dist > 0 ? 1 : 0;
      runLimit = (travel + numSteps - 1) / numSteps; // round up so the end is covered
      if (runLimit > 100000) runLimit = 100000;
      saveSettings();
      out.print("End marked at position ");
      out.println(pointB);
      out.print("Stack is ");
      out.print(travel * mmPerStep, 3);
      out.print("mm: ");
      out.print(runLimit);
      out.print(" moves, ");
      out.print(runLimit + 1);
      out.println(direction == 1 ? " frames, clockwise" : " frames, counterclockwise");
      out.println("Send g to return to the start, then s");
      break;
    }

    case 'g': {
      if (!hasA) {
        out.println("No start mark - set one with a");
        break;
      }
      if (moveActive) {
        out.println("Motor is still moving");
        break;
      }
      int dir = direction == 1 ? 1 : -1;
      long delta = pointA - position;
      if (backlashSteps > 0) {
        //Always arrive at the start travelling in the stack direction
        if (delta == 0 && lastMoveDir == dir) {
          out.println("Already at the start");
          break;
        }
        startMove(delta - dir * backlashSteps, dir * backlashSteps);
      } else {
        if (delta == 0) {
          out.println("Already at the start");
          break;
        }
        startMove(delta, 0);
      }
      out.println("Returning to start");
      break;
    }

    case 'u':
      startWifiUpload(src);
      break;

    default:
      out.print("Unknown command '");
      out.print(line);
      out.println("' - send h for help");
  }
}

// ---------------------------------------------------------------- Bluetooth

void checkBluetoothConnection() {
  bool connected = SerialBT.hasClient();
  if (connected == btConnected) return;
  btConnected = connected;
  if (connected) {
    Serial.println("Bluetooth client connected");
    printMenu();
  } else {
    Serial.println("Bluetooth client disconnected");
  }
}

// ---------------------------------------------------------------- WiFi firmware update

//Wait for one non-empty line from a port. Used only for the wifi prompts.
bool readLineBlocking(Stream& port, String& line, unsigned long timeoutMs) {
  line = "";
  unsigned long start = millis();
  unsigned long lastChar = start;
  while (millis() - start < timeoutMs) {
    if (port.available() > 0) {
      char c = (char)port.read();
      if (c == '\r' || c == '\n') {
        if (line.length() > 0) return true;
      } else {
        if (line.length() < 64) line += c;
        lastChar = millis();
      }
    } else {
      if (line.length() > 0 && millis() - lastChar > INPUT_IDLE_MS) return true;
      delay(5);
    }
  }
  return false;
}

void setupUpdateServer() {
  server.on("/", HTTP_GET, []() {
    if (!server.authenticate(otaUser, otaPin.c_str())) return server.requestAuthentication();
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", updatePage);
  });

  server.on("/update", HTTP_POST, []() {
    if (!server.authenticate(otaUser, otaPin.c_str())) return server.requestAuthentication();
    bool ok = otaUploadOk && !Update.hasError();
    server.sendHeader("Connection", "close");
    server.send(ok ? 200 : 500, "text/plain", ok ? "OK" : "Not a valid firmware file");
    if (ok) {
      delay(500);
      ESP.restart();
    }
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      otaUploadOk = false;
      otaAuthed = server.authenticate(otaUser, otaPin.c_str());
      if (!otaAuthed) return;
      out.print("Update: ");
      out.println(upload.filename);
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(out);
    } else if (!otaAuthed) {
      return;
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(out);
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        otaUploadOk = true;
        out.print("Update success, ");
        out.print(upload.totalSize);
        out.println(" bytes. Rebooting...");
      } else {
        Update.printError(out);
      }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
      Update.abort();
      out.println("Update aborted");
    }
  });
}

void startWifiUpload(Stream& src) {
  if (moveActive) {
    out.println("Motor is still moving");
    return;
  }
  if (otaMode) {
    out.println("Already in wifi upload mode - send p to leave it");
    return;
  }
  String ssid, password;
  out.println("Please enter WIFI SSID:");
  if (!readLineBlocking(src, ssid, 60000)) {
    out.println("No SSID entered, cancelled");
    return;
  }
  out.print("SSID: ");
  out.println(ssid);
  out.println("Please enter WIFI password (send - for an open network):");
  if (!readLineBlocking(src, password, 60000)) {
    out.println("No password entered, cancelled");
    return;
  }
  if (password == "-") password = "";

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());
  out.print("Connecting");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    out.print(".");
  }
  out.println();
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    out.println("Could not connect - check the SSID and password and send u to try again");
    return;
  }

  //A fresh PIN every time, shown only on the serial/Bluetooth console
  otaPin = String(100000 + (long)(esp_random() % 900000));
  bool mdns = MDNS.begin(otaHost);
  server.begin();
  otaMode = true;
  digitalWrite(ledPin, HIGH);

  out.print("Connected. Open http://");
  out.print(WiFi.localIP());
  if (mdns) {
    out.print(" or http://");
    out.print(otaHost);
    out.print(".local");
  }
  out.println();
  out.print("User: ");
  out.print(otaUser);
  out.print("   Password: ");
  out.println(otaPin);
  out.println("Send p to leave upload mode");
}

void stopWifiUpload() {
  server.stop();
  MDNS.end();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  otaMode = false;
  digitalWrite(ledPin, LOW);
  out.println("Left wifi upload mode");
}
