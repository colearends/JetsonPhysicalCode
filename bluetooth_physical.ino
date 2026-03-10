/*
  HMI / HC-05 INTEGRATED VERSION OF YOUR ORIGINAL SKETCH

  Preserved from original:
  - MCP4725 throttle via I2C with timeout + recovery
  - Watchdog enabled
  - Heartbeat on D13
  - Brake apply is time-limited one-shot with lockout
  - Steering + brake hall interrupts retained
  - Safety loop retained
  - USB serial commands retained: HELP, BRAKE_MS <ms>

  Changed:
  - Joystick X/Y input is replaced by HC-05 HMI commands
  - Joystick click arming is replaced by HMI arm/disarm
  - HC-05 uses Serial2 (NOT Serial1) because pins 18/19 are already used
    for steering hall sensors in your hardware

  HMI command format:
    CMD,<arm>,<steer>,<throttle>,<brake>

  Example:
    CMD,1,0,300,0
    CMD,1,-700,0,0
    CMD,1,0,0,1
    CMD,0,0,0,0

  Meanings:
    arm      = 0 or 1
    steer    = -1000 to +1000
    throttle = 0 to 1000
    brake    = 0 or 1

  Important:
  - HMI must continuously stream commands.
  - If commands stop for CMD_TIMEOUT_MS, controller goes safe.
*/

#include <Wire.h>
#include <avr/wdt.h>

// -------------------- MCP4725 --------------------
const uint8_t  MCP4725_ADDR = 0x60;
const uint16_t DAC_MAX      = 4095;
const uint16_t THROTTLE_MAX_DAC = 650; // keep your existing test cap

const uint8_t PIN_I2C_SDA = 20;
const uint8_t PIN_I2C_SCL = 21;

const uint32_t I2C_CLOCK_HZ      = 100000;
const uint16_t I2C_TIMEOUT_US    = 25000;
const uint8_t  DAC_WRITE_RETRIES = 2;

// -------------------- Pins --------------------
const uint8_t PIN_STEER_EXTEND  = 22;
const uint8_t PIN_STEER_RETRACT = 23;

const uint8_t PIN_BRAKE_EXTEND  = 24;
const uint8_t PIN_BRAKE_RETRACT = 25;

const uint8_t PIN_SAFETY_LOOP = 26;  // NC to GND

// Joystick pins kept here only as reference from your original hardware,
// but no longer used in the control path
const uint8_t PIN_JOY_SW = 27;
const uint8_t PIN_JOY_X  = A0;
const uint8_t PIN_JOY_Y  = A1;

const uint8_t PIN_STEER_HALL_A = 18;
const uint8_t PIN_STEER_HALL_B = 19;
const uint8_t PIN_BRAKE_HALL_A = 2;
const uint8_t PIN_BRAKE_HALL_B = 3;

// -------------------- Bluetooth --------------------
// IMPORTANT: use Serial2 on Mega because Serial1 = pins 18/19 conflicts
// with steering hall sensor inputs in your current wiring.
#define BT Serial2
const unsigned long BT_BAUD = 9600;
const unsigned long CMD_TIMEOUT_MS = 300;

// line parser buffer for BT
char btLineBuf[64];
uint8_t btLinePos = 0;

// parsed HMI command state
bool btHasValidCommand = false;
unsigned long lastCmdMs = 0;

bool cmdArm = false;         // 0/1
int  cmdSteer = 0;           // -1000..1000
int  cmdThrottle = 0;        // 0..1000
bool cmdBrake = false;       // 0/1

// -------------------- HEARTBEAT --------------------
const uint8_t PIN_HEARTBEAT_LED = 13;
const unsigned long HB_DISARM_MS = 500;
const unsigned long HB_ARMED_MS  = 120;
unsigned long lastHbMs = 0;
bool hbState = false;

// -------------------- Settings --------------------
const bool BRAKE_INVERT_DIR = false;
const bool SAFE_BRAKE_APPLY = false;

// -------------------- Command tuning --------------------
// Steering command threshold for HMI input
const int STEER_CMD_THRESH = 150;

// Brake command is explicit from HMI, so original joystick brake threshold
// is no longer used for command generation. Keeping release hysteresis logic
// is not needed with explicit brake commands.
const int BRAKE_RELEASE_HYST = 40; // retained only for reference

const unsigned long PRINT_MS = 150;

// -------------------- Time-based brake limit --------------------
// Preserved from your original code
unsigned long BRAKE_APPLY_MAX_MS = 115;

bool brakeBurstActive = false;
unsigned long brakeBurstStartMs = 0;

// Lockout after a burst finishes; prevents repeated bursts until brake command released
bool brakeApplyLockout = false;

// -------------------- State --------------------
volatile long steerCounts = 0;
volatile long brakeCounts = 0;

bool armed = false;

unsigned long lastPrintMs = 0;
uint16_t lastThrottleDAC  = 0;

// -------------------- Helpers --------------------
long readSteerCounts() { noInterrupts(); long v = steerCounts; interrupts(); return v; }
long readBrakeCounts() { noInterrupts(); long v = brakeCounts; interrupts(); return v; }

void isrSteerA() { steerCounts += (digitalRead(PIN_STEER_HALL_A) == digitalRead(PIN_STEER_HALL_B)) ? +1 : -1; }
void isrSteerB() { steerCounts += (digitalRead(PIN_STEER_HALL_A) != digitalRead(PIN_STEER_HALL_B)) ? +1 : -1; }
void isrBrakeA() { brakeCounts += (digitalRead(PIN_BRAKE_HALL_A) == digitalRead(PIN_BRAKE_HALL_B)) ? +1 : -1; }
void isrBrakeB() { brakeCounts += (digitalRead(PIN_BRAKE_HALL_A) != digitalRead(PIN_BRAKE_HALL_B)) ? +1 : -1; }

bool safetyClosed() { return digitalRead(PIN_SAFETY_LOOP) == LOW; }

bool commandFresh() {
  if (!btHasValidCommand) return false;
  return (millis() - lastCmdMs) <= CMD_TIMEOUT_MS;
}

// -------------------- Outputs --------------------
void steerStop()    { digitalWrite(PIN_STEER_EXTEND, LOW); digitalWrite(PIN_STEER_RETRACT, LOW); }
void steerExtend()  { digitalWrite(PIN_STEER_EXTEND, HIGH); digitalWrite(PIN_STEER_RETRACT, LOW); }
void steerRetract() { digitalWrite(PIN_STEER_EXTEND, LOW);  digitalWrite(PIN_STEER_RETRACT, HIGH); }

void brakeStop() {
  digitalWrite(PIN_BRAKE_EXTEND, LOW);
  digitalWrite(PIN_BRAKE_RETRACT, LOW);
}

void brakeApplyRaw() {
  if (!BRAKE_INVERT_DIR) {
    digitalWrite(PIN_BRAKE_EXTEND, LOW);
    digitalWrite(PIN_BRAKE_RETRACT, HIGH);
  } else {
    digitalWrite(PIN_BRAKE_EXTEND, HIGH);
    digitalWrite(PIN_BRAKE_RETRACT, LOW);
  }
}

void brakeRelease() {
  if (!BRAKE_INVERT_DIR) {
    digitalWrite(PIN_BRAKE_EXTEND, HIGH);
    digitalWrite(PIN_BRAKE_RETRACT, LOW);
  } else {
    digitalWrite(PIN_BRAKE_EXTEND, LOW);
    digitalWrite(PIN_BRAKE_RETRACT, HIGH);
  }
}

// -------------------- I2C recovery --------------------
void i2cRecoverBus() {
  Wire.end();

  pinMode(PIN_I2C_SCL, OUTPUT);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);

  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(5);

  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(PIN_I2C_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  }

  // STOP-ish
  pinMode(PIN_I2C_SDA, OUTPUT);
  digitalWrite(PIN_I2C_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_I2C_SDA, HIGH); delayMicroseconds(5);

  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
}

bool writeMCP4725(uint16_t val) {
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(0x40);
  Wire.write(val >> 4);
  Wire.write((val & 0x0F) << 4);
  return (Wire.endTransmission() == 0);
}

void setThrottleDAC(uint16_t val) {
  val = constrain(val, (uint16_t)0, (uint16_t)DAC_MAX);
  lastThrottleDAC = val;

  if (writeMCP4725(val)) return;

  for (uint8_t attempt = 0; attempt < DAC_WRITE_RETRIES; attempt++) {
    i2cRecoverBus();
    if (writeMCP4725(val)) return;
  }

  // Fail-safe if DAC cannot be written
  armed = false;
  Serial.println("ERR: DAC I2C failed -> DISARM");
}

// -------------------- Safe outputs --------------------
void resetBrakeLimiterState() {
  brakeBurstActive = false;
  brakeApplyLockout = false;
}

void forceSafeOutputs() {
  resetBrakeLimiterState();
  setThrottleDAC(0);
  steerStop();
  if (SAFE_BRAKE_APPLY) brakeApplyRaw();
  else brakeRelease();
}

// -------------------- Heartbeat --------------------
void updateHeartbeat() {
  unsigned long now = millis();

  if (!safetyClosed()) {
    digitalWrite(PIN_HEARTBEAT_LED, HIGH);
    hbState = true;
    return;
  }

  unsigned long period = armed ? HB_ARMED_MS : HB_DISARM_MS;
  if (now - lastHbMs >= period) {
    lastHbMs = now;
    hbState = !hbState;
    digitalWrite(PIN_HEARTBEAT_LED, hbState ? HIGH : LOW);
  }
}

// -------------------- Bluetooth parser --------------------
void applyParsedCommand(int armIn, int steerIn, int throttleIn, int brakeIn) {
  cmdArm = (armIn != 0);
  cmdSteer = constrain(steerIn, -1000, 1000);
  cmdThrottle = constrain(throttleIn, 0, 1000);
  cmdBrake = (brakeIn != 0);

  lastCmdMs = millis();
  btHasValidCommand = true;
}

bool parseCommandLine(char *line) {
  // Expected: CMD,<arm>,<steer>,<throttle>,<brake>
  if (strncmp(line, "CMD,", 4) != 0) return false;

  char *p = line + 4;
  char *tok1 = strtok(p, ",");
  char *tok2 = strtok(NULL, ",");
  char *tok3 = strtok(NULL, ",");
  char *tok4 = strtok(NULL, ",");

  if (!tok1 || !tok2 || !tok3 || !tok4) return false;

  int armIn      = atoi(tok1);
  int steerIn    = atoi(tok2);
  int throttleIn = atoi(tok3);
  int brakeIn    = atoi(tok4);

  applyParsedCommand(armIn, steerIn, throttleIn, brakeIn);
  return true;
}

void handleBluetooth() {
  while (BT.available()) {
    char c = (char)BT.read();

    if (c == '\r') continue;

    if (c == '\n') {
      btLineBuf[btLinePos] = '\0';

      if (btLinePos > 0) {
        bool ok = parseCommandLine(btLineBuf);

        if (ok) {
          BT.print("OK,");
          BT.print(cmdArm); BT.print(",");
          BT.print(cmdSteer); BT.print(",");
          BT.print(cmdThrottle); BT.print(",");
          BT.println(cmdBrake);
        } else {
          BT.println("ERR,BAD_CMD");
        }
      }

      btLinePos = 0;
      continue;
    }

    if (btLinePos < sizeof(btLineBuf) - 1) {
      btLineBuf[btLinePos++] = c;
    } else {
      // overflow -> reset line
      btLinePos = 0;
      BT.println("ERR,OVERFLOW");
    }
  }
}

// -------------------- Serial --------------------
void printHelp() {
  Serial.println("Commands:");
  Serial.println("  HELP");
  Serial.println("  BRAKE_MS <ms>   (time limit per apply burst)");
  Serial.println("Bluetooth HMI format:");
  Serial.println("  CMD,<arm>,<steer>,<throttle>,<brake>");
  Serial.println("Example:");
  Serial.println("  CMD,1,0,300,0");
}

void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  line.toUpperCase();

  if (line == "HELP") { printHelp(); return; }

  int sp = line.indexOf(' ');
  String cmd = (sp >= 0) ? line.substring(0, sp) : line;
  String arg = (sp >= 0) ? line.substring(sp + 1) : "";
  arg.trim();

  long val = arg.toInt();

  if (cmd == "BRAKE_MS") {
    if (val < 0) val = 0;
    BRAKE_APPLY_MAX_MS = (unsigned long)val;
    Serial.print("OK: BRAKE_APPLY_MAX_MS = ");
    Serial.println(BRAKE_APPLY_MAX_MS);
  } else {
    Serial.println("Unknown command. Type HELP.");
  }
}

// -------------------- Setup/Loop --------------------
void setup() {
  pinMode(PIN_STEER_EXTEND, OUTPUT);
  pinMode(PIN_STEER_RETRACT, OUTPUT);
  pinMode(PIN_BRAKE_EXTEND, OUTPUT);
  pinMode(PIN_BRAKE_RETRACT, OUTPUT);
  steerStop();
  brakeStop();

  pinMode(PIN_HEARTBEAT_LED, OUTPUT);
  digitalWrite(PIN_HEARTBEAT_LED, LOW);

  pinMode(PIN_SAFETY_LOOP, INPUT_PULLUP);
  pinMode(PIN_JOY_SW, INPUT_PULLUP);  // unused now, but harmless to keep

  Serial.begin(115200);
  Serial.setTimeout(20);

  BT.begin(BT_BAUD);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);

  pinMode(PIN_STEER_HALL_A, INPUT);
  pinMode(PIN_STEER_HALL_B, INPUT);
  pinMode(PIN_BRAKE_HALL_A, INPUT);
  pinMode(PIN_BRAKE_HALL_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_STEER_HALL_A), isrSteerA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_STEER_HALL_B), isrSteerB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BRAKE_HALL_A), isrBrakeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BRAKE_HALL_B), isrBrakeB, CHANGE);

  armed = false;
  btHasValidCommand = false;
  cmdArm = false;
  cmdSteer = 0;
  cmdThrottle = 0;
  cmdBrake = false;

  forceSafeOutputs();

  wdt_enable(WDTO_1S);

  Serial.println("READY: HMI/HC-05 integrated controller");
  Serial.println("BT on Serial2 @ 9600");
  Serial.print("BRAKE_APPLY_MAX_MS = "); Serial.println(BRAKE_APPLY_MAX_MS);
  printHelp();
}

void loop() {
  wdt_reset();

  unsigned long now = millis();

  updateHeartbeat();
  handleSerial();
  handleBluetooth();

  // Safety loop has highest priority
  if (!safetyClosed()) {
    armed = false;
    forceSafeOutputs();
    return;
  }

  // Loss of HMI command stream -> safe
  if (!commandFresh()) {
    armed = false;
    forceSafeOutputs();
    return;
  }

  // HMI arm/disarm controls armed state
  armed = cmdArm;

  if (!armed) {
    forceSafeOutputs();
    return;
  }

  // -------------------- Steering --------------------
  // Preserving your original open-loop directional steering behavior,
  // just replacing joystick dx with HMI steer command.
  int dx = cmdSteer;

  if (dx > STEER_CMD_THRESH) steerExtend();
  else if (dx < -STEER_CMD_THRESH) steerRetract();
  else steerStop();

  // -------------------- Throttle --------------------
  // Preserving your original "forward throttle only" behavior.
  // If brake is commanded, throttle is forced to zero.
  uint16_t dac = 0;
  if (!cmdBrake) {
    dac = (uint16_t)constrain(map(cmdThrottle, 0, 1000, 0, THROTTLE_MAX_DAC), 0, THROTTLE_MAX_DAC);
  }
  setThrottleDAC(dac);

  // -------------------- Brake: one-shot burst + lockout --------------------
  // Preserved from your original code, but now driven by explicit HMI brake command.
  bool wantApply = cmdBrake;
  bool releasedEnough = !cmdBrake;

  if (releasedEnough) {
    brakeApplyLockout = false;
    brakeBurstActive = false;
  }

  if (wantApply && !brakeApplyLockout) {
    if (!brakeBurstActive) {
      brakeBurstActive = true;
      brakeBurstStartMs = millis();
    }

    if (millis() - brakeBurstStartMs <= BRAKE_APPLY_MAX_MS) {
      brakeApplyRaw();
    } else {
      brakeStop();
      brakeApplyLockout = true;
    }
  } else {
    brakeRelease();
  }

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;

    Serial.print("ARM="); Serial.print(armed);
    Serial.print(" CMD_FRESH="); Serial.print(commandFresh());
    Serial.print(" steer="); Serial.print(cmdSteer);
    Serial.print(" throttle="); Serial.print(cmdThrottle);
    Serial.print(" brake="); Serial.print(cmdBrake);
    Serial.print(" lockout="); Serial.print(brakeApplyLockout);
    Serial.print(" burstActive="); Serial.print(brakeBurstActive);
    Serial.print(" BRAKE_MS="); Serial.print(BRAKE_APPLY_MAX_MS);
    Serial.print(" ThrottleDAC="); Serial.print(lastThrottleDAC);
    Serial.print(" steerCounts="); Serial.print(readSteerCounts());
    Serial.print(" brakeCounts="); Serial.println(readBrakeCounts());
  }
}