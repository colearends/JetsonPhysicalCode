#include <Wire.h>
#include <avr/wdt.h>

// -------------------- USER SETTINGS --------------------
const uint8_t PIN_HEARTBEAT_LED = 13;   // change to 12 if needed
const uint8_t PIN_SAFETY_LOOP   = 26;   // NC to GND
const unsigned long BT_BAUD     = 9600;
const unsigned long CMD_TIMEOUT_MS = 5000;
const unsigned long HB_ARMED_MS = 120;
const unsigned long PRINT_MS    = 250;

// Steering limits
const long STEER_LIMIT_POS = 2400;   // +2400
const long STEER_LIMIT_NEG = -1000;  // -1000

// -------------------- Steering actuator --------------------
const uint8_t PIN_STEER_EXTEND  = 22;
const uint8_t PIN_STEER_RETRACT = 23;
const int STEER_CMD_THRESH = 150;

// -------------------- Brake actuator --------------------
const uint8_t PIN_BRAKE_EXTEND  = 24;
const uint8_t PIN_BRAKE_RETRACT = 25;
const bool BRAKE_INVERT_DIR = true;

// -------------------- Steering hall feedback --------------------
const uint8_t PIN_STEER_HALL_A = 18;
const uint8_t PIN_STEER_HALL_B = 19;

// -------------------- MCP4725 --------------------
const uint8_t  MCP4725_ADDR      = 0x60;
const uint16_t DAC_MAX           = 4095;
const uint16_t THROTTLE_MAX_DAC  = 1100;

const uint8_t PIN_I2C_SDA = 20;
const uint8_t PIN_I2C_SCL = 21;

const uint32_t I2C_CLOCK_HZ      = 100000;
const uint16_t I2C_TIMEOUT_US    = 25000;
const uint8_t  DAC_WRITE_RETRIES = 2;

// -------------------- Bluetooth --------------------
#define BT Serial2

char btLineBuf[64];
uint8_t btLinePos = 0;

bool btHasValidCommand = false;
unsigned long lastCmdMs = 0;

bool cmdArm       = false;
int  cmdSteer     = 0;
int  cmdThrottle  = 0;
bool cmdBrake     = false;

// -------------------- State --------------------
bool armed = false;
bool hbState = false;
unsigned long lastHbMs = 0;
unsigned long lastPrintMs = 0;
uint16_t lastThrottleDAC = 0;

// -------------------- Feedback state --------------------
volatile long steerCounts = 0;
long steerZeroOffset = 0;
bool homingDone = false;

// -------------------- Steering feedback ISRs --------------------
void isrSteerA() {
  steerCounts += (digitalRead(PIN_STEER_HALL_A) == digitalRead(PIN_STEER_HALL_B)) ? +1 : -1;
}

void isrSteerB() {
  steerCounts += (digitalRead(PIN_STEER_HALL_A) != digitalRead(PIN_STEER_HALL_B)) ? +1 : -1;
}

// -------------------- Helpers --------------------
bool safetyClosed() {
  return digitalRead(PIN_SAFETY_LOOP) == LOW;
}

bool commandFresh() {
  if (!btHasValidCommand) return false;
  return (millis() - lastCmdMs) <= CMD_TIMEOUT_MS;
}

// -------------------- Steering outputs --------------------
void steerStop() {
  digitalWrite(PIN_STEER_EXTEND, LOW);
  digitalWrite(PIN_STEER_RETRACT, LOW);
}

void steerExtend() {
  digitalWrite(PIN_STEER_EXTEND, HIGH);
  digitalWrite(PIN_STEER_RETRACT, LOW);
}

void steerRetract() {
  digitalWrite(PIN_STEER_EXTEND, LOW);
  digitalWrite(PIN_STEER_RETRACT, HIGH);
}

// -------------------- Brake outputs --------------------
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

// -------------------- I2C / DAC --------------------
void i2cRecoverBus() {
  Wire.end();

  pinMode(PIN_I2C_SCL, OUTPUT);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);

  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(5);

  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(PIN_I2C_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(5);
  }

  pinMode(PIN_I2C_SDA, OUTPUT);
  digitalWrite(PIN_I2C_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SDA, HIGH);
  delayMicroseconds(5);

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

bool setThrottleDAC(uint16_t val) {
  val = constrain(val, (uint16_t)0, (uint16_t)DAC_MAX);
  lastThrottleDAC = val;

  if (writeMCP4725(val)) return true;

  for (uint8_t attempt = 0; attempt < DAC_WRITE_RETRIES; attempt++) {
    i2cRecoverBus();
    if (writeMCP4725(val)) return true;
  }

  Serial.println("ERR: DAC I2C write failed");
  return false;
}

// -------------------- Safe outputs --------------------
void forceSafeOutputs() {
  setThrottleDAC(0);
  brakeRelease();
}

// -------------------- Heartbeat --------------------
void updateHeartbeat() {
  unsigned long now = millis();

  if (!armed) {
    digitalWrite(PIN_HEARTBEAT_LED, HIGH);
    hbState = true;
    return;
  }

  if (now - lastHbMs >= HB_ARMED_MS) {
    lastHbMs = now;
    hbState = !hbState;
    digitalWrite(PIN_HEARTBEAT_LED, hbState ? HIGH : LOW);
  }
}

// -------------------- Command parsing --------------------
void applyParsedCommand(int armIn, int steerIn, int throttleIn, int brakeIn) {
  cmdArm      = (armIn != 0);
  cmdSteer    = constrain(steerIn, -1000, 1000);
  cmdThrottle = constrain(throttleIn, 0, 1000);
  cmdBrake    = (brakeIn != 0);

  lastCmdMs = millis();
  btHasValidCommand = true;
}

bool parseCommandLine(char *line) {
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
      btLinePos = 0;
      BT.println("ERR,OVERFLOW");
    }
  }
}

void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  line.toUpperCase();

  if (line == "HELP") {
    Serial.println("Bluetooth HMI format:");
    Serial.println("CMD,<arm>,<steer>,<throttle>,<brake>");
    Serial.println("Example: CMD,1,500,300,0");
    return;
  }

  Serial.println("Unknown command.");
}

// -------------------- Homing Sequence --------------------
void homeSteeringToCenter() {
  wdt_disable();
  Serial.println("Homing: simple timed extend...");

  const unsigned long EXTEND_MS = 15000;
  const long retractAmount = -2425;
  const int tolerance = 3;

  unsigned long start = millis();
  while (millis() - start < EXTEND_MS) {
    steerExtend();
    delay(10);
  }
  steerStop();
  Serial.println("Homing: finished timed extend.");

  noInterrupts();
  steerCounts = 0;
  interrupts();
  Serial.println("Homing: zero set at extension.");

  Serial.println("Homing: retracting to center...");

  while (true) {
    long raw = steerCounts;
    long error = retractAmount - raw;

    if (abs(error) <= tolerance) {
      steerStop();
      Serial.println("Homing: retract complete.");
      break;
    }

    if (error < 0) steerRetract();
    else steerExtend();

    delay(20);
  }

  noInterrupts();
  steerCounts = 0;
  interrupts();

  Serial.println("Homing complete: CENTER is now zero.");
  wdt_enable(WDTO_1S);
  homingDone = true;
}

// -------------------- Setup --------------------
void setup() {
  pinMode(PIN_HEARTBEAT_LED, OUTPUT);
  digitalWrite(PIN_HEARTBEAT_LED, LOW);

  pinMode(PIN_SAFETY_LOOP, INPUT_PULLUP);

  pinMode(PIN_STEER_EXTEND, OUTPUT);
  pinMode(PIN_STEER_RETRACT, OUTPUT);
  steerStop();

  pinMode(PIN_BRAKE_EXTEND, OUTPUT);
  pinMode(PIN_BRAKE_RETRACT, OUTPUT);
  brakeStop();

  pinMode(PIN_STEER_HALL_A, INPUT_PULLUP);
  pinMode(PIN_STEER_HALL_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_STEER_HALL_A), isrSteerA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_STEER_HALL_B), isrSteerB, CHANGE);

  Serial.begin(115200);
  Serial.setTimeout(20);

  BT.begin(BT_BAUD);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);

  armed = false;
  btHasValidCommand = false;
  cmdArm = false;
  cmdSteer = 0;
  cmdThrottle = 0;
  cmdBrake = false;
  steerCounts = 0;

  forceSafeOutputs();

  wdt_enable(WDTO_1S);

  Serial.println("READY: STEERING ENCODER + HOMING + LIMITS");

  homeSteeringToCenter();
}

// -------------------- Brake timed sequence --------------------
bool brakeSequenceActive = false;
unsigned long brakeStepStart = 0;
int brakeStep = 0;

void runBrakeSequence() {
  unsigned long now = millis();

  switch (brakeStep) {

    case 0:
      brakeApplyRaw();
      brakeStepStart = now;
      brakeStep = 1;
      break;

    case 1:
      if (now - brakeStepStart >= 85) {
        brakeStop();
        brakeStepStart = now;
        brakeStep = 2;
      }
      break;

    case 2:
      if (now - brakeStepStart >= 3000) {
        brakeRelease();
        brakeStepStart = now;
        brakeStep = 3;
      }
      break;

    case 3:
      if (now - brakeStepStart >= 3000) {
        brakeStop();
        brakeSequenceActive = false;
        brakeStep = 0;
      }
      break;
  }
}

// -------------------- Loop --------------------
void loop() {
  wdt_reset();

  handleSerial();
  handleBluetooth();

  armed = safetyClosed() && commandFresh() && cmdArm;

  updateHeartbeat();

  if (homingDone) {

    long scRaw = steerCounts;
    long sc    = scRaw - steerZeroOffset;

    // -------------------- Steering Control WITH LIMITS --------------------
    if (cmdSteer > STEER_CMD_THRESH) {
        if (sc < STEER_LIMIT_POS) steerExtend();
        else steerStop();
    }
    else if (cmdSteer < -STEER_CMD_THRESH) {
        if (sc > STEER_LIMIT_NEG) steerRetract();
        else steerStop();
    }
    else {
        steerStop();
    }

    // -------------------- Brake Control (Timed Sequence) --------------------
    if (cmdBrake && !brakeSequenceActive) {
        brakeSequenceActive = true;
        brakeStep = 0;
    }

    if (brakeSequenceActive) runBrakeSequence();
    else brakeStop();

    // -------------------- Throttle Output --------------------
    uint16_t dac = (uint16_t)constrain(
      map(cmdThrottle, 0, 1000, 0, THROTTLE_MAX_DAC),
      0,
      THROTTLE_MAX_DAC
    );

    setThrottleDAC(dac);
  }

  // -------------------- Debug Print --------------------
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;

    Serial.print("ARM=");      Serial.print(armed);
    Serial.print(" SAFETY=");  Serial.print(safetyClosed());
    Serial.print(" FRESH=");   Serial.print(commandFresh());
    Serial.print(" cmdArm=");  Serial.print(cmdArm);
    Serial.print(" steer=");   Serial.print(cmdSteer);
    Serial.print(" throttle=");Serial.print(cmdThrottle);
    Serial.print(" brake=");   Serial.print(cmdBrake);
    Serial.print(" DAC=");     Serial.print(lastThrottleDAC);
    Serial.print(" steerCounts="); Serial.println(steerCounts);

    BT.print("ARM=");
    BT.println(armed);
    BT.print("steerCounts=");
    BT.println(steerCounts);
  }
}
