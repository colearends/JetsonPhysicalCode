#include <Wire.h>
#include <avr/wdt.h>

// -------------------- USER SETTINGS --------------------
const uint8_t PIN_HEARTBEAT_LED = 13;   // change to 12 if needed
const uint8_t PIN_SAFETY_LOOP   = 26;   // NC to GND
const unsigned long BT_BAUD     = 9600;
const unsigned long CMD_TIMEOUT_MS = 5000;
const unsigned long HB_ARMED_MS = 120;
const unsigned long PRINT_MS    = 250;
const long STEER_LIMIT_POS = 1000;   // +1000
const long STEER_LIMIT_NEG = -1000;  // -1000
#include <EEPROM.h>
bool homingDone = false;

// EEPROM address for steering zero offset (4 bytes for a long)
const int EEPROM_ADDR_STEER_ZERO = 0;

// This will hold the saved zero offset
long steerZeroOffset = 0;

// -------------------- Steering actuator --------------------
const uint8_t PIN_STEER_EXTEND  = 22;
const uint8_t PIN_STEER_RETRACT = 23;
const int STEER_CMD_THRESH = 150;

// -------------------- Brake actuator --------------------
const uint8_t PIN_BRAKE_EXTEND  = 24;
const uint8_t PIN_BRAKE_RETRACT = 25;
const bool BRAKE_INVERT_DIR = true;   // flipped per your note

// -------------------- Steering hall feedback --------------------
const uint8_t PIN_STEER_HALL_A = 18;
const uint8_t PIN_STEER_HALL_B = 19;

// -------------------- Brake hall feedback --------------------
const uint8_t PIN_BRAKE_HALL_A = 2;
const uint8_t PIN_BRAKE_HALL_B = 3;

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
volatile long brakeCounts = 0;

// -------------------- Helpers --------------------
bool safetyClosed() {
  return digitalRead(PIN_SAFETY_LOOP) == LOW;
}

bool commandFresh() {
  if (!btHasValidCommand) return false;
  return (millis() - lastCmdMs) <= CMD_TIMEOUT_MS;
}

long readSteerCounts() {
  noInterrupts();
  long v = steerCounts;
  interrupts();
  return v;
}

long readBrakeCounts() {
  noInterrupts();
  long v = brakeCounts;
  interrupts();
  return v;
}

// -------------------- Steering feedback ISRs --------------------
void isrSteerA() {
  steerCounts += (digitalRead(PIN_STEER_HALL_A) == digitalRead(PIN_STEER_HALL_B)) ? +1 : -1;
}

void isrSteerB() {
  steerCounts += (digitalRead(PIN_STEER_HALL_A) != digitalRead(PIN_STEER_HALL_B)) ? +1 : -1;
}

// -------------------- Brake feedback ISRs --------------------
void isrBrakeA() {
  brakeCounts += (digitalRead(PIN_BRAKE_HALL_A) == digitalRead(PIN_BRAKE_HALL_B)) ? +1 : -1;
}

void isrBrakeB() {
  brakeCounts += (digitalRead(PIN_BRAKE_HALL_A) != digitalRead(PIN_BRAKE_HALL_B)) ? +1 : -1;
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
  //steerStop();
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

  if (line == "STEER_ZERO") {
    long sc = readSteerCounts();      // raw hall position right now
    steerZeroOffset = sc;             // save as new zero
    EEPROM.put(EEPROM_ADDR_STEER_ZERO, steerZeroOffset);

    Serial.print("OK: Steering zero set at raw = ");
    Serial.println(sc);
    return;
  }

  Serial.println("Unknown command.");
}
void homeSteeringToCenter() {
  wdt_disable();
  Serial.println("Homing: simple timed extend...");

  const unsigned long EXTEND_MS = 15000;  // run forward for exactly 2 seconds
  const long retractAmount = -2425;      // retract 1500 pulses
  const int tolerance = 3;

  // 1. Extend for exactly 2 seconds
  unsigned long start = millis();
  while (millis() - start < EXTEND_MS) {
    steerExtend();
    delay(10);
  }
  steerStop();
  Serial.println("Homing: finished timed extend.");

  // 2. Set zero at this fully-extended position
  noInterrupts();
  steerCounts = 0;
  interrupts();
  Serial.println("Homing: zero set at extension.");

  // 3. Retract 1500 pulses
  Serial.println("Homing: retracting 1500 pulses...");

  while (true) {
    long raw = readSteerCounts();
    long error = retractAmount - raw;

    if (abs(error) <= tolerance) {
      steerStop();
      Serial.println("Homing: retract complete.");
      break;
    }

    if (error < 0) {
      steerRetract();
    } else {
      steerExtend();
    }

    delay(20);
  }

  // 4. Set zero again at the new center position
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
  pinMode(PIN_BRAKE_HALL_A, INPUT_PULLUP);
  pinMode(PIN_BRAKE_HALL_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_STEER_HALL_A), isrSteerA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_STEER_HALL_B), isrSteerB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BRAKE_HALL_A), isrBrakeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BRAKE_HALL_B), isrBrakeB, CHANGE);

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
  brakeCounts = 0;

  forceSafeOutputs();

  wdt_enable(WDTO_1S);

  Serial.println("READY: DEBUG BUILD WITH STEERING + BRAKE FEEDBACK");
  // Load saved steering zero offset from EEPROM
  EEPROM.get(EEPROM_ADDR_STEER_ZERO, steerZeroOffset);
  Serial.print("Loaded steer zero offset: ");
  Serial.println(steerZeroOffset);
  
 homeSteeringToCenter();
}

// -------------------- Loop --------------------
void loop() {
  wdt_reset();

  handleSerial();
  handleBluetooth();

  armed = safetyClosed() && commandFresh() && cmdArm;

  updateHeartbeat();

  // Steering ALWAYS allowed after homing
  if (homingDone) {
     long scRaw = readSteerCounts();
     long sc    = scRaw - steerZeroOffset;

      if (cmdSteer > STEER_CMD_THRESH) {
         if (sc < 2400) {
             steerExtend();
          } else {
              steerStop();
          }
      }
      else if (cmdSteer < -STEER_CMD_THRESH) {
         if (sc > -1000) {
             steerRetract();
         } else {
             steerStop();
          }
     }
      else {
          steerStop();
      }

      // Brake with hard limits
      long bc = readBrakeCounts();   // current brake hall position

      if (cmdBrake) {
         // Apply brake (positive direction)
          if (bc < 50) {
              brakeApplyRaw();
          } else {
              brakeStop();   // hit +50 limit
          }
      } else {
          // Release brake (negative direction)
          if (bc > -5) {
              brakeRelease();
          } else {
              brakeStop();   // hit -50 limit
          }
      }


    // Throttle
    uint16_t dac = 0;
    dac = (uint16_t)constrain(
      map(cmdThrottle, 0, 1000, 0, THROTTLE_MAX_DAC),
      0,
      THROTTLE_MAX_DAC
    );


    setThrottleDAC(dac);
  }

  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;

    Serial.print("ARM=");          Serial.print(armed);
    Serial.print(" SAFETY=");      Serial.print(safetyClosed());
    Serial.print(" FRESH=");       Serial.print(commandFresh());
    Serial.print(" cmdArm=");      Serial.print(cmdArm);
    Serial.print(" steer=");       Serial.print(cmdSteer);
    Serial.print(" throttle=");    Serial.print(cmdThrottle);
    Serial.print(" brake=");       Serial.print(cmdBrake);
    Serial.print(" DAC=");         Serial.print(lastThrottleDAC);
    Serial.print(" steerCounts="); Serial.print(readSteerCounts());
    Serial.print(" brakeCounts="); Serial.println(readBrakeCounts());

    BT.print("steerCounts=");
    BT.print(readSteerCounts());
    BT.print(" brakeCounts=");
    BT.println(readBrakeCounts());
    BT.print("ARM=");
    BT.println(armed);

  }
}