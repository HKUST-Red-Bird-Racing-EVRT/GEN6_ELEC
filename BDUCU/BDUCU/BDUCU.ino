#include "pins.h"

constexpr int PRECHARGE_TARGET_PERCENT = 95;
constexpr int TS_VOLTAGE_PRESENT_MIN_RAW = 30;
constexpr int ADC_COUNTS_MAX = 1023;
constexpr int ADC_COUNTS_MID = 512;
constexpr int PACK_VOLTAGE_MIN_VX10 = 3080;
constexpr int PACK_VOLTAGE_MAX_VX10 = 3696;
constexpr int PACK_CURRENT_MAX_AX10 = 2600;
constexpr unsigned long CRITICAL_PERSIST_MS = 500UL;
constexpr unsigned long NEGATIVE_SETTLE_MS = 100UL;
constexpr unsigned long PRECHARGE_TIMEOUT_MS = 5000UL; //uint32_t

enum class AmsState {
  IDLE,
  NEGATIVE_ONLY,
  PRECHARGE,
  DRIVE,
  FAULT
};

AmsState amsState = AmsState::IDLE;
bool sdcArmed = false;
bool prevTsAct = false;
unsigned long voltageCriticalStartMs = 0;
unsigned long currentCriticalStartMs = 0;
unsigned long negativeOnlyStartMs = 0;
unsigned long prechargeStartMs = 0;

// Active-low outputs: LOW = ON, HIGH = OFF
void setAmsError(bool active) {
  digitalWrite(AMS_ERR, active ? LOW : HIGH);
}

void setRelays(bool airNegOn, bool airPosOn, bool preOn) {
  digitalWrite(AIR_NEG, airNegOn ? LOW : HIGH);
  digitalWrite(AIR_POS, airPosOn ? LOW : HIGH);
  digitalWrite(PRE, preOn ? LOW : HIGH);
}

bool isEnableRequestPresent() {
  return digitalRead(SDC) == HIGH && digitalRead(TS_ACT) == LOW;
}

bool isTsVoltagePresentWhileOpen() {
  return analogRead(TS_ISO) >= TS_VOLTAGE_PRESENT_MIN_RAW;
}

bool isStartRequestWithOrder() {
  const bool sdcHigh = digitalRead(SDC) == HIGH;
  const bool tsActActive = digitalRead(TS_ACT) == LOW;

  // Arm only when SDC is high while TS_ACT is inactive.
  if (sdcHigh && !tsActActive) {
    sdcArmed = true;
  }

  // Clear arming if SDC drops low.
  if (!sdcHigh) {
    sdcArmed = false;
  }

  const bool tsActFallingToActive = tsActActive && !prevTsAct;
  const bool orderedStart = sdcArmed && sdcHigh && tsActFallingToActive;

  prevTsAct = tsActActive;
  return orderedStart;
}

bool isPrechargeComplete() {
  const int acRaw = analogRead(AC_ISO);
  const int tsRaw = analogRead(TS_ISO);

  if (acRaw <= 0) {
    return false;
  }

  // Use long to avoid overflow on MCUs where int is 16-bit.
  return (static_cast<long>(tsRaw) * 100L) >= (static_cast<long>(acRaw) * PRECHARGE_TARGET_PERCENT);
}

int readPackVoltageVx10() {
  const int acRaw = analogRead(AC_ISO);
  // 600 V pack -> 4.7244 V sense; returns pack voltage in 0.1 V units.
  return static_cast<int>((static_cast<long>(acRaw) * 6350L) / ADC_COUNTS_MAX);
}

int readPackCurrentAx10() {
  const int hallRaw = analogRead(HALL_HI);
  int deltaCounts = hallRaw - ADC_COUNTS_MID;
  if (deltaCounts < 0) {
    deltaCounts = -deltaCounts;
  }

  // HALL_HI assumed +-500 A across 0..5 V; returns absolute current in 0.1 A units.
  return static_cast<int>((static_cast<long>(deltaCounts) * 5000L) / ADC_COUNTS_MID);
}

bool hasPersisted(unsigned long &startMs, bool active, unsigned long durationMs) {
  if (!active) {
    startMs = 0;
    return false;
  }

  if (startMs == 0) {
    startMs = millis();
  }

  return (millis() - startMs) >= durationMs;
}

void latchCriticalFault() {
  // AMS_ERR is expected to open the shutdown path in hardware.
  setAmsError(true);
  setRelays(false, false, false);
  amsState = AmsState::FAULT;
}

void setup() {
  // put your setup code here, to run once:
  int inputPins[] = {INPUT2, AC_ISO, TS_ISO, HALL_LO, HALL_HI, LVB_5V, TS_ACT, SDC};
  for (int i = 0; i < 8; i++) {
    pinMode(inputPins[i], INPUT);
  }

  int outputPins[] = {AMS_ERR, AIR_NEG, AIR_POS, PRE};
  for (int i = 0; i < 4; i++) {
    pinMode(outputPins[i], OUTPUT);
  }

  // Start in safe/open state.
  setAmsError(false);
  setRelays(false, false, false);

}

void loop() {
  const bool enableRequest = isEnableRequestPresent();
  const bool orderedStartRequest = isStartRequestWithOrder();

  const int packVoltageVx10 = readPackVoltageVx10();
  const bool voltageCritical = (packVoltageVx10 < PACK_VOLTAGE_MIN_VX10) ||
                               (packVoltageVx10 > PACK_VOLTAGE_MAX_VX10);
  const bool currentCritical = readPackCurrentAx10() > PACK_CURRENT_MAX_AX10;

  if (amsState != AmsState::FAULT) {
    const bool voltageFault = hasPersisted(voltageCriticalStartMs, voltageCritical, CRITICAL_PERSIST_MS);
    const bool currentFault = hasPersisted(currentCriticalStartMs, currentCritical, CRITICAL_PERSIST_MS);
    if (voltageFault || currentFault) {
      latchCriticalFault();
    }
  }

  if ((amsState == AmsState::IDLE) && isTsVoltagePresentWhileOpen()) {
    latchCriticalFault();
  }

  switch (amsState) {
    case AmsState::IDLE:
      setAmsError(false);
      setRelays(false, false, false);

      if (orderedStartRequest) {
        negativeOnlyStartMs = millis();
        setRelays(true, false, false);  // AIR- ON first
        amsState = AmsState::NEGATIVE_ONLY;
      }
      break;

    case AmsState::NEGATIVE_ONLY:
      if (!enableRequest) {
        setRelays(false, false, false);
        amsState = AmsState::IDLE;
        break;
      }

      if (millis() - negativeOnlyStartMs >= NEGATIVE_SETTLE_MS) {
        prechargeStartMs = millis();
        setRelays(true, false, true);  // Then enable precharge
        amsState = AmsState::PRECHARGE;
      }
      break;

    case AmsState::PRECHARGE: {
      if (!enableRequest) {
        setRelays(false, false, false);
        amsState = AmsState::IDLE;
        break;
      }

      if (isPrechargeComplete()) {
        setRelays(true, true, false);  // PRE off, both AIRs on
        amsState = AmsState::DRIVE;
        break;
      }

      if (millis() - prechargeStartMs >= PRECHARGE_TIMEOUT_MS) {
        setAmsError(true);
        setRelays(false, false, false);
        amsState = AmsState::FAULT;
      }
      break;
    }

    case AmsState::DRIVE:
      if (!enableRequest) {
        setRelays(false, false, false);
        amsState = AmsState::IDLE;
      }
      break;

    case AmsState::FAULT:
      // Fault is latched until reset/power cycle.
      setAmsError(true);
      setRelays(false, false, false);
      break;
  }
}
