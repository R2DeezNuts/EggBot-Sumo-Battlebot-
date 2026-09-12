#include <Arduino.h>

// ================= PINES MOTORES =================
// Motor A = DERECHA | Motor B = IZQUIERDA
const int A1_PIN = 33;
const int A2_PIN = 32;
const int PA_PIN = 25;
const int B1_PIN = 27;
const int B2_PIN = 14;
const int PB_PIN = 26;
const int PA_CHANNEL = 0;
const int PB_CHANNEL = 1;

// ================= PINES SENSORES RIVAL (E18) =================
const int SENSOR_FRONT_1_PIN = 23;        // Frente
const int SENSOR_FRONT_2_PIN = 34;        // Frente
const int SENSOR_FRONT_RIGHT_45_PIN = 18; // Frente/Derecha +45°
const int SENSOR_FRONT_LEFT_45_PIN = 35;  // Frente/Izquierda -45°

// ================= PINES SENSORES BORDE (QRE) =================
const int SENSOR_QRE_FRONT_LEFT_PIN = 21;  // Frontal izquierdo
const int SENSOR_QRE_FRONT_RIGHT_PIN = 19; // Frontal derecho

// ================= PIN SWITCH =================
const int SWITCH_PIN = 13; // Otro pin del switch a GND

// ================= CONFIG GENERAL =================
const bool DEBUG_SERIAL = true;
const uint32_t CONTROL_PERIOD_US = 1000;      // 1 kHz
const uint32_t TELEMETRY_PERIOD_MS = 100;     // 10 Hz
const uint16_t READY_WINDOW_MS = 5000;
const uint16_t SWITCH_DEBOUNCE_MS = 40;
const uint16_t KILL_REARM_HOLD_MS = 2000;     // rehabilitacion: mantener switch ON

// ================= LOGICA DE ENTRADAS =================
const bool SWITCH_ACTIVE_LOW = true;
const bool RIVAL_ACTIVE_LOW = true;

// ------QRE-----
// Lectura RC timing (QRE1113): blanco refleja mas => descarga mas rapida => tiempo menor.
const uint16_t QRE_CHARGE_US = 15;
const uint16_t QRE_TIMEOUT_US = 1200;
const uint16_t QRE_INVALID_LOW_US = 2;
const uint16_t QRE_INVALID_HIGH_MARGIN_US = 2;
const uint16_t QRE_FORCE_WHITE_US_L = 3;
const uint16_t QRE_FORCE_WHITE_US_R = 3;
const uint16_t QRE_FORCE_BLACK_US_L = 10;
const uint16_t QRE_FORCE_BLACK_US_R = 10;
const float QRE_REF_FAST_ALPHA = 0.22f;
const float QRE_REF_SLOW_ALPHA = 0.04f;
const float QRE_MIN_SPAN_US = 20.0f;
const float QRE_HYST_US = 6.0f;

// ================= PWM / MOTOR =================
const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const uint8_t PWM_MAX = 90;
const uint8_t PWM_SEARCH = 45;
const uint8_t PWM_ATTACK = 55;
const uint8_t PWM_ATTACK2 = 80;
const uint8_t PWM_TURN = 62;
const uint8_t PWM_ESCAPE = 62;
const uint16_t MOTOR_BRAKE_MS = 120;

// ================= HOLD / HISTÉRESIS =================
const uint16_t RIVAL_HOLD_MS = 60;
const uint16_t EDGE_HOLD_MS = 110;
const uint16_t ESCAPE_ACTION_MS = 300;
const uint16_t EDGE_RECOVERY_MS = 450;
const uint8_t EDGE_CONF_MAX = 8;
const uint8_t EDGE_CONF_ON_TH = 7;
const uint8_t EDGE_CONF_OFF_TH = 1;

enum RobotState { STANDBY, READY, VERSUS, KILL };

struct SensorFrame {
  uint8_t rival_mask;
  bool edge_left;
  bool edge_right;
  bool switch_on;
  uint32_t ts_us;
};

struct MotionCommand {
  int dirA;    // -1,0,+1
  int dirB;    // -1,0,+1
  uint8_t pwmA;
  uint8_t pwmB;
};

struct SensorFlags {
  uint8_t sensor_status;
  uint8_t sw_status_val;
};

const char* stateToText(RobotState s);

class SensorHub {
 public:
  void begin() {
    pinMode(SENSOR_FRONT_1_PIN, INPUT);
    pinMode(SENSOR_FRONT_2_PIN, INPUT);
    pinMode(SENSOR_FRONT_RIGHT_45_PIN, INPUT);
    pinMode(SENSOR_FRONT_LEFT_45_PIN, INPUT);

    pinMode(SENSOR_QRE_FRONT_LEFT_PIN, INPUT);
    pinMode(SENSOR_QRE_FRONT_RIGHT_PIN, INPUT);

    pinMode(SWITCH_PIN, INPUT_PULLUP);
  }

  SensorFrame readSensors() {
    const uint32_t now_us = micros();
    const uint32_t now_ms = millis();

    const bool front1 = readDigitalSensor(SENSOR_FRONT_1_PIN, RIVAL_ACTIVE_LOW);
    const bool front2 = readDigitalSensor(SENSOR_FRONT_2_PIN, RIVAL_ACTIVE_LOW);
    const bool right45 = readDigitalSensor(SENSOR_FRONT_RIGHT_45_PIN, RIVAL_ACTIVE_LOW);
    const bool left45 = readDigitalSensor(SENSOR_FRONT_LEFT_45_PIN, RIVAL_ACTIVE_LOW);

    bool edgeLRaw = readQreWhite(SENSOR_QRE_FRONT_LEFT_PIN, qre_left_, QRE_FORCE_WHITE_US_L, QRE_FORCE_BLACK_US_L);
    bool edgeRRaw = readQreWhite(SENSOR_QRE_FRONT_RIGHT_PIN, qre_right_, QRE_FORCE_WHITE_US_R, QRE_FORCE_BLACK_US_R);

    // Integrador + histeresis: robusto ante parpadeo por luz ambiente.
    edge_conf_l_ = updateEdgeConfidence(edge_conf_l_, edgeLRaw);
    edge_conf_r_ = updateEdgeConfidence(edge_conf_r_, edgeRRaw);
    edge_state_l_ = updateEdgeState(edge_state_l_, edge_conf_l_);
    edge_state_r_ = updateEdgeState(edge_state_r_, edge_conf_r_);

    // Latch temporal para asegurar reaccion de escape.
    if (edge_state_l_) edge_left_last_on_ms_ = now_ms;
    if (edge_state_r_) edge_right_last_on_ms_ = now_ms;
    bool edgeL = edge_state_l_ || ((now_ms - edge_left_last_on_ms_) <= EDGE_HOLD_MS);
    bool edgeR = edge_state_r_ || ((now_ms - edge_right_last_on_ms_) <= EDGE_HOLD_MS);

    uint8_t mask = 0;
    bitWrite(mask, 0, front1 ? 1 : 0);
    bitWrite(mask, 1, front2 ? 1 : 0);
    bitWrite(mask, 2, right45 ? 1 : 0);
    bitWrite(mask, 3, left45 ? 1 : 0);

    if (mask != 0) {
      rival_last_on_ms_ = now_ms;
    } else if ((now_ms - rival_last_on_ms_) <= RIVAL_HOLD_MS) {
      mask = rival_last_mask_;
    }
    if (mask != 0) rival_last_mask_ = mask;

    bool swStable = debounceSwitch(now_ms);

    SensorFrame out{};
    out.rival_mask = mask;
    out.edge_left = edgeL;
    out.edge_right = edgeR;
    out.switch_on = swStable;
    out.ts_us = now_us;

    last_flags_.sensor_status = mask;
    last_flags_.sw_status_val = swStable ? 0x01 : 0x00;
    bitWrite(last_flags_.sw_status_val, 1, edgeL ? 1 : 0);
    bitWrite(last_flags_.sw_status_val, 2, edgeR ? 1 : 0);

    return out;
  }

  SensorFlags lastFlags() const { return last_flags_; }

 private:
  struct QreAdapt {
    bool initialized = false;
    bool white_state = false;
    bool valid = false;
    float white_ref_us = 0.0f;
    float black_ref_us = 0.0f;
    float th_us = 0.0f;
    uint16_t last_us = 0;
  };

  bool readDigitalSensor(int pin, bool activeLow) const {
    int v = digitalRead(pin);
    return activeLow ? (v == LOW) : (v == HIGH);
  }

  bool debounceSwitch(uint32_t now_ms) {
    bool raw = readDigitalSensor(SWITCH_PIN, SWITCH_ACTIVE_LOW);
    if (raw != sw_last_raw_) {
      sw_last_change_ms_ = now_ms;
      sw_last_raw_ = raw;
    }
    if ((now_ms - sw_last_change_ms_) >= SWITCH_DEBOUNCE_MS) {
      sw_stable_ = sw_last_raw_;
    }
    return sw_stable_;
  }

  uint16_t readQreRcUs(uint8_t pin) const {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delayMicroseconds(QRE_CHARGE_US);

    pinMode(pin, INPUT);
    uint32_t start = micros();
    while (digitalRead(pin) == HIGH) {
      if ((uint32_t)(micros() - start) >= QRE_TIMEOUT_US) {
        return QRE_TIMEOUT_US;
      }
    }
    return (uint16_t)(micros() - start);
  }

  bool readQreWhite(uint8_t pin, QreAdapt& q, uint16_t forceWhiteUs, uint16_t forceBlackUs) {
    const uint16_t t = readQreRcUs(pin);
    q.last_us = t;
    const float tf = (float)t;
    const bool invalidLow = (t <= QRE_INVALID_LOW_US);
    const bool invalidHigh = (t >= (QRE_TIMEOUT_US - QRE_INVALID_HIGH_MARGIN_US));
    const bool valid = !invalidLow && !invalidHigh;
    q.valid = valid;
    if (invalidLow) {
      q.white_state = true;
      return true;
    }
    if (invalidHigh) {
      q.white_state = false;
      return false;
    }

    if (t <= forceWhiteUs) {
      q.white_state = true;
      return true;
    }
    if (t >= forceBlackUs) {
      q.white_state = false;
      return false;
    }

    if (!q.initialized) {
      q.white_ref_us = tf;
      q.black_ref_us = tf + QRE_MIN_SPAN_US;
      q.th_us = (q.white_ref_us + q.black_ref_us) * 0.5f;
      q.initialized = true;
      return true;
    }

    if (tf < q.white_ref_us) {
      q.white_ref_us = (1.0f - QRE_REF_FAST_ALPHA) * q.white_ref_us + QRE_REF_FAST_ALPHA * tf;
    } else {
      q.white_ref_us = (1.0f - QRE_REF_SLOW_ALPHA) * q.white_ref_us + QRE_REF_SLOW_ALPHA * tf;
    }

    if (tf > q.black_ref_us) {
      q.black_ref_us = (1.0f - QRE_REF_FAST_ALPHA) * q.black_ref_us + QRE_REF_FAST_ALPHA * tf;
    } else {
      q.black_ref_us = (1.0f - QRE_REF_SLOW_ALPHA) * q.black_ref_us + QRE_REF_SLOW_ALPHA * tf;
    }

    if (q.black_ref_us < q.white_ref_us) {
      float m = (q.black_ref_us + q.white_ref_us) * 0.5f;
      q.white_ref_us = m - (QRE_MIN_SPAN_US * 0.5f);
      q.black_ref_us = m + (QRE_MIN_SPAN_US * 0.5f);
    }

    float span = q.black_ref_us - q.white_ref_us;
    if (span < QRE_MIN_SPAN_US) {
      float m = (q.black_ref_us + q.white_ref_us) * 0.5f;
      q.white_ref_us = m - (QRE_MIN_SPAN_US * 0.5f);
      q.black_ref_us = m + (QRE_MIN_SPAN_US * 0.5f);
      span = QRE_MIN_SPAN_US;
    }
    q.th_us = q.white_ref_us + 0.5f * span;

    const float th_on = q.th_us - QRE_HYST_US;
    const float th_off = q.th_us + QRE_HYST_US;
    if (!q.white_state && tf <= th_on) q.white_state = true;
    else if (q.white_state && tf >= th_off) q.white_state = false;

    return q.white_state;
  }

  uint8_t updateEdgeConfidence(uint8_t conf, bool active) const {
    if (active) {
      if (conf < EDGE_CONF_MAX) conf++;
    } else {
      if (conf > 0) conf--;
    }
    return conf;
  }

  bool updateEdgeState(bool prevState, uint8_t conf) const {
    if (!prevState && conf >= EDGE_CONF_ON_TH) return true;
    if (prevState && conf <= EDGE_CONF_OFF_TH) return false;
    return prevState;
  }

  uint32_t edge_left_last_on_ms_ = 0;
  uint32_t edge_right_last_on_ms_ = 0;
  uint8_t edge_conf_l_ = 0;
  uint8_t edge_conf_r_ = 0;
  bool edge_state_l_ = false;
  bool edge_state_r_ = false;
  uint32_t rival_last_on_ms_ = 0;
  uint8_t rival_last_mask_ = 0;

  bool sw_last_raw_ = false;
  bool sw_stable_ = false;
  uint32_t sw_last_change_ms_ = 0;

  QreAdapt qre_left_{};
  QreAdapt qre_right_{};

  SensorFlags last_flags_{};
};

class MotorDriver {
 public:
  void begin() {
    pinMode(A1_PIN, OUTPUT);
    pinMode(A2_PIN, OUTPUT);
    pinMode(B1_PIN, OUTPUT);
    pinMode(B2_PIN, OUTPUT);

    ledcSetup(PA_CHANNEL, PWM_FREQ, PWM_RES);
    ledcSetup(PB_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(PA_PIN, PA_CHANNEL);
    ledcAttachPin(PB_PIN, PB_CHANNEL);

    applyImmediate({0, 0, 0, 0});
  }

  void applyMotion(const MotionCommand& cmd) {
    const uint32_t now = millis();

    if (inBrakeWindow_ && now < brakeUntilMs_) {
      applyImmediate({0, 0, 0, 0});
      return;
    }
    inBrakeWindow_ = false;

    bool reversingA = (cmd.dirA != 0 && lastDirA_ != 0 && cmd.dirA != lastDirA_);
    bool reversingB = (cmd.dirB != 0 && lastDirB_ != 0 && cmd.dirB != lastDirB_);

    if (reversingA || reversingB) {
      inBrakeWindow_ = true;
      brakeUntilMs_ = now + MOTOR_BRAKE_MS;
      applyImmediate({0, 0, 0, 0});
      return;
    }

    applyImmediate(cmd);
  }

  MotionCommand lastApplied() const { return lastApplied_; }

 private:
  void setDirA(int d) {
    if (d > 0) {
      digitalWrite(A1_PIN, LOW);
      digitalWrite(A2_PIN, HIGH);
    } else if (d < 0) {
      digitalWrite(A1_PIN, HIGH);
      digitalWrite(A2_PIN, LOW);
    } else {
      digitalWrite(A1_PIN, LOW);
      digitalWrite(A2_PIN, LOW);
    }
  }

  void setDirB(int d) {
    if (d > 0) {
      digitalWrite(B1_PIN, LOW);
      digitalWrite(B2_PIN, HIGH);
    } else if (d < 0) {
      digitalWrite(B1_PIN, HIGH);
      digitalWrite(B2_PIN, LOW);
    } else {
      digitalWrite(B1_PIN, LOW);
      digitalWrite(B2_PIN, LOW);
    }
  }

  void applyImmediate(MotionCommand cmd) {
    cmd.pwmA = min<uint8_t>(cmd.pwmA, PWM_MAX);
    cmd.pwmB = min<uint8_t>(cmd.pwmB, PWM_MAX);

    setDirA(cmd.dirA);
    setDirB(cmd.dirB);
    ledcWrite(PA_CHANNEL, cmd.pwmA);
    ledcWrite(PB_CHANNEL, cmd.pwmB);

    lastDirA_ = cmd.dirA;
    lastDirB_ = cmd.dirB;
    lastApplied_ = cmd;
  }

  int lastDirA_ = 0;
  int lastDirB_ = 0;
  bool inBrakeWindow_ = false;
  uint32_t brakeUntilMs_ = 0;
  MotionCommand lastApplied_{0, 0, 0, 0};
};

class BehaviorCore {
 public:
  RobotState updateState(const SensorFrame& frame) {
    const uint32_t now = millis();

    switch (state_) {
      case STANDBY:
        if (frame.switch_on) {
          readyStartedMs_ = now;
          state_ = READY;
        }
        break;

      case READY:
        if (!frame.switch_on) {
          state_ = KILL;
          killLatchedMs_ = now;
          break;
        }
        if ((now - readyStartedMs_) >= READY_WINDOW_MS) {
          state_ = VERSUS;
        }
        break;

      case VERSUS:
        if (!frame.switch_on) {
          state_ = KILL;
          killLatchedMs_ = now;
        }
        break;

      case KILL:
        // Rehabilitacion: mantener switch ON por KILL_REARM_HOLD_MS
        if (frame.switch_on) {
          if (!rearmCounting_) {
            rearmCounting_ = true;
            rearmStartMs_ = now;
          } else if ((now - rearmStartMs_) >= KILL_REARM_HOLD_MS) {
            rearmCounting_ = false;
            state_ = STANDBY;
          }
        } else {
          rearmCounting_ = false;
        }
        break;
    }

    return state_;
  }

  MotionCommand decideMotion(RobotState state, const SensorFrame& frame) {
    const uint32_t now = millis();

    if (state == STANDBY || state == READY || state == KILL) {
      postEdgeRightSpinPending_ = false;
      return stop();
    }

    if (now < forcedActionUntilMs_) {
      return forcedAction_;
    }
    if (postEdgeLeftSpinPending_) {
      forcedAction_ = { -1, +1, PWM_TURN, PWM_TURN };
      forcedActionUntilMs_ = now + 300;
      postEdgeLeftSpinPending_ = false;
      return forcedAction_;
    }
    if (postEdgeRightSpinPending_) {
      forcedAction_ = { +1, -1, PWM_TURN, PWM_TURN };
      forcedActionUntilMs_ = now + 300;
      postEdgeRightSpinPending_ = false;
      return forcedAction_;
    }

    // Prioridad 1: borde
    if (frame.edge_left && frame.edge_right) {
      forcedAction_ = { -1, -1, PWM_ESCAPE, PWM_ESCAPE };
      forcedActionUntilMs_ = now + ESCAPE_ACTION_MS;
      edgeRecoveryUntilMs_ = now + EDGE_RECOVERY_MS;
      searchBiasRight_ = true;
      return forcedAction_;
    }
    if (frame.edge_left) {
      // Retrocede y gira a derecha
      forcedAction_ = { -1, -1, PWM_ESCAPE, PWM_ESCAPE / 2 };
      forcedActionUntilMs_ = now + ESCAPE_ACTION_MS;
      edgeRecoveryUntilMs_ = now + EDGE_RECOVERY_MS;
      postEdgeLeftSpinPending_ = true;
      searchBiasRight_ = true;
      return forcedAction_;
    }
    if (frame.edge_right) {
      // Retrocede y gira a izquierda, luego añade giro sobre su eje a izquierda
      forcedAction_ = { -1, -1, PWM_ESCAPE / 2, PWM_ESCAPE };
      forcedActionUntilMs_ = now + ESCAPE_ACTION_MS;
      edgeRecoveryUntilMs_ = now + EDGE_RECOVERY_MS;
      postEdgeRightSpinPending_ = true;
      searchBiasRight_ = false;
      return forcedAction_;
    }

    // Prioridad 2: rival
    const bool front1 = bitRead(frame.rival_mask, 0);
    const bool front2 = bitRead(frame.rival_mask, 1);
    const bool frontAny = front1 || front2;
    const bool frontBoth = front1 && front2;
    const bool right45 = bitRead(frame.rival_mask, 2);
    const bool left45 = bitRead(frame.rival_mask, 3);
    const uint8_t frontPwm = frontBoth ? PWM_ATTACK2 : PWM_ATTACK;

    if (frontAny && !right45 && !left45) {
      return {+1, +1, frontPwm, frontPwm};
    }
    if (frontAny && right45 && !left45) {
      return {+1, +1, frontPwm, uint8_t(frontPwm * 0.72f)};
    }
    if (frontAny && left45 && !right45) {
      return {+1, +1, uint8_t(frontPwm * 0.72f), frontPwm};
    }
    if (right45 && !left45) {
      return {+1, +1, PWM_TURN, uint8_t(PWM_TURN * 0.55f)};
    }
    if (left45 && !right45) {
      return {+1, +1, uint8_t(PWM_TURN * 0.55f), PWM_TURN};
    }
    if (left45 && right45) {
      return {+1, +1, PWM_ATTACK, PWM_ATTACK};
    }

    // Prioridad 3: busqueda
    // Evita bucle: tras tocar borde, sesga busqueda temporalmente al lado opuesto.
    if (now < edgeRecoveryUntilMs_) {
      if (searchBiasRight_) {
        return {+1, +1, PWM_SEARCH, uint8_t(PWM_SEARCH * 0.58f)};
      }
      return {+1, +1, uint8_t(PWM_SEARCH * 0.58f), PWM_SEARCH};
    }
    if ((now / 700) % 2 == 0) {
      return {+1, +1, PWM_SEARCH, uint8_t(PWM_SEARCH * 0.58f)};
    }
    return {+1, +1, uint8_t(PWM_SEARCH * 0.58f), PWM_SEARCH};
  }

  RobotState state() const { return state_; }

 private:
  MotionCommand stop() const { return {0, 0, 0, 0}; }

  RobotState state_ = STANDBY;
  uint32_t readyStartedMs_ = 0;
  uint32_t killLatchedMs_ = 0;

  bool rearmCounting_ = false;
  uint32_t rearmStartMs_ = 0;

  MotionCommand forcedAction_{0, 0, 0, 0};
  uint32_t forcedActionUntilMs_ = 0;
  bool postEdgeLeftSpinPending_ = false;
  bool postEdgeRightSpinPending_ = false;
  uint32_t edgeRecoveryUntilMs_ = 0;
  bool searchBiasRight_ = true;
};

SensorHub sensorHub;
BehaviorCore behavior;
MotorDriver motorDriver;

uint32_t lastControlTickUs = 0;
uint32_t lastTelemetryMs = 0;
SensorFrame lastFrame{};
MotionCommand lastCmd{0, 0, 0, 0};

const char* stateToText(RobotState s) {
  switch (s) {
    case STANDBY: return "STANDBY";
    case READY: return "READY";
    case VERSUS: return "VERSUS";
    case KILL: return "KILL";
  }
  return "UNKNOWN";
}

void printTelemetry() {
  SensorFlags flags = sensorHub.lastFlags();
  MotionCommand applied = motorDriver.lastApplied();
  RobotState s = behavior.state();

  Serial.print("state=");
  Serial.print(stateToText(s));
  Serial.print(" switch=");
  Serial.print(lastFrame.switch_on ? "1" : "0");
  Serial.print(" edgeL=");
  Serial.print(lastFrame.edge_left ? "1" : "0");
  Serial.print(" edgeR=");
  Serial.print(lastFrame.edge_right ? "1" : "0");
  Serial.print(" rivalMask=0b");
  Serial.print(lastFrame.rival_mask, BIN);
  Serial.print(" sensor_status=0b");
  Serial.print(flags.sensor_status, BIN);
  Serial.print(" sw_status=0x");
  Serial.print(flags.sw_status_val, HEX);
  Serial.print(" cmd=[");
  Serial.print(lastCmd.dirA);
  Serial.print(",");
  Serial.print(lastCmd.dirB);
  Serial.print(",");
  Serial.print(lastCmd.pwmA);
  Serial.print(",");
  Serial.print(lastCmd.pwmB);
  Serial.print("] applied=[");
  Serial.print(applied.dirA);
  Serial.print(",");
  Serial.print(applied.dirB);
  Serial.print(",");
  Serial.print(applied.pwmA);
  Serial.print(",");
  Serial.print(applied.pwmB);
  Serial.println("]");
}

void setup() {
  Serial.begin(115200);

  sensorHub.begin();
  motorDriver.begin();

  lastControlTickUs = micros();
  lastTelemetryMs = millis();

  if (DEBUG_SERIAL) {
    Serial.println("ESP32 autonomous unified logic ready.");
  }
}

void loop() {
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlTickUs) >= CONTROL_PERIOD_US) {
    lastControlTickUs += CONTROL_PERIOD_US;

    lastFrame = sensorHub.readSensors();
    RobotState currentState = behavior.updateState(lastFrame);
    lastCmd = behavior.decideMotion(currentState, lastFrame);
    motorDriver.applyMotion(lastCmd);
  }

  const uint32_t nowMs = millis();

  if (DEBUG_SERIAL) {
    if ((uint32_t)(nowMs - lastTelemetryMs) >= TELEMETRY_PERIOD_MS) {
      lastTelemetryMs = nowMs;
      printTelemetry();
    }
  }

}
