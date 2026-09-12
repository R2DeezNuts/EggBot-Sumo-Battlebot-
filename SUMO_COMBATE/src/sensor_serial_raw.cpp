#include <Arduino.h>

// QRE RC timing test (solo QRE)
const int QRE_LEFT_PIN = 21;
const int QRE_RIGHT_PIN = 19;

const uint16_t CHARGE_US = 15;
const uint16_t TIMEOUT_US = 3500;
const uint16_t INVALID_LOW_US = 2;
const uint16_t INVALID_HIGH_MARGIN_US = 2;

const uint32_t SAMPLE_PERIOD_MS = 5;    // 200 Hz
const uint32_t PRINT_PERIOD_MS = 120;   // ~8 Hz

// Filtro adaptativo por sensor
const float REF_FAST_ALPHA = 0.15f;
const float REF_SLOW_ALPHA = 0.02f;
const float MIN_SPAN_US = 35.0f;
const float HYST_US = 10.0f;

struct QreState {
  uint8_t pin;
  uint16_t t_us;
  bool initialized;
  bool white;
  bool valid;
  float white_ref_us;
  float black_ref_us;
  float th_us;
};

QreState ql{QRE_LEFT_PIN, 0, false, false, false, 0.0f, 0.0f, 0.0f};
QreState qr{QRE_RIGHT_PIN, 0, false, false, false, 0.0f, 0.0f, 0.0f};

uint32_t lastSampleMs = 0;
uint32_t lastPrintMs = 0;

uint16_t readQreRcUs(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(CHARGE_US);

  pinMode(pin, INPUT);
  uint32_t start = micros();
  while (digitalRead(pin) == HIGH) {
    if ((uint32_t)(micros() - start) >= TIMEOUT_US) {
      return TIMEOUT_US;
    }
  }
  return (uint16_t)(micros() - start);
}

void clampRefs(QreState& s) {
  if (s.black_ref_us < s.white_ref_us) {
    float m = (s.black_ref_us + s.white_ref_us) * 0.5f;
    s.white_ref_us = m - (MIN_SPAN_US * 0.5f);
    s.black_ref_us = m + (MIN_SPAN_US * 0.5f);
  }

  float span = s.black_ref_us - s.white_ref_us;
  if (span < MIN_SPAN_US) {
    float m = (s.black_ref_us + s.white_ref_us) * 0.5f;
    s.white_ref_us = m - (MIN_SPAN_US * 0.5f);
    s.black_ref_us = m + (MIN_SPAN_US * 0.5f);
  }
}

void updateQre(QreState& s) {
  s.t_us = readQreRcUs(s.pin);
  float tf = (float)s.t_us;
  s.valid = (s.t_us > INVALID_LOW_US) && (s.t_us < (TIMEOUT_US - INVALID_HIGH_MARGIN_US));
  if (!s.valid) {
    s.white = false;
    return;
  }

  if (!s.initialized) {
    s.white_ref_us = tf;
    s.black_ref_us = tf + MIN_SPAN_US;
    s.th_us = (s.white_ref_us + s.black_ref_us) * 0.5f;
    s.white = true;
    s.initialized = true;
    return;
  }

  if (tf < s.white_ref_us) {
    s.white_ref_us = (1.0f - REF_FAST_ALPHA) * s.white_ref_us + REF_FAST_ALPHA * tf;
  } else {
    s.white_ref_us = (1.0f - REF_SLOW_ALPHA) * s.white_ref_us + REF_SLOW_ALPHA * tf;
  }

  if (tf > s.black_ref_us) {
    s.black_ref_us = (1.0f - REF_FAST_ALPHA) * s.black_ref_us + REF_FAST_ALPHA * tf;
  } else {
    s.black_ref_us = (1.0f - REF_SLOW_ALPHA) * s.black_ref_us + REF_SLOW_ALPHA * tf;
  }

  clampRefs(s);

  float span = s.black_ref_us - s.white_ref_us;
  s.th_us = s.white_ref_us + 0.5f * span;

  const float th_on = s.th_us - HYST_US;
  const float th_off = s.th_us + HYST_US;
  if (!s.white && tf <= th_on) s.white = true;
  else if (s.white && tf >= th_off) s.white = false;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(QRE_LEFT_PIN, INPUT);
  pinMode(QRE_RIGHT_PIN, INPUT);

  Serial.println("QRE adaptive test");
  Serial.println("Objetivo caso prueba: izquierda=negro(0) derecha=blanco(1)");
  Serial.println("Cols: tL tR | vL vR | thL thR | WL WR | BL BR | mask(WLbit0,WRbit1) | target");
}

void loop() {
  uint32_t now = millis();

  if ((uint32_t)(now - lastSampleMs) >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    updateQre(ql);
    updateQre(qr);
  }

  if ((uint32_t)(now - lastPrintMs) >= PRINT_PERIOD_MS) {
    lastPrintMs = now;

    const bool wl = ql.white;
    const bool wr = qr.white;
    const bool bl = !wl;
    const bool br = !wr;

    uint8_t mask = 0;
    bitWrite(mask, 0, wl ? 1 : 0);
    bitWrite(mask, 1, wr ? 1 : 0);

    const bool targetOk = (ql.valid && qr.valid && bl && wr); // izq negro + der blanco

    Serial.print(ql.t_us); Serial.print(' ');
    Serial.print(qr.t_us); Serial.print(" | ");
    Serial.print(ql.valid ? 1 : 0); Serial.print(' ');
    Serial.print(qr.valid ? 1 : 0); Serial.print(" | ");
    Serial.print((int)ql.th_us); Serial.print(' ');
    Serial.print((int)qr.th_us); Serial.print(" | ");
    Serial.print(wl ? 1 : 0); Serial.print(' ');
    Serial.print(wr ? 1 : 0); Serial.print(" | ");
    Serial.print(bl ? 1 : 0); Serial.print(' ');
    Serial.print(br ? 1 : 0); Serial.print(" | 0b");
    Serial.print(mask, BIN);
    Serial.print(" | ");
    if (!ql.valid || !qr.valid) {
      Serial.println("INVALID");
    } else {
      Serial.println(targetOk ? "OK" : "--");
    }
  }
}
