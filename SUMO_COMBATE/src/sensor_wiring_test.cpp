#include <Arduino.h>

// Test para validar cableado y niveles logicos con tu configuracion actual
// (incluyendo divisor 1k/2k si lo tienes montado).

struct PinStat {
  const char* name;
  uint8_t pin;
  int last;
  uint32_t highCount;
  uint32_t lowCount;
  uint32_t transitions;
};

// Sensores fotoelectricos (rival)
PinStat rivalPins[] = {
  {"FRONT_1", 23, HIGH, 0, 0, 0},
  {"FRONT_2", 34, HIGH, 0, 0, 0},
  {"RIGHT_45", 18, HIGH, 0, 0, 0},
  {"LEFT_45", 35, HIGH, 0, 0, 0},
};

// Sensores QRE (borde)
PinStat edgePins[] = {
  {"QRE_LEFT", 21, HIGH, 0, 0, 0},
  {"QRE_RIGHT", 19, HIGH, 0, 0, 0},
};

PinStat switchPin = {"SWITCH", 13, HIGH, 0, 0, 0};

const uint32_t SAMPLE_PERIOD_MS = 2;     // 500 Hz por pin
const uint32_t REPORT_PERIOD_MS = 500;   // 2 Hz reporte serial

uint32_t lastSampleMs = 0;
uint32_t lastReportMs = 0;
uint32_t bootMs = 0;

void setupPin(PinStat& p, bool pullup) {
  if (pullup) pinMode(p.pin, INPUT_PULLUP);
  else pinMode(p.pin, INPUT);
  p.last = digitalRead(p.pin);
}

void updatePin(PinStat& p) {
  int v = digitalRead(p.pin);
  if (v == HIGH) p.highCount++;
  else p.lowCount++;

  if (v != p.last) {
    p.transitions++;
    p.last = v;
  }
}

void printOne(const PinStat& p) {
  uint32_t total = p.highCount + p.lowCount;
  int highPct = 0;
  if (total > 0) {
    highPct = (int)((100UL * p.highCount) / total);
  }

  Serial.print(p.name);
  Serial.print("(GPIO");
  Serial.print(p.pin);
  Serial.print("): now=");
  Serial.print(p.last == HIGH ? "HIGH" : "LOW");
  Serial.print(" high=");
  Serial.print(highPct);
  Serial.print("% low=");
  Serial.print(100 - highPct);
  Serial.print("% trans=");
  Serial.println(p.transitions);
}

void printDiagnostic(const PinStat& p) {
  uint32_t total = p.highCount + p.lowCount;
  if (total < 50) return;

  if (p.transitions == 0) {
    Serial.print("  - ");
    Serial.print(p.name);
    Serial.print(": SIN CAMBIOS (revisa ajuste/cableado/polaridad) estado=");
    Serial.println(p.last == HIGH ? "HIGH" : "LOW");
  } else if (p.highCount > (total * 95UL) / 100UL) {
    Serial.print("  - ");
    Serial.print(p.name);
    Serial.println(": casi siempre HIGH (si es activo en LOW, aun no detecta objetivo). ");
  } else if (p.lowCount > (total * 95UL) / 100UL) {
    Serial.print("  - ");
    Serial.print(p.name);
    Serial.println(": casi siempre LOW (posible deteccion permanente o polaridad invertida). ");
  }
}

void printHelp() {
  Serial.println("=== SENSOR WIRING TEST ===");
  Serial.println("1) Mueve un objeto a 5-15cm frente a cada fotoelectrico.");
  Serial.println("2) Tapa/destapa cada QRE de borde.");
  Serial.println("3) Acciona el switch.");
  Serial.println("Si un pin no cambia transiciones, hay problema de cableado/ajuste/logica.");
  Serial.println("==========================");
}

void setup() {
  Serial.begin(115200);
  delay(400);

  // Importante: sin pullup interno en sensores para no enmascarar tu conexion externa.
  for (size_t i = 0; i < sizeof(rivalPins) / sizeof(rivalPins[0]); i++) {
    setupPin(rivalPins[i], false);
  }
  for (size_t i = 0; i < sizeof(edgePins) / sizeof(edgePins[0]); i++) {
    setupPin(edgePins[i], false);
  }
  // Switch si lleva GND al otro pin: INPUT_PULLUP interno ayuda.
  setupPin(switchPin, true);

  bootMs = millis();
  lastSampleMs = bootMs;
  lastReportMs = bootMs;

  printHelp();
}

void loop() {
  uint32_t now = millis();

  if ((uint32_t)(now - lastSampleMs) >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;

    for (size_t i = 0; i < sizeof(rivalPins) / sizeof(rivalPins[0]); i++) {
      updatePin(rivalPins[i]);
    }
    for (size_t i = 0; i < sizeof(edgePins) / sizeof(edgePins[0]); i++) {
      updatePin(edgePins[i]);
    }
    updatePin(switchPin);
  }

  if ((uint32_t)(now - lastReportMs) >= REPORT_PERIOD_MS) {
    lastReportMs = now;

    Serial.print("\n--- t=");
    Serial.print((now - bootMs) / 1000.0f, 1);
    Serial.println("s ---");

    Serial.println("[RIVAL]");
    for (size_t i = 0; i < sizeof(rivalPins) / sizeof(rivalPins[0]); i++) {
      printOne(rivalPins[i]);
      printDiagnostic(rivalPins[i]);
    }

    Serial.println("[EDGE]");
    for (size_t i = 0; i < sizeof(edgePins) / sizeof(edgePins[0]); i++) {
      printOne(edgePins[i]);
      printDiagnostic(edgePins[i]);
    }

    Serial.println("[SWITCH]");
    printOne(switchPin);
    printDiagnostic(switchPin);
  }
}
