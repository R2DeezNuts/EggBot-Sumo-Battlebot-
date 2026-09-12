---
layout: page
title: Documentación
---

# Deseño mecánico e de hardware

## Estrutura xeral
- **Dúas rodas motrices** (esquerda e dereita) con motores DC e reductora de ABS para reducir peso. 
- **Chasis plano** en dúas plantas:
  - **Inferior:** motores , batería e sensores.
  - **Superior:** electrónica de control e sensores.

## Dimensións orientativas
- **Largo:** ~20 cm
- **Ancho:** ~20 cm
- **Altura:** ~11 cm
- **Distancia entre rodas:** 12 cm
- **Altura sensores-chan:** 0,3;6 cm

## Sensores
- Barra con **3 sensores digitales** (ex. TCRT5000) colocados en liña na parte dianteira.
- Colocación centrada respecto ao eixe do robot.

## Materiais recomendados
- **Chasis:** acrílico, aluminio lixeiro ou impreso en 3D.
- **Motores:** micromotores DC con caixa reductora (relación 2,53:1–1:3).
- **Rodas:** diámetro 45 cm.
- **Batería:** LiPo 4S (840 mA) ou pack 4×AA NiMH.
- **Separadores e parafusos M3** para montar en dúas plantas.

## Esquema simple (vista superior)
<pre>
┌───────────────────────────┐
│     Sensores IR (5x)      │
│   [S1][S2][S3][S4][S5]    │
│                           │
│    Placa controladora     │
│                           │
[Roda L]Motor   Motor[Roda R]
│                           │
│     Batería no centro     │
└───────────-───────────────┘
</pre>

# Deseño electrónico

## Obxectivo
Placa compacta que aloxa:
- Un **ESP32** como MCU/Wi-Fi/Bluetooth.
- Control de **2 motores DC** (driver dual).
- Lectura de até **5 sensores IR** reflectivos.
- Xestión de **alimentación** desde LiPo / pack AA.
- Conectores para batería, motores, sensores e programación.

## Dimensións recomendadas
- **PCB:** ~46 × 90 mm (axustable segundo o chasis).
- **Capas:** 2 (suficiente para prototipo).


## Elementos principais (BOM orientativa)
- 1 × Módulo/IC **ESP32-WROOM**.
- 1 × Driver motores dual: Generico alta tensión.
- 1 × Regulador buck 5V (por exemplo MP1584) ou conversor LDO 5V si prefires sinxelo.
- 5 × Sensores IR (conector JST ou pin header).
- 2 × Terminal block ou pines para motores (2 polos cada un).
- 1 × Conector para batería (JST-SM ou pad para cable).
- 1 × Fusible para protección de inversión de polaridade de batería (opcional).
- Condensadores, resistencias, diodos de protección e ferrites.

## Esquema funcional (bloques)
- **Entrada batería** → proteción (fuse / diodo) → **buck 5V** → 5V rail
- 5V rail → **regulador 3.3V** → alimenta ESP32 (se o módulo non trae 3.3V)
- ESP32: pinos GPIO → drivers / sensores
- **Driver motores (TB6612)**: alimentado a 5V (VccM), sinais A1/A2/B1/B2 desde ESP32, pin PWM para velocidade (PA/PB)
- Sensores IR: 5 entradas analóxicas dixitalizadas (ADC) ou via divisor + comparador → conectadas a GPIOs ADC do ESP32
- Conectores: motores, batería, sensores, UART/USB.

## Regras de enrutado do PCB (suxestións)
- Lóxica (3.3V) separada do plano de potencia (motores).
- Liñas de alimentación robustas para ruta motor.
- Coloca o condensador grande preto do driver de motor.
- Mantén as trazas PWM curtas entre ESP32 e driver.
- Pensa en separar a masa GND en "star" ou con boa viaje entre GND-motores e GND-logic para evitar ruído (un só plano GND con vias de unión funciona ben en 2 capas).

## Protección e seguridade
- Fusible rearmable (PTC) na entrada da batería

## Estrutura de carpetas suxerida no repo

softwawe/
├── src/
│ ├── main.cpp
│ ├── motores.cpp
│ ├── sensores.cpp
│ └── control.cpp
├── include/
│ ├── motores.h
│ ├── sensores.h
│ └── control.h
├── platformio.ini # se usas PlatformIO
└── README.md

- `motores.cpp/h`: drivers PWM e dirección dos motores.
- `sensores.cpp/h`: lectura e calibración de sensores IR.
- `control.cpp/h`: algoritmo PID ou proporcional.
- `main.cpp`: loop principal e inicialización.