# Mapa de pines - SUMO_COMBATE

## Motores
- `GPIO33` -> `A1_PIN` (Motor A derecha, direccion)
- `GPIO32` -> `A2_PIN` (Motor A derecha, direccion)
- `GPIO25` -> `PA_PIN` (Motor A derecha, PWM)
- `GPIO27` -> `B1_PIN` (Motor B izquierda, direccion)
- `GPIO14` -> `B2_PIN` (Motor B izquierda, direccion)
- `GPIO26` -> `PB_PIN` (Motor B izquierda, PWM)

## Sensores fotoelectricos (tipo E18-D80NK)
- `GPIO23` -> `SENSOR_FRONT_1_PIN` (frente)
- `GPIO34` -> `SENSOR_FRONT_2_PIN` (frente)
- `GPIO18` -> `SENSOR_FRONT_RIGHT_45_PIN` (frente/derecha, +45°)
- `GPIO35` -> `SENSOR_FRONT_LEFT_45_PIN` (frente/izquierda, -45°)

## Sensores reflectivos (QRE1113 digital)
- `GPIO21` -> `SENSOR_QRE_FRONT_LEFT_PIN` (frontal izquierdo)
- `GPIO19` -> `SENSOR_QRE_FRONT_RIGHT_PIN` (frontal derecho)

## Switch
- `GPIO13` -> `SWITCH_PIN`
- Otro pin del switch -> `GND`
