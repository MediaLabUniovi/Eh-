# Classroom Noise Indicator - ESP32-S3 - v1

Proyecto para ESP32-S3 en PlatformIO.

## 1. Resumen del sistema

El ESP32-S3 lee:

- Un micrófono I2S tipo INMP441/HW-906CD.
- Un sensor BME280 por I2C.
- En el futuro, una batería mediante divisor 47k/47k.

Y controla:

- Un servo de 180º.
- Una tira NeoPixel de 7 LEDs.

El servo mueve un bloque rectangular mediante engranaje 2:1 multiplicador.
Por tanto, el bloque gira el doble que el servo.

| Estado | Bloque | Servo |
|---|---:|---:|
| QUIET | 0º | 0º |
| MEDIUM_NOISE | 90º | 45º |
| HIGH_NOISE | 180º | 90º |
| BATTERY_RESERVED | 270º | 135º |

El estado BATTERY_RESERVED está preparado en código, pero no se activa todavía.

---

## 2. Cableado propuesto

### Micrófono I2S HW-906CD / INMP441

| Pin micrófono | Conectar a ESP32-S3 | Nota |
|---|---|---|
| VDD | 3V3 | No usar 5V |
| GND | GND | Masa común |
| SCK | GPIO12 | Bit clock / BCLK |
| WS | GPIO11 | Word select / LRCLK |
| SD | GPIO10 | Datos I2S / DOUT |
| L/R | GND | Selecciona canal izquierdo |

No dejes el pin L/R al aire.

---

### Servo 180º

| Cable servo | Conectar a | Nota |
|---|---|---|
| Señal | GPIO5 ESP32-S3 | PWM servo |
| VCC | 5V fuente externa | Recomendado, no desde el ESP32-S3 |
| GND | GND fuente externa + GND ESP32-S3 | Masa común obligatoria |

Muy importante: une el GND de la fuente externa del servo con el GND del ESP32-S3.

---

### BME280

| Pin BME280 | Conectar a ESP32-S3 | Nota |
|---|---|---|
| VCC/VIN | 3V3 | Recomendado |
| GND | GND | Masa común |
| SDA | GPIO8 | I2C datos |
| SCL | GPIO9 | I2C reloj |

El código busca el BME280 en dirección 0x76 y, si no aparece, prueba 0x77.

---

### NeoPixel, 7 LEDs

| Pin tira | Conectar a | Nota |
|---|---|---|
| DIN | GPIO4 ESP32-S3 | Datos |
| 5V | 5V fuente | Puede ser la misma fuente de 5V del servo si aguanta corriente |
| GND | GND fuente + GND ESP32-S3 | Masa común obligatoria |

Recomendado:

- Poner un condensador de 470 µF o 1000 µF entre 5V y GND cerca de la tira.
- Poner una resistencia de 330-470 ohm en serie con DIN.
- Si hay fallos con la tira a 5V, usar adaptador de nivel lógico para pasar la señal de 3.3V a 5V.

---

### Divisor de batería futuro, reservado

Dos resistencias de 47k/47k.

| Punto | Conectar a |
|---|---|
| Batería + | Resistencia 47k superior |
| Punto medio del divisor | GPIO1 |
| Resistencia 47k inferior | GND |
| Batería - | GND común |

El código tiene la función `readBatteryReserved()`, pero no usa todavía ese valor para cambiar de estado.

---

## 3. Instalación en PlatformIO

1. Abre Visual Studio Code.
2. Instala la extensión PlatformIO si no la tienes.
3. Descomprime este ZIP.
4. Abre la carpeta `noise_classroom_esp32s3_v1` desde PlatformIO.
5. Conecta el ESP32-S3 por USB.
6. Compila con `Build`.
7. Sube con `Upload`.
8. Abre el monitor serie a 115200 baudios.

---

## 4. Calibración inicial

Al arrancar, el sistema calibra el ruido ambiente durante 5 segundos.
Durante ese tiempo conviene que la clase esté en silencio relativo.

Después calcula el ruido relativo sobre esa base.

En `src/main.cpp`, ajusta estos valores si hace falta:

```cpp
static const float QUIET_TO_MEDIUM_DB_OVER_FLOOR = 7.0f;
static const float MEDIUM_TO_HIGH_DB_OVER_FLOOR = 14.0f;
static const float MEDIUM_TO_QUIET_DB_OVER_FLOOR = 4.0f;
static const float HIGH_TO_MEDIUM_DB_OVER_FLOOR = 10.0f;
```

Si cambia demasiado tarde, baja los valores.
Si cambia demasiado pronto, súbelos.

---

## 5. Ajuste del servo

En `src/main.cpp` puedes cambiar:

```cpp
static const float SERVO_ANGLE_QUIET = 0.0f;
static const float SERVO_ANGLE_MEDIUM = 45.0f;
static const float SERVO_ANGLE_HIGH = 90.0f;
static const float SERVO_ANGLE_BATTERY_RESERVED = 135.0f;
```

Si el bloque no queda alineado con las caras, cambia estos ángulos.

Si el servo se mueve al revés, prueba algo como:

```cpp
QUIET = 135º
MEDIUM = 90º
HIGH = 45º
BATTERY = 0º
```

---

## 6. LEDs por confort térmico

El BME280 mide temperatura y humedad.
El código calcula una sensación térmica aproximada:

- Verde: confort OK.
- Naranja: cerca del límite.
- Rojo: supera el límite.
- Morado: error o BME280 no detectado.

Ajusta los umbrales aquí:

```cpp
static const float COMFORT_OK_MAX_C = 27.0f;
static const float COMFORT_WARNING_MAX_C = 29.0f;
```

---

## 7. Problemas típicos

### El micrófono no lee nada

- Comprueba VDD a 3.3V.
- Comprueba GND.
- Comprueba que L/R está conectado a GND.
- Revisa SCK, WS y SD.
- Prueba a cambiar `I2S_CHANNEL_FMT_ONLY_LEFT` por `I2S_CHANNEL_FMT_ONLY_RIGHT` si has puesto L/R a 3.3V.

### El servo tiembla

- Alimenta el servo con fuente externa de 5V.
- Une GND de la fuente con GND del ESP32-S3.
- Prueba otro cable o acorta cables.
- Baja carga mecánica del bloque.

### Los NeoPixel hacen cosas raras

- Une todas las masas.
- Usa condensador entre 5V y GND.
- Usa resistencia en DIN.
- Si la tira va a 5V y falla, usa adaptador de nivel para DIN.

### El BME280 no aparece

- Revisa SDA/SCL.
- Revisa alimentación.
- Prueba dirección 0x76/0x77. El código ya prueba ambas.
