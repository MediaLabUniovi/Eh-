/*
  Classroom Noise Indicator - ESP32-S3 - v1

  Función:
  - Lee un micrófono I2S tipo INMP441/HW-906CD.
  - Clasifica el ruido de una clase en 3 estados:
      QUIET, MEDIUM_NOISE, HIGH_NOISE.
  - Deja preparado BATTERY_RESERVED para una futura lectura de batería.
  - Mueve un servo 180º suavemente.
  - El servo mueve el bloque con relación 2:1, así que:
      45º servo = 90º bloque.
  - Lee un BME280 y pone una tira NeoPixel de 7 LEDs:
      verde = confort OK,
      naranja = cerca del límite,
      rojo = supera el límite.

  Notas importantes:
  - El servo NO debe alimentarse desde el pin 5V/3V3 del ESP32-S3.
    Usa fuente externa de 5V para el servo y une GND de la fuente con GND del ESP32-S3.
  - El micrófono I2S va a 3.3V.
  - El pin L/R del micrófono se conecta a GND para canal izquierdo.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <ESP32Servo.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include "driver/i2s.h"
#include "esp_intr_alloc.h"

#ifndef I2S_COMM_FORMAT_STAND_I2S
  #define I2S_COMM_FORMAT_STAND_I2S I2S_COMM_FORMAT_I2S
#endif

// ============================================================
// ======================== PINES =============================
// ============================================================

// Micrófono I2S tipo INMP441/HW-906CD
static const int I2S_SCK_PIN = 12;   // SCK / BCLK
static const int I2S_WS_PIN  = 11;   // WS / LRCLK
static const int I2S_SD_PIN  = 10;   // SD / DOUT

// Servo
static const int SERVO_PIN = 5;

// NeoPixel
static const int NEOPIXEL_PIN = 4;
static const int NEOPIXEL_COUNT = 7;

// BME280 I2C
static const int BME_SDA_PIN = 8;
static const int BME_SCL_PIN = 9;

// Batería futura: divisor 47k/47k al ADC.
// De momento está preparado pero NO se usa para cambiar el estado.
static const int BATTERY_ADC_PIN = 1;

// ============================================================
// =================== CONFIGURACIÓN GENERAL ==================
// ============================================================

static const uint32_t SERIAL_BAUD = 115200;

// Decisión de ruido cada 5 segundos
static const uint32_t DECISION_INTERVAL_MS = 5000;

// Ventana de lectura de micrófono dentro de cada ciclo
static const uint32_t NOISE_MEASUREMENT_WINDOW_MS = 1000;

// Calibración inicial del ruido ambiente
static const uint32_t CALIBRATION_SECONDS = 5;

// I2S
static const i2s_port_t I2S_PORT = I2S_NUM_0;
static const int I2S_SAMPLE_RATE = 16000;
static const size_t I2S_READ_BUFFER_SAMPLES = 256;

// Servo
static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2500;
static const uint32_t SERVO_STEP_INTERVAL_MS = 20;
static const float SERVO_STEP_DEG = 1.0f;

// Como el bloque gira el doble que el servo:
// Bloque 0º   -> Servo 0º
// Bloque 90º  -> Servo 45º
// Bloque 180º -> Servo 90º
// Bloque 270º -> Servo 135º
// Bloque 360º -> Servo 180º
static const float SERVO_ANGLE_QUIET = 0.0f;
static const float SERVO_ANGLE_MEDIUM = 45.0f;
static const float SERVO_ANGLE_HIGH = 90.0f;
static const float SERVO_ANGLE_BATTERY_RESERVED = 135.0f;

// Umbrales de ruido relativos al ruido ambiente calibrado.
// Estos valores son de primera versión: probablemente tendrás que ajustarlos en clase.
static const float QUIET_TO_MEDIUM_DB_OVER_FLOOR = 7.0f;
static const float MEDIUM_TO_HIGH_DB_OVER_FLOOR = 14.0f;

// Histéresis para bajar de estado. Bajamos con umbrales más bajos que los de subida.
static const float MEDIUM_TO_QUIET_DB_OVER_FLOOR = 4.0f;
static const float HIGH_TO_MEDIUM_DB_OVER_FLOOR = 10.0f;

// Confirmaciones necesarias para aceptar un cambio de estado.
// 1 = responde cada 5 s.
// 2 = necesita dos ciclos seguidos, es decir, unos 10 s.
static const uint8_t CONFIRMATIONS_TO_CHANGE_STATE = 1;

// BME280 / confort térmico.
// En esta primera versión se usa índice de calor aproximado.
// Si la sensación térmica supera COMFORT_OK_MAX_C, se avisa.
static const float COMFORT_OK_MAX_C = 27.0f;
static const float COMFORT_WARNING_MAX_C = 29.0f;

// Batería futura con divisor 47k/47k.
static const float BATTERY_DIVIDER_RATIO = 2.0f;
static const float BATTERY_CALIBRATION = 1.0f;

// ============================================================
// ========================= OBJETOS ==========================
// ============================================================

Servo servo;
Adafruit_BME280 bme;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================
// ========================= ESTADOS ==========================
// ============================================================

enum SystemState {
  QUIET,
  MEDIUM_NOISE,
  HIGH_NOISE,
  BATTERY_RESERVED
};

struct BMEData {
  bool ok = false;
  float temperatureC = NAN;
  float humidityPct = NAN;
  float pressureHpa = NAN;
  float comfortC = NAN;
};

SystemState currentState = QUIET;
SystemState pendingState = QUIET;
uint8_t pendingStateCount = 0;

float noiseFloorDb = 0.0f;
float lastNoiseDb = 0.0f;
float lastRelativeNoiseDb = 0.0f;

float currentServoAngle = SERVO_ANGLE_QUIET;
float targetServoAngle = SERVO_ANGLE_QUIET;
uint32_t lastServoStepMs = 0;
uint32_t lastDecisionMs = 0;

bool bmeAvailable = false;

// ============================================================
// =================== DECLARACIÓN FUNCIONES ==================
// ============================================================

bool setupI2S();
float readNoiseLevel(uint32_t windowMs);
void calibrateNoiseFloor();
SystemState calculateCandidateNoiseState(float relativeDb);
void updateNoiseState(float relativeDb);
const char* stateToString(SystemState state);
float stateToServoAngle(SystemState state);
void setServoTargetForState(SystemState state);
void moveServoSmooth();
bool setupBME280();
BMEData readBME280();
float calculateComfortValue(float tempC, float humidityPct);
void updateNeoPixels(const BMEData& data);
float readBatteryReserved();
void printStatus(const BMEData& data);

// ============================================================
// ========================== SETUP ===========================
// ============================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1200);

  Serial.println();
  Serial.println("==============================================");
  Serial.println(" Classroom Noise Indicator - ESP32-S3 - v1");
  Serial.println("==============================================");

  // ADC futuro para batería
  analogReadResolution(12);
  pinMode(BATTERY_ADC_PIN, INPUT);

  // NeoPixel
  pixels.begin();
  pixels.clear();
  pixels.show();

  // I2C / BME280
  Wire.begin(BME_SDA_PIN, BME_SCL_PIN);
  bmeAvailable = setupBME280();

  // Servo
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo.write((int)SERVO_ANGLE_QUIET);
  currentServoAngle = SERVO_ANGLE_QUIET;
  targetServoAngle = SERVO_ANGLE_QUIET;

  // I2S / micrófono
  if (!setupI2S()) {
    Serial.println("ERROR: no se pudo iniciar I2S. Revisa pines y plataforma.");
    // Seguimos para que BME/LED/servo puedan probarse.
  }

  calibrateNoiseFloor();

  currentState = QUIET;
  pendingState = QUIET;
  setServoTargetForState(currentState);

  Serial.println("Sistema iniciado.");
  Serial.println();
}

// ============================================================
// =========================== LOOP ===========================
// ============================================================

void loop() {
  // Movimiento suave del servo. Se llama siempre y no bloquea apenas.
  moveServoSmooth();

  const uint32_t now = millis();

  if (now - lastDecisionMs >= DECISION_INTERVAL_MS) {
    lastDecisionMs = now;

    lastNoiseDb = readNoiseLevel(NOISE_MEASUREMENT_WINDOW_MS);
    lastRelativeNoiseDb = lastNoiseDb - noiseFloorDb;
    if (lastRelativeNoiseDb < 0.0f) {
      lastRelativeNoiseDb = 0.0f;
    }

    updateNoiseState(lastRelativeNoiseDb);
    setServoTargetForState(currentState);

    BMEData bmeData = readBME280();
    updateNeoPixels(bmeData);

    printStatus(bmeData);
  }
}

// ============================================================
// ========================= MIC I2S ==========================
// ============================================================

bool setupI2S() {
  i2s_config_t i2sConfig = {};
  i2sConfig.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  i2sConfig.sample_rate = I2S_SAMPLE_RATE;
  i2sConfig.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  i2sConfig.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  i2sConfig.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2sConfig.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2sConfig.dma_buf_count = 8;
  i2sConfig.dma_buf_len = 256;
  i2sConfig.use_apll = false;
  i2sConfig.tx_desc_auto_clear = false;
  i2sConfig.fixed_mclk = 0;

  i2s_pin_config_t pinConfig = {};
  pinConfig.bck_io_num = I2S_SCK_PIN;
  pinConfig.ws_io_num = I2S_WS_PIN;
  pinConfig.data_out_num = I2S_PIN_NO_CHANGE;
  pinConfig.data_in_num = I2S_SD_PIN;

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2sConfig, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("i2s_driver_install error: %d\n", err);
    return false;
  }

  err = i2s_set_pin(I2S_PORT, &pinConfig);
  if (err != ESP_OK) {
    Serial.printf("i2s_set_pin error: %d\n", err);
    return false;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  Serial.println("I2S iniciado.");
  return true;
}

float readNoiseLevel(uint32_t windowMs) {
  int32_t samples[I2S_READ_BUFFER_SAMPLES];
  size_t bytesRead = 0;

  const uint32_t startMs = millis();
  uint32_t sampleCount = 0;

  // Welford para calcular desviación/RMS quitando DC.
  double mean = 0.0;
  double m2 = 0.0;

  while (millis() - startMs < windowMs) {
    esp_err_t err = i2s_read(
      I2S_PORT,
      (void*)samples,
      sizeof(samples),
      &bytesRead,
      pdMS_TO_TICKS(100)
    );

    if (err != ESP_OK || bytesRead == 0) {
      continue;
    }

    const size_t count = bytesRead / sizeof(int32_t);
    for (size_t i = 0; i < count; i++) {
      // INMP441 entrega 24 bits dentro de una palabra de 32 bits.
      // Para cálculo relativo no necesitamos unidades absolutas.
      const double x = (double)(samples[i] >> 8);

      sampleCount++;
      const double delta = x - mean;
      mean += delta / sampleCount;
      const double delta2 = x - mean;
      m2 += delta * delta2;
    }
  }

  if (sampleCount < 10) {
    return 0.0f;
  }

  const double variance = m2 / sampleCount;
  const double rms = sqrt(variance);

  // dB relativos, no dB SPL reales. Sirve para comparar con el ruido ambiente.
  const float db = 20.0f * log10((float)rms + 1.0f);
  return db;
}

void calibrateNoiseFloor() {
  Serial.println();
  Serial.println("Calibrando ruido ambiente...");
  Serial.println("Mantén la clase/entorno en silencio relativo.");

  float sumDb = 0.0f;
  uint32_t validReads = 0;

  for (uint32_t i = 0; i < CALIBRATION_SECONDS; i++) {
    float db = readNoiseLevel(1000);
    if (db > 0.0f) {
      sumDb += db;
      validReads++;
    }
    Serial.printf("  Calibración %lu/%lu: %.2f dB relativos\n",
                  (unsigned long)(i + 1),
                  (unsigned long)CALIBRATION_SECONDS,
                  db);
  }

  if (validReads == 0) {
    noiseFloorDb = 0.0f;
    Serial.println("AVISO: no se recibieron lecturas válidas del micrófono.");
  } else {
    noiseFloorDb = sumDb / validReads;
  }

  Serial.printf("Ruido base calibrado: %.2f dB relativos\n", noiseFloorDb);
  Serial.println();
}

// ============================================================
// ====================== MÁQUINA DE ESTADOS ==================
// ============================================================

SystemState calculateCandidateNoiseState(float relativeDb) {
  switch (currentState) {
    case QUIET:
      if (relativeDb >= MEDIUM_TO_HIGH_DB_OVER_FLOOR) return HIGH_NOISE;
      if (relativeDb >= QUIET_TO_MEDIUM_DB_OVER_FLOOR) return MEDIUM_NOISE;
      return QUIET;

    case MEDIUM_NOISE:
      if (relativeDb >= MEDIUM_TO_HIGH_DB_OVER_FLOOR) return HIGH_NOISE;
      if (relativeDb <= MEDIUM_TO_QUIET_DB_OVER_FLOOR) return QUIET;
      return MEDIUM_NOISE;

    case HIGH_NOISE:
      if (relativeDb <= HIGH_TO_MEDIUM_DB_OVER_FLOOR) {
        if (relativeDb <= MEDIUM_TO_QUIET_DB_OVER_FLOOR) return QUIET;
        return MEDIUM_NOISE;
      }
      return HIGH_NOISE;

    case BATTERY_RESERVED:
      // De momento no se activa. Si se entrase manualmente, vuelve a ruido normal.
      return QUIET;
  }

  return QUIET;
}

void updateNoiseState(float relativeDb) {
  const SystemState candidate = calculateCandidateNoiseState(relativeDb);

  if (candidate == currentState) {
    pendingState = candidate;
    pendingStateCount = 0;
    return;
  }

  if (candidate == pendingState) {
    pendingStateCount++;
  } else {
    pendingState = candidate;
    pendingStateCount = 1;
  }

  if (pendingStateCount >= CONFIRMATIONS_TO_CHANGE_STATE) {
    Serial.printf("Cambio de estado: %s -> %s\n",
                  stateToString(currentState),
                  stateToString(candidate));
    currentState = candidate;
    pendingStateCount = 0;
  }
}

const char* stateToString(SystemState state) {
  switch (state) {
    case QUIET: return "QUIET";
    case MEDIUM_NOISE: return "MEDIUM_NOISE";
    case HIGH_NOISE: return "HIGH_NOISE";
    case BATTERY_RESERVED: return "BATTERY_RESERVED";
  }
  return "UNKNOWN";
}

float stateToServoAngle(SystemState state) {
  switch (state) {
    case QUIET: return SERVO_ANGLE_QUIET;
    case MEDIUM_NOISE: return SERVO_ANGLE_MEDIUM;
    case HIGH_NOISE: return SERVO_ANGLE_HIGH;
    case BATTERY_RESERVED: return SERVO_ANGLE_BATTERY_RESERVED;
  }
  return SERVO_ANGLE_QUIET;
}

// ============================================================
// =========================== SERVO ==========================
// ============================================================

void setServoTargetForState(SystemState state) {
  targetServoAngle = stateToServoAngle(state);
}

void moveServoSmooth() {
  const uint32_t now = millis();
  if (now - lastServoStepMs < SERVO_STEP_INTERVAL_MS) {
    return;
  }
  lastServoStepMs = now;

  if (fabs(currentServoAngle - targetServoAngle) <= SERVO_STEP_DEG) {
    currentServoAngle = targetServoAngle;
    servo.write((int)round(currentServoAngle));
    return;
  }

  if (currentServoAngle < targetServoAngle) {
    currentServoAngle += SERVO_STEP_DEG;
  } else {
    currentServoAngle -= SERVO_STEP_DEG;
  }

  currentServoAngle = constrain(currentServoAngle, 0.0f, 180.0f);
  servo.write((int)round(currentServoAngle));
}

// ============================================================
// =========================== BME280 =========================
// ============================================================

bool setupBME280() {
  bool ok = bme.begin(0x76, &Wire);
  if (!ok) {
    ok = bme.begin(0x77, &Wire);
  }

  if (ok) {
    Serial.println("BME280 detectado.");
  } else {
    Serial.println("AVISO: BME280 no detectado en 0x76 ni 0x77.");
  }

  return ok;
}

BMEData readBME280() {
  BMEData data;

  if (!bmeAvailable) {
    data.ok = false;
    return data;
  }

  data.temperatureC = bme.readTemperature();
  data.humidityPct = bme.readHumidity();
  data.pressureHpa = bme.readPressure() / 100.0f;

  if (isnan(data.temperatureC) || isnan(data.humidityPct)) {
    data.ok = false;
    return data;
  }

  data.comfortC = calculateComfortValue(data.temperatureC, data.humidityPct);
  data.ok = true;
  return data;
}

float calculateComfortValue(float tempC, float humidityPct) {
  // Índice de calor aproximado.
  // Para temperaturas interiores bajas/medias, devolvemos la propia temperatura.
  // El cálculo clásico tiene sentido sobre todo a partir de aprox. 27 ºC.
  if (tempC < 27.0f) {
    return tempC;
  }

  const float T = tempC * 9.0f / 5.0f + 32.0f; // ºF
  const float R = humidityPct;

  float HI = -42.379f
             + 2.04901523f * T
             + 10.14333127f * R
             - 0.22475541f * T * R
             - 0.00683783f * T * T
             - 0.05481717f * R * R
             + 0.00122874f * T * T * R
             + 0.00085282f * T * R * R
             - 0.00000199f * T * T * R * R;

  return (HI - 32.0f) * 5.0f / 9.0f; // ºC
}

// ============================================================
// ========================== NEOPIXEL ========================
// ============================================================

void updateNeoPixels(const BMEData& data) {
  uint32_t color;

  if (!data.ok) {
    // Morado: sensor no detectado o lectura inválida.
    color = pixels.Color(80, 0, 80);
  } else if (data.comfortC > COMFORT_WARNING_MAX_C) {
    color = pixels.Color(255, 0, 0); // rojo
  } else if (data.comfortC > COMFORT_OK_MAX_C) {
    color = pixels.Color(255, 90, 0); // naranja
  } else {
    color = pixels.Color(0, 180, 0); // verde
  }

  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

// ============================================================
// ===================== BATERÍA RESERVADA ====================
// ============================================================

float readBatteryReserved() {
  // Preparado para futuro divisor 47k/47k.
  // No se usa todavía para activar BATTERY_RESERVED.
  const int raw = analogRead(BATTERY_ADC_PIN);
  const float pinVoltage = (float)raw * 3.3f / 4095.0f;
  const float batteryVoltage = pinVoltage * BATTERY_DIVIDER_RATIO * BATTERY_CALIBRATION;
  return batteryVoltage;
}

// ============================================================
// =========================== SERIAL =========================
// ============================================================

void printStatus(const BMEData& data) {
  Serial.println("-------------------- STATUS --------------------");
  Serial.printf("Ruido bruto:      %.2f dB relativos\n", lastNoiseDb);
  Serial.printf("Ruido base:       %.2f dB relativos\n", noiseFloorDb);
  Serial.printf("Ruido sobre base: %.2f dB\n", lastRelativeNoiseDb);
  Serial.printf("Estado:           %s\n", stateToString(currentState));
  Serial.printf("Servo actual:     %.1fº\n", currentServoAngle);
  Serial.printf("Servo objetivo:   %.1fº\n", targetServoAngle);

  if (data.ok) {
    Serial.printf("Temperatura:      %.2f ºC\n", data.temperatureC);
    Serial.printf("Humedad:          %.2f %%\n", data.humidityPct);
    Serial.printf("Presión:          %.2f hPa\n", data.pressureHpa);
    Serial.printf("Confort/sens.:    %.2f ºC\n", data.comfortC);

    if (data.comfortC > COMFORT_WARNING_MAX_C) {
      Serial.println("NeoPixel:         ROJO - supera el límite");
    } else if (data.comfortC > COMFORT_OK_MAX_C) {
      Serial.println("NeoPixel:         NARANJA - cerca del límite");
    } else {
      Serial.println("NeoPixel:         VERDE - confort OK");
    }
  } else {
    Serial.println("BME280:           no disponible");
    Serial.println("NeoPixel:         MORADO - error BME280");
  }

  Serial.printf("Batería futura:   %.2f V, no usada\n", readBatteryReserved());
  Serial.println("------------------------------------------------");
  Serial.println();
}
