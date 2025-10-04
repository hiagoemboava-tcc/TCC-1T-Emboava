#include <Wire.h> 
#include <SPI.h>
#include <mcp_can.h>
#include <NewPing.h>
#include <math.h>
#include <limits.h>

// -------------------- PINOS --------------------
#define PINO_CORRENTE   A1
#define TRIG_RE         26
#define ECHO_RE         27
#define TRIG_VANTE      28
#define ECHO_VANTE      29
#define PINO_RELE_VANTE 23   // ativo em LOW
#define PINO_RELE_RE    22   // ativo em LOW
#define CAN_CS          53
#define MCP_CLK         MCP_8MHZ
MCP_CAN CAN(CAN_CS);

// -------------------- SENSORES ----------------
NewPing sensorRe(TRIG_RE, ECHO_RE, 200);
NewPing sensorVante(TRIG_VANTE, ECHO_VANTE, 200);

// -------------------- CORRENTE (ACS712-5A) ----
const float VREF       = 5.0f;
const float ACS_OFFSET = 2.5f;
const float ACS_SENS   = 0.185f;

// --- TMP102 ---
#define TMP102_ADDR 0x48

float readTMP102C(){
  Wire.beginTransmission(TMP102_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return NAN;
  Wire.requestFrom((uint8_t)TMP102_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return NAN;
  uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
  raw >>= 4;
  if (raw & 0x800) raw |= 0xF000;
  return ((int16_t)raw) * 0.0625f;
}

// -------------------- CONTROLE DE ENVIO -------
unsigned long ultimaLeitura = 0;
const unsigned long intervaloEnvio = 100;

// -------------------- HISTERESE BOMBAS --------
float LIMIAR_ON   = 1.0f;   // liga acima de ±1°
float LIMIAR_OFF  = 0.5f;   // desliga abaixo de ±0.5°
const unsigned long TEMPO_MIN_LIGADA_MS = 2000; // 2s
bool bombaVante = false;
bool bombaRe    = false;
unsigned long tOnVante = 0;
unsigned long tOnRe    = 0;

// -------------------- FILTROS EMA --------------
struct EMAF {
  float a, y; bool has;
  EMAF(float alpha=0.2f): a(alpha), y(0), has(false) {}
  inline float update(float x) {
    if (!has) { y = x; has = true; return y; }
    y += a * (x - y);
    return y;
  }
};
EMAF emaI(0.12f);
EMAF emaRe(0.30f);
EMAF emaVa(0.30f);
EMAF emaPitch(0.5f);   // suavização mais leve e rápida

// -------------------- FUNÇÕES ------------------
unsigned int distancia10x_cm(NewPing& s){
  unsigned int uS = s.ping_median(5);
  if (uS == 0) return 0;
  unsigned int cm = NewPing::convert_cm(uS);
  if (cm > 500) cm = 500;
  return (unsigned int)(cm * 10U);
}

void setRele(uint8_t pino, bool on){
  digitalWrite(pino, on ? LOW : HIGH); // ativo em LOW
}

float readCorrenteA_mediaEMA() {
  const uint8_t N_I = 16;
  float accV = 0.0f;
  for (uint8_t i=0; i<N_I; i++) {
    accV += analogRead(PINO_CORRENTE) * (VREF / 1023.0f);
  }
  float vSense = accV / N_I;
  float corrente = (vSense - ACS_OFFSET) / ACS_SENS;
  return emaI.update(corrente);
}

void i2cScanOnce() {
  Serial.println(F("I2C scan (uma passada):"));
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  achou em 0x"));
      Serial.println(addr, HEX);
      delay(3);
    }
  }
}

// -------------------- MPU-6050 ----------------
const uint8_t MPU = 0x68;
float pitch=0.0f;
float alpha = 0.90;   // filtro complementar mais rápido
unsigned long t_prev=0;

// offsets globais
float ax_off=0, ay_off=0, az_off=0;
float gx_off=0, gy_off=0, gz_off=0;

void mpuWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void lerAccel(float &Ax, float &Ay, float &Az){
  int16_t AcX,AcY,AcZ;
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU, (uint8_t)6);
  AcX = (Wire.read()<<8) | Wire.read();
  AcY = (Wire.read()<<8) | Wire.read();
  AcZ = (Wire.read()<<8) | Wire.read();
  Ax = (AcX - ax_off) / 16384.0f;  // ±2g
  Ay = (AcY - ay_off) / 16384.0f;
  Az = (AcZ - az_off) / 16384.0f;
}

void lerGyro(float &Gx, float &Gy, float &Gz){
  int16_t GyX,GyY,GyZ;
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU, (uint8_t)6);
  GyX = (Wire.read()<<8) | Wire.read();
  GyY = (Wire.read()<<8) | Wire.read();
  GyZ = (Wire.read()<<8) | Wire.read();
  Gx = (GyX - gx_off) / 131.0f;   // ±250 °/s
  Gy = (GyY - gy_off) / 131.0f;
  Gz = (GyZ - gz_off) / 131.0f;
}

void calibraMPU() {
  const int N = 1000;
  long sAx=0, sAy=0, sAz=0, sGx=0, sGy=0, sGz=0;

  Serial.println(F("Calibrando MPU-6050, mantenha-o parado..."));
  delay(500);

  for(int i=0;i<N;i++){
    int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
    Wire.beginTransmission(MPU); Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU, (uint8_t)6);
    AcX = (Wire.read()<<8) | Wire.read();
    AcY = (Wire.read()<<8) | Wire.read();
    AcZ = (Wire.read()<<8) | Wire.read();

    Wire.beginTransmission(MPU); Wire.write(0x43); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU, (uint8_t)6);
    GyX = (Wire.read()<<8) | Wire.read();
    GyY = (Wire.read()<<8) | Wire.read();
    GyZ = (Wire.read()<<8) | Wire.read();

    sAx += AcX; sAy += AcY; sAz += AcZ;
    sGx += GyX; sGy += GyY; sGz += GyZ;
    delay(2);
  }

  ax_off = (float)sAx/N;
  ay_off = (float)sAy/N;
  az_off = (float)sAz/N - 16384.0f; // remove 1g em Z
  gx_off = (float)sGx/N;
  gy_off = (float)sGy/N;
  gz_off = (float)sGz/N;

  Serial.println(F("Calibracao concluida."));
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(PINO_RELE_VANTE, OUTPUT);
  pinMode(PINO_RELE_RE, OUTPUT);
  setRele(PINO_RELE_VANTE, false);
  setRele(PINO_RELE_RE, false);

  // CAN
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_CLK) != CAN_OK) {
    Serial.println("Erro ao iniciar CAN, tentando...");
    delay(200);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN OK");

  // Scanner I2C
  i2cScanOnce();

  // Inicializa MPU
  mpuWrite(0x6B, 0x00); // wake up
  mpuWrite(0x1B, 0x00); // gyro ±250°/s
  mpuWrite(0x1C, 0x00); // acc ±2g
  delay(100);

  // Calibra offsets
  calibraMPU();

  // Pitch inicial
  float Ax, Ay, Az;
  lerAccel(Ax, Ay, Az);
  pitch = atan2f(-Ax, sqrtf(Ay*Ay + Az*Az)) * 180.0f/PI;
  t_prev = micros();
}

// -------------------- LOOP --------------------
void loop() {
  unsigned long agora = millis();

  // --- Atualiza pitch do MPU-6050 ---
  unsigned long t_now = micros();
  float dt = (t_now - t_prev) * 1e-6f;
  t_prev = t_now;

  float Ax, Ay, Az, Gx, Gy, Gz;
  lerAccel(Ax, Ay, Az);
  lerGyro(Gx, Gy, Gz);

  float pitch_acc = atan2f(-Ax, sqrtf(Ay*Ay + Az*Az)) * 180.0f/PI;
  pitch = alpha * (pitch + Gy * dt) + (1.0f - alpha) * pitch_acc;
  pitch = emaPitch.update(pitch);   // suavização leve (0.4)

  // --- Controle pelas leituras do PITCH ---
  if (agora - ultimaLeitura >= intervaloEnvio) {
    ultimaLeitura = agora;

    if (pitch > LIMIAR_ON && !bombaVante) {
      bombaVante = true; bombaRe = false;
      tOnVante = agora;
      setRele(PINO_RELE_VANTE, true);
      setRele(PINO_RELE_RE, false);
      Serial.println(">> Bomba de Vante LIGADA");
    }
    else if (pitch < -LIMIAR_ON && !bombaRe) {
      bombaRe = true; bombaVante = false;
      tOnRe = agora;
      setRele(PINO_RELE_RE, true);
      setRele(PINO_RELE_VANTE, false);
      Serial.println(">> Bomba de Ré LIGADA");
    }
    else if (fabsf(pitch) <= LIMIAR_OFF) {
      if (bombaVante && (agora - tOnVante) >= TEMPO_MIN_LIGADA_MS) {
        bombaVante = false; setRele(PINO_RELE_VANTE, false);
        Serial.println(">> Bomba de Vante DESLIGADA");
      }
      if (bombaRe && (agora - tOnRe) >= TEMPO_MIN_LIGADA_MS) {
        bombaRe = false; setRele(PINO_RELE_RE, false);
        Serial.println(">> Bomba de Ré DESLIGADA");
      }
    }

    // ---- Sensores auxiliares ----
    unsigned int re10 = distancia10x_cm(sensorRe);
    unsigned int va10 = distancia10x_cm(sensorVante);
    float re_cm = emaRe.update(re10 / 10.0f);
    float va_cm = emaVa.update(va10 / 10.0f);

    float tempC = readTMP102C();
    float corrente = readCorrenteA_mediaEMA();

    // ---- TRANSMISSÃO CAN ----
    int16_t pitch100 = (int16_t)roundf(pitch * 100.0f);
    byte payloadPitch[2] = { (byte)(pitch100 >> 8), (byte)(pitch100 & 0xFF) };
    CAN.sendMsgBuf(0x105, 0, 2, payloadPitch);

    uint16_t re10u = (uint16_t)(re_cm * 10.0f);
    byte payloadRE[2] = { (byte)(re10u >> 8), (byte)(re10u & 0xFF) };
    CAN.sendMsgBuf(0x101, 0, 2, payloadRE);

    uint16_t va10u = (uint16_t)(va_cm * 10.0f);
    byte payloadVA[2] = { (byte)(va10u >> 8), (byte)(va10u & 0xFF) };
    CAN.sendMsgBuf(0x102, 0, 2, payloadVA);

    int16_t t100 = isfinite(tempC) ? (int16_t)roundf(tempC * 100.0f) : (int16_t)INT16_MIN;
    byte payloadT[2] = { (byte)(t100 >> 8), (byte)(t100 & 0xFF) };
    CAN.sendMsgBuf(0x103, 0, 2, payloadT);

    int16_t i100 = (int16_t)roundf(corrente * 100.0f);
    byte payloadI[2] = { (byte)(i100 >> 8), (byte)(i100 & 0xFF) };
    CAN.sendMsgBuf(0x104, 0, 2, payloadI);

    // ---- PRINT DEBUG ----
    Serial.print("Pitch="); Serial.print(pitch,2); Serial.print("° | ");
    Serial.print("Re="); Serial.print(re_cm,1); Serial.print("cm | ");
    Serial.print("Vante="); Serial.print(va_cm,1); Serial.print("cm | ");
    Serial.print("Temp="); Serial.print(tempC,2); Serial.print("°C | ");
    Serial.print("I="); Serial.print(corrente,2); Serial.println(" A");
  }
}
