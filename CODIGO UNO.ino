#include <SPI.h>
#include <mcp_can.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>  // Biblioteca para LCD I2C

// -------------------- PINOS --------------------
#define CAN_CS   10
#define MCP_CLK  MCP_8MHZ   // ou MCP_16MHZ se seu módulo tiver cristal de 16 MHz
MCP_CAN CAN(CAN_CS);

// Bluetooth em pinos 2 (RX) e 3 (TX)
SoftwareSerial BT(3 , 2); // RX, TX

// LCD I2C (endereço padrão 0x27 ou 0x3F dependendo do módulo)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variáveis para armazenar dados recebidos
float pitch = 0.0, pot = 0.0, nivelRe = 0.0, nivelVante = 0.0;
float tempC = 0.0, corrente = 0.0;
float Ax=0, Ay=0, Az=0;

unsigned long lastUpdate = 0;
unsigned long tela = 0; // alternar telas no LCD

void setup() {
  Serial.begin(9600);  // Debug no PC via USB
  BT.begin(9600);      // Comunicação Bluetooth (HC-05)

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_CLK) == CAN_OK) {
    Serial.println("CAN OK - aguardando dados do MEGA...");
  } else {
    Serial.println("Erro ao iniciar CAN!");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);

  // Inicia LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Sistema USV");
  lcd.setCursor(0,1);
  lcd.print("Iniciando...");
  delay(1500);
  lcd.clear();

  // Cabeçalho CSV
  Serial.println("Timestamp_ms,Pitch,Pot,NivelRe,NivelVante,Temp,Corrente,Ax,Ay,Az");
  BT.println("Timestamp_ms,Pitch,Pot,NivelRe,NivelVante,Temp,Corrente,Ax,Ay,Az");
}

void loop() {
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    if (rxId == 0x100) { // Potenciômetro
      int8_t val = rxBuf[0];
      bool neg = val & 0x80;
      int mag = val & 0x0F;
      pot = (neg ? -1.0 : 1.0) * mag;
      Serial.print("Pot recebido: "); Serial.println(pot);
    }
    else if (rxId == 0x101) { // Nível Ré
      int16_t re10 = (rxBuf[0] << 8) | rxBuf[1];
      nivelRe = re10 / 10.0f;
      Serial.print("Nivel Re recebido: "); Serial.println(nivelRe);
    }
    else if (rxId == 0x102) { // Nível Vante
      int16_t va10 = (rxBuf[0] << 8) | rxBuf[1];
      nivelVante = va10 / 10.0f;
      Serial.print("Nivel Vante recebido: "); Serial.println(nivelVante);
    }
    else if (rxId == 0x103) { // Temperatura
  int16_t t100 = ((int16_t)rxBuf[0] << 8) | rxBuf[1];

  // Sempre mostra o valor bruto recebido
  Serial.print("DEBUG UNO Temp raw = ");
  Serial.println(t100);

  if (t100 == -32768) {
    Serial.println(">> Temperatura inválida recebida (-32768)");
    tempC = 0.0; // mantém 0.0 para o CSV
  } else {
    tempC = t100 / 100.0f;
    Serial.print("Temperatura recebida = ");
    Serial.print(tempC);
    Serial.println(" °C");
  }
}
    else if (rxId == 0x104) { // Corrente
      int16_t i100 = (rxBuf[0] << 8) | rxBuf[1];
      corrente = i100 / 100.0f;
      Serial.print("Corrente recebida = ");
      Serial.print(corrente);
      Serial.println(" A");
    }
    else if (rxId == 0x105) { // Pitch
      int16_t pitch100 = (rxBuf[0] << 8) | rxBuf[1];
      pitch = pitch100 / 100.0f;
      Serial.print("Pitch recebido = ");
      Serial.println(pitch);
    }
    else if (rxId == 0x107) { // Acelerômetro (se começar a enviar no MEGA)
      int16_t ax1000 = (rxBuf[0] << 8) | rxBuf[1];
      int16_t ay1000 = (rxBuf[2] << 8) | rxBuf[3];
      int16_t az1000 = (rxBuf[4] << 8) | rxBuf[5];
      Ax = ax1000 / 1000.0f;
      Ay = ay1000 / 1000.0f;
      Az = az1000 / 1000.0f;
      Serial.print("Accel recebido Ax="); Serial.print(Ax);
      Serial.print(" Ay="); Serial.print(Ay);
      Serial.print(" Az="); Serial.println(Az);
    }

    // Monta linha CSV
    unsigned long ts = millis();
    String linha = String(ts) + "," +
                   String(pitch, 2) + "," +
                   String(pot, 1) + "," +
                   String(nivelRe, 1) + "," +
                   String(nivelVante, 1) + "," +
                   String(tempC, 2) + "," +
                   String(corrente, 2) + "," +
                   String(Ax, 3) + "," +
                   String(Ay, 3) + "," +
                   String(Az, 3);

    // Envia para PC (USB) e Bluetooth (HC-05)
    Serial.println(linha);
    BT.println(linha);
  }

  // Atualiza LCD a cada 2 segundos
  if (millis() - lastUpdate > 2000) {
    lastUpdate = millis();
    lcd.clear();
    if (tela == 0) {
      lcd.setCursor(0, 0);
      lcd.print("Temp:");
      lcd.print(tempC, 1);
      lcd.print((char)223); // símbolo grau
      lcd.print("C");

      lcd.setCursor(0, 1);
      lcd.print("I:");
      lcd.print(corrente, 2);
      lcd.print("A");
    }
    else if (tela == 1) {
      lcd.setCursor(0, 0);
      lcd.print("Pitch:");
      lcd.print(pitch, 1);
      lcd.print((char)223);

      lcd.setCursor(0, 1);
      lcd.print("Pot:");
      lcd.print(pot, 1);
    }
    else if (tela == 2) {
      lcd.setCursor(0, 0);
      lcd.print("NivelRe:");
      lcd.print(nivelRe, 0);
      lcd.print("cm");

      lcd.setCursor(0, 1);
      lcd.print("NivelVa:");
      lcd.print(nivelVante, 0);
      lcd.print("cm");
    }
    tela = (tela + 1) % 3; // alterna entre 3 telas
  }
}
