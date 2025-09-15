// Exemplo de uso combinado
// ADC+BANDGAP+EEPROM+Serial

#include <EEPROM.h>
#include "CRC8.h"
#include <LiquidCrystal.h>

// Ajuste os pinos conforme sua ligação
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Definições dos botões
const int BT0 = 7, BT1 = 6;

#define NUM_ANALOG 16  // Arduino Mega tem A0 até A15
long int li_ADCConvert[NUM_ANALOG] = { 0 };

CRC8 crc;
typedef union uINT  // nome da union
{
  int i16;
  uint8_t ui8[2];
} u_INT;                   // pseudonimo para union uINT (cria um novo tipo)
typedef struct sGAINCRC {  // nome do struct
  u_INT uiGAIN;
  uint8_t u8CRC;
} s_GAINCRC;  // pseudonimo para struct sGAINCRC (cria um novo tipo)
s_GAINCRC stGAINCH0 = { .uiGAIN = { .i16 = 0 }, .u8CRC = 0 };

const long InternalVoltageReference = 1100L;  // 1100mV

// Struct para controle do menu LCD
#define DEF_MAX_PAGES ((NUM_ANALOG + 1) / 2)  // Ajustado para o número de canais
struct menuLCD {
  int ipagAtual;
  int iflagAut;
  int iTMR;
};
struct menuLCD mLCD = { 0, 0, 0 };

int readVDDBandgap(void);
void vADCPrint(void);
void vCargaInicialEEPROM(void);
void vBlink(void);
void vGRAVAEEPROM(void);
void vProcessaSerial(void);
void vBTNGerent(void);
void vLCDGerent(void);
//void vMostraLCD();

// Valor padrao do ganho
static int iADCGain = 4000;

// Variáveis para debounce dos botões
unsigned long ul_contadorBT0 = 0, ul_contadorBT1 = 0;

// Teste de comunicacao do ATMEGA2560
void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  // Configurar pinos dos botões
  pinMode(BT0, INPUT_PULLUP);
  pinMode(BT1, INPUT_PULLUP);

  Serial.begin(9600);
  analogRead(A0);  // Ligar e configurar o ADC.

  vCargaInicialEEPROM();
  lcd.begin(16, 2);  // LCD 16 colunas, 2 linhas
  lcd.print("Inicializando...");
  delay(1000);
  lcd.clear();
}

unsigned long int ul_BlinkTimer = 0;
unsigned long int ul_ADCPrintTimer = 0;
unsigned long int ul_ADCReadTimer = 0, ul_ADCVDDTimer = 0;

void loop() {
  // put your main code here, to run repeatedly:
  vBlink();
  vADCPrint();
  vProcessaSerial();
  vBTNGerent(); 
  vLCDGerent();   
  
}

// void vADCPrint(void) {
//   static int iVDD = 0;
  
//   if (millis() - ul_ADCVDDTimer > 1001) {
//     ul_ADCVDDTimer = millis();
//     iVDD = readVDDBandgap();
//   }
//   if (millis() - ul_ADCReadTimer > 200) {
//     ul_ADCReadTimer = millis();

//     for (uint8_t ch = 0; ch < NUM_ANALOG; ch++) {
//       li_ADCConvert[ch] = (long)analogRead(ch) * iVDD;
//       li_ADCConvert[ch] /= 1024;
//       li_ADCConvert[ch] *= (long)iADCGain;
//       li_ADCConvert[ch] /= 1000L;
//     }
//   }
//   if (millis() - ul_ADCPrintTimer > 550) {
//     ul_ADCPrintTimer = millis();
//     Serial.println("VDD: " + String(iVDD) + "mV");
//     for (uint8_t ch = 0; ch < NUM_ANALOG; ch++) {
//       Serial.print("A");
//       Serial.print(ch);
//       Serial.print(": ");
//       Serial.print(li_ADCConvert[ch]);
//       Serial.println(" mV");
//       //Serial.println("A0: " + String(li_ADCConvert[ch]) + "mV");
//     }
//   }
// }

// Adicione estas variáveis globais
uint8_t u8_currentChannel = 0;  // Canal atual sendo lido

// Modifique a função vADCPrint()
void vADCPrint(void) {
  static int iVDD = 0;
  
  if (millis() - ul_ADCVDDTimer > 1001) {
    ul_ADCVDDTimer = millis();
    iVDD = readVDDBandgap();
  }
  
  // Leitura escalonada - um canal por vez
  if (millis() - ul_ADCReadTimer > 20) {  // Reduzido para 20ms
    ul_ADCReadTimer = millis();
    
    // Lê apenas UM canal por vez
    li_ADCConvert[u8_currentChannel] = (long)analogRead(u8_currentChannel) * iVDD;
    li_ADCConvert[u8_currentChannel] /= 1024;
    li_ADCConvert[u8_currentChannel] *= (long)iADCGain;
    li_ADCConvert[u8_currentChannel] /= 1000L;
    
    // Avança para o próximo canal
    u8_currentChannel = (u8_currentChannel + 1) % NUM_ANALOG;
  }
  
  if (millis() - ul_ADCPrintTimer > 550) {
    ul_ADCPrintTimer = millis();
    Serial.println("VDD: " + String(iVDD) + "mV");
    for (uint8_t ch = 0; ch < NUM_ANALOG; ch++) {
      Serial.print("A");
      Serial.print(ch);
      Serial.print(": ");
      Serial.print(li_ADCConvert[ch]);
      Serial.println(" mV");
    }
  }
}

void vBlink(void) {
  if (millis() - ul_BlinkTimer > 250) {
    ul_BlinkTimer = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void vCargaInicialEEPROM(void)
// Bloco de carga inicial dos ganhos da EEPROM
{
  crc.reset();
  EEPROM.get(0, stGAINCH0);
  crc.add(stGAINCH0.uiGAIN.ui8[0]);
  crc.add(stGAINCH0.uiGAIN.ui8[1]);
  if (stGAINCH0.u8CRC == crc.calc()) {
    // Temos dado valido em uniADCGainTemp.i16
    iADCGain = stGAINCH0.uiGAIN.i16;
    Serial.println("Carga do iGain OK! Ganho:" + String(iADCGain));
  } else {
    Serial.println("Carga do iGain não OK!");
    vGRAVAEEPROM();
  }
}

void vGRAVAEEPROM(void) {
  stGAINCH0.uiGAIN.i16 = iADCGain;
  crc.reset();
  crc.add(stGAINCH0.uiGAIN.ui8[0]);
  crc.add(stGAINCH0.uiGAIN.ui8[1]);
  stGAINCH0.u8CRC = crc.calc();
  EEPROM.put(0, stGAINCH0);  // Grava na EEPROM
}

void vProcessaSerial(void) {
  char ch_msg[10];
  if (Serial.available()) {
    int iBytesRead = Serial.readBytesUntil('\n', ch_msg, sizeof(ch_msg) - 1);
    ch_msg[iBytesRead] = '\0';  // Null
    uint8_t u8digitCheck = 0;
    for (uint8_t u8k = 0; u8k < iBytesRead; u8k++) {
      if (isDigit(ch_msg[u8k])) u8digitCheck++;
    }
    if (u8digitCheck == (iBytesRead - 1)) {  // Converte para inteiro
      int i16_temp = 0;
      sscanf(ch_msg, "%i", &i16_temp);
      i16_temp = constrain(i16_temp, 0, 32767);
      iADCGain = i16_temp;
      Serial.println("Novo ganho: " + String(iADCGain));
      vGRAVAEEPROM();
    } else {
      Serial.println("Erro de valor");
    }
  }
}

int readVDDBandgap(void) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  //ADMUX[5:0] = 011110;
  //impoe a entrada em 1100mV (band gap)
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
#else  // Para qualquer outro ATMEGA
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
#endif
  delayMicroseconds(500);
  // Inicia a conversao
  ADCSRA |= _BV(ADSC);
  // Espera a conversao
  while (((ADCSRA & (1 << ADSC)) != 0))
    ;
  int value = ((InternalVoltageReference * 1024L) / ADC);
  return value;
}


void vBTNGerent(void)
{
  if(!digitalRead(BT0))
  {
    if(millis()-ul_contadorBT0 > 100)
    {
      if(mLCD.ipagAtual > 0) mLCD.ipagAtual--;     // Decrementa pagina do LCD
      ul_contadorBT0 = millis();
      lcd.clear(); // Limpa LCD ao mudar página
    }
  } else {
    ul_contadorBT0 = millis();
  }
  
  if(!digitalRead(BT1))
  {
    if(millis()-ul_contadorBT1 > 100)
    {
      if(mLCD.ipagAtual < DEF_MAX_PAGES - 1) mLCD.ipagAtual++;    // Incrementa pagina do LCD
      ul_contadorBT1 = millis();
      lcd.clear(); // Limpa LCD ao mudar página
    }
  } else {
    ul_contadorBT1 = millis();
  }
}

unsigned long ul_tempoRefreshLCD = 0;

void vLCDGerent(void)
{
  if(millis() - ul_tempoRefreshLCD > 500)  // Refresh a cada 500ms
  {
    ul_tempoRefreshLCD = millis();
    
    lcd.clear();
    
    // Primeira linha - Canal par
    int canal_par = mLCD.ipagAtual * 2;
    if (canal_par < NUM_ANALOG) {
      lcd.setCursor(0, 0);
      lcd.print("A");
      lcd.print(canal_par);
      lcd.print(":");
      lcd.print(li_ADCConvert[canal_par]);
      lcd.print("mV");
    }
    
    // Segunda linha - Canal ímpar
    int canal_impar = mLCD.ipagAtual * 2 + 1;
    if (canal_impar < NUM_ANALOG) {
      lcd.setCursor(0, 1);
      lcd.print("A");
      lcd.print(canal_impar);
      lcd.print(":");
      lcd.print(li_ADCConvert[canal_impar]);
      lcd.print("mV");
    }
    
    // Indicadores de navegação
    lcd.setCursor(15, 0);
    if (mLCD.ipagAtual > 0) lcd.print("↑");
    
    lcd.setCursor(15, 1);
    if (mLCD.ipagAtual < DEF_MAX_PAGES - 1) lcd.print("↓");
  }
}
