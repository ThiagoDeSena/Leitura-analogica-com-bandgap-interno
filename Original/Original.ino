// Exemplo de uso combinado
// ADC+BANDGAP+EEPROM
// + Serial
// (c) LDSB - 2025
#include <EEPROM.h>
#include "CRC8.h"

CRC8  crc;
typedef union uINT  // nome da union
{ int i16;
  uint8_t ui8[2];
} u_INT;  // pseudonimo para union uINT (cria um novo tipo)
typedef struct sGAINCRC { // nome do struct
  u_INT uiGAIN;
  uint8_t u8CRC;
} s_GAINCRC;  // pseudonimo para struct sGAINCRC (cria um novo tipo)
s_GAINCRC stGAINCH0 = {.uiGAIN = {.i16 = 0}, .u8CRC = 0};

const long InternalVoltageReference = 1100L; // 1100mV

int readVDDBandgap(void);
void vADCPrint(void);   void vCargaInicialEEPROM(void);
void vBlink(void);      void vGRAVAEEPROM(void);
void vProcessaSerial(void);

// Valor padrao do ganho
static int iADCGain = 4000;
// Teste de comunicacao do ATMEGA2560
void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  analogRead(A0);   // Ligar e configurar o ADC.
  vCargaInicialEEPROM();
  
}

unsigned long int ul_BlinkTimer = 0;
unsigned long int ul_ADCPrintTimer = 0;
unsigned long int ul_ADCReadTimer = 0, ul_ADCVDDTimer = 0;
void loop() {
  // put your main code here, to run repeatedly:
  vBlink();
  vADCPrint();
  vProcessaSerial();
}
void vADCPrint(void)
{
  static int iVDD = 0;
  //long int li_ADCConvert = ((analogRead(A0)*iVDD*iADCGain)/1024L)/1000L;
  static long int li_ADCConvert = 0;
  if (millis() - ul_ADCVDDTimer > 1001)
  { ul_ADCVDDTimer = millis();
    iVDD = readVDDBandgap();
  }
  if (millis() - ul_ADCReadTimer > 200)
  { ul_ADCReadTimer = millis();
    li_ADCConvert = (long)analogRead(A0) * iVDD;
    li_ADCConvert /= 1024;
    li_ADCConvert *=  (long)iADCGain;
    li_ADCConvert /= 1000L;
  }
  if (millis() - ul_ADCPrintTimer > 550)
  { ul_ADCPrintTimer = millis();
    Serial.println("VDD: " + String(iVDD) + "mV");
    Serial.println("A0: " + String(li_ADCConvert) + "mV");
  }
}
void vBlink(void)
{
  if (millis() - ul_BlinkTimer > 250)
  {
    ul_BlinkTimer = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void vCargaInicialEEPROM(void)
// Bloco de carga inicial dos ganhos da EEPROM
{
  crc.reset();
  EEPROM.get(0,    stGAINCH0);
  crc.add(stGAINCH0.uiGAIN.ui8[0]); crc.add(stGAINCH0.uiGAIN.ui8[1]);
  if (stGAINCH0.u8CRC == crc.calc())
  {
    // Temos dado valido em uniADCGainTemp.i16
    iADCGain = stGAINCH0.uiGAIN.i16;
    Serial.println("Carga do iGain OK! Ganho:" + String(iADCGain));
  } else {
    Serial.println("Carga do iGain não OK!");
    vGRAVAEEPROM();
  }
}

void vGRAVAEEPROM(void)
{
  stGAINCH0.uiGAIN.i16 = iADCGain;
  crc.reset();
  crc.add(stGAINCH0.uiGAIN.ui8[0]); crc.add(stGAINCH0.uiGAIN.ui8[1]);
  stGAINCH0.u8CRC = crc.calc();
  EEPROM.put(0, stGAINCH0); // Grava na EEPROM
}

void vProcessaSerial(void)
{
  char ch_msg[10];
  if (Serial.available())
  {
    int iBytesRead = Serial.readBytesUntil('\n', ch_msg, sizeof(ch_msg)-1);
    ch_msg[iBytesRead] = '\0'; // Null
    uint8_t u8digitCheck = 0;
    for(uint8_t u8k = 0; u8k<iBytesRead; u8k++)
    {
      if(isDigit(ch_msg[u8k]))  u8digitCheck++;
    }
    if(u8digitCheck == (iBytesRead - 1))
    { // Converte para inteiro
      int i16_temp = 0;
      sscanf(ch_msg,"%i",&i16_temp);
      i16_temp = constrain(i16_temp, 0, 32767);
      iADCGain = i16_temp;
      Serial.println("Novo ganho: "+String(iADCGain));
      vGRAVAEEPROM();
    } else {
      Serial.println("Erro de valor");
    }  
  }
}

int readVDDBandgap(void)
{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  //ADMUX[5:0] = 011110;
  //impoe a entrada em 1100mV (band gap)
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
#else // Para qualquer outro ATMEGA
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
#endif
  delayMicroseconds(500);
  // Inicia a conversao
  ADCSRA |= _BV( ADSC );
  // Espera a conversao
  while ( ( (ADCSRA & (1 << ADSC) ) != 0 )  );
  int value = ( ( InternalVoltageReference * 1024L ) / ADC );
  return  value;
}