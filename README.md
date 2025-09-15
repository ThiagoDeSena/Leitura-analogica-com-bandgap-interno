# Sistema de Aquisição de Dados com Calibração por Bandgap e Persistência em EEPROM

![Arduino Mega](https://img.shields.io/badge/Platform-Arduino%20Mega-00979D?style=for-the-badge&logo=arduino)
![AVR Architecture](https://img.shields.io/badge/Architecture-AVR-0078D4?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)

Sistema profissional de aquisição de dados com 8 canais analógicos, implementando calibração dinâmica via bandgap interno e persistência de parâmetros em EEPROM com verificação de integridade CRC-8.

## 📋 Características Principais

- **Aquisição de 8 canais analógicos** com leitura escalonada
- **Calibração precisa** usando referência interna de bandgap (1.1V)
- **Persistência em EEPROM** com verificação de integridade CRC-8
- **Interface LCD 16x2** com navegação por páginas
- **Comunicação serial** para ajuste remoto de parâmetros
- **Sistema robusto** com tratamento de erros e debounce de botões

## 🏗️ Arquitetura do Sistema

### Diagrama de Blocos
```
+----------------+    +-------------+    +-----------------+
|   8 Canais     |    |  Referência |    |    Subsystem    |
|   Analógicos   |--->|   Bandgap   |--->|    de Aquisição |
|  (A0-A7)       |    |  Interna    |    |      ADC        |
+----------------+    +-------------+    +-----------------+
                                 |               |
                                 |               |
+----------------+    +-------------+    +-----------------+
|   Interface    |    |  Sistema de |    |    Persistência |
|     LCD        |<---|  Navegação  |<---|     EEPROM      |
|   16x2 + BTNs  |    |             |    |   com CRC-8     |
+----------------+    +-------------+    +-----------------+
                                 |               |
                                 |               |
+----------------+    +-------------+    +-----------------+
|  Comunicação   |    |   Controle  |    |    Processo     |
|    Serial      |<-->|  Principal  |<-->|    de Dados     |
|   (9600 baud)  |    |   (loop)    |    |                 |
+----------------+    +-------------+    +-----------------+
```

## 🔧 Especificações Técnicas

### Hardware Requerido
- Arduino Mega 2560 (ou Arduino Mega 1280)
- LCD 16x2 com interface parallel
- 2 botões para navegação
- Fontes de sinal analógico (0-5V)

### Configuração de Pinos
```cpp
// LCD - ajuste conforme sua ligação
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Botões
const int BT0 = 7, BT1 = 6;
```

## ⚙️ Funcionalidades Técnicas

### 1. Sistema de Aquisição ADC
Leitura multiplexada de 8 canais com escalonamento temporal:
```cpp
// Leitura rotativa - um canal por vez (20ms por canal)
u8_currentChannel = (u8_currentChannel + 1) % NUM_ANALOG;
li_ADCConvert[u8_currentChannel] = (long)analogRead(u8_currentChannel) * iVDD;
```

### 2. Calibração por Bandgap Interno
Medição precisa de VDD usando referência interna de 1.1V:
```cpp
int readVDDBandgap(void) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | 
          (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
#endif
  // ... conversão e cálculo
  return ((InternalVoltageReference * 1024L) / ADC);
}
```

**Fórmula de cálculo:**
```
VDD_real = (1100mV × 1024) / ADC_bandgap
```

### 3. Persistência com Verificação CRC
Estrutura de dados para armazenamento seguro:
```cpp
typedef union uINT {
  int i16;
  uint8_t ui8[2];
} u_INT;

typedef struct sGAINCRC {
  u_INT uiGAIN;    // Union para acesso a int16/bytes
  uint8_t u8CRC;   // Checksum CRC-8
} s_GAINCRC;
```

Verificação de integridade na inicialização:
```cpp
crc.reset();
crc.add(stGAINCH0.uiGAIN.ui8[0]);
crc.add(stGAINCH0.uiGAIN.ui8[1]);
if (stGAINCH0.u8CRC == crc.calc()) {
  // Dados válidos - carga do ganho
}
```

### 4. Interface de Comunicação
Protocolo serial para ajuste de ganho:
- Baud rate: 9600
- Formato: valor numérico + '\n'
- Exemplo: `4000\n` define ganho para 4000

### 5. Sistema de Navegação LCD
- Paginação automática baseada no número de canais
- Display de 2 canais por página
- Indicadores visuais de navegação (↑↓)

## 📊 Equações e Cálculos

**Conversão ADC para Tensão:**
```
Vchannel = (ADC_value × VDD_real × Gain) / (1024 × 1000)
```

**Cálculo de VDD Real:**
```
VDD_real = (Vbandgap × 1024) / ADC_bandgap
```

## 🚀 Como Utilizar

1. **Carregue o código** no Arduino Mega
2. **Conecte os sinais analógicos** nos canais A0-A7
3. **Ajuste o ganho** via comunicação serial:
   ```
   Envie: 4000\n
   Resposta: "Novo ganho: 4000"
   ```
4. **Navegue pelas páginas** usando os botões BT0 (anterior) e BT1 (próxima)
5. **Monitore os valores** no LCD e via serial

## 📝 Estrutura do Código

```
loop()
├── vBlink()            - LED heartbeat
├── vADCPrint()         - Leitura e exibição ADC
├── vProcessaSerial()   - Comunicação serial
├── vBTNGerent()        - Controle de botões
└── vLCDGerent()        - Atualização do display
```

## 🔍 Detalhes de Implementação

### Otimizações
- Leitura escalonada de canais ADC
- Refresh assíncrono do display
- Debounce de botões com temporização
- Verificação de integridade de dados

### Tratamento de Erros
- Verificação CRC na inicialização
- Constraint nos valores de ganho (0-32767)
- Validação de comandos serial

## 📈 Aplicações

- Sistemas de monitoramento industrial
- Bancadas de teste e medição
- Aquisição de dados com alta precisão
- Sistemas embarcados com requerimento de persistência
- Projetos educacionais avançados

## 📚 Referências Técnicas

- **Datasheet ATmega2560** - Seção ADC (28.7) e Bandgap Reference
- **CRC-8 Algorithm** - Detecção de erros em comunicação de dados
- **EEPROM Data Storage** - Técnicas de persistência em memória não-volátil

## 🛠️ Personalização

Ajuste os parâmetros according to your needs:

```cpp
// Número de canais analógicos
#define NUM_ANALOG 8

// Valor de referência interno (em mV)
const long InternalVoltageReference = 1100L;

// Pinos de hardware
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
const int BT0 = 7, BT1 = 6;
```

## 📄 Licença

Este projeto está sob licença MIT. Veja o arquivo LICENSE para detalhes.

## 🤝 Contribuições

Contribuições são bem-vindas! Sinta-se à vontade para:
- Reportar issues
- Sugerir melhorias
- Enviar pull requests

---

**Desenvolvido para aplicações profissionais de aquisição de dados e sistemas embarcados**
