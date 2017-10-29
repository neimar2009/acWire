// acWire.cpp

#include <Arduino.h>
#include "acWire.h"

#define writeI2C 0
#define readI2C  1
#define ACK      0
#define NOACK    1

uint8_t digitalOpenDran(uint8_t pin, uint8_t val);

#define SDA_WRITE_BIT(_pin, _bit)  {digitalOpenDran((_pin), (_bit & 0x80) ? HIGH : LOW); }
#define SDA_READ_BIT(_pin, _byte)  {digitalOpenDran((_pin), (HIGH)) ? _byte |= 1: _byte &= 0xFE; }

#define TWI_CLOCK_LIMIT            400000  // 400kHz
#define TWI_PERIOD_HALF            ((uint32_t)(((1/TWI_CLOCK_LIMIT)/2) * 1000000))

/*********************************************************************************/
/*
static void turnOffPWM(uint8_t timer)
{
  switch (timer)
  {
    #if defined(TCCR1A) && defined(COM1A1)
    case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
    #endif
    #if defined(TCCR1A) && defined(COM1B1)
    case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
    #endif
    #if defined(TCCR1A) && defined(COM1C1)
    case TIMER1C:   cbi(TCCR1A, COM1C1);    break;
    #endif
    
    #if defined(TCCR2) && defined(COM21)
    case  TIMER2:   cbi(TCCR2, COM21);      break;
    #endif
    
    #if defined(TCCR0A) && defined(COM0A1)
    case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
    #endif
    
    #if defined(TCCR0A) && defined(COM0B1)
    case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
    #endif
    #if defined(TCCR2A) && defined(COM2A1)
    case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
    #endif
    #if defined(TCCR2A) && defined(COM2B1)
    case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
    #endif
    
    #if defined(TCCR3A) && defined(COM3A1)
    case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
    #endif
    #if defined(TCCR3A) && defined(COM3B1)
    case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
    #endif
    #if defined(TCCR3A) && defined(COM3C1)
    case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
    #endif

    #if defined(TCCR4A) && defined(COM4A1)
    case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
    #endif          
    #if defined(TCCR4A) && defined(COM4B1)
    case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
    #endif
    #if defined(TCCR4A) && defined(COM4C1)
    case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
    #endif      
    #if defined(TCCR4C) && defined(COM4D1)
    case TIMER4D: cbi(TCCR4C, COM4D1);  break;
    #endif      
      
    #if defined(TCCR5A)
    case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
    case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
    case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
    #endif
  }
}

// val = LOW, output low (Sink); val = HIGH, input with pull-up
uint8_t digitalOpenDran(uint8_t pin, uint8_t val) {

  uint8_t timer = digitalPinToTimer(pin);
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *reg, *out;

  if (port == NOT_A_PIN) return LOW;

  // If the pin that support PWM output, we need to turn it off
  // before doing a digital write.
  if (timer != NOT_ON_TIMER) turnOffPWM(timer);

  reg = portModeRegister(port);
  out = reg + 1; //portOutputRegister(port);

  uint8_t ret = LOW;
  uint8_t oldSREG = SREG;
  cli();

    // \
    |      |        |    PUD     |         |         |                                             \
    | DDxn | PORTxn | (in MCUCR) |   I/O   | Pull-up | Comment                                     \
    |------+--------+------------+---------+---------+---------------------------------------------\
    |  0   |   0    |     X      |  Input  |   No    | Tri-state (Hi-Z)                            \
    |  0   |   1    |     0      |  Input  |   Yes   | Pxn will source current if ext. pulled low. \
    |  0   |   1    |     1      |  Input  |   No    | Tri-state (Hi-Z)                            \
    |  1   |   0    |     X      |  Output |   No    | Output Low (Sink)                           \
    |  1   |   1    |     X      |  Output |   No    | Output High (Source)                        |

  if (val == LOW) {
    *out &= ~bit;   // This position avoids conflict of state.
    *reg |=  bit;
  } else {
    *reg &= ~bit;
    *out |=  bit;
    ret = (*(reg - 1) & bit); // portInputRegister(port);
  }

  SREG = oldSREG;
  return ret;
}
*/

// val = LOW, output low (Sink); val = HIGH, input with pull-up
uint8_t digitalOpenDran(uint8_t pin, uint8_t val) {

  if (val == LOW) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  } else {
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);
    if (digitalRead(pin)) return HIGH;
  }
  return LOW;
}


/*********************************************************************************/

uint8_t acWireClass::TWI_readBit(uint8_t data) {
  //          __
  // SCL  ___/  \_
  //      _ .. ___
  // SDA  _X..X___
  //
  data <<= 1;                       // Desloca data para esquerda...
  digitalOpenDran(pinSDA, HIGH);    // <- Mesmo que 'pinMode(pinSDA, INPUT)'
  delayMicroseconds(periodHalf);    // Período de preparação do bit que o escravo vai enviar.
  digitalOpenDran(pinSCL, HIGH);    // open-dran
  delayMicroseconds(periodHalf);    // Período de leitura.
  SDA_READ_BIT(pinSDA, data);       // Rotina de leitura do bit enviado pelo escravo.
  digitalOpenDran(pinSCL, LOW);     // Clock baixo, fim da leitura.
  return data;                      // Retorna data prepara para a próxima leitura.
}

uint8_t acWireClass::TWI_writeBit(uint8_t data) {
  //          _
  // SCL  ___/ \_
  //      _ _____
  // SDA  _X_____
  //
  SDA_WRITE_BIT(pinSDA, data);  // Rotina de escrita do bit para leitura pelo escravo.
  delayMicroseconds(periodHalf);    // Período de estabilização do bit que será lido pelo escravo.
  digitalOpenDran(pinSCL, HIGH);
  delayMicroseconds(periodHalf);    // Período de leitura do bit pelo escravo.
  digitalOpenDran(pinSCL, LOW);
  data <<= 1;                   // Desloca data para esquerda...
  return data;                  // Retorna data prepara para a próxima escrita.
}

//== Inicialização ==========================================================

acWireClass::acWireClass(uint8_t pinSDA, uint8_t pinSCL, boolean mode = true)
  : pinSDA(pinSDA), pinSCL(pinSCL) {

  periodHalf = TWI_PERIOD_HALF;   // <- Meio período definico para 400kHz.
  if(!mode) periodHalf *= 4; // <- para 'mode' de baixa velocidade 100kHz
  releaseSDA();
}

void acWireClass::begin(uint8_t slave) {

  slaveID = slave <<= 1;
}

/*  Reads data  ******************************************/
uint8_t acWireClass::readBegin(uint8_t* data, uint8_t len) {

  beginMultiTransactions(readI2C);
  modeACK = ACK;   // Ativado resposta ACK em todas as leituras.
  flagI2C = readI2C;
  return read(data, len);
}

uint8_t acWireClass::readBegin() {

  uint8_t data = 0;
  readBegin(&data, 1);
  return data;
}

uint8_t acWireClass::read(uint8_t* data, uint8_t len) {

  //  Header 
  if(!inMuiltiTransaction) { 
    if(!sendHeader(readI2C)) return 0;
  } else {
    if(flagI2C == writeI2C) {
      reopenTransaction();
      if(!sendHeader(readI2C)) return 0;
    }
  }
  flagI2C = readI2C;
  //  Reading
  uint8_t d = 0;
  while (d < len) {
    data[d] = receiveByte(d == (len-1) && modeACK);
    d++;
  }

  //  End
  if(!inMuiltiTransaction) closeTransaction();
  return d;
}

uint8_t acWireClass::read() {

  uint8_t data = 0;
  if(read(&data, 1) == 0) return 0;
  return data;
}

uint8_t acWireClass::readEnd(uint8_t* data, uint8_t len) {

  modeACK = NOACK; // Libera a resposta NOACK na última leitura.
  uint8_t l = read(data, len);
  endMultiTransactions();
  return l;
}

uint8_t acWireClass::readEnd() {

  uint8_t data = 0;
  if(readEnd(&data, 1) == 0) return 0;
  return data;
}

/*  Writes data  ******************************************/
uint8_t acWireClass::writeBegin(uint8_t* data, uint8_t len) {

  beginMultiTransactions(writeI2C);
  return write(data, len);
}

uint8_t acWireClass::writeBegin(uint8_t data) {

  return writeBegin(&data, 1);
}

uint8_t acWireClass::write(uint8_t* data, uint8_t len) {

  if(!inMuiltiTransaction && !sendHeader(writeI2C)) return 0;

  flagI2C = writeI2C;
  uint8_t d = 0;
  while (d < len) {
    if(!sendByte(data[d])) break;
    d++;
  }  

  if(!inMuiltiTransaction) closeTransaction();
  return d; // return (d == len);
}

uint8_t acWireClass::write(uint8_t data) {

  return write(&data, 1);
}

uint8_t acWireClass::writeEnd(uint8_t* data, uint8_t len) {

  uint8_t l = write(data, len);
  endMultiTransactions();
  return l;
}

uint8_t acWireClass::writeEnd(uint8_t data) {

  return writeEnd(&data, 1);
}

/************************************************************************************************/

void acWireClass::openTransaction() {
  //      __
  // SCL    \_
  //      _
  // SDA   \__
  //
  releaseSDA();
  digitalOpenDran(pinSDA, LOW);           // Indica iníco de operação.
  delayMicroseconds(periodHalf);          // Tempo de identificação de início.
  digitalOpenDran(pinSCL, LOW);           // Indica iníco de operação.
  delayMicroseconds(periodHalf);          // Tempo de identificação de início.
}

void acWireClass::closeTransaction() {
  //         __
  // SCL  __/ 
  //      _   _
  // SDA  _\_/
  //
  digitalOpenDran(pinSDA, LOW);           // Leva SDA para baixo.
  delayMicroseconds(periodHalf);          // Período de acomadação do escravo.
  while (!digitalOpenDran(pinSCL, HIGH)); // Clock alto: intervalo de reconhecimento de ação.
  delayMicroseconds(periodHalf);          // Período de latência para o sinal de encerramento.
  digitalOpenDran(pinSDA, HIGH);          // SDA alto após Clock alto indica encerramento.
  delayMicroseconds(periodHalf);          // Período de latência para o sinal de encerramento.
  releaseSDA();
}

void acWireClass::reopenTransaction() {
  //            _
  // SCL  _____/ 
  //      _   ___
  // SDA  _\_/
  //
  digitalOpenDran(pinSDA, LOW);           // Leva SDA para baixo.
  delayMicroseconds(periodHalf);          // Período de acomadação do escravo.
  digitalOpenDran(pinSDA, HIGH);          // Prepara SDA para o sinal de reabertura.
  delayMicroseconds(periodHalf);          // Tempo de identificação de estado.
  while (!digitalOpenDran(pinSCL, HIGH)); // Indica operação de reabertura.
  delayMicroseconds(periodHalf);          // Tempo de identificação de início.
  openTransaction();                      // Conclui a reabertura.
}

void acWireClass::releaseSDA() {

  // Espera pela estabilização de SDA.
  while(!digitalOpenDran(pinSDA, HIGH)) {
    digitalOpenDran(pinSCL, LOW);         // É ingetado meio período baixo e meio período alto
    delayMicroseconds(periodHalf);        // para completar um um ciclo. Sem ativida em SDA
    digitalOpenDran(pinSCL, HIGH);        // que deve permanecer alto (open-dran). Isto faz
    delayMicroseconds(periodHalf);        // os escravos liberem SDA para trabalho.
  }
}

bool acWireClass::beginMultiTransactions(uint8_t Rw) {

  if(!sendHeader(Rw)) return false;
  inMuiltiTransaction = 1;
  return true;
}

void acWireClass::endMultiTransactions() {

  closeTransaction();
  inMuiltiTransaction = 0;
}

bool acWireClass::sendHeader(uint8_t Rw) {

  if(!inMuiltiTransaction) openTransaction();

  if(!sendByte(slaveID + Rw)){  // Prepara e envia o bit identificador do escravo.
    closeTransaction();         //<- Em caso de falha força o fim da transação.
    return false;
  }
  return true;       
}

bool acWireClass::sendByte(uint8_t b) {

  for (uint8_t i = 0; i < 8; ++i) {           // Sequência a transação de 8 bits.
    b = TWI_writeBit(b);                      // Retorna o estado do bit para o próxomo envio.
  }
  uint8_t ack = 0;
  return (TWI_readBit(ack) == ACK);           // Dá a resposta da leitura do bit ACK.
}

uint8_t acWireClass::receiveByte(uint8_t ack) {

  uint8_t b = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    b = TWI_readBit(b);
  }
  TWI_writeBit(ack);
  return b;
}
