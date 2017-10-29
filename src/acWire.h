/*
 * Copyright (c) 2017 by Acácio Neimar de Oliveira <neimar2009@gmail.com>
 * acWire.h
 */

class acWireClass {
public:
  // mode is 100kHz or 400kHz; false or true.
  acWireClass(uint8_t pinSDA, uint8_t pinSCL, boolean mode = true);
  ~acWireClass() {};
  void begin(uint8_t slave);
  uint8_t readBegin(uint8_t* data, uint8_t len);
  uint8_t readBegin();
  uint8_t read(uint8_t* data, uint8_t len);
  uint8_t read();
  uint8_t readEnd(uint8_t* data, uint8_t len);
  uint8_t readEnd();
  uint8_t writeBegin(uint8_t* data, uint8_t len);
  uint8_t writeBegin(uint8_t data);
  uint8_t write(uint8_t* data, uint8_t len);
  uint8_t write(uint8_t data);
  uint8_t writeEnd(uint8_t* data, uint8_t len);
  uint8_t writeEnd(uint8_t data);
private:
  uint8_t slaveID = 0;
  uint8_t pinSDA;
  uint8_t pinSCL;
  uint8_t periodHalf;
  uint8_t inMuiltiTransaction = 0;
  uint8_t modeACK = 1; //<- NOACK
  uint8_t flagI2C = 0; //<- writeI2C
  uint8_t TWI_readBit(uint8_t data);
  uint8_t TWI_writeBit(uint8_t data);
  void openTransaction();
  void closeTransaction();
  void reopenTransaction();
  void releaseSDA();
  bool beginMultiTransactions(uint8_t Rw);
  void endMultiTransactions();
  bool sendHeader(uint8_t Rw);
  bool sendByte(uint8_t b);
  uint8_t receiveByte(uint8_t ack);
};

