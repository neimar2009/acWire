
# Library for I2C communication, for ATtiny85 and equivalents.

Description
------------

* For ATtiny85 and equivalents.
* Individual instance for each slave device.

Help
------------

### acWireClass
```
acWireClass(uint8_t pinSDA, uint8_t pinSCL, boolean mode = true);
```
Instância a classe.    
`pinSDA` | número do pino para transferência de dados.    
`pinSCL` | número do pino do sinal de Clock.    
`mode` | velocidade de operação. 'true' para 400kHz e 'false' para 100kHz

### begin
```
begin(uint8_t slave);
```
Inicia a configuração necessária para funcionemento.    
`slave` | endereço de identificação do escravo.

### readBegin
```
readBegin(uint8_t* data, uint8_t len);    
```
Dá inicia a uma sequência de comunicação. Lê uma sequência de dados. Deve terminar com o seu fechamento.
`data` | ponteiro para variável que receberá os dados a serem lidos.
`len` | tamanho do espaço alocado para os dados.    
retorno | quantidade de dados lidos. 

### readBegin
```
readBegin();
```
Dá inicia a uma sequência de comunicação. Lê uma dado. Deve terminar com o seu fechamento.
retorno | um dado lido.

### read
```
read(uint8_t* data, uint8_t len);
```
Lê um sequência de dados.    
`data` | ponteiro para variável que receberá os dados a serem lidos.
`len` | tamanho do espaço alocado para 'data', é quantidade de dados a ser lido.
retorno | quantidade de dados lidos. 

### read
```
read();
```
Lê um dado.
retorno | o dado lido.    

### readEnd
```
readEnd(uint8_t* data, uint8_t len);

```
Lê uma sequência de dados e finaliza uma sequência de comunicações já aberta. Deve ser chamado para fechar uma sequência de comunicações aberta.    
`data` | ponteiro para variável que receberá os dados a serem lidos.
`len` | tamanho do espaço alocado para os dados.
retorno | quantidade de dados lidos. 

### readEnd
```
readEnd();

```
Lê uma dado e finaliza uma sequência de comunicações já aberta. Deve ser chamado para fechar uma sequência de comunicações aberta.    
retorna | um dado lido.

================

### writeBegin
```
writeBegin(uint8_t* data, uint8_t len);    
```
Dá inicia a uma sequência de comunicação. Escreve uma sequência de dados. Deve terminar com o seu fechamento.
`data` | ponteiro para variável que contém os dados a serem transmitidos.
`len` | tamanho do espaço alocado para os dados.    
retorno | quantidade de dados transmitidos. 

### writeBegin
```
writeBegin();
```
Dá inicia a uma sequência de comunicação. Escreve uma dado. Deve terminar com o seu fechamento.
retorno | 1 para dado escrito e 0 para erro.

### write
```
write(uint8_t* data, uint8_t len);
```
Escreve uma sequência de dados.    
`data` | ponteiro para variável que contém os dados a serem transmitido.
`len` | tamanho do espaço alocado para os dados.
retorno | quantidade de dados transmitidos. 

### write
```
write();
```
Escreve um dado.
retorno | 1 para dado escrito e 0 para erro.    

### writeEnd
```
writeEnd(uint8_t* data, uint8_t len);

```
Escreve uma sequência de dados e finaliza uma sequência de comunicações já aberta. Deve ser chamado para fechar uma sequência de comunicações aberta.    
`data` | ponteiro para variável que contém os dados a serem transmitidos.
`len` | tamanho do espaço alocado para os dados.
retorno | quantidade de dados lidos. 

### writeEnd
```
writeEnd();

```
Escreve uma dado e finaliza uma sequência de comunicações já aberta. Deve ser chamado para fechar uma sequência de comunicações aberta.    
retorno | 1 para dado escrito e 0 para erro.    

Example
------------

```
  ...
acWireClass DS3231
uint8_t data[7];
uint8_t len = 7; 
  ...
```
 
 
```
void begin() {

  DS3231.begin(0x68);
  ...
}
```
```
void loop() { 

  DS3231.write(0);          // Registro inicial para leitura.
  DS3231.read( data, len);  // 

  // Transformar dados contidos em data de bcd para decimal.
  ...

  Serial.print(data[6]+2000); // Year
  Serial.print("-");
  Serial.print(data[5]);      // month
  Serial.print("-");
  Serial.print(data[4]);      // day
  Serial.print(" ");
  Serial.print(data[3]);      // day of week
  Serial.print(" ");
  Serial.print(data[2]);      // hour
  Serial.print(":");
  Serial.print(data[1]);      // minute
  Serial.print(":");
  Serial.print(data[0]);      // second
  Serial.println();

  while(true);

}
```


Help me
------------
  Due to the limited time available for development, I present this project in the
  way that you see it. Personally, I'm sorry, but so far I've been able to develop.
  
  My English is weak, to the extent possible, depending on available time, I will
  translate.
  
  Comments and suggestions will help in improving the project. Welcome.


Thanks
------------
  **I thank God.**
  
------------
