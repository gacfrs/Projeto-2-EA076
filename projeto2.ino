/* Protocolo de aplicacao - Implementacao usando rotina de interrupcao e
  Baseado no programa do Prof.Tiago F. Tavares

  Dimitri Reis
  Guilherme Frauches
*/

#include <stdio.h>
#include <EEPROM.h>
#include <Wire.h>

/****************************************************** Declaração de variáveis *********************************************/
/**Conversor ADC**/
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
int sensorValue = 0;        // value read from the pot

/**EEPROM**/
int n_elements;
int address;
#define Index 0

/******************************************************** CODIGO PRONTO DO TAVARES***************************************/
/* Rotina auxiliar para comparacao de strings */
int str_cmp(char *s1, char *s2, int len) {
  /* Compare two strings up to length len. Return 1 if they are
      equal, and 0 otherwise.
  */
  int i;
  for (i = 0; i < len; i++) {
    if (s1[i] != s2[i]) return 0;
    if (s1[i] == '\0') return 1;
  }
  return 1;
}

/* Processo de bufferizacao..... */

/* Buffer de dados recebidos */
#define MAX_BUFFER_SIZE 15
typedef struct {
  char data[MAX_BUFFER_SIZE];
  unsigned int tam_buffer;
} serial_buffer;

/* Teremos somente um buffer... */
volatile serial_buffer Buffer;

/* Todas as funcoes a seguir... */

/* Limpa buffer */
void buffer_clean() {
  Buffer.tam_buffer = 0;
}

/* Adiciona caractere ao buffer */
int buffer_add(char c_in) {
  if (Buffer.tam_buffer < MAX_BUFFER_SIZE) {
    Buffer.data[Buffer.tam_buffer++] = c_in;
    return 1;
  }
  return 0;
}

/* Flags globais para controle de processos da interrupcao */
volatile int flag_check_command = 0;

/* Rotinas de interrupcao */

/* Ao receber evento da UART */
void serialEvent() {
  char c;
  while (Serial.available()) {
    c = Serial.read();
    if (c == '\n') {
      buffer_add('\0'); /* Se recebeu um fim de linha, coloca um terminador de string no buffer */
      flag_check_command = 1;
    } else {
      buffer_add(c);
    }
  }
}
/******************************************************** FIM CODIGO PRONTO DO TAVARES***************************************/

/************************************************************ EEPROM *********************************************************/
void write_byte (int ee_address, char data) {
  Wire.beginTransmission(0x50);
  Wire.write(ee_address);
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

char read_byte (int ee_address) {
  byte rdata;
  Wire.beginTransmission(0x50);
  Wire.write(ee_address);
  Wire.endTransmission();
  Wire.requestFrom(0x50, 1);

  if (Wire.available())
    rdata = Wire.read();

  return rdata;
}
/************************************************************ FIM EEPROM *********************************************************/

/* Funcoes internas ao void main() */
void setup() {
  /* Inicializacao */
  buffer_clean();
  flag_check_command = 0;
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  int x, y, n;
  char out_buffer[20];
  int flag_write = 0;

  /* A flag_check_command permite separar.... */
  if (flag_check_command == 1) {                             //OK
    if (str_cmp(Buffer.data, "PING", 4)) {
      sprintf(out_buffer, "PONG\n");
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "ID", 2)) {               //OK
      sprintf(out_buffer, "DATALOGGER DO TIAGO\n");
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "SUM", 3)) {              //OK
      sscanf(Buffer.data, "%*s %d %d", &x, &y);
      sprintf(out_buffer, "SUM = %d\n", x + y);
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "MEASURE", 7)) {          //OK
      // read the analog in value:
      sensorValue = analogRead(analogInPin);

      // print the results to the serial monitor:
      sprintf(out_buffer, "MEASURE = %d\n", sensorValue);
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "MEMSTATUS", 9)) {        //FALTA TESTAR
      n_elements = read_byte(Index);
      sprintf(out_buffer, "MEMSTATUS = %d\n", n_elements);
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "RESET", 5)) {           //FALTA TESTAR
      write_byte(Index, (int) 0);
      /*
          n_elements = read_byte(Index);
          sprintf(out_buffer, "RESET = %d\n", n_elements);
      */
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "RECORD", 6)) {           //FALTA TESTAR
      /* lê o sensor
         lê a posiçao do indice
         grava numa posição
         tentar entender direito o  N*x[Index]+1
         incrementa o x[Index] */

      sensorValue = analogRead(analogInPin);
      n_elements = read_byte(Index)+1;
      write_byte(n_elements, sensorValue);
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "GET N", 5)) {                // FALTA TESTAR
      sscanf(Buffer.data, "%*s %d", &n);
      sprintf(out_buffer, "GET N = %c\n", read_byte(n));
      flag_write = 1;
    }

    else {                                                    //OK
      sprintf(out_buffer, "ERROR\n");
      flag_write = 1;
    }
    flag_check_command = 0;
  }

  /* Posso construir uma dessas estruturas... */
  if (flag_write == 1) {
    Serial.write(out_buffer);
    buffer_clean();
    flag_write = 0;
  }

}

/***
  rEFERENCIAS^: https://www.arduino.cc/en/Tutorial/EEPROMWrite
  http://www.hobbytronics.co.uk/arduino-external-eeprom
  http://pdf.datasheetcatalog.com/datasheet/atmel/doc0180.pdf

  Pino EEEPROM - Arduino
  1(A0)- gnd
  2(A1)- gnd
  3(A2)- gnd
  4(Vss)- Gnd
  5(SDA)- A4 - trafegam os dados
  6(SCL)- A5 - carrega o sinal de clock
  7(WP)- Gnd
  8(Vcc)- Vcc

  colocar indice no byte 0 - quantos elementos estao gravados na memoria
  analisar o estado inicial e entao usar a proxima posição
  para formatar - escrever 0 na posição de índice

  Master envia o endereço do slave
  envia o que quiser
  manda parar

  /* Escreve dado numa posição da memória **
  void write_byte(int posição, char dado);

  /* Le dado de uma posição da memória **
  char read_byte(int posição);



***/
