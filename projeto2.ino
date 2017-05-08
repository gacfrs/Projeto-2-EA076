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
unsigned int sensorValue;        // value read from the pot

/**EEPROM**/
int n_elements;
#define Index 0

/**Teclado Matricial**/
const int L[3];
const int C[2];

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
void write_byte (int address, char data) {
  Wire.beginTransmission(0x50);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

char read_byte (int address) {
  byte rdata;
  Wire.beginTransmission(0x50);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(0x50, 1);

  if (Wire.available())
    rdata = Wire.read();

  return rdata;
}
/************************************************************ FIM EEPROM *********************************************************/

/************************************************************************ TECLADO MATRICIAL **************************************/
void setup_teclado() {
  pinMode(L[0], OUTPUT);
  pinMode(L[1], OUTPUT);
  pinMode(L[2], OUTPUT);
  pinMode(L[3], OUTPUT);

  pinMode(C[0], INPUT_PULLUP);
  pinMode(C[1], INPUT_PULLUP);
  pinMode(C[2], INPUT_PULLUP);

  digitalWrite(L[0], HIGH);
  digitalWrite(L[1], HIGH);
  digitalWrite(L[2], HIGH);
  digitalWrite(L[3], HIGH);
}
/*
  char caracter(int linha, int coluna){
  if(L[0] && L[1]
  }

  /*
  int Varredura(){
  int i, j, coluna;
  for(i=0; i<=3; i++){
    digitalWrite(L[i], LOW);
    for(j=0; j<=2; j++){
      coluna = digitalRead(C[j]);
      if (!coluna)
    }
  }
  }

  função Varredura();
  Para cada pino de saída x[i]:
    Configura x[i] como ativo;
    Para cada pino de entrada y[j]:
      Se y[j] está ativo:
        retorna caractere na posição (i,j);
    Configura x[i] como inativo;

  função interrupção_periodica();
  Se nao estou em deboucing:
    A=Varredura();
    Se A é um caractere válido:
      Camada_de_aplicação(A);
      muda para modo debouncing;
      contador_deboucing recebe maximo;
  Se estou em debouycing:
  //Porcesso analogo ao scheduling
  Decremento contador_deboucing;
  Se contador_deboucning é zero:
    sai do modo deboucing;


  /*********************************************************************END TECLADO MATRICIAL **************************************/

/* Funcoes internas ao void main() */
void setup() {
  /* Inicializacao */
  buffer_clean();
  flag_check_command = 0;
  Serial.begin(9600);
  Wire.begin();
  setup_teclado();
}

void loop() {
  int x, y, memoria;
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
      sensorValue = map(analogRead(analogInPin), 0, 1023, 0, 255);


      // print the results to the serial monitor:
      sprintf(out_buffer, "MEASURE = %d\n", sensorValue);
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "MEMSTATUS", 9)) {        //OK
      n_elements = read_byte(Index);
      sprintf(out_buffer, "MEMSTATUS = %d\n", n_elements);
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "RESET", 5)) {           //OK
      write_byte(Index, (int) 0);
      n_elements = read_byte(Index);
      sprintf(out_buffer, "RESET = %d\n", n_elements);
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "RECORD", 6)) {           //ok
      sensorValue = map(analogRead(analogInPin), 0, 1023, 0, 255);
      n_elements = read_byte(Index) + 1;
      write_byte(n_elements, sensorValue);
      write_byte(Index, n_elements);
      sprintf(out_buffer, "RECORDED %d\n", sensorValue);
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "GET", 3)) {                // OKKKKK RUMO AO GG
      sscanf(Buffer.data, "%*s %d", &x);

      if (x > n_elements)
        sprintf(out_buffer, "ESPACO DE MEMORIA NAO GRAVADO \n");
      else {
        memoria = read_byte(x);
        sprintf(out_buffer, "GET %d = %d\n", x, memoria);
      }
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

  0000001101000111
  01000111

***/
