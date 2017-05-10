

/* Protocolo de aplicacao - Implementacao usando rotina de interrupcao e
  Baseado no programa do Prof.Tiago F. Tavares
  Dimitri Reis
  Guilherme Frauches
*/

#include <stdio.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Keypad.h>

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
#define LED 13;
int k = 0, med_auto = 0;

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
//Definicao da quantidade de linhas e colunas
const byte LINHAS = 4;
const byte COLUNAS = 3;

//Matriz de caracteres
char matriz_teclas[LINHAS][COLUNAS] =
{
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

//Definicao dos pinos das linhas
byte PinosLinhas[LINHAS] = {4, 5, 6, 7};
//Definicao dos pinos das colunas
byte PinosColunas[COLUNAS] = {8, 9, 10};

//Inicializa o teclado
Keypad meuteclado = Keypad( makeKeymap(matriz_teclas), PinosLinhas, PinosColunas, LINHAS, COLUNAS);
/**************************************************************** FIM TECLADO MATRICIAL ***************************************/


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
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  buffer_clean();
  flag_check_command = 0;
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  int x, y, memoria, pisca = 0, timer;
  char out_buffer[20], teclado[2], tecla_pressionada;
  int flag_write = 0;

  /*Varredura*/
  tecla_pressionada = meuteclado.getKey();
  if (tecla_pressionada) {
    teclado[k] = tecla_pressionada;
    k++;
    if (k > 2) {
      k = 0;
      flag_check_command == 1;
    }
  }

  if (pisca) {
    if (digitalRead(LED) == HIGH)
      digitalWrite(LED, LOW);
    else
      digitalWrite(LED, HIGH);
    for (timer = 0, timer <= 1000, timer++)
    }

  if (med_auto) {
    // read the analog in value:
    sensorValue = analogRead(analogInPin) / 4;

    // print the results to the serial monitor:
    sprintf(out_buffer, "MEASURE = %d\n", sensorValue);
    flag_write = 1;
  }

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
      sensorValue = analogRead(analogInPin) / 4;

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
      sensorValue = analogRead(analogInPin) / 4;
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
    /**************************** FALTA TESTAR ************************/
    else if (str_cmp(teclado, "#1*", 3)) { //Pisca um led dizendo que o sitema esta responsivo
      if (pisca)
        pisca = 0;
      else
        pisca = 1;
      flag_write = 1;
    }

    else if (str_cmp(teclado, "#2*", 3)) { //Realiza uma medição e grava o valor na memória
      sensorValue = analogRead(analogInPin) / 4;
      n_elements = read_byte(Index) + 1;
      write_byte(n_elements, sensorValue);
      write_byte(Index, n_elements);
      sprintf(out_buffer, "RECORDED %d\n", sensorValue); //apagar depois, é só para testar
      flag_write = 1;
    }

    else if (str_cmp(teclado, "#3*", 3)) { //Ativa o modo de medição automatica
      med_auto = 1;
      flag_write = 1;
    }

    else if (str_cmp(teclado, "#4*", 3)) { //Encerra o modo de medição automática
      med_auto = 0;
      flag_write = 1;
    }
    /********************************************FIM FALTA TESTAR ****************/
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
*/
