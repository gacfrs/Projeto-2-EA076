/****************************************************************************
 * 
 *   EA076 C - Projeto 2 - projeto2.io
 *  Dimitri Reis        RA 145869
 *  Guilherme Frauches  RA 155591
 *
 *  Protocolo de aplicacao - Implementacao usando rotina de interrupcao e
 *  baseado no programa do Prof.Tiago F. Tavares. 
 *  
 ****************************************************************************/

#include <stdio.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Keypad.h>
#include "TimerOne.h"

/* Variáveis de Controle */
const int analogInPin = A0;   // Pino referente ao LDR
unsigned int sensorValue;     // Variavel que representa o valor lido do LDR
unsigned int n_elements;      // Numero elementos atuais na memoria
int k = 0, med_auto = 0;
#define Index 0               // Posicao 0 da EEPROM que armazena o numero de elementos gravados
#define ledPin 13

/** As funcoes abaixo sao relativas a utilizacao da UART **/
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

/* Processo de bufferizacao. Caracteres recebidos sao armazenados em um buffer. Quando um caractere
    de fim de linha ('\n') e recebido, todos os caracteres do buffer sao processados simultaneamente.
*/

/* Buffer de dados recebidos */
#define MAX_BUFFER_SIZE 15
typedef struct {
  char data[MAX_BUFFER_SIZE];
  unsigned int tam_buffer;
} serial_buffer;

/* Teremos somente um buffer em nosso programa, O modificador volatile
    informa ao compilador que o conteudo de Buffer pode ser modificado a qualquer momento. Isso
    restringe algumas otimizacoes que o compilador possa fazer, evitando inconsistencias em
    algumas situacoes (por exemplo, evitando que ele possa ser modificado em uma rotina de interrupcao
    enquanto esta sendo lido no programa principal).
*/
volatile serial_buffer Buffer;

/* Todas as funcoes a seguir assumem que existe somente um buffer no programa e que ele foi
    declarado como Buffer. Esse padrao de design - assumir que so existe uma instancia de uma
    determinada estrutura - se chama Singleton (ou: uma adaptacao dele para a programacao
    nao-orientada-a-objetos). Ele evita que tenhamos que passar o endereco do
    buffer como parametro em todas as operacoes (isso pode economizar algumas instrucoes PUSH/POP
    nas chamadas de funcao, mas esse nao eh o nosso motivo principal para utiliza-lo), alem de
    garantir um ponto de acesso global a todas as informacoes contidas nele.
*/

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
int pisca = 0;

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
/** Fim das funcoes relativas a UART **/

/**  As funcoes abaixo sao relativas a memoria EEPROM **/
/* Processo de escrita de um dado em um certo endereço da memória EEPROM */
void write_byte (unsigned int address, char data) {
  Wire.beginTransmission(0x50);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

/* Processo de leitura de um certo endereço da memória EEPROM */
byte read_byte (unsigned int address) {
  byte rdata = 0xF;
  Wire.beginTransmission(0x50);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(0x50, 1);

  if (Wire.available())
    rdata = Wire.read();

  return rdata;
}
/** Fim das funcoes relativas a memoria EEPROM **/

/**  As funcoes abaixo sao relativas ao Teclado Matricial **/
/* Definicao da quantidade de linhas e colunas */
const byte LINHAS = 4;
const byte COLUNAS = 3;

/* Matriz de caracteres */
char matriz_teclas[LINHAS][COLUNAS] =
{
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

/* Definicao dos pinos das linhas e colunas */
byte PinosLinhas[LINHAS] = {4, 5, 6, 7};
byte PinosColunas[COLUNAS] = {8, 9, 10};

/* Inicializa o teclado */
Keypad meuteclado = Keypad( makeKeymap(matriz_teclas), PinosLinhas, PinosColunas, LINHAS, COLUNAS);

/** Fim das funcoes relativas ao Teclado Matricial **/

/* Funcoes internas ao void main() */
void setup() {
  pinMode(ledPin, OUTPUT);            // Inicializacao do led que sera
  digitalWrite(ledPin, LOW);          // comandado pelo teclado matricial

  buffer_clean();
  flag_check_command = 0;

  Timer1.initialize();                // Inicializacao da interrupcao por tempo
  Timer1.attachInterrupt(Int_Timer);

  Serial.begin(9600);
  Wire.begin();
}

/* Realiza o tratamento da interrupção */
void Int_Timer() {
  sensorValue = analogRead(analogInPin) / 4;          // Faz a leitura do LDR e converte em 8 bits
  if (pisca)                                          // Pisca o LED a cada 1 seg, se necessario
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
  else
    digitalWrite (ledPin, LOW);
}

void loop() {
  int x, y, flag_write = 0;
  unsigned int memoria;
  char out_buffer[20], teclado[2];

  char tecla_pressionada = meuteclado.getKey();   // Processo de varredura do teclado matricial

  if (tecla_pressionada)
  {
    teclado[k] = tecla_pressionada;
    k++;
    if (k > 2) {                                  // Os comandos do teclado possuem exatamente 3 digitos
      if (str_cmp(teclado, "#1*", 3)) {           // Comando "#1*" Liga/Desliga o processo de piscagem do LED
        if (pisca)
          pisca = 0;
        else
          pisca = 1;
        sprintf(out_buffer, "");
        flag_write = 1;
      }

      else if (str_cmp(teclado, "#2*", 3)) {      // Comando "#2*" realiza uma medicao e grava o valor na memoria
        n_elements = read_byte(Index) + 1;
        write_byte(n_elements, sensorValue);
        write_byte(Index, n_elements);
        sprintf(out_buffer, "RECORDED %d\n", sensorValue);
        flag_write = 1;
      }

      else if (str_cmp(teclado, "#3*", 3)) {      // Comando "#3*" ativa o modo de medicao automatica
        med_auto = 1;
        flag_write = 1;
      }

      else if (str_cmp(teclado, "#4*", 3)) {      // Comando "#4*" desativa o modo de medicao automatica
        med_auto = 0;
        flag_write = 1;
        sprintf(out_buffer, "");
      }
      k = 0;
    }
  }

  /* Realiza o comando "#3*" */
  if (med_auto) {
    sprintf(out_buffer, "MEASURE = %d\n", sensorValue);
    flag_write = 1;
  }

  /* A flag_check_command permite separar a recepcao de caracteres
      (vinculada a interrupca) da interpretacao de caracteres. Dessa forma,
      mantemos a rotina de interrupcao mais enxuta, enquanto o processo de
      interpretacao de comandos - mais lento - nao impede a recepcao de
      outros caracteres. Como o processo nao 'prende' a maquina, ele e chamado
      de nao-preemptivo.
  */
  if (flag_check_command == 1) {

    if (str_cmp(Buffer.data, "PING", 4)) {                  // Retorna "PONG" ao enviar "PING" pela UART
      sprintf(out_buffer, "PONG\n");
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "ID", 2)) {               // Retorna "DATALOGGER DO TIAGO" ao enviar "ID" pela UART
      sprintf(out_buffer, "DATALOGGER DO TIAGO\n");
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "SUM", 3)) {              // Realiza a soma de dois números enviados pela UART
      sscanf(Buffer.data, "%*s %d %d", &x, &y);
      sprintf(out_buffer, "SUM = %d\n", x + y);
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "MEASURE", 7)) {          // Realiza uma medicao no LDR
      sprintf(out_buffer, "MEASURE = %d\n", sensorValue);
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "MEMSTATUS", 9)) {        // Retorna o numero de elementos gravados na memoria EEPROM
      n_elements = read_byte(Index);
      sprintf(out_buffer, "MEMSTATUS = %d\n", n_elements);
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "RESET", 5)) {            // Zera o numero de elementos gravados na memoria
      write_byte(Index, (int) 0);
      n_elements = read_byte(Index);
      sprintf(out_buffer, "RESETED\n");
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "RECORD", 6)) {           // Realiza uma medicao no LDR e grava o dado na memoria EEPROM
      sensorValue = analogRead(analogInPin) / 4;
      n_elements = read_byte(Index) + 1;
      write_byte(n_elements, sensorValue);
      write_byte(Index, n_elements);
      sprintf(out_buffer, "RECORDED %d\n", sensorValue);
      flag_write = 1;
    }

    else if (str_cmp(Buffer.data, "GET", 3)) {              // Retorna o n-esimo dado gravado na memoria EEPROM
      sscanf(Buffer.data, "%*s %d", &x);

      if (x > n_elements)
        sprintf(out_buffer, "ESPACO DE MEMORIA INVALIDO \n");
      else {
        memoria = read_byte(x);
        sprintf(out_buffer, "GET %d = %d\n", x, memoria);
      }
      flag_write = 1;
    }

    else {                                                   // Retorna "ERROR" se o comando inserido for invalido
      sprintf(out_buffer, "ERROR\n");
      flag_write = 1;
    }
    flag_check_command = 0;
  }

  /* Posso construir uma dessas estruturas if(flag) para cada funcionalidade
      do sistema. Nesta a seguir, flag_write e habilitada sempre que alguma outra
      funcionalidade criou uma requisicao por escrever o conteudo do buffer na
      saida UART.
  */
  if (flag_write == 1) {
    Serial.write(out_buffer);
    buffer_clean();
    flag_write = 0;
  }
}

/* 
 * Referencias para a implementacao do projeto:
 * Kernel de tempo real e comunicação UART - https://github.com/embarcados-unicamp/rt-kernel/blob/master/src/rtkernel.ino
 * EEPROM - https://www.arduino.cc/en/Tutorial/EEPROMWrite
 * Teclado Matricial - http://www.arduinoecia.com.br/2015/05/teclado-matricial-membrana-4x3-arduino.html
 */
