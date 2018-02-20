/**
 *       
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "NilGPRS.h"



// Declare and initialize the semaphore.
static SEMAPHORE_DECL(GPRSNewMsg, 0);

// both buffers are static global vars in this file. interrupt handlers
// need to access them
static gsm_rx_ring_buffer rx_buffer;
static gsm_tx_ring_buffer tx_buffer;
// Local static variables
static uint8_t _msg;
static uint8_t _ATreply[120]; // buffer to accomodate AT reply  

/**
 * Initialize the USART module with the BAUD rate predefined
 * in HardwareSerial.h
 *
 * This may implement passing addresses of registers, like the HardwareSerial
 * class from the Arduino library, but that's not necessary at this point.
 *
 */
NilGPRS::NilGPRS(gsm_rx_ring_buffer *rx_buffer_ptr, gsm_tx_ring_buffer *tx_buffer_ptr) {
 //   _rx_buffer = rx_buffer_ptr;
 //   _tx_buffer = tx_buffer_ptr;
}

/**
 * enable receiver, transmitter ...
 *    
 */
void NilGPRS::begin(unsigned long baud) {
  uint16_t baud_setting;
  if (F_CPU != 16000000UL || baud != 57600) {
    // Double the USART Transmission Speed
    UCSR1A = 1 << U2X1;
    baud_setting = (F_CPU / 4 / baud - 1) / 2;
  } else {
    // hardcoded exception for compatibility with the bootloader shipped
    // with the Duemilanove and previous boards and the firmware on the 8U2
    // on the Uno and Mega 2560.
    UCSR1A = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }
  // assign the baud_setting
  UBRR1H = baud_setting >> 8;
  UBRR1L = baud_setting;
  // enable transmit and receive
  UCSR1B |= (1 << TXEN1) | (1 << RXEN1) | (1 << RXCIE1);

  _msg = 0;

}

/**
 * Fulush RX buffer
 */
void NilGPRS::flushRX(void) {
  rx_buffer.tail = rx_buffer.head;
  _msg = 0;
} 

/**
 * Read procedure
 */
uint8_t NilGPRS::read(void) {
  uint8_t datal  = rx_buffer.buffer[rx_buffer.tail];
  rx_buffer.tail = (rx_buffer.tail + 1) % GSM_USART_RX_BUFFER_SIZE;
  if (datal == 0x0A) { // NL
    _msg --;
  }
  return datal;
} 

/**
 * Read message
 */
uint8_t NilGPRS::read(uint8_t *msg) {
  if (_msg == 0) return 0; // no message
  uint8_t count = 0;
  uint8_t rb;
  do {
    rb = read();
    //Serial.print('|'); Serial.print((char)rb); Serial.print('-'); Serial.print(rb, HEX); // Debug 
    if (rb != 0x0A && rb != 0x0D) {
      msg[count] = rb;  // not CR and NL
      count++;          // We have at least one byte present
    } 
  } while (rb != 0x0A); // NL
  msg[count] = 0;     // Terminate 
  //Serial.println(); // Debug  
  return count;
} 

/**
 * Messages count
 */
uint8_t NilGPRS::isMsg(void) {
  return _msg;
} 


/**
 * Receive handler
 */
NIL_IRQ_HANDLER(USART1_RX_vect) {
  
  /* On AVR this forces compiler to save registers r18-r31.*/
  NIL_IRQ_PROLOGUE();
  /* Nop on AVR.*/
  nilSysLockFromISR();
  

  uint8_t datal  = UDR1;
  rx_buffer.buffer[rx_buffer.head] = datal;  // save byte
  rx_buffer.head = (rx_buffer.head + 1) % GSM_USART_RX_BUFFER_SIZE;

  if (datal == 0x0A) { // NL
    _msg ++;
  }

  if ((rx_buffer.head) == rx_buffer.tail) { // full, we are loosing data
    rx_buffer.tail = (rx_buffer.tail + 1) % GSM_USART_RX_BUFFER_SIZE;
  }
  
  /* Nop on AVR.*/
  nilSysUnlockFromISR();
  /* Epilogue performs rescheduling if required.*/
  NIL_IRQ_EPILOGUE();
}


/**
 * Data Register Empty Handler
 */
NIL_IRQ_HANDLER(USART1_UDRE_vect) {
  /* On AVR this forces compiler to save registers r18-r31.*/
  NIL_IRQ_PROLOGUE();
  /* Nop on AVR.*/
  nilSysLockFromISR();

  if (tx_buffer.head == tx_buffer.tail) {
      UCSR1B &= ~(1<<UDRIE1); // Buffer is empty, disable the interrupt
      //tx_buffer.head = tx_buffer.tail = 0;
  } else {
      UDR1 = tx_buffer.buffer[tx_buffer.tail];
      tx_buffer.tail = (tx_buffer.tail+1) % GSM_USART_TX_BUFFER_SIZE;
  }
  
  /* Nop on AVR.*/
  nilSysUnlockFromISR();
  /* Epilogue performs rescheduling if required.*/
  NIL_IRQ_EPILOGUE();
}

/**
 * Transmit Complete Interrupt Handler
 *  
 * Automatically cleared when the interrupt is executed. 
 
NIL_IRQ_HANDLER(USART0_TX_vect) {
  
  NIL_IRQ_PROLOGUE();
  nilSysLockFromISR();  
  
  nilSysUnlockFromISR();
  NIL_IRQ_EPILOGUE();  
}
*/

size_t NilGPRS::write(uint8_t data) {
  // Save data byte at end of buffer
  tx_buffer.buffer[tx_buffer.head] = data;
  // Increment the head
  tx_buffer.head = (tx_buffer.head+1) % GSM_USART_TX_BUFFER_SIZE;

  if ((tx_buffer.head) == tx_buffer.tail) { // full, we are loosing data
    tx_buffer.tail = (tx_buffer.tail + 1) % GSM_USART_TX_BUFFER_SIZE;
  }

  UCSR1B |= (1<<UDRIE1);  // Enable Data Register Empty interrupt, start transmitting
  //UCSR1B |= (1<<TXCIE1);  // Enable Transmit Complete interrupt (USART_TXC) 
  return 1;
}

void NilGPRS::printP(const unsigned char *str) {
  while (uint8_t value = pgm_read_byte(str++)) {
    write(value);
  }
}

//
uint8_t NilGPRS::WaitMsg(uint16_t _wait = AT_WAIT){ 
  uint8_t _at_wait = 0;

  while ((!isMsg()) && (_at_wait < _wait)) {
    nilThdSleepMilliseconds(AT_DELAY);
    _at_wait++;
    //Serial.print(_at_wait);Serial.print('.');
  }
  //Serial.println(F("<"));
  if (_at_wait == _wait) return 0;
  else return 1;
} 

// 
int8_t NilGPRS::sendCmd(char *what){ 
  uint8_t t_size;
  int8_t  at_tmp;

  //[SEND] AT
  //AT
  //OK
  flushRX();

  print(what); print("\n"); // \n
  Serial.print(F("*>")); Serial.println(what);

  if (!WaitMsg()) return -20;              // timeout reached
  t_size = read(_ATreply);                 // read serial
  Serial.print(F("1>")); Serial.println((char*)_ATreply);
  /*
    Serial.println(F("~~"));
    Serial.print(t_size);
    for(uint8_t i = 0; i < t_size; i++) { Serial.print((char)_ATreply[i]); }
    Serial.println();
  */
  if (t_size != strlen(what)) return -11;  // echo not match
  at_tmp = memcmp(_ATreply, what, t_size);
  if (at_tmp != 0) return -1;              // echo not match

  if (!WaitMsg()) return -21;              // timeout reached
  t_size = read(_ATreply);                 // read serial
  Serial.print(F("2>")); Serial.println((char*)_ATreply);
  /*
    Serial.println(F("~~"));
    Serial.print(t_size);
    for(uint8_t i = 0; i < t_size; i++) { Serial.print((char)_ATreply[i]); }
    Serial.println();
  */
  if (t_size != 2) return -12;             // 'OK' size
  at_tmp = memcmp(AT_OK, _ATreply, t_size);// compare
  if (at_tmp != 0) return -2;              // OK not received
  else return 1;
}

// 
int8_t NilGPRS::sendCmdWR(char *what, uint8_t *response) { 
  uint8_t t_size, r;
  int8_t  at_tmp;

  //[SEND] AT+CSQ
  //AT+CSQ
  //+CSQ: 24,0
  //
  //OK
  flushRX();

  print(what); print("\n"); // \n
  Serial.print(F("*>")); Serial.println(what);

  // get echo
  if (!WaitMsg()) return -20;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("1>")); Serial.println((char*)_ATreply);
  if (t_size != strlen(what)) return -11;   // echo not match
  at_tmp = memcmp(_ATreply, what, t_size);
  if (at_tmp != 0) return -1;               // echo not match

  // get output
  if (!WaitMsg()) return -21;               // timeout reached
  r = read(response);                       // read serial
  response[r] = 0;                          // terminate the response by null
  Serial.print(F("2>")); Serial.println((char*)response);

  // empty line
  if (!WaitMsg()) return -22;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  if (t_size != 0) return -2;               // not empty line
  
  // get OK / ERROR
  if (!WaitMsg()) return -23;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("3>")); Serial.println((char*)_ATreply);
  if (t_size != 2) return -13;              // 'OK' size
  at_tmp = memcmp(AT_OK, _ATreply, t_size); // compare
  if (at_tmp != 0) return -3;               // OK not received
  else return r;                            // size of reply
}

// 
int8_t NilGPRS::sendCmdWR(char *what, uint8_t *response, uint8_t index) { 
  uint8_t t_size, r;
  int8_t  at_tmp;
  char * pch;

  //[SEND] AT+CSQ
  //AT+CSQ
  //+CSQ: 24,0
  //
  //OK
  flushRX();

  print(what); print("\n"); // \n
  Serial.print(F("*>")); Serial.println(what);

  // get echo
  if (!WaitMsg()) return -20;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("1>")); Serial.println((char*)_ATreply);
  if (t_size != strlen(what)) return -11;   // echo not match
  at_tmp = memcmp(_ATreply, what, t_size);
  if (at_tmp != 0) return -1;               // echo not match

  // get output
  if (!WaitMsg()) return -21;               // timeout reached
  r = read(response);                       // read serial
  response[r] = 0;                          // terminate the response by null
  Serial.print(F("2>")); Serial.println((char*)response);

  // get index
  pch = strtok((char*)response," ,.-");
  r = 0;
  while (pch != NULL){
    ++r;
    if (r == index) break;
    pch = strtok(NULL, " ,.-");
  }
  strncpy ((char*)response, pch, strlen(pch)); // ** strlen
  response[strlen(pch)] = 0;  // ** strlen

  // empty line
  if (!WaitMsg()) return -22;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  if (t_size != 0) return -2;              // not empty line
  
  // get OK / ERROR
  if (!WaitMsg()) return -23;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("3>")); Serial.println((char*)_ATreply);
  if (t_size != 2) return -13;              // 'OK' size
  at_tmp = memcmp(AT_OK, _ATreply, t_size); // compare
  if (at_tmp != 0) return -3;               // OK not received
  else return r;                            // size of reply
}

int8_t NilGPRS::sendSMSBegin(char *number) {
  uint8_t t_size;
  int8_t  at_tmp;
  flushRX();

  // SMS header
  print(AT_send_sms); print('"'); print(number); println('"');// print("\n"); // \n
  if (!WaitMsg()) return -20;             // timeout reached
  t_size = read(_ATreply);                  // read serial
  //Serial.print(F("-sms b-")); Serial.println((char*)_ATreply);
  at_tmp = memcmp(AT_send_sms, _ATreply, strlen(AT_send_sms));     // compare
  if (at_tmp != 0) return -1;               // echo not match
  else return 1;
}

int8_t NilGPRS::sendSMSEnd(char *what) {
  uint8_t t_size;
  int8_t  at_tmp;
  flushRX();
  
  // End of SMS
  print(what); write(26);
  
  /*
  // empty line
  if (!WaitMsg()) return -20;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  if (t_size != 0) return -1;               // not empty line
  else Serial.println(F("-sms el-")); 
  */

  // read reply
  if (!WaitMsg()) return -21;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("-sms e1-")); Serial.println((char*)_ATreply);
  at_tmp = memcmp(what, _ATreply, strlen(what));       // compare
  if (at_tmp != 0) return -2;               // not received

  if (!WaitMsg(600)) return -22;            // timeout reached
  t_size = read(_ATreply);                  // read serial 
  Serial.print(F("-sms e2-")); Serial.println((char*)_ATreply);
  at_tmp = memcmp(AT_send_sms_reply, _ATreply, strlen(AT_send_sms_reply));  // compare strlen
  if (at_tmp != 0) return -3;               // not received

  // empty line
  if (!WaitMsg()) return -23;             // timeout reached
  t_size = read(_ATreply);                  // read serial
  if (t_size != 0) return -4;               // not empty line
  else Serial.println(F("-sms el-"));
  
  // get OK / ERROR
  if (!WaitMsg()) return -24;             // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("-sms e3-")); Serial.println((char*)_ATreply);
  if (t_size != strlen(AT_OK)) return -15;              // 'OK' size
  at_tmp = memcmp(AT_OK, _ATreply, t_size); // compare
  if (at_tmp != 0) return -5;               // OK not received
  else return 1;                            // size of reply
}


int8_t NilGPRS::sendQueryWR(char *what, uint8_t *response) { 
  uint8_t t_size;
  int8_t  at_tmp;

  //[SEND] AT+CIPGSMLOC=1,1
  //AT+CIPGSMLOC=1,1
  //+CIPGSMLOC: 0,16.878613,49.193176,2017/12/29,18:35:51
  flushRX();

  print(what); print("\n"); // \n
  // get echo
  if (!WaitMsg()) return -20;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("1>")); Serial.println((char*)_ATreply);
  if (t_size != strlen(what)) return -11;   // echo not match
  at_tmp = memcmp(_ATreply, what, t_size);
  if (at_tmp != 0) return -1;               // echo not match

  // get output
  if (!WaitMsg()) return -21;               // timeout reached
  at_tmp = read(response);                  // read serial
  response[at_tmp] = 0;                     // terminate the response by null
  Serial.print(F("2>")); Serial.println((char*)response);
 
  return at_tmp;                            // size of reply
}

// 
int8_t NilGPRS::sendOKQueryWR(char *what, uint8_t *response, uint8_t index) { 
  uint8_t t_size, r;
  int8_t  at_tmp;
  char * pch;

  //[SEND] AT+HTTPACTION=1
  //AT+HTTPACTION=1
  //OK
  //
  //+HTTPACTION: 1,200,0

  flushRX();

  print(what); print("\n"); // \n 
  // get echo
  if (!WaitMsg()) return -21;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("1>")); Serial.println((char*)_ATreply);
  if (t_size != strlen(what)) return -11;   // echo not match
  at_tmp = memcmp(_ATreply, what, t_size);
  if (at_tmp != 0) return -1;               // echo not match

  // get OK / ERROR
  if (!WaitMsg()) return -22;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("2>")); Serial.println((char*)_ATreply);
  if (t_size != strlen(AT_OK)) return -12;  // 'OK' size
  at_tmp = memcmp(AT_OK, _ATreply, t_size); // compare
  if (at_tmp != 0) return -2;               // OK not received
  
  // empty line
  if (!WaitMsg()) return -23;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  if (t_size != 0) return -3;               // not empty line  

  // get output
  if (!WaitMsg()) return -24;               // timeout reached
  r = read(response);                       // read serial
  response[r] = 0;                          // terminate the response by null

  // get index
  pch = strtok((char*)response," ,.-");
  r = 0;
  while (pch != NULL){
    ++r;
    if (r == index) break;
    pch = strtok(NULL, " ,.-");
  }
  strncpy ((char*)response, pch, strlen(pch)); // ** strlen
  response[strlen(pch)] = 0;  // ** strlen

  return 1;
}

/*
// 
int8_t NilGPRS::sendData(char *what, uint8_t length = 0, uint16_t timeout = DEFAULT_TIMEOUT) { 
  uint8_t t_size;
  int8_t  at_tmp;

  //[SEND] AT+HTTPDATA=40,10000
  //AT+HTTPDATA=48,10000
  //DOWNLOAD
  // ... send data ...
  // 
  //OK

  flushRX();

  print(AT_HTTPDATA); print(length); print(','); print(timeout); print("\n"); // \n
  // get echo
  if (!WaitMsg()) return -21;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("1>")); Serial.println((char*)_ATreply);
  if (t_size < 12) return -11;              // echo not match, compare only AT+HTTPDATA=
  at_tmp = memcmp(what, _ATreply, 12);      // compare only AT+HTTPDATA=
  if (at_tmp != 0) return -1;               // echo not match

  // get DOWNLOAD
  if (!WaitMsg()) return -22;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("2>")); Serial.println((char*)_ATreply);
  if (t_size < 8) return -12;               // echo not match, compare only DOWNLOAD
  at_tmp = memcmp(AT_DOWNLOAD, _ATreply, 8);// compare only DOWNLOAD
  if (at_tmp != 0) return -2;               // DOWNLOAD not match

  // Send data
  print(what);

  // empty line
  if (!WaitMsg(timeout)) return -23;        // timeout reached
  t_size = read(_ATreply);                  // read serial
  if (t_size != 0) return -3;               // not empty line  

  // get OK 
  if (!WaitMsg()) return -24;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  if (t_size != 2) return -14;              // 'OK' size
  at_tmp = memcmp(AT_OK, _ATreply, t_size); // compare
  if (at_tmp != 0) return -4;               // OK not received
  else return 1;  
}
*/

// 
int8_t NilGPRS::sendData(char *what) { 
  uint8_t t_size;
  int8_t  at_tmp;
  uint8_t length = strlen(what);

  //[SEND] AT+HTTPDATA=40,10000
  //AT+HTTPDATA=48,10000
  //DOWNLOAD
  // ... send data ...
  // 
  //OK
  flushRX();

  print(AT_HTTPDATA); print(length); print(','); print(DEFAULT_TIMEOUT); print("\n"); // \n
  // get echo
  if (!WaitMsg()) return -21;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("1>")); Serial.println((char*)_ATreply);
  if (t_size < 12) return -11;              // echo not match, compare only AT+HTTPDATA=
  at_tmp = memcmp(AT_HTTPDATA, _ATreply, 12);  // compare only AT+HTTPDATA=
  if (at_tmp != 0) return -1;               // echo not match

  // get DOWNLOAD
  if (!WaitMsg()) return -22;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("2>")); Serial.println((char*)_ATreply);
  if (t_size != strlen(AT_DOWNLOAD)) return -12;               // echo not match, compare only DOWNLOAD
  at_tmp = memcmp(AT_DOWNLOAD, _ATreply, strlen(AT_DOWNLOAD)); // compare only DOWNLOAD
  if (at_tmp != 0) return -2;               // DOWNLOAD not match

  // Send data
  print(what);

  // empty line
  if (!WaitMsg()) return -23;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  if (t_size != 0) return -3;               // not empty line  

  // get OK 
  if (!WaitMsg()) return -24;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("3>")); Serial.println((char*)_ATreply);
  if (t_size != 2) return -14;              // 'OK' size
  at_tmp = memcmp(AT_OK, _ATreply, t_size); // compare
  if (at_tmp != 0) return -4;               // OK not received
  else return 1;  
}


// 
int8_t NilGPRS::getData(uint8_t *response) { 
  uint8_t t_size, data_size;
  int8_t  at_tmp;
  char * pch;  

  //[SEND] AT+HTTPREAD
  //AT+HTTPREAD
  //+HTTPREAD: 100
  // ... data ...
  // 
  //OK
  flushRX();

  print(AT_GET_DATA); print("\n"); // \n
  // get echo
  if (!WaitMsg()) return -21;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("1>")); Serial.println((char*)_ATreply);
  if (t_size != strlen(AT_GET_DATA)) return -11;                 // echo not match, compare only AT+HTTPDATA=
  at_tmp = memcmp(AT_GET_DATA, _ATreply, strlen(AT_GET_DATA));   // compare only AT+HTTPDATA=
  if (at_tmp != 0) return -1;               // echo not match

  // get +HTTPREAD: #
  if (!WaitMsg()) return -22;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("2>")); Serial.println((char*)_ATreply);
  if (t_size < strlen(AT_DATA_SIZE)) return -12;                 // echo not match, compare only +HTTPREAD:
  at_tmp = memcmp(AT_DATA_SIZE, _ATreply, strlen(AT_DATA_SIZE) );// compare only +HTTPREAD:
  if (at_tmp != 0) return -2;               // DOWNLOAD not match

  // get data size
  pch = strtok((char*)_ATreply," ");
  if (pch != NULL) pch = strtok(NULL, " ");
  data_size = strtol(pch, NULL, 10);
  Serial.print(F("~>")); Serial.println(data_size);

  // get output
  if (!WaitMsg()) return -23;               // timeout reached
  at_tmp = read(response);                  // read serial
  response[at_tmp] = 0;                     // terminate the response by null
  Serial.print(F("3>")); Serial.println((char*)response);

  /*
  // empty line
  if (!WaitMsg()) return -24;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  if (t_size != 0) return -4;               // not empty line  
  */
  // get OK 
  if (!WaitMsg()) return -25;               // timeout reached
  t_size = read(_ATreply);                  // read serial
  Serial.print(F("4>")); Serial.println((char*)_ATreply);
  if (t_size != 2) return -15;              // 'OK' size
  at_tmp = memcmp(AT_OK, _ATreply, t_size); // compare
  if (at_tmp != 0) return -5;               // OK not received
  else return data_size;  
}


/** Sleep while waiting for new message.
 * @note This function should not be used in the idle thread.
 * @return true if success or false if overrun or error.
 */
bool NilGPRS::nilWaitGPRSNewMsg() {
  // Idle thread can't sleep.
  if (nilIsIdleThread()) return false;
  if (nilSemWaitTimeout(&GPRSNewMsg, TIME_IMMEDIATE) != NIL_MSG_TMO) return false;
  nilSemWait(&GPRSNewMsg);
  return true;
}

NilGPRS GPRS(&rx_buffer, &tx_buffer);