/**
 * RS485 protocol library
 * Adam Baron - vysocan(c) 2011
 * Adam Baron - vysocan(c) 2019
 * 
 * For Atmega168, 328
 *
 * 1.0 
 * Adjustments for ACK/NACK and STM32 
 * 
 * 0.9 Beta  
 * Tested on two breadboards, sending read data or commands looks fine. 
 * Sending junk random data looks fine. One of many is picked up as general
 * command. I guess when the random 2 bytes matches the xor. No random data is
 * pushed through 2 byte crc.         
 *  
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 
 * Packet structure: 
 * *************~~~~~~~~~~~~~~~~~~
 * * A * P * X *  D0 ~ D63 C0 ~ C1 
 * *************~~~~~~~~~~~~~~~~~~
 * 
 * * Required fields
 * ~ Data and signature fields
 * A Address - 4 bits form and 4 bits to address.
 *   0  is master
 *   15 is broadcast to all
 *   14 addresses left for other devices
 * 
 * P Packet definition - 2 bits type and 6 bits length.
 *   Flag bit configuration:
 *   FLAG_ACK 1
 *   FLAG_NACK 0
 *   FLAG_CMD 1
 *   FLAG_DTA 0
 *    
 *   Types: CMD - Command - user defined general commands value 0 - 63
 *          DTA - Data - user data length D0 - D63 and CRC S0, S1
 *          ACK, NACK - flags for data, using signature of data packet
 *          
 * X XOR of A and P, for basic transfer safety.
 * 
 * D0 - D63 user data.     
 *                      
 * C0 ~ C1  CRC for user data or signature for ACK, NAK.  
 * 
 *
 * Application notes:
 * Baud rate should be bigger then 9600bps for a 16 MHz crystal, due
 * to stochastic wait for empty line to work properly.
 *       
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>

#include "LibRS485v2.h"

// macros
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(port,pin) port &= ~(1<<pin)
#define set_output(port,pin) port |= (1<<pin)

#ifndef BAUD_TOL
#   define BAUD_TOL 2
#endif

// both buffers are static global vars in this file. interrupt handlers
// need to access them
volatile rx_ring_buffer rx_buffer;
volatile tx_ring_buffer tx_buffer;
// Local static variables
volatile uint8_t _msg_length, _data_length, _address;
volatile rs485transceiver_t _state = TRC_UNINIT;

/**
 * Initialize 
 */
LibRS485v2::LibRS485v2(){
}

/**
 * Initalize RS485
 * 
 * Set baudrate, 9N1, MPCM
 * enable receiver, transmitter ...
 *    
 */
void LibRS485v2::begin(unsigned long baud, uint8_t address) {
    set_output(PORTD, 3);   // Assign WriteEnable as output
    output_low(PORTD, 3);   // Disable WriteEnable on RS485 chip   
    _address = address;     // Assign address
    // Tick usec, 11 bits for one byte, _delay_loop_2 executes four CPU cycles
    // per iteration. The maximal possible delay is 262.14 ms / F_CPU in MHz (16 or 8).
    _tick = (F_CPU * 11) / (4 * baud);  
    // Microsecond delay for one character, 11 bits 
    //tick = 11000000 / baud;
    
    // taken from <util/setbaud.h>
    uint8_t use2x = 0;
    uint16_t ubbr =  (F_CPU + 8UL * baud) / (16UL * baud) - 1UL;
    if ( (100 * (F_CPU)) > (16 * (ubbr + 1) * (100 * ubbr + ubbr * BAUD_TOL)) ) {
        use2x = 1;
        ubbr = (F_CPU + 4UL * baud) / (8UL * baud) - 1UL;
    }

    UBRR0L = ubbr & 0xff;
    UBRR0H = ubbr >> 8;
    if (use2x) {
        UCSR0A |= (1 << U2X0);
    } else {
        UCSR0A &= ~(1 << U2X0);
    }

    // Flush buffers
    tx_buffer.head = 0;
    tx_buffer.tail = 0;
    rx_buffer.head = 0;
    rx_buffer.tail = 0;
    _state = TRC_READY;
        
    UCSR0A |= (1<<MPCM0);  // Set MPCM
    UCSR0B = (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0) | (1<<UCSZ02);
    // 
    // POZOR UCSRC = (1<<URSEL) pro nastaveni UCSRC 
    //
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
    
    // Bit          7 6 5 4 3 2 1 0
    //              RXCIEn TXCIEn UDRIEn RXENn TXENn UCSZn2 RXB8n TXB8n UCSRnB
    //              R/W R/W R/W R/W R/W R/W R R/W
    //Initial Value 0 0 0 0 0 0 0 0
//    UCSR0B |= (1<<TXEN0);  // Enable Transmitter
//    UCSR0B |= (1<<RXEN0);  // Enable Reciever
//    UCSR0B |= (1<<RXCIE0); // Enable Rx Complete Interrupt
//    UCSR0B |= (1<<UCSZ02); // 9 Bit
// UCSRC is set by default
}
/**
 * Receive handler
 */

void LibRS485v2::write(uint8_t data) {  
  // Advance the head, store the data    
  tx_buffer.buffer[tx_buffer.head++] = data;
}

uint8_t LibRS485v2::read(void) {
  return rx_buffer.buffer[rx_buffer.tail++];
} 

void LibRS485v2::flushRX(void) {
  rx_buffer.head = 0;
  rx_buffer.tail = 0;
  _state = TRC_READY;
  UCSR0A |= (1<<MPCM0); // set MPCM
}

ISR(USART_RX_vect) {
  // full, do not receive anything 
  if ((rx_buffer.head + 1) == rx_buffer.tail) { 
    UCSR0A |= (1<<MPCM0); // set MPCM
    rx_buffer.head -= _msg_length; // return head
    return;
  }
  
  uint8_t status = UCSR0A; // Get status and 9th bit, then data from buffer
  uint8_t datah  = UCSR0B; 
  uint8_t datal  = UDR0;
  uint8_t _tmp, _tmp1;
  
  /* If error, return -1 */
  //if ( status & (1<<FE0)|(1<<DOR0)) output_high(PORTB, 5);
  //if ( status & (1<<FE0)) output_high(PORTB, 5);    
  // buffer overflow!
  
  datah = (datah >> RXB80) & 0b1; // Filter the 9th bit to 1 or 0
  // Check 9th bit
  if (datah){ 
    // Address frame in middle tranmission = error or contention
    if (!(status & (1 << MPCM0))) { // MPCM0 bit is not set
      UCSR0A |= (1<<MPCM0); // set MPCM
      rx_buffer.head = 0;
      rx_buffer.tail = 0; // Flush RX
      _state = TRC_READY;
      return; // escape  
    } else {
      _tmp = datal & 0b1111; //* _tmp = datal & 0b00001111;
      // Our address or broadcast, we start receiveing data
      if ((_tmp == _address) || (_tmp == 15)) {
        UCSR0A &= ~(1<<MPCM0); // clear MPCM
        _msg_length = 1;
        _data_length = 0;
        if (_state == TRC_READY) _state = TRC_RECEIVING;
      }          
      else { // not our address
        _state = TRC_READY;
        return; // escape 
      }
    }        
  } else { 
    _msg_length++;
    if (_msg_length == 2) { // control byte
      // We are receiving DATA + MSG_CRC_SIZE, ACK is 0 + MSG_CRC_SIZE 
      if (((datal >> 6) & 0b1) == FLAG_DTA) {
        _data_length = (datal & 0b111111) + MSG_CRC_SIZE;
      }
    }
    if (_msg_length == 3) { // XOR check
      if ((rx_buffer.buffer[(uint8_t)(rx_buffer.head-2)] ^ rx_buffer.buffer[(uint8_t)(rx_buffer.head-1)]) != datal) { // error
        UCSR0A |= (1<<MPCM0); // set MPCM
        rx_buffer.head = 0;
        rx_buffer.tail = 0; // Flush RX
        _state = TRC_READY;
        return; // escape 
      } 
    }
  }
  
  rx_buffer.buffer[rx_buffer.head++] = datal;  // save byte, advance head 
  
  if (_msg_length == MSG_HEADER_SIZE + _data_length) { // we have complete message
    UCSR0A |= (1<<MPCM0); // set MPCM
    // Receiving new packet
    if (_state == TRC_RECEIVING) {
      // If ACK requested, and length not 0
      if ((((rx_buffer.buffer[1] >> 7) & 0b1) == FLAG_ACK) &&
        ((rx_buffer.buffer[1] & 0b111111) != 0)) {
        tx_buffer.tail = 0;
        tx_buffer.head = 3;
        // Write header
        tx_buffer.buffer[0] = (_address << 4) | ((rx_buffer.buffer[0] >> 4) & 0b1111);    // To address
        // ACK, Data or CMD, Length is 0
        tx_buffer.buffer[1] = ((FLAG_ACK & 0b1) << 7) | (((rx_buffer.buffer[1] >> 6) & 0b1) << 6) | 0b000000; 
        // write XOR header
        tx_buffer.buffer[2] = (tx_buffer.buffer[0] ^ tx_buffer.buffer[1]); 
    
        // For data send CRC
        if (((rx_buffer.buffer[1] >> 6) & 0b1) == FLAG_DTA) {
            tx_buffer.buffer[3] = rx_buffer.buffer[rx_buffer.head - 2]; 
            tx_buffer.buffer[4] = rx_buffer.buffer[rx_buffer.head - 1];
            tx_buffer.head += 2;
        }
        
        _state = TRC_ACKING;
        // Go for write
        output_high(PORTD, 3);  // Enable WriteEnable on RS485 chip    
        UCSR0B |= (1<<UDRIE0);  // Enable Data Register Empty interrupt, start transmitting
        UCSR0B |= (1<<TXCIE0);  // Enable Transmit Complete interrupt (USART_TXC) 
      } else {
        _state = TRC_RECEIVED;
      }
    }
    if (_state == TRC_WAITING_ACK) {
      _state = TRC_ACKED;
      //output_low(PORTC, 3);
    }
  }
         
}

/**
 * Read procedure
 */
int8_t LibRS485v2::readMsg(RS485_msg *msg) {

  if (_state != TRC_RECEIVED) return 0; // no message
  
  uint8_t tmp_head;
  uint16_t tmp_crc = 0xFFFF; 
  
  // Read header
  tmp_head = read();
  msg->address     = tmp_head >> 4;  // from address

  tmp_head = read();
  msg->ack         = (tmp_head >> 7) & 0b1;
  msg->ctrl        = (tmp_head >> 6) & 0b1;
  msg->data_length = tmp_head & 0b111111;

  // compare XOR of header is done in interrupt
  tmp_head = read(); 
  
  // Read data part
  if (msg->ctrl == FLAG_DTA) {
    for(tmp_head = 0; tmp_head < msg->data_length; tmp_head++) {
      msg->buffer[tmp_head] = read();
      tmp_crc = _crc_xmodem_update(tmp_crc, msg->buffer[tmp_head]);
    }
    // Read DATA CRC or ACK signature, 
    tmp_head = read();
    msg->crc = (tmp_head << 8) | read();
  }
 
  // Flush RX
  rx_buffer.head = 0;
  rx_buffer.tail = 0;
  _state = TRC_READY;
 
  // Compare data CRC and computed CRC
  if ((msg->ctrl == FLAG_DTA) && (msg->data_length > 0) && (tmp_crc != msg->crc)) return -1; // CRC not valid
  
  return 1;
}

/**
 * Data Register Empty Handler
 */
ISR(USART_UDRE_vect) {
    if (tx_buffer.head == tx_buffer.tail) {
        UCSR0B &= ~(1<<UDRIE0); // Buffer is empty, disable the interrupt
        tx_buffer.head = 0;
        tx_buffer.tail = 0;
    } else {
        // Address or data  
        if (tx_buffer.tail == 0) {
          UCSR0B |= (1<<TXB80);  //set
        } else {
          UCSR0B &= ~(1<<TXB80); //clear
        }
        UDR0 = tx_buffer.buffer[tx_buffer.tail];
        tx_buffer.tail++;
    }
}

/**
 * Transmit Complete Interrupt Handler
 *  
 * Automatically cleared when the interrupt is executed. 
 */
ISR(USART_TX_vect) {
  output_low(PORTD, 3); // Disable WriteEnable on RS485 chip   
  // ACK sent, we raise flag received
  if (_state == TRC_ACKING) _state = TRC_RECEIVED;
  // Sending message
  if (_state == TRC_SENDING_WITH_ACK) _state = TRC_WAITING_ACK;
  if (_state == TRC_SENDING) _state = TRC_READY;
}


/**
 * Prepare and write message
 *   
 */
int8_t LibRS485v2::msg_write(RS485_msg *msg) {
    
  if (_state != TRC_READY) return 0; // Transmitter is busy;
  
  uint8_t tmp_head1, tmp_head2;

  tx_buffer.head = 0;
  tx_buffer.tail = 0; // Null buffer head and tail 
    
  // Write header
  tmp_head1 = _address << 4;               // Our address
  tmp_head1 = tmp_head1 + msg->address;    // To address
  write(tmp_head1);
  // Make sure that ACK NAK signature is only MSG_CRC_SIZE (2) bytes
  // Make sure that CMD data_length is 0 bytes
  tmp_head2  = ((msg->ack  & 0b1) << 7);
  tmp_head2 |= ((msg->ctrl & 0b1) << 6);
  tmp_head2 |= (msg->data_length & 0b111111);
  write(tmp_head2);
  write(tmp_head1 ^ tmp_head2);            // write XOR header
  
  // Write data 
  if (msg->ctrl == FLAG_DTA) {
    if (msg->data_length > 0) msg->crc = 0xFFFF; // Initialize CRC, but not for ACK
    for(tmp_head1 = 0; tmp_head1 < msg->data_length; tmp_head1++) {
      write(msg->buffer[tmp_head1]);
      msg->crc = _crc_xmodem_update(msg->crc, msg->buffer[tmp_head1]); // Is done only when (msg->data_length > 0), ie not for ACK
    }
    // Write CRC or ACK signature
    write(msg->crc >> 8);
    write(msg->crc & 0b11111111);
  }
    
  // Check incoming messages
  if (!(UCSR0A & (1 << MPCM0))) { 
    tx_buffer.head = 0;
    tx_buffer.tail = 0; // Null buffer head and tail
    return -1; // line is too busy, quit
  }
  
  // wait for line ready
  UCSR0B &= ~(1<<RXCIE0);  // Disable Rx Complete Interrupt
  UCSR0A &= ~(1<<MPCM0);   // Disable MPCM
  tmp_head1 = 0;
  do {
    tmp_head2 = UDR0;        //just empty rx buffer 
    //_delay_loop_2(_tick * ((random() % 15) + 1)); // random time, max is 15 nodes
    _delay_loop_2(_tick * (_address + 2)); // my address as time, max is 15 nodes
    tmp_head1++;
    if ( tmp_head1 == LINE_READY_TIME_OUT) {
      tx_buffer.head = 0;
      tx_buffer.tail = 0; // Null buffer head and tail
      UCSR0A |= (1<<MPCM0);  // Enable MPCM
      UCSR0B |= (1<<RXCIE0); // Enable Rx Complete Interrupt
      return -2; // line is too busy, quit
      } 
  } while (UCSR0A & (1<<RXC0)); // no char received, line is free      
  // Set DE to block receiving
  output_high(PORTD, 3);  // Enable WriteEnable on RS485 chip                       
  // Enable back 
  UCSR0A |= (1<<MPCM0);   // Enable MPCM
  UCSR0B |= (1<<RXCIE0);  // Enable Rx Complete Interrupt
  // End of wait for line ready
  
  // Go for write
  UCSR0B |= (1<<UDRIE0);  // Enable Data Register Empty interrupt, start transmitting
  UCSR0B |= (1<<TXCIE0);  // Enable Transmit Complete interrupt (USART_TXC) 
  
  if (msg->ack == FLAG_ACK) {
    _state = TRC_SENDING_WITH_ACK;
    //output_high(PORTC, 3);
  }
  else _state = TRC_SENDING;
  
  return 1;
}

int8_t LibRS485v2::sendMsg(RS485_msg *msg){
  // Force NACK
  msg->ack = FLAG_NACK;
  
  return msg_write(msg);
}

int8_t LibRS485v2::sendMsgWithAck(RS485_msg *msg, uint8_t repeat = 5){
  uint8_t _count_ack = 0, _count_send = 0;
  int8_t resp;
  
  // Force ACK
  msg->ack = FLAG_ACK;
  
  do {
    resp = msg_write(msg);
    if (resp == -1) return -1; // Cannot send while receiving
    if (resp == 1) {
      //output_high(PORTC, 5);
      _count_ack = 0;
      do {
        _delay_loop_2(_tick); // delay 1 character 
        if ((_state == TRC_ACKED) &&
          (msg->ctrl == ((rx_buffer.buffer[1] >> 6) & 0b1)) && 
          (msg->ack  == ((rx_buffer.buffer[1] >> 7) & 0b1)) &&
          ((rx_buffer.buffer[1] & 0b111111) == 0)
          ){
            // Data or CMD
            if (((rx_buffer.buffer[1] >> 6) & 0b1) == FLAG_DTA) {
              // For data check also CRC
              if ((uint16_t)((rx_buffer.buffer[3] << 8) | (rx_buffer.buffer[4])) == msg->crc) {
                flushRX();
                //output_low(PORTC, 5);
                return 1;
              }
            } else {
              flushRX();
              //output_low(PORTC, 5);
              return 1;
            }
          }
        _count_ack++;
      } while (_count_ack < USART_TX_BUFFER_SIZE);
      //output_low(PORTC, 5);
      flushRX();
    }
    _count_send++;
  } while (_count_send < repeat);
  return 0; // Send failed
}


LibRS485v2 RS485;//(&rx_buffer, &tx_buffer);
