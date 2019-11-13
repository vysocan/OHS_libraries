/**
 *
 * 
*/

#ifndef LibRS485v2_h
#define LibRS485v2_h

#include <inttypes.h>

#include "Print.h"

#include <util/delay_basic.h>

// Define message parameters
#define MSG_HEADER_SIZE 3        // 2 bytes and XOR
#define MSG_DATA_SIZE 64         // 64 bytes
#define MSG_CRC_SIZE 2           // 2 bytes data CRC16
#define LINE_READY_TIME_OUT 100  // max 255
// Define buffer sizes
#define USART_RX_BUFFER_SIZE 256 // must be 256
#define USART_TX_BUFFER_SIZE (MSG_HEADER_SIZE + MSG_DATA_SIZE + MSG_CRC_SIZE)
// Define the amount of tolerance upon which x2 will be enabled, in percent
#define BAUD_TOL 2

// RS485 header bits
#define FLAG_ACK  1
#define FLAG_NACK 0
#define FLAG_CMD  1
#define FLAG_DTA  0

typedef enum { 
    TRC_UNINIT = 0,
    TRC_STOP = 1,
    TRC_READY = 2,
    TRC_SENDING = 3,
    TRC_SENDING_WITH_ACK = 4,
    TRC_WAITING_ACK = 5,
    TRC_RECEIVING = 6,
    TRC_ACKING = 7,
    TRC_RECEIVED = 8,
    TRC_ACKED = 9 
} rs485transceiver_t;

struct rx_ring_buffer {
    uint8_t buffer[USART_TX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
};

struct tx_ring_buffer {
    uint8_t buffer[USART_TX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
};

struct RS485_msg {
  unsigned char buffer[MSG_DATA_SIZE]; 
  uint8_t address;
  uint8_t data_length;
  uint8_t ctrl;
  uint8_t ack;
  uint16_t crc;
};

class LibRS485v2 // : public Print
{
  private:
    void write(uint8_t);
    uint8_t read(void);
    int16_t _tick;    
    int8_t msg_write(RS485_msg *msg); 
    
  public:
    LibRS485v2();//(rx_ring_buffer *, tx_ring_buffer *);
    void begin(unsigned long, uint8_t);
    void flushRX(void);
    int8_t readMsg(RS485_msg *msg);
    int8_t sendMsg(RS485_msg *msg);
    int8_t sendMsgWithAck(RS485_msg *msg, uint8_t repeat);

};

extern LibRS485v2 RS485;

#endif
 
