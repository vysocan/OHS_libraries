/*
 *
 *
 *
*/

#ifndef NilGPRS_h
#define NilGPRS_h
#include <Arduino.h>

#include <NilRTOS.h>

#define AT_WAIT            50          // how many delay loops wait for modem response
#define AT_DELAY           100         // delay in millis for modem response 
// Commands
#define AT_is_alive        "AT"        // --'CR'OK'CR''CR'
#define AT_model           "AT+CGMM"   // --'CR'TC35'CR''CR'OK'CR'
#define AT_registered      "AT+CREG?"  // --'CR'+CREG: 0,1'CR''CR'OK'CR'
#define AT_signal_strength "AT+CSQ"    // --'CR'+CSQ: 21,99'CR''CR'OK'CR'
#define AT_set_sms_to_text "AT+CMGF=1" // --'CR'OK'CR'
#define AT_set_sms_receive "AT+CNMI=1,2,0,0,0" // Set modem to dump SMS to serial
#define AT_send_sms        "AT+CMGS="  // --"+6421494481" followed by message then CTRL-Z then enter
#define AT_CLIP_ON         "AT+CLIP=1" // Set CLI On
#define AT_CLIP_OFF        "AT+CLIP=0" // Set CLI Off
#define AT_D               "ATD"       // Dial number
#define AT_H               "ATH"       // Hang up
#define AT_modem_info      "ATI"       // 
#define AT_GET_DATA        "AT+HTTPREAD" //
// Replies
#define AT_OK              "OK"        // 
#define AT_CMT_reply       "+CMT"      // Incoming SMS
#define AT_send_sms_reply  "+CMGS:"    // SMS sent and number
#define AT_DOWNLOAD        "DOWNLOAD"  // 
#define AT_HTTPDATA        "AT+HTTPDATA="
#define AT_DATA_SIZE       "+HTTPREAD:"

extern "C" void USART0_TX_vect(void) __attribute__ ((signal));
extern "C" void USART0_RX_vect(void) __attribute__ ((signal));
extern "C" void USART0_UDRE_vect(void) __attribute__ ((signal));

#include <inttypes.h>

// Define buffer sizes
#define GSM_USART_RX_BUFFER_SIZE 1024
#define GSM_USART_TX_BUFFER_SIZE 128
#define DEFAULT_TIMEOUT          5000L

struct gsm_rx_ring_buffer {
    uint16_t buffer[GSM_USART_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
};

struct gsm_tx_ring_buffer {
    uint8_t buffer[GSM_USART_TX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
};


#ifdef __cplusplus
extern "C" {
#endif
 bool nilWaitGPRSNewMsg();
#ifdef __cplusplus
}
#endif

class NilGPRS : public Print {
  private:
    

  public:
    //uint8_t _msg;

    NilGPRS(gsm_rx_ring_buffer *, gsm_tx_ring_buffer *);
    void    begin(unsigned long);
    void    flushRX(void);
    uint8_t read(void);
    uint8_t read(uint8_t *msg);
    size_t  write(uint8_t);
    using   Print::write;
    void    printP(const unsigned char *str);
    
    uint8_t isMsg(void);

    uint8_t WaitMsg(uint16_t _wait);
    int8_t  sendCmd(char *what);
    int8_t  sendCmdWR(char *what, uint8_t *response);
    int8_t  sendCmdWR(char *what, uint8_t *response, uint8_t index);
    int8_t  sendSMSBegin(char *number);
    int8_t  sendSMSEnd(char *what);

    int8_t  sendQueryWR(char *what, uint8_t *response);
    int8_t  sendOKQueryWR(char *what, uint8_t *response, uint8_t index);
    //int8_t  sendData(char *what, uint8_t length = 0, uint16_t timeout = DEFAULT_TIMEOUT);
    int8_t  sendData(char *what);
    int8_t  getData(uint8_t *response);

    int8_t  sendATTest();
    //Result  configureBearer(const char *apn);


    bool    nilWaitGPRSNewMsg();

};

extern NilGPRS GPRS;

#endif
 