// Test sketch that is loaded on slave Moteinos
// It will send an encrypted message to the master/gateway every TRANSMITPERIOD
// It will respond to any ACKed messages from the master
// Every 3rd message will also be ACKed (request ACK from master).
#include <RFM12B.h>
#include <avr/sleep.h>

#define MYID        2       // node ID used for this unit
#define NETWORKID   1
#define GATEWAYID   1
#define FREQUENCY  RF12_868MHZ //Match this with the version of your Moteino! (others: RF12_433MHZ, RF12_915MHZ)
#define KEY  "ABCDABCDABCDABCD"
#define TRANSMITPERIOD 2000 //transmit a packet to gateway so often (in ms)

#define SERIAL_BAUD      115200
#define ACK_TIME             50  // # of ms to wait for an ack

int interPacketDelay = 5000; //wait this many ms between sending packets
char input = 0;
RFM12B radio;

boolean requestACK = false;
byte sendSize=0;
char payload[] = "123 ABCDEFGHIJKLMNOPQRSTUVWXYZ";

void setup()
{
  Serial.begin(SERIAL_BAUD);
  radio.Initialize(MYID, FREQUENCY, NETWORKID, 0);
  radio.Encrypt((byte*)KEY);
  char buff[50];
  sprintf(buff, "Transmitting at %d Mhz...", FREQUENCY == RF12_433MHZ ? 433 : FREQUENCY== RF12_868MHZ ? 868 : 915);
  Serial.println(buff);
}

long lastPeriod = -1;
void loop()
{
  //process any serial input
  if (Serial.available() > 0) {
    input = Serial.read();
    if (input >= 48 && input <= 57) //[0,9]
    {
      interPacketDelay = 100 * (input-48);
      if (interPacketDelay == 0) interPacketDelay = 1000;
      Serial.print("\nChanging delay to ");
      Serial.print(interPacketDelay);
      Serial.println("ms\n");
    }
  }

  //check for any received packets
  if (radio.ReceiveComplete())
  {
    if (radio.CRCPass())
    {
      Serial.print("Receive [");
      Serial.print(radio.GetSender(), DEC);
      Serial.print("]:");
      for (byte i = 0; i < *radio.DataLen; i++)
        Serial.print((char)radio.Data[i]);

      if (radio.ACKRequested())
      {
        radio.SendACK();
        Serial.println(" - ACK sent.");
      }
    } else Serial.print(F("BAD-CRC"));
  }
  
  if ((int)(millis()/TRANSMITPERIOD) > lastPeriod)
  {
    lastPeriod++;
    //Send data periodically to GATEWAY
    Serial.print("Sending [");
    Serial.print(sendSize);
    Serial.print("]: ");
    for(byte i = 0; i < sendSize; i++)
      Serial.print((char)payload[i]);
    
    requestACK = ((sendSize % 3) == 0); //request ACK every 3rd xmission
    radio.Send(GATEWAYID, payload, sendSize, requestACK);
    if (requestACK)
    {
      Serial.print(" - waiting for ACK...");
      if (waitForAck(GATEWAYID)) Serial.print("ok!");
      else Serial.print("nothing...");
    }
    
    sendSize = (sendSize + 1) % 31;
    Serial.println();
  }
}

// wait a few milliseconds for proper ACK to me, return true if indeed received
static bool waitForAck(byte theNodeID) {
  long now = millis();
  while (millis() - now <= ACK_TIME) {
    if (radio.ACKReceived(theNodeID))
      return true;
  }
  return false;
}
