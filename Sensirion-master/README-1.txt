Sensirion: An Arduino Library for the Sensirion SHT1x, SHT7x, SHT2x, SHT3x 
family of temperature and humidity sensors.

!!! WARNING SHT2x is a 3.3V Sensor. Mine survive to 5V but avoid it !!!

Created by Markus Schatzl, November 28, 2008
Revised   (v1.1)  by Carl Jackson, August 4, 2010
Rewritten (v2.0)  by Carl Jackson, December 8, 2010
Rewritten (v3.01) by Thierry Couquillou, June 03, 2015

Revision History

1.0 -  Original code provides a constructor, two public functions, plus
       several private functions.  The primary public function, "measure",
       commands the sensor to perform both a temperature and a humidity
       measurement and then calculates the dewpoint.  Total execution time
       is approximately 400 milliseconds.
       
1.1 -  Added several new functions while touching as little as possible of
       the original code.  The primary new feature is the ability to perform
       non-blocking measurements, ie, to return control to the calling routine
       after sending a command to the sensor rather than to spin waiting for
       the measurement to complete.  Also added the ability to set the sensor
       measurement precision (14-bit/12-bit Temp/RH vs. 12-bit/8-bit Temp/RH)
       for precision vs. speed trade-off.  Updated equation coefficients for
       the V4 version of the sensors per Sensirion recommendations.
       
2.0 -  Extensive changes for robustness, code size, and new features.  Added
       CRC checking, consistent handling of the data pin internal pullup, and
       improved error reporting.  Added sensor status register read function
       and expanded status register write function to cover all setable bits.
       
3.0 -  Extensive change for real non-blocking, easy to use interface. Only
       one interface remain: sht.measure(&temp,&humi,&dew,...), the one used 
       before for blocking mode. Now always in non-blocking mode + CRC + pull_up. 
       If you really want blocking (and you never want this as it's evil) use
       something like while(sht.measure(&temp,&humi,&dew,...) != S_Meas_Rdy).
       
       CRC calculation have been updated to be fully compatible with SHT1x 
       and SHT2x sensors using SHT1x protocol 
       (before connect SHT2x sensors cause CRC error)
       
       I2c protocol used by new sensors SHT2x and SHT3x have been implemented
       so all sensors SHT1x, SHT7x, SHT2x, SHT3x should works if address is
       set correctly in constructor Sensirion(dataPin, sclkPin, address).
       
       I2c No Hold Master have been implemented to avoid freezing I2c network
       that maybe used for something else.
       
       Was tested with SHT10, SHT20 in SHT1x protocol, SHT2x holdmaster and
       SHT2x noholdmaster protocol OK. SHT30 was not tested.

       Many sensors can be used in same time using dedicated pin.
      
3.01 - Modify and test for STM32F1 architecture (Maple Mini clone) + BugFix
  
     - Replace deprecated pinMode(INPUT) + digitalWrite(HIGH) by 
       pinMode(INPUT_PULLUP) to be compatible with STM32F1
         
     - Remove forgotten debug stuff using pin4 to trigger my scope
       
     - Tweak timings to be able to work without any additional component
       on 3.3v MCU (like Maple Mini STM32F1) no aditional pullup or
       filter capacitor.


 Usage Information
-------------------

CRC error detection is always enabled.

The library header file defines two macros (PULSE_SHORT and PULSE_LONG)
that are used in generating the sensor interface signaling.  By default,
PULSE_SHORT delays 15 microsecond and PULSE_LONG delays 30 microseconds.

Very long connections may require increase to the delay macros. 
If not sufficient, additional pullup termination resistor and low pass 
filtering (capacitor) will improve the sensor interface signal integrity.

To avoid self heating of the sensor, Sensirion recommends that the
sensor not be active for more than 10% of the time. So SHT1x sensors will
perform 1 measurement every 4s and SHT2x SHT3x sensors every second

 Prototype
-----------

Sensirion(uint8_t dataPin, uint8_t clockPin, uint8_t address = 0x00, bool noholdmaster = false)
  New parameters "address" and "noholdmaster" are optionals

int8_t  measure(float *temp = NULL, float *humi = NULL, float *dew = NULL, float temp0 = 0, float *humi0 = NULL);   
  All parameters are optionals
  temperature in oC, humidity in %, dewpoint in oC
  temp0 and humi0 are used to return equivalent humidity at a given theorical temperature
  
Return code :
  S_Err_TO     = -4; // Timeout
  S_Err_CRC    = -3; // CRC failure
  S_Err_NoACK  = -2; // ACK expected but not received
  S_Err_Param  = -1; // Parameter error in function call

  S_Meas_Wait  = 0;  // Wait for sensor cooling down
  S_Temp_Req   = 1;  // Temperature request (pulse)
  S_Temp_Wait  = 2;  // Wait for temperature measurement
  S_Humi_Req   = 3;  // Humidity request (pulse) means temperature measurement was successfull
  S_Humi_Wait  = 4;  // Wait for humidity measurement
  S_Calc_Run   = 5;  // Calculation in progress

  S_Meas_Rdy   = 6;  // All measurement was successfull (pulse)
  
 Usage Example
---------------

Sensirion sht1 = Sensirion(2, 3);

void loop()
{
  ret = sht1.measure(&temperature, &humidity, &dewpoint);
  
  if (ret == S_Meas_Rdy) // A new measurement is available
  {    
    ... use measurements ...
  }
}


