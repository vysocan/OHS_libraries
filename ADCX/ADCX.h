#ifndef ADC_X_h
#define ADC_X_h

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <stdint.h>

/* Manages an ADC-X Add-on for BoardX
 * More information available here: http://upgradeindustries.com/product/13/ADC-X-Add-on
 * Library written by Kevin Green for UPGRADE INDUSTRIES
*/
class ADCX {

// using ref = ADCX_REF_MODE_EXTERNAL  =>  External reference voltage on Ref pin.
// using ref = ADCX_REF_MODE_INTERNAL  =>  Internal 2.5V reference voltage (Ref pin left open).

enum ADCX_CONSTANTS{
	ADCX_REF_MODE_EXTERNAL=0,
	ADCX_REF_MODE_INTERNAL=1,
	ADCX_SINGLE_ENDED=0x80
};


private:

	uint8_t	 _adc_i2c_address;
	uint16_t _samples_taken;
	uint16_t _error_code;
	uint16_t _reference_type;

	/* Take a reading on a channel
	 * param channel the channel number to sample
	 * return a sample
	*/
	uint8_t adcSampleRaw(uint8_t channel,  uint8_t reference_mode_bits);


public:

	static const uint8_t ADCX_I2C_DEFAULT_ADDRESS = 0x90;

	/* Constructor, initializes an ADC on the I2C bus.
	 * param i2c_address 8-bit unshifted address on I2C bus. LSB should always be 0
	*/
	ADCX(uint8_t i2c_address);

	/* Read a channel on the ADC
	 * param channel	The channel to read
	 * return a sample
	*/
	uint8_t read(uint8_t channel);
	
	/* Sets the reference type
	 * param reference	reference spec
	*/
	void setReference(uint16_t reference);

	/* Get the number of samples taken initialization
	 * return number of samples total
	*/		
	uint16_t samplesTaken();

	/* Get the any errors that are picked up
	 * return error associated with I2C transmissions
	*/		
	uint16_t errors();

};


#endif