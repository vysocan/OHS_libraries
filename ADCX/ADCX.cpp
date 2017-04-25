/* Manages an ADC-X Add-on for BoardX
 * More information available here: http://upgradeindustries.com/product/13/ADC-X-Add-on
 * Library written by Kevin Green for UPGRADE INDUSTRIES
*/

#if ARDUINO >= 100
	#include <Arduino.h>
	
	//for compatibility with new function names in Arduion 1.0+
	#define I2C_WRITE(...) Wire.write(__VA_ARGS__)
	#define I2C_READ(...)  Wire.read(__VA_ARGS__)

#else

  	#include <WProgram.h>
	#define I2C_WRITE(...) Wire.send(__VA_ARGS__)
	#define I2C_READ(...)  Wire.receive(__VA_ARGS__)

#endif

#include "ADCX.h"
#include <stdint.h>
#include "../Wire/Wire.h"

/* Constructor, initializes an ADC on the I2C bus.
 * param i2c_address 8-bit unshifted address on I2C bus. LSB should always be 0
*/
ADCX::ADCX(uint8_t i2c_address) : _adc_i2c_address(i2c_address), _error_code(0), _samples_taken(0), _reference_type(ADCX_REF_MODE_INTERNAL){

	//shift down one so we don't have to keep doing that.
	_adc_i2c_address = i2c_address>>1;
}

/* Read a single ended channel on the ADC
 * param channel The channel to read
 * return a sample
*/
uint8_t ADCX::read(uint8_t channel){

	channel = (((channel>>1) | (channel&0x01)<<2)<<4) | ADCX_SINGLE_ENDED;

	return adcSampleRaw(channel, _reference_type);
}

/* Sets the reference type
 * param reference	reference spec
*/
void ADCX::setReference(uint16_t reference){
	_reference_type = reference;
}

/* Get the number of samples taken initialization
 * return number of samples total
*/		
uint16_t ADCX::samplesTaken(){
	return _samples_taken;
}

/* Get the any errors that are picked up
 * return error associated with I2C transmissions
*/		
uint16_t ADCX::errors(){
	return _error_code;	
}

/* Take a reading on a channel
 * param channel the channel number to sample
 * return a sample
*/
uint8_t ADCX::adcSampleRaw(uint8_t channel, uint8_t reference_mode_bits)
{
	uint8_t ret_buffer;

	// combine raw channel and reference bits
	channel &= 0xF0;
	channel |= reference_mode_bits;

	// start conversion on requested channel
	Wire.beginTransmission(_adc_i2c_address);
	I2C_WRITE(channel);
	_error_code = Wire.endTransmission();

	//read result
	Wire.requestFrom(_adc_i2c_address, (uint8_t)1);

	while(Wire.available())
		ret_buffer = I2C_READ();

	_samples_taken++;	

	return ret_buffer;

}