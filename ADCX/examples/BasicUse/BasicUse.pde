#include <Wire.h>
#include <ADCX.h>

//Initialize the ADC and use the default 
//ADC address that's provided
ADCX adc(ADCX::ADCX_I2C_DEFAULT_ADDRESS);

//this is where we will store the values we read from the ADC
static unsigned char adc_channel_values[8] = {0,0,0,0,0,0,0,0};

void setup(){
	Wire.begin(); //we must call this before using any I2C functions!
	Serial.begin(9600);
}

void loop(){

	//max is 8
	unsigned char number_of_channels_to_read = 8;

	//loop through the channels
	for(int i=0; i<number_of_channels_to_read; i++){
		
		//save the reading of channel "i"
		adc_channel_values[i] = adc.read(i);
		
		//print it out
		Serial.print("ADC Channel ");
		Serial.print(i, DEC);
		Serial.print(" was ");
		Serial.print(adc_channel_values[i],DEC);
		Serial.println(" ");
	}


	//Handle any I2C errors
	unsigned int error = adc.errors();
	Serial.print("Error code: ");
	Serial.print(error,HEX);
	
	switch(error)
	{
		case 0: Serial.print(" [Success!] "); break;
		case 1: Serial.print(" [Data too large for buffer] "); break;
		case 2: Serial.print(" [I2C Address was not found] "); break;
		case 3: Serial.print(" [Data was not acknowledged] "); break;
		default:Serial.print(" [There was some other error] "); break;
	}
	Serial.println(" ");

	delay(5000);
}
