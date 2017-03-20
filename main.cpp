/*
 * TEC_controler_A9.cpp
 *
 * Created: 08/02/2017 08:30:08
 * Author : David
 * License: https://creativecommons.org/licenses/by-sa/3.0/
 */ 

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//function prototypes
uint16_t getADC(uint8_t sel_pin); //sel_pin = ADC_V or ADC_I
float ADCtoVolt(uint16_t adc); //convert ADC bits to volts (for logging)
float ADCtoAmp(uint16_t adc); //convert ADC bits to amps (for logging)
	//I2C - control
void sendSPIByte(uint8_t byte0, uint8_t byte1, uint8_t byte2); //SPI - DAC
	//UART - logging
void initClk(); //initialise clock0
void calcPID(); //calc and send PID


//Global values
//DAC ref to external 2.5V so DAC = ((1024/2.5V)*Vout)-1
//1.25V = zero, x < 1.25V cool, x > 1.25V heat
volatile uint16_t vControl = 511; //TEC heat/cool magnitude, DAC-port A; 1.25V
volatile uint16_t currTemp; //current temperature 
volatile uint16_t iLimCool = 900; //TEC cooling current limit DAC-port D; 2.2V
volatile uint16_t iLimHeat = 900; //TEC heating current limit DAC-port B; 2.2V
volatile uint16_t vLim = 162; //TEC voltage limit DAC-port C; 0.4V
volatile uint16_t setTemp = 668; //requested constant temperature for TEC (ADC 10bit value = 25dec C)
//setup PID values
/*One generic tuning process is 
- Set all gains to zero.
 - Increase the P gain until the response to a disturbance is steady oscillation.
 - Increase the D gain until the the oscillations go away (i.e. it's critically damped).
 - Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
 - Set P and D to the last stable values.
Increase the I gain until it brings you to the setpoint with the number of oscillations 
desired (normally zero but a quicker response can be had if you don't mind a couple 
oscillations of overshoot)*/
volatile int16_t lastProcessValue; //Last process value, used to find derivative of process value.
volatile int16_t sumError; //Summation of errors, used for integrate calculations
volatile int16_t P_Factor = 80; //The Proportional tuning constant, multiplied with SCALING_FACTOR
volatile int16_t I_Factor = 2; //The Integral tuning constant, multiplied with SCALING_FACTOR
volatile int16_t D_Factor = 1200; //The Derivative tuning constant, multiplied with SCALING_FACTOR
volatile uint8_t enPID = 1; //enable PID
//setup counter
volatile uint16_t count = 0; //counter based on timer A interrupt up to dt * 2^16 = 214s


//DAC selection bytes, CODEn & LOAD_ALL
//b19	b18	b17	b16
//0		0	0	0	DAC A
//0		0	0	1	DAC B
//0		0	1	0	DAC C
//0		0	1	1	DAC D
uint8_t dacA = 0b00110000;
uint8_t dacB = 0b00110001;
uint8_t dacC = 0b00110010;
uint8_t dacD = 0b00110011;

//Pin numbers for TEC project
#define CLR PIN1_bm //PA1 - CLR (active low)
#define ADC_K ADC_MUXPOS_AIN5_gc //PA5 - THERM
#define ADC_V ADC_MUXPOS_AIN6_gc //PA6 - VTEC
#define ADC_I ADC_MUXPOS_AIN7_gc //PA7 - ITEC
//PB0 - SCL
//PB1 - SDA
//PB2 - TX
//PB3 - RX
#define enTEC PIN5_bm //PB5 - Enable TEC
#define SCK PIN0_bm //PC0 - SCK
#define MISO PIN1_bm //PC1 - MISO
#define MOSI PIN2_bm //PC2 - MOSI
#define SS PIN3_bm //PC3 - SS (active low)


//Read ADC on clock interrupt when triggered by compare value 0
ISR (TCA0_CMP0_vect)
{
	//clear overflow flags
	TCA0.SINGLE.INTFLAGS = 0;
	//restart counter
	TCA0.SINGLE.CTRLESET |= TCA_SINGLE_CMD_RESTART_gc;
	//read ADC
	currTemp = getADC(ADC_K);  
	//calc PID if enabled
	if (enPID != 0)
	{
		calcPID();
	}
	count++; //increment general counter/timer
}

//interrupt for reading I2C commands

//interrupt for transmitting serial data



//regularly used function to send byte to SPI
static inline uint8_t spi(uint8_t c) //static inline - suggestion to compiler to optimize
{
	SPI0.DATA = c;
	while ((SPI0.INTFLAGS & SPI_RXCIF_bm) == 0);
	return SPI0.DATA;
}



///////////////////////////////////////////////////////////////////////////////
///////////////////////// START of MAIN ///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
int main(void)
{
	//TINY - setup port A (1 = output)
	// 7 6 5 4 3 2 1 RESET
	// 5 = Thermistor (input)
	// 3 = TEC current (input)
	// 2 = TEC voltage (input)
	// 1 = DAC CLR (Active low) (output)
	PORTA.DIRSET = CLR;
	PORTA.OUTSET = CLR; //portA pin 1 = high 


	//TINY - setup port B (1 = output)
	// 7 6 5 4 RX TX SDA SCL
	// 5 = shut down (output)
	// 4 = temp good (input)
	PORTB.DIRSET = enTEC;
	PORTB.OUTCLR = enTEC; //set enable TEC to low /off at startup


	//TINY - setup SPI MAX5713 (max 50MHz)
	PORTMUX.CTRLB = PORTMUX_SPI0_ALTERNATE_gc; //SPI1 pins so switch MUX
	//SPI - configure SCK, MOSI, SS pins to output
	PORTC.DIRSET = SCK | MOSI | SS;
	//SPI - set SS high (active low)
	PORTC.OUTSET = SS;
	//SPI - select SPI as master
	SPI0.CTRLA |= SPI_MASTER_bm;
	//SPI - SPI Mode 0
	SPI0.CTRLB |= SPI_MODE_0_gc;
	//SPI - select the clock speed prescaler 64
	SPI0.CTRLA |= SPI_PRESC_DIV64_gc;
	//SPI - Slave Select Disable
	SPI0.CTRLB |= SPI_SSD_bm;
	//SPI - enable SPI
	SPI0.CTRLA |= SPI_ENABLE_bm;
	
	
	//better to set the DAC up each time rather than try and rely on EEPROM settings
	PORTA.OUTCLR = CLR; //portA pin 1 = low
	_delay_ms(1);
	PORTA.OUTSET = CLR; //portA pin 1 = high
	//DAC - setup with initial outputs
	//CONFIG = all DACs, latch is transparent
	sendSPIByte(0b01101001, 0b00001111, 0x00);
	//DAC - setup REF to external
	//REF = 0b01110000; //last two bytes are zeros
	sendSPIByte(0b01110000, 0x00, 0x00);
	//DAC - set vControl
	//shift 16bit value right to get high byte, shift left 4 bits to get low byte
	//DAC A - set output TEC
	sendSPIByte(dacA, (uint8_t)(vControl >> 2), (uint8_t)(vControl << 6));
	//DAC B - set iLimHeat
	sendSPIByte(dacB, (uint8_t)(iLimHeat >> 2), (uint8_t)(iLimHeat << 6));
	//DAC C - set vLim
	sendSPIByte(dacC, (uint8_t)(vLim >> 2), (uint8_t)(vLim << 6));
	//DAC D - set iLimCool
	sendSPIByte(dacD, (uint8_t)(iLimCool >> 2), (uint8_t)(iLimCool << 6));
	
	
	
	//TINY - setup ADC
	//VREF - setup voltage reference to 2.5V
	VREF.CTRLA |= VREF_ADC0REFSEL_2V5_gc;
	//VREF - enable ADC ref
	VREF.CTRLB |= VREF_ADC0REFEN_bm;
	//ADC - select resolution 10bit
	ADC0.CTRLA |= ADC_RESSEL_10BIT_gc;
	//ADC - number of convertions accumulated per measurement
	ADC0.CTRLB |= ADC_SAMPNUM_ACC8_gc; //update temp conversion if changing number of samples
	//ADC - select reference
	ADC0.CTRLC |= ADC_REFSEL_INTREF_gc;
	//ADC - sampling rate pre-scaler ~1.25MHz
	ADC0.CTRLC |= ADC_PRESC_DIV16_gc;
	//ADC - initial input PA5 / AIN5
	ADC0.MUXPOS |= ADC_MUXPOS_AIN5_gc;
	//ADC - enable start event (start measuring on enable)
	ADC0.EVCTRL |= ADC_STARTEI_bm;
	//ADC - enable ADC (ready for measurement trigger)
	ADC0.CTRLA |= ADC_ENABLE_bm;
	
	
	
	//TINY - setup UART for debugging current and voltage data
	//UART -
	


	//TINY  - setup I2C Slave
	//I2C - enable slave
	TWI0.SCTRLA |= TWI_AP_ADR_gc;
	//I2C - set address
	TWI0.SADDR |= 0b00111110;
	
	
	//load PID settings from EEPROM
	
	
	
	//initialise clock & enable interrupts
	initClk();
	sei();
	
	PORTB.OUTSET = enTEC; //enable TEC
	
	
    while (1) 
    {
		//calcPID();
		//getVoltage();
		//getCurrent();
		_delay_ms(100);
		
		
    }
}
///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// END of MAIN ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


//read ADC value
uint16_t getADC(uint8_t sel_pin)
{
	//wait for current measurement to finish
	while((ADC0.COMMAND & ADC_STCONV_bm) == 1){}
	//select input PA2 / AIN2
	ADC0.MUXPOS = sel_pin;
	//trigger reading
	ADC0.COMMAND |= ADC_STCONV_bm;
	//wait while reading taken
	while((ADC0.COMMAND & ADC_STCONV_bm) == 1){}
	//process result
	uint16_t result = (ADC0.RES/8); //divide by number of samples accumulated and convert to voltage
	return result;
}


float ADCtoVolt(uint16_t adc)
{
	//convert to Volts
	float result = ((adc*0.00423828)-1.25)*4; //(x-(0.5*2.5v))4 >> (x-1.25)*4
	return result;
}


float ADCtoAmp(uint16_t adc)
{
	//convert to Volts
	float result = ((adc*0.00423828)-1.25)*2; //(x-(0.5*2.5v))/(25*0.02Ohms) >> (x-1.25)*2
	return result;
}


//send 24bit string to DAC
void sendSPIByte(uint8_t byte0, uint8_t byte1, uint8_t byte2)
{
	//lower SS - start transmit
	PORTC.OUTCLR = SS; //set SS low (active low)
	//load data
	spi(byte0);
	spi(byte1);
	spi(byte2);
	//raise SS - END of transmit
	PORTC.OUTSET = SS; //set SS high (active low)
}



//setup clock
void initClk()
{
	/*interval timer is set using the master clock divider/prescaler and then setting
	the value at which the clock is compared. if the values match then the interrupt
	is triggered and the clock and interrupt flags are re-set*/ 
	//select clock divider 1/64 on 16bit clock
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc; // 20MHz / 64 = 312.5kHz
	//clear overflow flags
	TCA0.SINGLE.INTFLAGS = 0;
	//enable compare channel 0
	TCA0.SINGLE.INTCTRL |= TCA_SINGLE_CMP0_bm;
	//set compare value - trigger value for interrupt reading 11th bit (10bits rollover) (305Hz)
	TCA0.SINGLE.CMP0 = 0b0010000000000000;
	//enable overflow interrupt ctrl
	TCA0.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm;
	//enable timer
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
	
}


//PID controller
void calcPID()
{
	int16_t error;
	int32_t p_term;
	int32_t d_term;
	int32_t i_term;
	int32_t temp;

	
	//calculate error
	error = setTemp - currTemp; // +/-ve value
	
	// Calculate Pterm and limit error overflow
	if (error > 511) //positive limit
	{
		p_term = 511;
	}
	else if (error < -511) //negative limit
	{
		p_term = -511;
	}
	else
	{
		p_term = P_Factor * error;
	}

	// Calculate Iterm and limit integral runaway
	temp = sumError + error;
	if (temp > 511)
	{
		i_term = 511;
		sumError = 511;
	} 
	else if (temp < -511)
	{
		i_term = -511;
		sumError = -511;
	}
	else
	{
		sumError = temp;
		i_term = I_Factor * sumError;
	}

	// Calculate Dterm
	d_term = (D_Factor * (lastProcessValue - vControl))/128;
	
	lastProcessValue = vControl;
	
	temp = 511 - ((p_term + i_term + d_term)/256); //nominal mid point, use temp as signed holder
	
	//check for valid range (10bit)
	if (temp > 1023)
	{
		temp = 1023;
	} 
	else if (temp < 0)
	{
		temp = 0;
	}
	
	vControl = temp; 
	
	//set new value
	//DAC A - set output TEC
	sendSPIByte(dacA, (uint8_t)(vControl >> 2), (uint8_t)(vControl << 6));
	//update previous values

}


