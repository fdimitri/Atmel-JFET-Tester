/*
 * ATTiny1634Test.c
 *
 * Created: 10/1/2015 2:33:47 PM
 * Author : FrankD
 */ 
#define F_CPU 8000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

#undef ADC_LTC2418
#define ADC_LTC2439

#define ADC_INPUTS 16

#define SPI_WAIT_MSG 0x01
#define SPI_SENDING_MSG 0x02
#define SPI_SENT_MSG 0x04
#define SPI_COMPLETE_MSG 0x08

#define MODE_IDSS	1
#define MODE_VP		2



#ifdef ADC_LTC2418
#define SPI_MESSAGE_LENGTH 4
#endif

#ifdef ADC_LTC2439
#define SPI_MESSAGE_LENGTH 2
#endif


#define MAKE_BINARY_STRING(byte) 	(byte & 0x80 ? 1 : 0), 	(byte & 0x40 ? 1 : 0), 	(byte & 0x20 ? 1 : 0), 	(byte & 0x10 ? 1 : 0), 	(byte & 0x08 ? 1 : 0), 	(byte & 0x04 ? 1 : 0), 	(byte & 0x02 ? 1 : 0), 	(byte & 0x01 ? 1 : 0)
#undef MAKE_BINARY_STRING



static uint8_t x = 0;
void serialMsgFmt(char *fmt, ...);
void setup(void);
void loop(void);
void USART_Init(unsigned int baud);
uint8_t spiTransfer8(uint8_t data);
uint8_t spiTransferMulti(unsigned char *string, unsigned char *retString, unsigned char len);
uint8_t spiTransferBit(uint8_t data);
uint_fast32_t readInput_LTC2418(uint8_t muxAddr);
uint_fast32_t readInput_LTC2439(uint8_t muxAddr);
uint_fast32_t readInput(uint8_t adcPin);
void readAllInputs(void);
void serialMsg(char *msg);
void setMode(uint8_t newMode);
void sleepADC();
void wakeADC();
uint8_t testEOC_LTC2439();


unsigned char bufUSIDR[4];
uint_fast32_t results[ADC_INPUTS];
unsigned char spiFlags;
char genBuf[256];

ISR(USI_OVERFLOW_vect) {
	// We got a byte on SPI!
	TIMSK &= ~(1 << OCIE0A);
	// Copy USIDR to buffer to prevent overwrite on next transfer.
	bufUSIDR[x] = USIDR;
	// Update flags and clear USI counter
	USISR = (1 << USIOIF);
	if (spiFlags & SPI_WAIT_MSG) {
		if (x < (SPI_MESSAGE_LENGTH - 1)) ++x;
		else {
			spiFlags |= SPI_COMPLETE_MSG;
			x = 0;
		}
	}
}



int main(void) {
	sleepADC();
	_delay_ms(2000);
	setup();
    while (1) {
		loop();
    }
}

void setup(void) {
	// SPI setup
	//USICR = (1<<USIOIE) | (1<<USIWM0) | (1<<USICS1) | (1<<USICLK) | (1<<USITC); //counter overflow interrupt enable, and 3 wire mode, external register clk
	USICR =   (1<<USIOIE) | (1<<USIWM0) | (1<<USICS1) | (1<<USICS0) | (1<<USICLK) | (1<<USITC);
	DDRA |= (0b000111000);
	
	TCCR0A = (1<<WGM01) | 0x02;
	OCR0A = 31;
	
	USART_Init((uint_fast16_t) 38400);
}

void loop(void) {
	setMode(MODE_IDSS);
	readAllInputs();
	setMode(MODE_VP);
	readAllInputs();
	
}

void sleepADC() {
	PORTA |= (1 << 3);
}

void wakeADC() {
	PORTA |= (1 << 3);
	PORTA &= ~(1 << 3);
	PORTA |= (1 << 3);
	PORTA &= ~(1 << 3);

}


void setMode(uint8_t newMode) {
	switch(newMode) {
		case MODE_IDSS:
			PORTA |= (1 << 4);
		break;
		case MODE_VP:
			PORTA &= ~(1 << 4);
		break;
	}
}

void serialSendByte(char b) {
	while (!(UCSR0A & (1<<UDRE0)));
	// set data into data register
	UDR0 = b;
}

void serialMsg(char *msg) {
	while (*msg) {
		serialSendByte(*msg++);
	}
	return;
}

void serialMsgFmt(char *fmt, ...) {
	memset((void *) genBuf, 0, sizeof(genBuf));
	va_list args;
	va_start(args, fmt);
	vsprintf(genBuf, fmt, args);
	va_end(args);
	serialMsg(genBuf);
	serialMsg("\r\n");
	return;
}


void readAllInputs(void) {
	uint8_t adcPin;
	readInput(1);
	_delay_ms(500);
	for (adcPin = 1; adcPin < ADC_INPUTS; adcPin++) {
		results[adcPin - 1] = readInput(adcPin);
		_delay_ms(500);
	}
	_delay_ms(500);
	results[adcPin - 1] = readInput(adcPin);
}

uint_fast32_t readInput(uint8_t adcPin) {
	uint_fast32_t result;
	adcPin -= 1; // ie 16 Becomes 15
	uint8_t muxAddr;
	// Set 0b10 mandatory, 0bXX1 to select new address
	muxAddr = 0xA0;
	if (adcPin & 0x1) {
		// Set SGL and SIGN to TRUE
		muxAddr |= 0x18;
	}
	else {
		// Set SGL to TRUE and SIGN to FALSE
		muxAddr |= 0x10;
	}
	// Throw away the LSB
	adcPin = adcPin >> 1;
	// OR it with 1_0_EN_SGL_SIGN -- we now have the translated mux address
	muxAddr |= adcPin;
	#ifdef ADC_LTC2418
	result = readInput_LTC2418(muxAddr);
	#endif
	#ifdef ADC_LTC2439
	result = readInput_LTC2439(muxAddr);
	#endif
	return(result);

}

uint8_t testEOC_LTC2439() {
	uint8_t result;
	wakeADC();
	result = spiTransferBit(0);
	result |= (spiTransferBit(0) << 1);
	if ((result & 0xb11) == 0b01) {
		serialMsgFmt("Second bit from LTC2439 was 1, this should always be 0");
		return(-1);
	}
	if ((result & 0xb11) == 0b10) {
		serialMsgFmt("EOC incomplete, wait for EOC to be complete..");
		return(-1);
	}
	if ((result & 0xb11) == 0xb00) {
		serialMsgFmt("EOC complete, ready to go!");
		_delay_ms(100);
		return(0);
	}
	_delay_ms(100);
	return(0);
}

uint_fast32_t readInput_LTC2439(uint8_t muxAddr) {
	unsigned char spiMsg[2] = {muxAddr,0xC0};
	unsigned char spiRetMsg[2];
	unsigned char x;
	unsigned char eocResult;
	uint_fast32_t rVal = 0;
	wakeADC();
	USICR = (1<<USIWM0)|(1<<USICS0);
	eocResult = spiTransferBit(1);
	eocResult |= (spiTransferBit(0) << 1);
	spiTransferMulti(spiMsg, spiRetMsg, SPI_MESSAGE_LENGTH);
	spiFlags |= SPI_WAIT_MSG;
	serialMsgFmt("Waiting for SPI transfer to complete..");
	while (!(spiFlags & SPI_COMPLETE_MSG ));
	spiFlags &= ~SPI_WAIT_MSG;
	serialMsgFmt("SPI Transfer complete, value: %x", spiRetMsg);
	serialMsgFmt("EOC Result value: %x", eocResult);
	for (x = 0; x < SPI_MESSAGE_LENGTH; x++) {
		if (x) rVal |= bufUSIDR[x] << (8 * x);
		else rVal |= bufUSIDR[x];
	}
	x = (bufUSIDR[SPI_MESSAGE_LENGTH - 1] & 0x3E) >> 1;
	if (x == muxAddr) {
		return(rVal);
	}
	else {
		serialMsgFmt("x did not equal muxAddr.. %x %x\n", x, muxAddr);
	}
	return(0);
}

uint_fast32_t readInput_LTC2418(uint8_t muxAddr) {
	unsigned char spiMsg[4] = {muxAddr,0x0,0x0,0x0};
	unsigned char spiRetMsg[4];
	unsigned char x;
	uint_fast32_t rVal = 0;
	spiTransferMulti(spiMsg, spiRetMsg, SPI_MESSAGE_LENGTH);
	spiFlags |= SPI_WAIT_MSG;
	serialMsgFmt("Waiting for SPI transfer to complete..");
	while (!(spiFlags & SPI_COMPLETE_MSG ));
	spiFlags &= ~SPI_WAIT_MSG;
	serialMsgFmt("SPI Transfer complete, value: %x", spiRetMsg);
	for (x = 0; x < SPI_MESSAGE_LENGTH; x++) {
		rVal |= bufUSIDR[x] << (8 * x);
	}
	x = (bufUSIDR[SPI_MESSAGE_LENGTH - 1] & 0x3E) >> 1;
	if (x == muxAddr) {
		return(rVal);
	}
	else {
		serialMsgFmt("x did not equal muxAddr.. %x %x", x, muxAddr);
	}
	return(0);
}


uint8_t spiTransferMulti(unsigned char *string, unsigned char *retString, unsigned char len) {
	unsigned char x = len;
	while (x) {
		retString[x-1] = spiTransfer8(string[len - x]);
		x--;
	}
	return(x);
}

uint8_t spiTransfer8(uint8_t data) {
	serialMsgFmt("Sending a byte of data over USI-SPI: %x", data);
	USIDR = data;
	USISR = _BV(USIOIF); // clear flag
	while ( (USISR & _BV(USIOIF)) == 0 ) {
		USICR = (1<<USIWM1) | (1<<USICS1) | (1<<USICLK) | (1<<USITC);
	}
	data = USIDR;
	serialMsgFmt("Shifted in a byte from USI-SPI: %x", data);
	return(data);
	/*
	USISR = (1<<USIOIF);             // clear IRQ
	USICR = (1<<USIWM0)|(1<<USICS0); // start clock
	while ((USISR & (1<<USIOIF)) == 0); // send 8 bits
	USICR = (1<<USIWM0)|(0<<USICS0); // stop clock after
	*/
}

uint8_t spiTransferBit(uint8_t data) {
	serialMsgFmt("Sending a bit of data over USI-SPI: %x", data);
	USIDR = data;
	USISR = _BV(USIOIF); // clear flag
	USICR = (1<<USIWM0) | (1<<USICS1) | (1<<USICLK) | (1<<USITC);
	data = USIDR;
	serialMsgFmt("Shifted in a bit from USI-SPI: %x", data);
	return(data);
	/*
	USISR = (1<<USIOIF);             // clear IRQ
	USICR = (1<<USIWM0)|(1<<USICS0); // start clock
	while ((USISR & (1<<USIOIF)) == 0); // send 8 bits
	USICR = (1<<USIWM0)|(0<<USICS0); // stop clock after
	*/
}

void USART_Init(uint_fast16_t desiredBaud ) {
	/* Set baud rate */
	uint_fast16_t baud = ((8000000L / (desiredBaud * 16L)) - 1);
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0L = (unsigned char)baud;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (1<<USBS0) | (1<<UCSZ00) | (1<<UCSZ01); 
}

