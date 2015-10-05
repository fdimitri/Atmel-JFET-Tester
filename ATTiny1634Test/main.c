/*
 * ATTiny1634Test.c
 *
 * Created: 10/1/2015 2:33:47 PM
 * Author : FrankD
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define ADC_LTC2418
#undef ADC_LT2439

#define ADC_INPUTS 16

#define SPI_WAIT_MSG 0x01
#define SPI_SENDING_MSG 0x02
#define SPI_SENT_MSG 0x04
#define SPI_COMPLETE_MSG 0x08


void setup(void);
void loop(void);
void USART_Init(unsigned int baud);
uint8_t spiTransfer8(uint8_t data);
uint8_t spiTransferMulti(unsigned char *string, unsigned char *retString, unsigned char len);
uint_fast32_t readInput_LTC2418(uint8_t muxAddr);
uint_fast32_t readInput(uint8_t adcPin);
void readAllInputs(void);
void serialMsg(char *msg);
void serialMsgFmt(char *fmt, ...);


unsigned char bufUSIDR[4];
uint_fast32_t results[ADC_INPUTS];
unsigned char spiFlags;
char genBuf[256];
int main(void) {
	setup();
    while (1) {
		loop();
    }
}

void setup(void) {
	// SPI setup
	//USICR = (1<<USIOIE) | (1<<USIWM0) | (1<<USICS1) | (1<<USICLK) | (1<<USITC); //counter overflow interrupt enable, and 3 wire mode, external register clk
	USICR = (1<<USIOIE) | (1<<USIWM0) | (1<<USICS1) | (1<<USICS0) |(1<<USICLK);

	TCCR0A = (1<<WGM01) | 0x02;
	OCR0A = 31;
	
	USART_Init((unsigned int) 115200);
}

void loop(void) {
	unsigned char junkBytes[] = { 0x80, 0xFF, 0x00, 0x7F };
	unsigned char returnString[4];
	spiTransferMulti(&junkBytes[0], &returnString[0], 4);
	readAllInputs();
}

void serialSendByte(char b) {
	
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
	return;
}


void readAllInputs(void) {
	uint8_t adcPin;
	for (adcPin = 1; adcPin < ADC_INPUTS; adcPin++) {
		results[adcPin - 1] = readInput(adcPin);
	}
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
	return(result);

}

uint_fast32_t readInput_LTC2418(uint8_t muxAddr) {
	unsigned char spiMsg[4] = {muxAddr,0x0,0x0,0x0};
	unsigned char spiRetMsg[4];
	unsigned char x;
	uint_fast32_t rVal = 0;
	spiTransferMulti(spiMsg, spiRetMsg, 4);
	spiFlags |= SPI_WAIT_MSG;
	while (!(spiFlags & SPI_COMPLETE_MSG ));
	spiFlags &= ~SPI_WAIT_MSG;
	
	for (x = 0; x < 4; x++) {
		rVal |= bufUSIDR[x] << (8 * x);
	}
	x = (bufUSIDR[3] & 0x3E) >> 1;
	if (x == muxAddr) {
		return(rVal);
	}
	else {
		serialMsgFmt("x did not equal muxAddr.. %x %x\n", x, muxAddr);
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
	USIDR = data;
	USISR = _BV(USIOIF); // clear flag
	while ( (USISR & _BV(USIOIF)) == 0 ) {
		USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC);
	}
	return USIDR;
	/*
	USISR = (1<<USIOIF);             // clear IRQ
	USICR = (1<<USIWM0)|(1<<USICS0); // start clock
	while ((USISR & (1<<USIOIF)) == 0); // send 8 bits
	USICR = (1<<USIWM0)|(0<<USICS0); // stop clock after
	*/
}

void USART_Init( unsigned int baud ) {
	/* Set baud rate */
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0L = (unsigned char)baud;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}


ISR(USI_OVERFLOW_vect) {
	// We got a byte on SPI!
	static uint8_t x = 0;
	TIMSK &= ~(1<<OCIE0A);
	// Copy USIDR to buffer to prevent overwrite on next transfer.
	bufUSIDR[x] = USIDR;
	// Update flags and clear USI counter
	USISR = (1<<USIOIF);
	if (spiFlags & SPI_WAIT_MSG) {
		if (x < 3) ++x;
		else {
			spiFlags |= SPI_COMPLETE_MSG;
			x = 0;
		}
	}
}
