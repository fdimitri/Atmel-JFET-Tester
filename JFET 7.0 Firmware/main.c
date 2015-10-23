/*
 * ATTiny1634Test.c
 *
 * Created: 10/1/2015 2:33:47 PM
 * Author : FrankD
 */ 
#define F_CPU 8000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#undef ADC_LTC2418
#define ADC_LTC2439

#define FET_DDR DDRA
#define FET_PORT PORTA
#define FET_PIN 4

#define ADC_CSDDR DDRA
#define ADC_CSPORT PORTA
#define ADC_CSPIN 3

#define ADC_SDDDR DDRB
#define ADC_SDPORT PORTB
#define ADC_SDPINR PINB

#define ADC_SDIPIN 1
#define ADC_SDOPIN 2

#define ADC_SCKDDR DDRC
#define ADC_SCKPORT PORTC
#define ADC_SCKPIN 1

struct adcTable {
	uint8_t adcType;
	uint16_t adcPortAddr;
	uint8_t adcPin;
};
#define ADC_CSBV (1 << ADC_CSPIN)
#define ADC_SDIBV (1 << ADC_SDIPIN)
#define ADC_SDOBV (1 << ADC_SDOPIN)
#define ADC_SCKBV (1 << ADC_SCKPIN)
#define FET_PINBV (1 << FET_PIN)
#define ADC_CSHIGH ADC_CSPORT |= ADC_CSBV
#define ADC_CSLOW ADC_CSPORT &= ~ADC_CSBV
#define ADC_SDOHIGH ADC_SDPORT |= ADC_SDOBV
#define ADC_SDOLOW ADC_SDPORT &= ~ADC_SDOBV
#define ADC_SCKHIGH ADC_SCKPORT |= ADC_SCKBV
#define ADC_SCKLOW ADC_SCKPORT &= ~ADC_SCKBV
#define ADC_SCKPULSE ADC_SCKHIGH; ADC_SCKLOW
#define FET_HIGH FET_PORT |= FET_PINBV
#define FET_LOW FET_PORT &= ~FET_PINBV

#define ADC_SDIVAL ((ADC_SDPINR & ADC_SDIBV) ? 1 : 0)
#define ADC_CSVAL ((ADC_CSPORT & ADC_CSBV) ? 1 : 0)
#define ADC_SCKVAL ((ADC_SCKPORT & ADC_SCKBV) ? 1 : 0)

#define ADC_INPUTS 16
#define AREF_VOLTAGE 5.0
#define RDIV1 10000.0
#define RDIV2 14700.0
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



void serialMsgFmt(const char *fmt, ...);
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
uint8_t testEOC_LTC2439();
void delay_ms(uint_fast32_t ms);
char serialWaitForChar();
void serialSendByte(char b);

//char binString[] = "%d%d%d%d%d%d%d%d";
//unsigned char bufUSIDR[4];
uint_fast32_t results[ADC_INPUTS];
unsigned char spiFlags;
char genBuf[64];

void delay_ms(uint_fast32_t ms) {
	while (--ms) {
		_delay_ms(1);
	}
}

int main(void) {
	ADC_CSHIGH;
	delay_ms(200);
	setup();
    while (1) {
		loop();
    }
}

void setup(void) {
	ADC_SDDDR |= ADC_SDOBV;
	ADC_SDDDR &= ~ADC_SDIBV;
	FET_DDR |= FET_PINBV;
	ADC_SCKDDR |= ADC_SCKBV;
	ADC_CSDDR |= ADC_CSBV;
	USART_Init((uint_fast16_t) 9600);
	serialMsgFmt("Finished with setup!");
	serialMsgFmt("DDRA: %d", DDRA);
	serialMsgFmt("DDRB: %d", DDRB);
	serialMsgFmt("DDRC: %d", DDRC);
	delay_ms(1000);
}

#define BUF_SIZE 16

uint_fast32_t waitForInput() {
	char c;
	uint_fast32_t rVal;
	char buf[BUF_SIZE];
	char *r;
	unsigned char x = 0;
	r = (char *) &rVal;
	serialMsg("> ");
	while (c != '\n' && c != '\r' && x < BUF_SIZE) {
		c = serialWaitForChar();
		buf[x++] = c;
		serialSendByte(c);
	}
	if (buf[0] == 'i') {
		*(r + 1) = MODE_IDSS;
		
	}
	else if (buf[0] == 'v') {
		*(r + 1) = MODE_VP;
	}
	*(r) = buf[1] - '0';
	if ('0' <= buf[2] && buf[2] <= '9') {
		if (buf[1] == '1')	*(r) += 9;
		else *(r) = 0;
		*(r) += buf[2] - '0';
	}
	serialMsg("\r\n");
	return(rVal);
}

char serialWaitForChar() {
	while(!(UCSR0A & (1<<RXC0)));
	return(UDR0);
}

void serialSendByte(char b) {
	while (!(UCSR0A & (1<<UDRE0)));
	// set data into data register
	UDR0 = b;
}


void loop(void) {
	uint_fast32_t cmdType;
	char *r;
	cmdType = waitForInput();
	r = (char *) &cmdType;
	if (*(r+1) == MODE_IDSS) setMode(MODE_IDSS);
	if (*(r+1) == MODE_VP) setMode(MODE_VP);
	readInput(*r);
	
	/*setMode(MODE_IDSS);
	readAllInputs();
	setMode(MODE_VP);
	readAllInputs();
	delay_ms(200);*/
}


void setMode(uint8_t newMode) {
	switch(newMode) {
		case MODE_IDSS:
			FET_HIGH;
		break;
		case MODE_VP:
			FET_LOW;
		break;
	}
}


void serialMsg(char *msg) {
	while (*msg) {
		serialSendByte(*msg++);
	}
	serialSendByte('\r');
	serialSendByte('\n');
	return;
}

void serialMsgFmt(const char *fmt, ...) {
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
	delay_ms(50);
	for (adcPin = 1; adcPin <= ADC_INPUTS; adcPin++) {
		results[adcPin - 1] = readInput(adcPin);
	}
}

uint_fast32_t readInput(uint8_t adcPin) {
	uint_fast32_t result;
	serialMsgFmt("Entered readInput for pin %d", adcPin);
	adcPin -= 1; // ie 16 Becomes 15
	uint8_t muxAddr;
	// Set 0b10000000 mandatory, b00000XX1 to select new address
	muxAddr = 0xA0;
	if (adcPin & 0x1) {
		// Set SGL and SIGN to TRUE
		muxAddr |= 0x18;
	}
	else {
		// Set SGL to TRUE and SIGN to FALSE
		muxAddr |= 0x10;
	}
	// Throw away the LSB and shift right
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
	uint8_t rVal;
	ADC_SCKLOW;
	ADC_CSHIGH;
	ADC_CSLOW;
	delay_ms(1024);
	rVal = ADC_SDIVAL;
	ADC_CSHIGH;
	return(rVal);
}

uint_fast32_t readInput_LTC2439(uint8_t muxAddr) {
	unsigned char spiMsg[4] = {0, 0, 0, 0};
	unsigned char spiRetMsg[4] = {0, 0, 0, 0};
	unsigned char result;
	uint_fast32_t rVal = 0;
	uint_fast32_t *testMsg = (uint_fast32_t *) &spiRetMsg[0];
	//uint_fast32_t *sendMsg = (uint_fast32_t *) &spiMsg[0];
	uint_fast32_t tMsg = 0;
	spiMsg[2] = muxAddr;
	
	//serialMsgFmt("Entered LTC2439 Read Input Routine");
	//serialMsgFmt("Looping until we get valid EOC");
	while (testEOC_LTC2439()) { result++; }
		
	//serialMsgFmt("Sending message: 0x%lX", *sendMsg);
	spiTransferMulti(spiMsg, &spiRetMsg[0], 3);
	//*testMsg &= ~(0x1F);
	tMsg = 0;
	tMsg |= spiRetMsg[0];
	tMsg |= (((uint_fast32_t) spiRetMsg[1]) << 8);
	tMsg |= (((uint_fast32_t) spiRetMsg[2]) << 16);

	rVal = *testMsg;

//	serialMsgFmt("SPI Transfer complete, raw value: 0x%lX, 0x%lX", *testMsg, tMsg);
	*testMsg = *testMsg >> 5;		// Discard 5 bits of junk because we did a 24 bit transfer for a 19 bit result
	*testMsg &= 0xFFFF;				// Discard EOC and "0" which shouldn't affect our result anyway -- although we also throw away "SIG"
	if (*testMsg & (1L << 16)) {
		// Result is negative
		*testMsg = *testMsg | ~(0xFFFF);
	}
	tMsg >>= 5;
	tMsg &= 0xFFFF;
	
//	serialMsgFmt("SPI Transfer complete, raw value: 0x%lX, 0x%lX", *testMsg, tMsg);
	
//	serialMsgFmt("SPI Transfer complete, value: 0x%lX -- 0x%X -- 0x%X -- 0x%lX", *testMsg, spiRetMsg[0], spiRetMsg[1], rVal);
	
	while (testEOC_LTC2439()) { result++; }
	
//	serialMsgFmt("Sending message: 0x%lX", *sendMsg);
	spiTransferMulti(spiMsg, &spiRetMsg[0], 3);
	tMsg = 0;
	tMsg |= spiRetMsg[2];
	tMsg |= (((uint_fast32_t) spiRetMsg[1]) << 8);
	tMsg |= (((uint_fast32_t) spiRetMsg[0]) << 16);
//	serialMsgFmt("SPI Transfer complete, raw value: 0x%lX, 0x%lX", *testMsg, tMsg);

	*testMsg = *testMsg >> 5;		// Discard 5 bits of junk because we did a 24 bit transfer for a 19 bit result
	if ((*testMsg & 0x18000) == 0x18000) {
		serialMsg("SM: 11");
	}
	else if ((*testMsg & 0x18000) == 0x08000) {
		serialMsg("SM: 01");
	}
	else if ((*testMsg & 0x18000) == 0x10000) {
		serialMsg("SM: 10");
	}
	if (*testMsg & (1L << 16)) {
		// Result is negative
		*testMsg = (*testMsg  | ~0xFFFF);			// Took this from somewhere else, it's obviously wrong.
	}

	*testMsg &= 0xFFFF;				// Discard EOC and "0" and SIG which shouldn't affect our result anyway
	
	tMsg = tMsg >> 5;
	tMsg &= 0xFFFF;
	rVal = *testMsg;
//	serialMsgFmt("tMsg after shift and mask: 0x%lX", tMsg);
//	serialMsgFmt("SPI Transfer complete, value: 0x%lX -- 0x%X -- 0x%X -- 0x%lX -- 0x%X", *testMsg, spiRetMsg[0], spiRetMsg[1], rVal, tMsg);
	serialMsgFmt("rVal: 0x%X", rVal);
	
	float iVal = rVal * AREF_VOLTAGE;
	iVal *= (RDIV2 + RDIV1) / RDIV1;
	iVal /= 65535.0;
	serialMsgFmt("%f", iVal);
	
	return(rVal);
}

uint_fast32_t readInput_LTC2418(uint8_t muxAddr) {
	unsigned char spiMsg[4] = {muxAddr,0x0,0x0,0x0};
	unsigned char spiRetMsg[4];
	unsigned char x;
	uint_fast32_t rVal = 0;
	//serialMsgFmt("Entered LTC2418 Read Input Routine");
	spiTransferMulti(spiMsg, spiRetMsg, SPI_MESSAGE_LENGTH);
	spiFlags |= SPI_WAIT_MSG;
	//serialMsgFmt("Waiting for SPI transfer to complete..");
	while (!(spiFlags & SPI_COMPLETE_MSG ));
	spiFlags &= ~SPI_WAIT_MSG;
	//serialMsgFmt("SPI Transfer complete, value: %x", spiRetMsg);
	for (x = 0; x < SPI_MESSAGE_LENGTH; x++) {
		//rVal |= bufUSIDR[x] << (8 * x);
	}
	//x = (bufUSIDR[SPI_MESSAGE_LENGTH - 1] & 0x3E) >> 1;
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
	//serialMsgFmt("spiTransferMulti bytes:");
	while (x != 0) {
		x--;
		retString[x] = spiTransfer8(string[x]);
		//serialMsgFmt("spiTransferMulti(): Byte %d is 0b%d%d%d%d%d%d%d%d", x, MAKE_BINARY_STRING(retString[x]));
	}
	ADC_CSHIGH;
	return(x);
}

uint8_t spiTransfer8(uint8_t data) {
	ADC_CSLOW;
	uint8_t b = 0;
	uint8_t x;
	for (x = 7; x != 255; x--) {
		//serialMsgFmt("Shifting out bit: %x..", (data & (1 << x)) ? 1 : 0);
		b |= spiTransferBit(data & (1 << x) ? 1 : 0) << x;
	}
	return(b);
}

/* LTC2439: The serial clock signal present on SCK (Pin 18) is used to
synchronize the data transfer. Each bit of data is shifted
out the SDO pin on the falling edge of the serial clock and
each input bit is shifted in the SDI pin on the rising edge
of the serial clock. */

uint8_t spiTransferBit(uint8_t data) {
	uint8_t portVal = ADC_SDPORT;
	//char *valFmt = "Port: %x; SDI: %x; CSVAL: %x, SCK: %x";
	if (data) {
		portVal |= ADC_SDOBV;
	}
	else {
		portVal &= ~ADC_SDOBV;
	}
	ADC_SCKLOW;
	ADC_SDPORT = portVal;
	portVal = ADC_SDIVAL;
	ADC_SCKHIGH;
	return(portVal);
}

void USART_Init(uint_fast16_t desiredBaud ) {
	/* Set baud rate */
	uint_fast16_t baud = ((F_CPU / (desiredBaud * 16L)) - 1);
//	uint_fast16_t baud = 207; 1MHz 300bps
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0L = (unsigned char)baud;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (1<<USBS0) | (1<<UCSZ00) | (1<<UCSZ01); 
}

