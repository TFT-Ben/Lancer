//Aston Lancer mk1 Avionics code
//Authors:  Ben Cartwright, Thomas Lewis, Matthew Marriott, Sion Abraham
//started: 05/02/2016
//v1.0
//designed for use with the ATmega328P

//******************
//   LIBRAriES
//******************

#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/crc16.h>
#include <avr/wdt.h>													//watchdog timer !!write gps error handling

//******************
//   DEFINITIONS
//******************

//defines CPU clock such that 1s = 8m clock cycles
#ifndef F_CPU
#define F_CPU 8000000UL
#endif

//GPS_USART
#define GPS_BAUD 9600													//defines baud rate
#define UBRR_VALUE (((F_CPU / (GPS_BAUD * 16UL))) - 1)					//defines UBBR value to achieve GPS_BAUD at F_CPU speed

//GPS
#define GPSEN PC1														//defines GPS enable pin

//Radio
#define reload 45536													//defines the reload value for the overflow timer at 8MHz, prescale 1/8, 50 baud
#define RDEN PC0														//defines RADIO enable pin
#define RDTx PD2														//defines RADIO Tx pin

#define LED PC2															//LED for debugging

//******************
//   PROTOTYPES
//******************

void GPS_TxChar(uint8_t ch);											//transmit one character via built in USART
void RADIO_TxBit(int bit);												//transmit one bite via software serial to radio
void GPS_ParseSentence(uint8_t *String);					//parse a sentence of NMEA ($xxGGA)
void GPS_Init();														//initialise gps
void GPS_HandleRx();													//trigger when GPS pins receive signal
void RADIO_HandleTx();													//handle transmission of datastring
uint16_t RADIO_checksum(char *String);					//calculate CRC16 checksum for the transmission string
void buildDatastring();									//build the transmission datastring from the latest telemetry

//******************
//   FIELDS
//******************

//GPS
volatile int gpsRxFLAG = 0;												//flag for GPS Rx being available
volatile char gpsRxChar;												//storage for received character
bool gpsInitialised = false;											//false until airborne dynamic mode successful

uint8_t b;																//these variables for gps init
uint8_t ackByteID = 0;
uint8_t ackPacket[10];

uint8_t gpsString[82];									//GPS string receive buffer.  82 is max NMEA sentence size
uint8_t gpsStringIndex;									//variable to store where we are in the gpsString
bool gpsStartSynced = false;											//variable to sync up start of sentence $ - default unsynced
uint8_t gpsLatestReading[82];											//variable to store latest reading of NMEA sentence we want
bool latestGPSParsed;													//variable so we only parse a new reading once

uint8_t gpsTime[9];														//time read from GPS
uint8_t gpsLat[10];														//latitude
char gpsLatDir;															//latitude direction
uint8_t gpsLon[10];														//longitude
char gpsLonDir;															//longitude direction
uint8_t gpsNumSat[2];													//number of satellites
uint8_t gpsAlt[7];														//altitude
char gpsAltUnit;														//altitude unit
uint32_t gpsGeoSep[7];													//geoidal separation
char gpsGeoSepUnit;														//geoidal separation unit

//radio
char tx_datastring[256];												//final transmission string
uint8_t messageID = 0;													//message ID
char callsign[5] = "ASTL1";												//craft callsign
volatile int radioTxFlag = 0;											//if we need to transmit datastring bit

int currentCharacter = 0;																//index variable for the datastring
int currentBit = 0;																//bit index variable for the specific char in the datastring

//******************
//   MAIN
//******************

int main(void) {
	
	cli();																//make sure interrupts disabled during setup
	
	//******************
	//   I/O INIT
	//******************
	
	DDRC = 0x07;														//set PORTC PC0 (TxEN) and PC1 (GPSEN) and PC2 (LED) to output
	DDRD = 0x04;														//set PORTD PD2 (Tx) to output
	
	//******************
	//   TIMER INIT
	//******************
	
	TCNT1 = reload; 													//load reload into counter register
	TIMSK1 = 1<<TOIE1; 													//enable overflow interrupt
	TCCR1B = 1<<CS11; 													//enable clock prescale 1/8
	
	//******************
	//   GPS INIT
	//******************
	
	UBRR0 = UBRR_VALUE;													//sets baud rate
	UCSR0B |= 1<<RXEN0 | 1<<TXEN0 | 1<<RXCIE0;							//enables USART TX and RX - enables interrupt on Rx complete
	UCSR0C |= 1<<USBS0 | 1<<UCSZ01 | 1<<UCSZ00;							//configure USART sentence - asynchronous, 2 stop bits, 8 data bits
	gpsStringIndex = 0;													//begin at the start of the input string
	
	sei();																//enable global interrupts
	
	PORTC |= 1<<RDEN;													//initialise radio module
	GPS_Init();															//initialise GPS module for airborne dynamic mode

	//******************
	//   LOOP
	//******************
	
	while(1) {
		if(gpsInitialised) {
			PORTC |= 1<<LED;
		}
		
		GPS_HandleRx();
		
		if(gpsStartSynced) {											//if currently reading
			if(gpsStringIndex >= 6 && !(gpsString[0] == '$' && gpsString[1] == 'G' && gpsString[3] == 'G' && gpsString[4] == 'G' && gpsString[5] == 'A')) {
				gpsStartSynced = false;									//stop reading in bits if it's not the sentence we want to save time
			}															//due to this gpsLatestReading can only contain GxGGA
		}
		if(!latestGPSParsed) {											//if we haven't parsed this reading yet
			GPS_ParseSentence(gpsLatestReading);						//parse data out of sentence
		}
	}
}

//******************
//   OTHER FUNCTIONS
//******************

void GPS_TxChar(uint8_t ch) {
	while(!(UCSR0A & 1<<UDRE0)) {										//check to see if Tx buffer is clear
		UDR0 = ch;														//write char to buffer
	}
}

void GPS_ParseSentence(uint8_t *String) {								//parse NMEA sentence, comma delimited
	latestGPSParsed = true;												//parsing being completed

	int i, j, k;														//these will help parse each section
	
	for(i = 7, j = 1, k = 0; i < sizeof(gpsString) && j < 9; i++) {		//i set to 7 so we start at the actual data
		if(String[i] == ',') {											//comma delimiter
			j++;														//j designates data section
			k = 0;														//k is the index into that data section
		}
		else {
			switch(j) {
				case 1 :												//gps time
					gpsTime[k++] = String[i];
					break;
				case 2 :												//latitude
					gpsLat[k++] = String[i];
					break;
				case 3 :												//latitude direction
					gpsLatDir = String[i];
					break;
				case 4 :												//longitude
					gpsLon[k++] = String[i];
					break;
				case 5 :												//longitude direction
					gpsLonDir = String[i];
					break;
				case 7 :												//number of satellites
					gpsNumSat[k++] = String[i];
					break;
				case 9 :												//altitude
					gpsAlt[k++] = String[i];
					break;
				case 10 :												//altitude unit
					gpsAltUnit = String[i];
					break;
				case 11 : 												//geoidal separation
					gpsGeoSep[k++] = String[i];
					break;
				case 12 : 												//geoidal separation unit
					gpsGeoSepUnit = String[i];
					break;
			}
		}
	}
}

void GPS_Init() {
	uint8_t setdm6[] = {
	 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
	 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
	 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
	 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
	 
																		//construct the expected ACK packet
	ackPacket[0] = 0xB5;												//header
	ackPacket[1] = 0x62; 												//header
	ackPacket[2] = 0x05; 												//class
	ackPacket[3] = 0x01; 												//id
	ackPacket[4] = 0x02; 												//length
	ackPacket[5] = 0x00;
	ackPacket[6] = setdm6[2]; 											//ACK class
	ackPacket[7] = setdm6[3]; 											//ACK id
	ackPacket[8] = 0; 													//CK_A
	ackPacket[9] = 0; 													//CK_B
	 
	for (uint8_t ubxi=2; ubxi<8; ubxi++) {								//calculate the checksums
		ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
		ackPacket[9] = ackPacket[9] + ackPacket[8];
	}
	
	PORTC |= 1<<GPSEN;													//set GPSEN high
	
	int len = sizeof(setdm6)/sizeof(uint8_t);							//set length of setup string
	
	for(int i = 0; i < len; i++) {
		GPS_TxChar(setdm6[i]);											//send setup bytes
	}
}

void GPS_HandleRx() {														//handle GPS input
	if(gpsRxFLAG == 1) {													//new Rx data received
		gpsRxFLAG = 0;														//reset flag
		
		if(!gpsInitialised) {												//if not set up		
			//!!timeout
			
			b = gpsRxChar;
			if(b == ackPacket[ackByteID]) {									//check each packet against the expected message
				ackByteID++;												//check next packet if successful
			}
			else {
				ackByteID = 0;												//reset and look again if not
				GPS_Init();													//!!not needed when timeout implemented
			}
			if(ackByteID > 9) {												//if all packets are in order
				gpsInitialised = true;
			}
		}
		else {		
			if(gpsStartSynced == false) {									//if we're in the middle of some other string
				if(gpsRxChar == '$') {										//and we detect the start of an NMEA sentence
					gpsStartSynced = true;									//begin to read
				}
			}
			
			if (gpsStartSynced) {													//if we're at the start of the string
				gpsString[gpsStringIndex++] = gpsRxChar;							//read each bit into gpsString input buffer
				
				if(gpsStringIndex == sizeof(gpsString) || gpsRxChar == '*') {		//if the string is full or the sentence ends
					gpsStringIndex = 0;												//return to start of string
					gpsStartSynced = false;											//wait until next sentence starts  !!ignores checksum
					memcpy(gpsLatestReading, gpsString, sizeof gpsLatestReading);	//copy the complete sentence to a new location for parsing
					latestGPSParsed = false;										//signal that it's a new result
				}
			}
		}
	}
}

uint16_t RADIO_checksum (char *String) {
	size_t i;
	uint16_t crc;
	uint8_t c;
	 
	crc = 0xFFFF;
	 
	for (i = 2; i < strlen(String); i++) { 											//calculate checksum ignoring the first two $s
		c = String[i];
		crc = _crc_xmodem_update (crc, c);
	}
	 
	return crc;
}

void RADIO_TxBit(int bit) {
	if(bit) {
		//high
		PORTD |= 1<<RDTx;
	}
	else {
		//low
		PORTD &= ~(1<<RDTx);
	}
}

void RADIO_HandleTx() {	
	if(radioTxFlag) {
		radioTxFlag = 0;
		
		switch(currentBit) {
			case 0 :
				RADIO_TxBit(0);												//start bit
				currentBit++;
				break;
			case 9 :
				RADIO_TxBit(1);												//stop bit
				currentBit++;
				break;
			case 10 :
				RADIO_TxBit(1);												//stop bit
				currentBit = 0;
				ri++;
				break;
			default :
				{
				char ch = tx_datastring[ri];								//get the datastring char we're at
				ch = ch >> (currentBit - 1);										//shift it however many places
				
				if(ch & 1) {
					RADIO_TxBit(1);
				}
				else {
					RADIO_TxBit(0);
				}
				currentBit++;
				}
		}
		
		if(currentBit == 10 && currentCharacter == strlen(tx_datastring)) {						//if we've just transmitted the last bit of the datastring
			buildDatastring();												//generate new datastring
		}
	}
}

void buildDatastring() {
	memset(tx_datastring, 0, sizeof(tx_datastring));					//reset tx_datastring to null
	char interimData[256] = { 0 };										//string to hold data for checksum calc
	char checksum[10];													//string to hold checksum

																		//build interim data string
	snprintf(interimData, sizeof(interimData), "$$%s,%i,%s,%s%c,%s%c,%s%c", callsign, messageID, gpsTime, gpsLat, gpsLatDir, gpsLon, gpsLonDir, gpsAlt, gpsAltUnit);	
	
																		//calculate checksum
	snprintf(checksum, sizeof(checksum), "*%04X\n", RADIO_checksum(interimData));

																		//append data and checksum to transmission buffer
	snprintf(tx_datastring, sizeof(tx_datastring), "%s%s", interimData, checksum);
}

//******************
//   INTERRUPTS
//******************

ISR(USART_RX_vect) {													//this interrupt reads in GPS data from the USART
	gpsRxChar = UDR0;													//read the received character
	gpsRxFLAG = 1;														//raise flag
}

ISR(TIMER1_OVF_vect) {													//this interrupt sets the flag alerting the program
	radioTxFlag = 1;													//to transmit the next datastring bit
}
