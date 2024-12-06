// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// Copyright (c) 2024 Upside Down Labs - contact@upsidedownlabs.tech
// Author: Deepak Khatri
//
// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

#include <Arduino.h>

// Definitions
#define NUM_CHANNELS 6									// Number of channels supported
#define HEADER_LEN 3									// Header = SYNC_BYTE_1 + SYNC_BYTE_2 + Counter
#define PACKET_LEN (NUM_CHANNELS * 2 + HEADER_LEN + 1)	// Packet length = Header + Data + END_BYTE
#define SAMP_RATE 250.0									// Sampling rate (250 for UNO R3)
#define SYNC_BYTE_1 0xC7								// Packet first byte
#define SYNC_BYTE_2 0x7C								// Packet second byte
#define END_BYTE 0x01									// Packet last byte
#define BAUD_RATE 230400								// Serial connection baud rate

// Global constants and variables
uint8_t packetBuffer[PACKET_LEN];	// The transmission packet
uint8_t currentChannel;				// Current channel being sampled
uint16_t ADCValue = 0;				// ADC current value
bool timerStatus = false;			// SATUS bit
bool bufferReady = false;			// Buffer ready status bit

bool timerStart()
{
	timerStatus = true;
	digitalWrite(LED_BUILTIN, HIGH);
	// Enable Timer1 Compare A interrupt
	return TIMSK1 |= (1 << OCIE1A);
}

bool timerStop()
{
	timerStatus = false;
	bufferReady = false;
	digitalWrite(LED_BUILTIN, LOW);
	// Disable Timer1 Compare A interrupt
	return TIMSK1 &= ~(1 << OCIE1A);
}

// ISR for Timer1 Compare A match (called based on the sampling rate)
ISR(TIMER1_COMPA_vect)
{
	if (!timerStatus or Serial.available())
	{
		timerStop();
		return;
	}

	// Read 6ch ADC inputs and store current values in packetBuffer
	for (currentChannel = 0; currentChannel < NUM_CHANNELS; currentChannel++)
	{
		ADCValue = analogRead(currentChannel);									   // Read Analog input
		packetBuffer[((2 * currentChannel) + HEADER_LEN)] = highByte(ADCValue);	   // Write High Byte
		packetBuffer[((2 * currentChannel) + HEADER_LEN + 1)] = lowByte(ADCValue); // Write Low Byte
	}

	// Increment the packet counter
	packetBuffer[2]++;

	// Set bufferReady status bit to true
	bufferReady = true;
}

bool timerBegin(float sampling_rate)
{
	cli(); // Disable global interrupts

	// Calculate the time interval in seconds from the sampling rate
	// Interval = 1 / sampleRateHz
	float intervalSeconds = 1.0 / sampling_rate;

	// Calculate OCR1A based on the interval
	// OCR1A = (16MHz / (Prescaler * Desired Time)) - 1
	// Prescaler options: 1, 8, 64, 256, 1024
	unsigned long ocrValue = (16000000 / (64 * sampling_rate)) - 1;

	// Configure Timer1 for CTC mode (Clear Timer on Compare Match)
	TCCR1A = 0; // Clear control register A
	TCCR1B = 0; // Clear control register B

	// Set CTC mode (WGM12 bit) and set the prescaler to 64
	TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); // Prescaler = 64

	// Set the calculated value in OCR1A register
	OCR1A = ocrValue;

	sei(); // Enable global interrupts
}

void setup()
{

	Serial.begin(BAUD_RATE);
	while (!Serial)
	{
		; // Wait for serial port to connect. Needed for native USB
	}

	// Status LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Initialize packetBuffer
	packetBuffer[0] = SYNC_BYTE_1;			// Sync 0
	packetBuffer[1] = SYNC_BYTE_2;			// Sync 1
	packetBuffer[2] = 0;					// Packet counter
	packetBuffer[PACKET_LEN-1] = END_BYTE;	// End Byte

	// Setup timer
	timerBegin(SAMP_RATE);
}

void loop()
{
	if(timerStatus and bufferReady){
		// Send Packet over serial
		Serial.write(packetBuffer, PACKET_LEN);
		bufferReady = false;
	}

	if (Serial.available())
	{
		// Read command
		String command = Serial.readString();
		command.trim();

		// Who are you?
		if (command == "WHORU")
		{
			Serial.println("UNO-R3");
		}

		// Start data acquisition
		if (command == "START")
		{
			timerStart();
		}

		// Stop data acquisition
		if (command == "STOP")
		{
			timerStop();
		}

		// Get status
		if (command == "STATUS")
		{
			if (timerStatus)
			{
				Serial.println("START");
			}
			else
			{
				Serial.println("STOP");
			}
		}
	}
}
