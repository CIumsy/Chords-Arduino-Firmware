
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

#include "FspTimer.h"
#include <Arduino.h>

// Definitions
#define NUM_CHANNELS 6								   // Number of channels supported
#define HEADER_LEN 3								   // Header = SYNC_BYTE_1 + SYNC_BYTE_2 + Counter
#define PACKET_LEN (NUM_CHANNELS * 2 + HEADER_LEN + 1) // Packet length = Header + Data + END_BYTE
#define SAMP_RATE 500.0								   // Sampling rate (250/500 for UNO R4)
#define SYNC_BYTE_1 0xC7							   // Packet first byte
#define SYNC_BYTE_2 0x7C							   // Packet second byte
#define END_BYTE 0x01								   // Packet last byte
#define BAUD_RATE 230400							   // Serial connection baud rate

// Global constants and variables
uint8_t packetBuffer[PACKET_LEN];	// The transmission packet
uint8_t currentChannel;				// Current channel being sampled
uint16_t ADCValue = 0;				// ADC current value
bool timerStatus = false;			// Timer status bit
bool bufferReady = false;			// Buffer ready status bit			

FspTimer ChordsTimer;

bool timerStart()
{
	timerStatus = true;
	digitalWrite(LED_BUILTIN, HIGH);
	return ChordsTimer.start();
}

bool timerStop()
{
	timerStatus = false;
  bufferReady = false;
	digitalWrite(LED_BUILTIN, LOW);
	return ChordsTimer.stop();
}

// callback method used by timer
void timerCallback(timer_callback_args_t __attribute((unused)) * p_args)
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
	uint8_t timer_type = GPT_TIMER;
	int8_t timer_channel = FspTimer::get_available_timer(timer_type);
	if (timer_channel != -1)
	{
		ChordsTimer.begin(TIMER_MODE_PERIODIC, timer_type, timer_channel, sampling_rate, 0.0f, timerCallback);
		ChordsTimer.setup_overflow_irq();
		ChordsTimer.open();
		return true;
	}
	else
	{
		return false;
	}
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
	packetBuffer[0] = SYNC_BYTE_1; // Sync 0
	packetBuffer[1] = SYNC_BYTE_2; // Sync 1
	packetBuffer[2] = 0;		   // Packet counter
	packetBuffer[PACKET_LEN-1] = END_BYTE;   // End Byte

	timerBegin(SAMP_RATE);

	analogReadResolution(14);
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
			Serial.println("UNO-R4");
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
