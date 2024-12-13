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
#include <Arduino_AdvancedAnalog.h>

// Definitions
#define NUM_CHANNELS 6                                  // Number of channels supported
#define HEADER_LEN 3                                    // Header = SYNC_BYTE_1 + SYNC_BYTE_2 + Counter
#define PACKET_LEN (NUM_CHANNELS * 2 + HEADER_LEN + 1)  // Packet length = Header + Data + END_BYTE
#define ADC_SAMPLING 16000                              // ADC sampling rate
#define ADC_QUEUE 256                                   // ADC Qeueue depth
#define SAMPLES_CHANNEL 32                              // Samples per channel
#define SAMP_RATE ADC_SAMPLING / SAMPLES_CHANNEL        // CHORDS Sampling rate (250/500 for GIGA R1 WiFi)
#define SYNC_BYTE_1 0xC7                                // Packet first byte
#define SYNC_BYTE_2 0x7C                                // Packet second byte
#define END_BYTE 0x01                                   // Packet last byte
#define BAUD_RATE 230400                                // Serial connection baud rate

// Global constants and variables
uint8_t packetBuffer[PACKET_LEN];  // The transmission packet
uint8_t currentChannel;            // Current channel being sampled
uint16_t adcValue = 0;             // ADC current value

// Channel to use from A0 - A11
AdvancedADC adc(A0, A1, A2, A3, A4, A5);

void adcStart() {
  digitalWrite(LED_BUILTIN, LOW);
  // Resolution, sample rate, number of samples per channel, queue depth.
  if (!adc.begin(AN_RESOLUTION_16, ADC_SAMPLING, SAMPLES_CHANNEL, ADC_QUEUE, true)) {
    Serial.println("Failed to start analog acquisition!");
    while (1)
      ;
  }
}

void adcStop() {
  digitalWrite(LED_BUILTIN, HIGH);
  adc.stop();
}

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ;  // Wait for serial port to connect. Needed for native USB
  }

  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize packetBuffer
  packetBuffer[0] = SYNC_BYTE_1;            // Sync 0
  packetBuffer[1] = SYNC_BYTE_2;            // Sync 1
  packetBuffer[2] = 0;                      // Packet counter
  packetBuffer[PACKET_LEN - 1] = END_BYTE;  // End Byte
}

void loop() {
  // Send data if the buffer is ready and the timer is activ
  if (adc.available()) {
    SampleBuffer adcBuffer = adc.read();
    // Read 6ch ADC inputs and store current values in packetBuffer
    for (currentChannel = 0; currentChannel < NUM_CHANNELS; currentChannel++) {
      adcValue = adcBuffer[currentChannel];                                       // Read Analog input
      packetBuffer[((2 * currentChannel) + HEADER_LEN)] = highByte(adcValue);     // Write High Byte
      packetBuffer[((2 * currentChannel) + HEADER_LEN + 1)] = lowByte(adcValue);  // Write Low Byte
    }

    // Increment the packet counter
    packetBuffer[2]++;

    // Release the buffer to return it to the pool.
    adcBuffer.release();

    // Write packetBuffer to Serial
    Serial.write(packetBuffer, PACKET_LEN);
  }

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();         // Remove extra spaces or newline characters
    command.toUpperCase();  // Normalize to uppercase for case-insensitivity

    if (command == "WHORU")  // Who are you?
    {
      Serial.println("GIGA-R1");
    } else if (command == "START")  // Start data acquisition
    {
      adcStart();
    } else if (command == "STOP")  // Stop data acquisition
    {
      adcStop();
    } else if (command == "STATUS")  // Get status
    {
      Serial.println(adcStatus ? "RUNNING" : "STOPPED");
    } else {
      Serial.println("UNKNOWN COMMAND");
    }
  }
}