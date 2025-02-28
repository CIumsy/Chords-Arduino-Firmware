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
// Copyright (c) 2024 - 2025 Upside Down Labs - contact@upsidedownlabs.tech
// Author: Deepak Khatri
//
// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

#include <Arduino.h>

// Definitions
#define LED_BUILTIN 6
#define TIMER_FREQ 1000000
#define NUM_CHANNELS 3                                  // Number of channels supported
#define HEADER_LEN 3                                    // Header = SYNC_BYTE_1 + SYNC_BYTE_2 + Counter
#define PACKET_LEN (NUM_CHANNELS * 2 + HEADER_LEN + 1)  // Packet length = Header + Data + END_BYTE
#define SAMP_RATE 500.0                                 // Sampling rate (250/500 for UNO R4)
#define SYNC_BYTE_1 0xC7                                // Packet first byte
#define SYNC_BYTE_2 0x7C                                // Packet second byte
#define END_BYTE 0x01                                   // Packet last byte
#define BAUD_RATE 230400                                // Serial connection baud rate

// Global constants and variables
uint8_t packetBuffer[PACKET_LEN];  // The transmission packet
uint8_t currentChannel;            // Current channel being sampled
uint16_t adcValue = 0;             // ADC current value
bool timerStatus = false;          // Timer status bit
bool bufferReady = false;          // Buffer ready status bit

hw_timer_t *timer_1 = NULL;

void IRAM_ATTR ADC_ISR()
{
    if (!timerStatus or Serial.available()) {
    timerStop();
    return;
  }

  // Set bufferReady status bit to true
  bufferReady = true;
}

void timerStart() {
  timerStatus = true;
  timerStart(timer_1);
  digitalWrite(LED_BUILTIN, HIGH);
}

void timerStop() {
  timerStatus = false;
  bufferReady = false;
  timerStop(timer_1);
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {

  Serial.begin(BAUD_RATE);
  Serial.setTimeout(100);
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

  timer_1 = timerBegin(1000000);
  timerAttachInterrupt(timer_1, &ADC_ISR);
  timerAlarm(timer_1, (int)(TIMER_FREQ / SAMP_RATE), true, 0);
  timerStop(timer_1);
  analogReadResolution(12);
}

void loop() {
  // Send data if the buffer is ready and the timer is activ
  if (timerStatus and bufferReady) {

    // ADC value Reading, Converting, and Storing:
    for (currentChannel = 0; currentChannel < NUM_CHANNELS; currentChannel++) {

      // Read ADC input
      adcValue = analogRead(currentChannel);

      // Store current values in packetBuffer to send.
      packetBuffer[((2 * currentChannel) + HEADER_LEN)] = highByte(adcValue);     // Write High Byte
      packetBuffer[((2 * currentChannel) + HEADER_LEN + 1)] = lowByte(adcValue);  // Write Low Byte
    }

    // Increment the packet counter
    packetBuffer[2]++;
    // Send the packetBuffer to the Serial port
    Serial.write(packetBuffer, PACKET_LEN);
    // Reset the bufferReady flag
    bufferReady = false;
  }

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();         // Remove extra spaces or newline characters
    command.toUpperCase();  // Normalize to uppercase for case-insensitivity

    if (command == "WHORU")  // Who are you?
    {
      Serial.println("NPG-LITE");
    } else if (command == "START")  // Start data acquisition
    {
      timerStart();
    } else if (command == "STOP")  // Stop data acquisition
    {
      timerStop();
    } else if (command == "STATUS")  // Get status
    {
      Serial.println(timerStatus ? "RUNNING" : "STOPPED");
    } else {
      Serial.println("UNKNOWN COMMAND");
    }
  }
}
