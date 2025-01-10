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

#include "mbed.h"
#include <Arduino.h>
#include "hardware/adc.h"

// Definitions
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

mbed::Ticker ticker;

// callback method used by timer
void timerCallback() {
  // Read ADC inputs and store current values in packetBuffer
  for (currentChannel = 0; currentChannel < NUM_CHANNELS; currentChannel++) {
    adc_select_input(currentChannel);
    adcValue = adc_read();                                                      // Read Analog input
    packetBuffer[((2 * currentChannel) + HEADER_LEN)] = highByte(adcValue);     // Write High Byte
    packetBuffer[((2 * currentChannel) + HEADER_LEN + 1)] = lowByte(adcValue);  // Write Low Byte
  }

  // Increment the packet counter
  packetBuffer[2]++;

  bufferReady = true;
}

void timerStart() {
  timerStatus = true;
  auto interval = std::chrono::microseconds(static_cast<int>(1e6 / SAMP_RATE));
  ticker.attach(&timerCallback, interval);
  digitalWrite(LED_BUILTIN, HIGH);
}

void timerStop() {
  timerStatus = false;
  bufferReady = false;
  ticker.detach();
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
  // Initialize ADC
  adc_init();         // Initialize the ADC hardware
  adc_gpio_init(26);  // Initialize GPIO26 -> ADC0
  adc_gpio_init(27);  // Initialize GPIO27 -> ADC1
  adc_gpio_init(28);  // Initialize GPIO28 -> ADC2

  // Set ADC resolution (the RP2040 ADC supports native 12-bit resolution)
  adc_set_clkdiv(1);  // Ensure maximum ADC clock speed
  adc_fifo_setup(
    true,   // Enable FIFO
    false,  // No DMA requested
    1,      // DREQ (threshold for DMA) not used
    true,   // Set to true to shift results to 12 bits
    false   // Don't enable error on overflow
  );
}

void loop() {
  // Send data if the buffer is ready and the timer is activ
  if (timerStatus and bufferReady) {
    Serial.write(packetBuffer, PACKET_LEN);
    bufferReady = false;
  }

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();         // Remove extra spaces or newline characters
    command.toUpperCase();  // Normalize to uppercase for case-insensitivity

    if (command == "WHORU")  // Who are you?
    {
      Serial.println("RPI-PICO-RP2040");
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
