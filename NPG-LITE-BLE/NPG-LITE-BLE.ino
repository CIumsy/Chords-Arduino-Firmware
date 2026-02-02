/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <https://www.gnu.org/licenses/>.

   BLE connectivity adapted from the ESP32 BLE Server example by Random Nerd Tutorials:
   https://randomnerdtutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/.

   Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
   Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
   Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

   At Upside Down Labs, we create open‐source DIY neuroscience hardware and software.
   Our mission is to make neuroscience affordable and accessible for everyone.
   By supporting us with your purchase, you help spread innovation and open science.
   Thank you for being part of this journey with us!
*/


// ----- Existing Includes -----
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEUtils.h>
#include <Adafruit_NeoPixel.h>
#include <sdkconfig.h>
#include "hal/efuse_hal.h"
#include "esp_gap_ble_api.h"
#include "esp_idf_version.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


// ----- New low-power / BLE / ADC includes -----
#include "esp_bt.h"                  // release Classic BT memory
#include "esp_adc/adc_continuous.h"  // ADC continuous (DMA) driver
#include "hal/adc_types.h"           // adc_atten_t, bit width, etc.
#include "soc/soc_caps.h"            // SOC_ADC_DIGI_RESULT_BYTES


// ----- Chip-specific Pin Definitions -----
//
// Use the ESP-IDF config macros to detect the chip.
#if defined(CONFIG_IDF_TARGET_ESP32C6)
// Store chip revision number (for optional raw fixup if needed)
uint32_t chiprev = efuse_hal_chip_revision();
#define LED_BUILTIN 7
#define PIXEL_PIN 15
#define PIXEL_COUNT 6
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#define LED_BUILTIN 6
#define PIXEL_PIN 3
#define PIXEL_COUNT 4
#else
#error "Unsupported board: Please target either ESP32-C6 or ESP32-C3 in your Board Manager."
#endif


#define PIXEL_BRIGHTNESS 7                                // Brightness of Neopixel LED
#define NUM_CHANNELS 4                                    // Number of BioAmp channels + 1 channel for battery
#define SINGLE_SAMPLE_LEN (2 * (NUM_CHANNELS-1) + 1)      // Each sample: (No. of bioAmp channels * 2 bytes) + 1 counter
#define BLOCK_COUNT 10                                    // Batch size: 10 samples per notification
#define NEW_PACKET_LEN (BLOCK_COUNT * SINGLE_SAMPLE_LEN)  // New packet length (70 bytes)
#define SAMP_RATE 500.0                                   // Sampling rate per channel (500 Hz)
#define ADC_CONV_BYTES SOC_ADC_DIGI_RESULT_BYTES          // Number of bytes per ADC conversion result in continuous mode
#define BATTERY_PIN A6


// Onboard Neopixel at PIXEL_PIN
Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);


// Battery monitoring variables
static unsigned long lastBatteryCheck = 0;
static const unsigned long BATTERY_CHECK_INTERVAL = 10000;  // Interval in milliseconds
static BLEServer *pBLEServer = nullptr;                      // Store server reference for disconnect


// LUT for 1S LiPo (Voltage in ascending order)
const float voltageLUT[] = {
  3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82,
  3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20
};


const int percentLUT[] = {
  0, 5, 10, 15, 20, 25, 30, 35, 40, 45,
  50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
};


const int lutSize = sizeof(voltageLUT) / sizeof(voltageLUT[0]);


// Linear interpolation function
float interpolatePercentage(float voltage) {
  // Handle out-of-range voltages
  if (voltage <= voltageLUT[0]) return 0;
  if (voltage >= voltageLUT[lutSize - 1]) return 100;


  // Find the nearest LUT entries
  int i = 0;
  while (i < lutSize - 1 && voltage > voltageLUT[i + 1]) i++;


  // Interpolate
  float v1 = voltageLUT[i], v2 = voltageLUT[i + 1];
  int p1 = percentLUT[i], p2 = percentLUT[i + 1];
  return p1 + (voltage - v1) * (p2 - p1) / (v2 - v1);
}


// BLE UUIDs – change if desired.
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DATA_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"     // For ADC data (Notify only)
#define CONTROL_CHAR_UUID "0000ff01-0000-1000-8000-00805f9b34fb"  // For commands (Read/Write/Notify)
#define BATTERY_CHAR_UUID "f633d0ec-46b4-43c1-a39f-1ca06d0602e1"  // For battery status (Notify only)


// ----- Global Variables -----
uint8_t batchBuffer[NEW_PACKET_LEN] = { 0 };  // Buffer to accumulate BLOCK_COUNT samples
volatile int sampleIndex = 0;     // How many samples accumulated in current batch
volatile bool streaming = false;  // True when "START" command is received
uint8_t mac[6];                   // Array to store 6-byte MAC address

// Flags to start/stop adc_continuous_mode
static volatile bool adc_start_requested = false;
static volatile bool adc_stop_requested = false;


BLECharacteristic *pDataCharacteristic;
BLECharacteristic *pControlCharacteristic;
BLECharacteristic *pBatteryCharacteristic;


// Global sample counter (each sample's packet counter)
uint8_t overallCounter = 0;


// Battery monitoring - stores latest ADC reading from A6
static volatile uint16_t latestBatteryRaw = 2111;  // Initially set to 2111 indicating 100% battery to avoid connection issues
// Rolling average buffer for battery (1000 samples = 2 seconds @ 500Hz)
#define BATTERY_AVG_SAMPLES 1000
static uint16_t batteryBuffer[BATTERY_AVG_SAMPLES] = {0};
static uint16_t batteryIndex = 0;
static uint32_t batterySum = 0;  // Initialize with startup value
static bool batteryBufferFilled = false;

// ----- ADC DMA (continuous mode) globals -----
static adc_continuous_handle_t adc_handle = nullptr;
static bool adc_started = false;
static SemaphoreHandle_t adc_data_semaphore = nullptr;
static esp_ble_adv_params_t advParams = {
  .adv_int_min = 0x0680,
  .adv_int_max = 0x0680,
  .adv_type = ADV_TYPE_IND,
  .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
  .channel_map = ADV_CHNL_ALL,
  .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

// Helper macros to parse DMA results as TYPE2 format on C3/C6
// (channel and data fields are in adc_digi_output_data_t::type2)
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_GET_CHANNEL(p) ((p)->type2.channel)
#define ADC_GET_DATA(p) ((p)->type2.data)


// Forward declarations
static void adc_dma_init();
static void adc_dma_start();
static void adc_dma_stop();
static void handle_adc_dma_and_notify();
static inline uint16_t fix_raw_if_needed(uint16_t raw);


// ----- BLE Server Callbacks -----
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    pixels.setPixelColor(0, pixels.Color(0, PIXEL_BRIGHTNESS, 0));  // Green
    pixels.show();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);


    // Apply -3 dBm to the active connection (handle 0 for first/only connection)
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_N3);
    esp_ble_gap_stop_advertising();  // Explicitly stop advertising
  }


  void onDisconnect(BLEServer *pServer) override {
    pixels.setPixelColor(0, pixels.Color(PIXEL_BRIGHTNESS, 0, 0));  // Red
    pixels.show();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);


    streaming = false;
    adc_stop_requested = true;  // Request stop
    esp_ble_gap_start_advertising(&advParams);
  }
};


// ----- BLE Control Characteristic Callback -----
// Handles incoming commands ("START", "STOP", "WHORU", "STATUS")
class ControlCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String cmd = characteristic->getValue();
    cmd.trim();
    cmd.toUpperCase();


    if (cmd == "START") {
      pixels.setPixelColor(0, pixels.Color(0, 0, PIXEL_BRIGHTNESS));  // Blue
      pixels.show();
      overallCounter = 0;
      sampleIndex = 0;
      streaming = true;
      adc_start_requested = true;  // Request start 
    } else if (cmd == "STOP") {
      pixels.setPixelColor(0, pixels.Color(0, PIXEL_BRIGHTNESS, 0));  // Green
      pixels.show();
      streaming = false;
      adc_stop_requested = true;  // Request stop 
    } else if (cmd == "WHORU") {
      characteristic->setValue("NPG-LITE");
      characteristic->notify();
    } else if (cmd == "STATUS") {
      characteristic->setValue(streaming ? "RUNNING" : "STOPPED");
      characteristic->notify();
    } else {
      characteristic->setValue("UNKNOWN COMMAND");
      characteristic->notify();
    }
  }
};


void checkBatteryAndDisconnect() {
  float voltage = (latestBatteryRaw / 1000.0) * 2;  // ESP32C6 v0.1
  voltage = voltage - 0.02;
  float percentage = interpolatePercentage(voltage);
  // Send battery percentage as single byte (0-100)
  uint8_t batteryByte = (uint8_t)percentage;
  pBatteryCharacteristic->setValue(&batteryByte, 1);
  pBatteryCharacteristic->notify();
  if(percentage > 50.0)
  {
    pixels.setPixelColor(1, pixels.Color(0, PIXEL_BRIGHTNESS, 0));  // Green when above 50%
    pixels.show();
  }
  else if(percentage <= 50.0 && percentage >= 5.0 )
  {
    pixels.setPixelColor(1, pixels.Color(15, 4, 0));  // Orange when below 50%
    pixels.show();
  }
  else if (percentage < 5.0) {
    // Stop streaming
    streaming = false;
    adc_stop_requested = true;  // Request stop

    pixels.setPixelColor(1, pixels.Color(PIXEL_BRIGHTNESS, 0, 0));  // Red when below 5%
    pixels.show();

    // Disconnect BLE client if connected
    if (pBLEServer != nullptr && pBLEServer->getConnectedCount() > 0) {
      // Get connection ID from first connected client
      std::map<uint16_t, conn_status_t> peerDevices = pBLEServer->getPeerDevices(false);
      for (auto const &entry : peerDevices) {
        uint16_t connId = entry.first;
        pBLEServer->disconnect(connId);  // Use public disconnect method
      }
    }
  }
}


void setup() {
  // ----- LEDs -----
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(PIXEL_BRIGHTNESS, 0, 0));  // Red (power on)
  pixels.show();


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);


  setCpuFrequencyMhz(80);


  // Create binary semaphore for ADC data ready signaling
  adc_data_semaphore = xSemaphoreCreateBinary();
  if (adc_data_semaphore == nullptr) {
    while (1);  // Halt
  }


  esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY);


  // ----- BLE-only memory footprint (free Classic BT) -----
  // Must be called before the BLE stack is initialized
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);


  // ----- Initialize BLE -----
  char deviceName[36];
  sprintf(deviceName, "NPG-%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  BLEDevice::init(deviceName);


  // Set BLE TX power to -3 dBm for default/advertising/scan
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N3);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N3);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_N3);

  // Optional larger MTU for efficiency (doesn't change packet format)
  BLEDevice::setMTU(500);


  pBLEServer = BLEDevice::createServer();
  pBLEServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pBLEServer->createService(SERVICE_UUID);


  // Data Characteristic (Notify only) for ADC data
  pDataCharacteristic = pService->createCharacteristic(
    DATA_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);
  pDataCharacteristic->addDescriptor(new BLE2902());


  // Control Characteristic (Read/Write/Notify)
  pControlCharacteristic = pService->createCharacteristic(
    CONTROL_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pControlCharacteristic->setCallbacks(new ControlCallback());

  // Battery Characteristic (Read/Notify)
  pBatteryCharacteristic = pService->createCharacteristic(
    BATTERY_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pBatteryCharacteristic->addDescriptor(new BLE2902());


  pService->start();


  // Configure advertising data to include device name
  esp_ble_adv_data_t adv_data = {};
  adv_data.set_scan_rsp = false;
  adv_data.include_name = true;  // ← KEY: include "NPG-<MAC>" in advertising
  adv_data.include_txpower = false;
  adv_data.min_interval = 0x0006;
  adv_data.max_interval = 0x0010;
  adv_data.appearance = 0x00;
  adv_data.manufacturer_len = 0;
  adv_data.p_manufacturer_data = nullptr;
  adv_data.service_data_len = 0;
  adv_data.p_service_data = nullptr;
  adv_data.service_uuid_len = 0;
  adv_data.p_service_uuid = nullptr;
  adv_data.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);


  esp_ble_gap_config_adv_data(&adv_data);


  // Stop Arduino's advertising helper and start with our params
  BLEDevice::getAdvertising()->stop();  // if it was started elsewhere
  esp_ble_gap_start_advertising(&advParams);
}


void loop() {
  // Handle start/stop requests of adc_continuous_mode
  if (adc_start_requested) {
    adc_dma_start();
    adc_start_requested = false;
  }
  if (adc_stop_requested) {
    adc_dma_stop();
    adc_stop_requested = false;
  }

  if (streaming) {
    // Battery check only when streaming (every 10 seconds)
    unsigned long currentMillis = millis();
    if (currentMillis - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
      lastBatteryCheck = currentMillis;
      checkBatteryAndDisconnect();
    }

    // Block until semaphore is given by ISR (allows deep sleep)
    if (xSemaphoreTake(adc_data_semaphore, portMAX_DELAY) == pdTRUE) {
      handle_adc_dma_and_notify();
    }
  } else {
    // Longer delay when idle to maximize sleep time
    delay(100);
  }
}


// ====== ADC DMA implementation ======

// Maps physical ADC channel id → logical index 0..NUM_CHANNELS-1
static int8_t hw2idx[10];

#if NUM_CHANNELS < 7
  static const uint8_t hw_chs[NUM_CHANNELS] = { 0, 1, 2, 6 };
#else
  static const uint8_t hw_chs[NUM_CHANNELS] = { 0, 1, 2, 3, 4, 5, 6 };
#endif


static void adc_dma_init() {


  static adc_digi_pattern_config_t pattern[NUM_CHANNELS];


  // Build pattern from the single channel list
  for (int i = 0; i < NUM_CHANNELS; i++) {
    pattern[i].atten = ADC_ATTEN_DB_11;
    pattern[i].channel = hw_chs[i];  // ← use physical channel id
    pattern[i].unit = ADC_UNIT_1;    // ADC1 only
    pattern[i].bit_width = ADC_BITWIDTH_12;
  }


  for (int i = 0; i < (int)sizeof(hw2idx); i++) hw2idx[i] = -1;
  for (int i = 0; i < NUM_CHANNELS; i++) hw2idx[hw_chs[i]] = i;


  // Create driver handle and configure continuous conversion
  adc_continuous_handle_cfg_t handle_cfg = {
    .max_store_buf_size = NUM_CHANNELS * ADC_CONV_BYTES * BLOCK_COUNT * 5,
    .conv_frame_size = NUM_CHANNELS * ADC_CONV_BYTES * BLOCK_COUNT,
#if defined(ESP_IDF_VERSION) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
    .flags = { .flush_pool = 1 },  // Only for newer IDF that supports it
#endif
  };
  if (adc_handle == nullptr) {
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));
  }


  adc_continuous_evt_cbs_t cbs = {
    .on_conv_done = [](adc_continuous_handle_t handle,
                       const adc_continuous_evt_data_t *edata,
                       void *user_data) -> bool {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(adc_data_semaphore, &xHigherPriorityTaskWoken);
      return (xHigherPriorityTaskWoken == pdTRUE);  // Yield if higher priority task woken
    },
  };
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, nullptr));


  adc_continuous_config_t dig_cfg = {
    .pattern_num = NUM_CHANNELS,
    .adc_pattern = pattern,
    // total sample rate = per-channel * number of channels
    .sample_freq_hz = (uint32_t)(SAMP_RATE * NUM_CHANNELS),  // 2000 SPS total
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_OUTPUT_TYPE,  // TYPE2 (unit, channel, data)
  };
  ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
}


static void adc_dma_start() {
  // Reinitialize ADC if it was deinited
  if (adc_handle == nullptr) {
    adc_dma_init();
  }


  if (adc_handle && !adc_started) {
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    adc_started = true;
  }
}


static void adc_dma_stop() {
  if (adc_handle && adc_started) {
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
    adc_started = false;


    // DEINITIALIZE ADC to save power
    ESP_ERROR_CHECK(adc_continuous_deinit(adc_handle));
    adc_handle = nullptr;
  }
}


static inline uint16_t fix_raw_if_needed(uint16_t raw) {
#if defined(CONFIG_IDF_TARGET_ESP32C6)
  // Optional: match your prior scaling workaround for C6 rev1 if needed
  if (chiprev == 1) {
    // scale raw (0..~3249) to 0..4095
    uint32_t v = (uint32_t)raw * 4095u / 3249u;
    if (v > 4095u) v = 4095u;
    return (uint16_t)v;
  }
#endif
  return raw;
}


static void handle_adc_dma_and_notify() {
  // Read whatever DMA has buffered; non-blocking with short buffer
  uint8_t dma_buf[NUM_CHANNELS * ADC_CONV_BYTES * BLOCK_COUNT];
  uint32_t ret_len = 0;
  esp_err_t ret = adc_continuous_read(adc_handle, dma_buf, sizeof(dma_buf), &ret_len, 0);
  if (ret != ESP_OK || ret_len == 0) {
    return;
  }


  // Assemble triplets (ch0, ch1, ch2) into your 7-byte sample packets
  // Maintain a small staging for latest values per channel and a mask
  static uint16_t last_vals[NUM_CHANNELS] = { 0 };
  static uint8_t have_mask = 0;
  const uint8_t FULL_MASK = (1u << NUM_CHANNELS) - 1;


  for (uint32_t i = 0; i + ADC_CONV_BYTES <= ret_len; i += ADC_CONV_BYTES) {
    auto *p = (const adc_digi_output_data_t *)&dma_buf[i];
    uint8_t ch_hw = ADC_GET_CHANNEL(p);  // physical channel id from TYPE2
    uint16_t raw = ADC_GET_DATA(p);
    // map physical channel → logical index (0..NUM_CHANNELS-1)
    int8_t idx = (ch_hw < (uint8_t)sizeof(hw2idx)) ? hw2idx[ch_hw] : -1;
    if (idx >= 0) {
      // Apply fix only to BioAmp channels (A0-A5), NOT battery (A6)
      if (idx < (NUM_CHANNELS-1)) {
        last_vals[idx] = fix_raw_if_needed(raw);
      } else {
        last_vals[idx] = raw;  // Battery channel: use raw value
      }
      have_mask |= (1u << idx);


      // Store battery reading with rolling average
      if (idx == (NUM_CHANNELS-1)) {
        uint16_t newSample = last_vals[idx];
        
        // Subtract oldest value from sum
        batterySum -= batteryBuffer[batteryIndex];
        
        // Add new value to buffer and sum
        batteryBuffer[batteryIndex] = newSample;
        batterySum += newSample;
        
        // Update circular buffer index
        batteryIndex++;
        if (batteryIndex >= BATTERY_AVG_SAMPLES) {
          batteryIndex = 0;
          batteryBufferFilled = true;  // Buffer is now full
        }
        
        // Calculate and store average (only after buffer is filled for accuracy)
        if (batteryBufferFilled) {
          latestBatteryRaw = (uint16_t)(batterySum / BATTERY_AVG_SAMPLES);
        } else {
          // Before buffer fills, use current value (or could use partial average)
          latestBatteryRaw = newSample;
        }
      }
    }

    // When we have all 3 channels, emit one 7-byte record DIRECTLY to batchBuffer
    if (have_mask == FULL_MASK) {
      // Calculate offset in batchBuffer
      uint16_t offset = sampleIndex * SINGLE_SAMPLE_LEN;

      // Write counter directly to batchBuffer
      batchBuffer[offset] = overallCounter;
      overallCounter = (overallCounter + 1) & 0xFF;


      // Big-endian packing directly to batchBuffer (no intermediate samplePacket)
      for (uint8_t c = 0; c < NUM_CHANNELS - 1; c++) {
        uint16_t v = last_vals[c];
        batchBuffer[offset + 1 + c * 2] = (uint8_t)((v >> 8) & 0xFF);
        batchBuffer[offset + 1 + c * 2 + 1] = (uint8_t)(v & 0xFF);
      }


      sampleIndex++;


      // Notify every BLOCK_COUNT samples
      if (sampleIndex >= BLOCK_COUNT) {
        pDataCharacteristic->setValue(batchBuffer, NEW_PACKET_LEN);
        pDataCharacteristic->notify();
        sampleIndex = 0;
      }


      have_mask = 0;  // reset for next triplet
    }
  }
}