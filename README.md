# Chords Arduino Firmware

**Chords** is an open-source toolkit developed by Upside Down Labs to transform Arduino-compatible 
boards into bio-potential data acquisition devices when paired with BioAmp hardware.

## Tools

1. [Chords-Web](https://chords.upsidedownlabs.tech/)
2. [Chords-Python](https://github.com/upsidedownlabs/Chords-Python)

> [!NOTE]
> Flash Arduino code to your hardware from the list below to use these tools.

## Supported boards

| Board | Voltage | Resolution | SamplingRate | BaudRate | Code |
| ----- | ------- | ---------- | ------------ | -------- | ---- |
| Arduino GIGA R1 (WiFi) | 3V3 | 16-bit | 500 | 230400 | [GIGA-R1.ino](GIGA-R1/GIGA-R1.ino) |
| Arduino UNO R4 Minima/WiFi | 5V | 14-bit | 500 | 230400 | [UNO-R4.ino](UNO-R4/UNO-R4.ino) |
| Arduino UNO R3 | 5V | 10-bit | 250 | 230400 | [UNO-R3.ino](UNO-R3/UNO-R3.ino) |
| Raspberry PI Pico | 3V3 | 12-bit | 500 | 230400 | [RPI-PICO-RP2040.ino](RPI-PICO-RP2040/RPI-PICO-RP2040.ino) |
| UNO R3 Clone (CH340) | 5V | 10-bit | 250 | 115200 | [UNO-CLONE.ino](UNO-CLONE/UNO-CLONE.ino) |