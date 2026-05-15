# ESP32-S3-DevKitC-1 (N16R8) Pinout Reference

## Overview

**Board:** ESP32-S3-DevKitC-1
**Module:** ESP32-S3-WROOM-2 (N16R8 variant)
**Flash:** 16 MB (Octal SPI)
**PSRAM:** 8 MB (Octal SPI)
**Total Pins:** 44 (22 per side)
**CPU:** Xtensa LX7 dual-core @ 240 MHz

## Important Notes

⚠️ **RESERVED PINS (Do NOT use for general I/O):**
- **GPIO 26-32:** Connected to internal SPI flash (Octal SPI)
- **GPIO 33-37:** Connected to internal PSRAM (Octal SPI)

These pins are dedicated to internal flash and PSRAM communication. Using them will cause system instability.

⚠️ **STRAPPING PINS (Avoid in projects if possible):**
- **GPIO 0:** Boot mode selection (pulled HIGH = normal boot, LOW = download mode)
- **GPIO 3:** JTAG enable
- **GPIO 45:** VDD_SPI voltage
- **GPIO 46:** ROM logging enable

These pins have special boot-time behavior. Avoid using them unless necessary.

## Complete Pinout (Official Espressif Documentation)

### J1 Header (Left Side - 22 pins)

| Pin No. | Name | Type | Function |
|---------|------|------|----------|
| 1 | 3V3 | P | 3.3V power supply |
| 2 | 3V3 | P | 3.3V power supply |
| 3 | RST | I | EN (Reset) |
| 4 | 4 | I/O/T | RTC_GPIO4, GPIO4, TOUCH4, ADC1_CH3 |
| 5 | 5 | I/O/T | RTC_GPIO5, GPIO5, TOUCH5, ADC1_CH4 |
| 6 | 6 | I/O/T | RTC_GPIO6, GPIO6, TOUCH6, ADC1_CH5 |
| 7 | 7 | I/O/T | RTC_GPIO7, GPIO7, TOUCH7, ADC1_CH6 |
| 8 | 15 | I/O/T | RTC_GPIO15, GPIO15, U0RTS, ADC2_CH4, XTAL_32K_P |
| 9 | 16 | I/O/T | RTC_GPIO16, GPIO16, U0CTS, ADC2_CH5, XTAL_32K_N |
| 10 | 17 | I/O/T | RTC_GPIO17, GPIO17, U1TXD, ADC2_CH6 |
| 11 | 18 | I/O/T | RTC_GPIO18, GPIO18, U1RXD, ADC2_CH7, CLK_OUT3 |
| 12 | 8 | I/O/T | RTC_GPIO8, GPIO8, TOUCH8, ADC1_CH7, SUBSPICS1 |
| 13 | 3 | I/O/T | RTC_GPIO3, GPIO3, TOUCH3, ADC1_CH2 ⚠️ **Strapping** |
| 14 | 46 | I/O/T | GPIO46 ⚠️ **Strapping** |
| 15 | 9 | I/O/T | RTC_GPIO9, GPIO9, TOUCH9, ADC1_CH8, FSPIHD, SUBSPIHD |
| 16 | 10 | I/O/T | RTC_GPIO10, GPIO10, TOUCH10, ADC1_CH9, FSPICS0, FSPIIO4, SUBSPICS0 |
| 17 | 11 | I/O/T | RTC_GPIO11, GPIO11, TOUCH11, ADC2_CH0, FSPID, FSPIIO5, SUBSPID |
| 18 | 12 | I/O/T | RTC_GPIO12, GPIO12, TOUCH12, ADC2_CH1, FSPICLK, FSPIIO6, SUBSPICLK |
| 19 | 13 | I/O/T | RTC_GPIO13, GPIO13, TOUCH13, ADC2_CH2, FSPIQ, FSPIIO7, SUBSPIQ |
| 20 | 14 | I/O/T | RTC_GPIO14, GPIO14, TOUCH14, ADC2_CH3, FSPIWP, FSPIDQS, SUBSPIWP |
| 21 | 5V | P | 5V power supply |
| 22 | G | G | Ground |

### J3 Header (Right Side - 22 pins)

| Pin No. | Name | Type | Function |
|---------|------|------|----------|
| 1 | G | G | Ground |
| 2 | TX | I/O/T | U0TXD, GPIO43, CLK_OUT1 |
| 3 | RX | I/O/T | U0RXD, GPIO44, CLK_OUT2 |
| 4 | 1 | I/O/T | RTC_GPIO1, GPIO1, TOUCH1, ADC1_CH0 |
| 5 | 2 | I/O/T | RTC_GPIO2, GPIO2, TOUCH2, ADC1_CH1 |
| 6 | 42 | I/O/T | MTMS, GPIO42 |
| 7 | 41 | I/O/T | MTDI, GPIO41, CLK_OUT1 |
| 8 | 40 | I/O/T | MTDO, GPIO40, CLK_OUT2 |
| 9 | 39 | I/O/T | MTCK, GPIO39, CLK_OUT3, SUBSPICS1 |
| 10 | 38 | I/O/T | GPIO38, FSPIWP, SUBSPIWP |
| 11 | 37 | I/O/T | SPIDQS, GPIO37, FSPIQ, SUBSPIQ 🔴 **RESERVED (PSRAM)** |
| 12 | 36 | I/O/T | SPIIO7, GPIO36, FSPICLK, SUBSPICLK 🔴 **RESERVED (PSRAM)** |
| 13 | 35 | I/O/T | SPIIO6, GPIO35, FSPID, SUBSPID 🔴 **RESERVED (PSRAM)** |
| 14 | 0 | I/O/T | RTC_GPIO0, GPIO0 ⚠️ **Boot Button, Strapping** |
| 15 | 45 | I/O/T | GPIO45 ⚠️ **Strapping** |
| 16 | 48 | I/O/T | GPIO48, SPICLK_N, SUBSPICLK_N_DIFF, **RGB LED** |
| 17 | 47 | I/O/T | GPIO47, SPICLK_P, SUBSPICLK_P_DIFF |
| 18 | 21 | I/O/T | RTC_GPIO21, GPIO21 |
| 19 | 20 | I/O/T | RTC_GPIO20, GPIO20, U1CTS, ADC2_CH9, CLK_OUT1, USB_D+ |
| 20 | 19 | I/O/T | RTC_GPIO19, GPIO19, U1RTS, ADC2_CH8, CLK_OUT2, USB_D- |
| 21 | G | G | Ground |
| 22 | G | G | Ground |

