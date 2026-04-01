/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "nrf.h"

const uint32_t g_ADigitalPinMap[] =
{
  // D0 .. D19
   2,  // D0  is P0.02 
   3,  // D1  is P0.03 
  28,  // D2  is P0.28
  29,  // D3  is P0.29 (SPI CS)
   4,  // D4  is P0.04 (SDA)
   5,  // D5  is P0.05 (SCL)
  43,  // D6  is P1.11 (UART TX) 
  44,  // D7  is P1.12 (UART RX) 
  45,  // D8  is P1.13 (SPI SCK)
  46,  // D9  is P1.14 (SPI MISO)
  47,  // D10 is P1.15 (SPI MOSI)
  15,  // D11 is P0.15 (I2S SD)
  19,  // D12 is P0.19 (I2S SCK)
  33,  // D13 is P1.01 (I2S WS)
   9,  // D14 is P0.09 (NFC1 / RX1)
  10,  // D15 is P0.10 (NFC2 / TX1)
  31,  // D16 is P0.31 (VBAT)
  35,  // D17 is P1.03 (SPI SCK1)
  37,  // D18 is P1.05 (SPI MISO1)
  39,  // D19 is P1.07 (SPI MOSI1)

  // D20 .. D23 (Leds and Buttons)
  26,  // D20 is P0.26 (LED_R)
  30,  // D21 is P0.30 (LED_G)
   6,  // D22 is P0.06 (LED_B)
  18,  // D23 is P0.18 (SW1 / RESET) 

  // D24 .. D26 BAT / Charger
  13,  // D24 is P0.13 (HICHG)
  17,  // D25 is P0.17 (LED1)
  14,  // D26 is P0.14 (VBAT_en)

  // D27 .. D30 (Internal I2C - 6D IMU)
   7,  // D27 is P0.07 (SDA1)
  27,  // D28 is P0.27 (SCL1)
  11,  // D29 is P0.11 (6D INT)
  40,  // D30 is P1.08 (6D PWR)

  // D31 .. D33 (Internal MIC)
  32,  // D31 is P1.00 (MIC CLK)
  16,  // D32 is P0.15 (MIC DATA)
  42,  // D33 is P1.10 (MIC PWR)

  // D34 .. D39 (QSPI)
  20,  // D34 is P0.20 (QSPI D0)
  24,  // D35 is P0.24 (QSPI D1)
  22,  // D36 is P0.22 (QSPI D2)
  23,  // D37 is P0.23 (QSPI D3)
  21,  // D38 is P0.21 (QSPI CLK)
  25,  // D39 is P0.25 (QSPI CS)

};

void initVariant()
{
  // LED1 & LED2
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);

  pinMode(PIN_LED2, OUTPUT);
  ledOff(PIN_LED2);

  pinMode(PIN_LED3, OUTPUT);
  ledOff(PIN_LED3);
}

