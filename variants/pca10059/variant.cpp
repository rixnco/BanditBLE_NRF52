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
  // D0 .. D15
  13,  // D0  is P0.25 (UART RX)
  15,  // D1  is P0.24 (UART TX) 
  17,  // D2  is P0.17
  20,  // D3  is P0.20
  22,  // D4  is P0.22
  24,  // D5  is P0.24
  32,  // D6  is P1.00
   9,  // D7  is P0.09
  10,  // D8  is P0.10

  42,  // D9  is P1.10 
  45,  // D10 is P1.13
  47,  // D11 is P1.15
   2,  // D12 is P0.02 (AIN0)
  29,  // D13 is P0.29 (AIN5)
  31,  // D14 is P0.31 (AIN7)

       // D15 .. D20 (Leds and Buttons)
   6,  // D15 is P0.06 LED1
   8,  // D16 is P0.08 LED_R
  41,  // D17 is P1.09 LED_G
  12,  // D18 is P0.12 LED_B
  38,  // D19 is P1.06 SW1
  18,  // D20 is P0.18 SW2

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

