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

#ifndef _VARIANT_PCA10059_DONGLE_
#define _VARIANT_PCA10059_DONGLE_

/** Master clock frequency */
#define VARIANT_MCK       (64000000ul)

#define USE_LFXO      // Board uses 32khz crystal for LF
// define USE_LFRC    // Board uses RC for LF

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
  
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT           (40)
#define NUM_DIGITAL_PINS     (40)
#define NUM_ANALOG_INPUTS    (7)
#define NUM_ANALOG_OUTPUTS   (0)

// LEDs

#define LED_STATE_ON         (0)    // State when LED is litted
#define LED_STATE_OFF        (1)  


#define PIN_LED1             (11)		// Led RED
#define PIN_LED2             (12)		// Led GREEN
#define PIN_LED3             (13)		// Led BLUE

#define LED_BUILTIN          PIN_LED1

#define LED_RED              PIN_LED1
#define LED_GREEN            PIN_LED2
#define LED_BLUE             PIN_LED3

// Buttons
#define PIN_BUTTON1          (23)		// RESET BTN

/*
 * Analog pins
 */
#define PIN_A0                (0)
#define PIN_A1                (1)
#define PIN_A2                (2)
#define PIN_A3                (3) 
#define PIN_A4                (4)
#define PIN_A5                (5)
#define PIN_A6               (16) // VBAT

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
static const uint8_t A6  = PIN_A6 ;
#define ADC_RESOLUTION    14


/*
 * Serial interfaces
 */
#define PIN_SERIAL1_TX       (6)
#define PIN_SERIAL1_RX       (7)

#define PIN_SERIAL2_TX       (15)
#define PIN_SERIAL2_RX       (14)


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

#define PIN_SPI_SCK          (8)
#define PIN_SPI_MISO         (9)
#define PIN_SPI_MOSI         (10)

static const uint8_t SS   = (3);
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

#define PIN_SPI1_SCK          (17)
#define PIN_SPI1_MISO         (18)
#define PIN_SPI1_MOSI         (19)


/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         (5)
#define PIN_WIRE_SCL         (6)

#define PIN_WIRE1_SDA         (25)
#define PIN_WIRE1_SCL         (26)



#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
