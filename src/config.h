#ifndef __CONFIG_H__
#define __CONFIG_H__

// ==================== User/Board configuration ====================


#define LED_PIN     LED_RED
#define LED_ON      LED_STATE_ON
#define LED_OFF     !LED_STATE_ON




// Crankshaft sensor feature.
#define CRANKSHAFT_INT_PIN    2   // Crankshaft VRS sensor
#define CRANKSHAFT_TEETH      22   // number of VRS pulses per crankshaft revolution



// Wheel speed sensor feature.
// Comment to disable.
#define ENABLE_SPEED_SENSOR

#ifdef ENABLE_SPEED_SENSOR
#define WHEEL_INT_PIN          5   // VSS capteur boîte de vitesses (P0.05)

// VSS boîte de vitesses GSF650 : ~4 impulsions / tour arbre secondaire.
// Rapport final GSF650 : 43/17 = 2.529 → ~10.1 impulsions / tour roue.
// !! VALEUR A CALIBRER : compter les impulsions à vitesse connue et ajuster.
// Procédure : rouler à 50 km/h, mesurer fréquence (Hz) → pulses/km = Hz / (50/3.6)
#define WHEEL_ABS_TEETH       10   // impulsions VSS par tour roue (à calibrer !)

#define WHEEL_CIRCUMF_MM    2150   // Circonférence pneu 140/70R17 en mm
#endif

// Gear feature
#define GEAR_PIN              PIN_A2

// Default ADC thresholds (8-bit) used to map gearbox sensor value to gears.
// Measured values: G1=90 / G2=114 / G3=149 / G4=179 / G5=212 / G6=227 / N=242
#define DEFAULT_GEAR1         102
#define DEFAULT_GEAR2         131
#define DEFAULT_GEAR3         164
#define DEFAULT_GEAR4         195
#define DEFAULT_GEAR5         219
#define DEFAULT_GEAR6         234


// Storage feature

// Persist settings in external SPI flash (P25Q16H).
// Comment this define to use BLEBondStore internal storage instead.
#define SETTINGS_USE_EXTERNAL_FLASH

// Address of the settings page in external flash (last 4 KB sector)
#ifdef SETTINGS_USE_EXTERNAL_FLASH
#define SETTINGS_FLASH_ADDR   0x1FF000UL
#endif



#endif
