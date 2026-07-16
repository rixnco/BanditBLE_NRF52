# Architecture découplée : Protocol & Settings

## Vue d'ensemble

L'architecture a été refactorisée pour découpler les responsabilités :

```
┌──────────────────────────────────┐
│       main.cpp (Application)     │
│  Orchestre et connecte les       │
│        services                  │
└──────────────────────────────────┘
         ↓         ↓         ↓
    ┌────┴─┐  ┌────┴───┐  ┌─┴────┐
    │Sensor│  │Settings│  │Output│
    │Logic │  │Manager  │  │Stream│
    └────┬─┘  └────┬───┘  └─┬────┘
         └──────┬───┴────┬──┘
              ┌─┴──────┐ │
              │Protocol│ │
              │Handler │◄┘
              └────────┘
```

## Composants

### 1. **Print** (Arduino standard)
Interface abstraite pour les sorties, fournie par le framework Arduino.

```cpp
// Print est utilisé directement (Serial, nrfuart héritent de Print)
Print& output = Serial;
output.print("Hello");
output.println("World");
```

**Avantages:**
- ✅ Standard Arduino, zéro dépendance externe
- ✅ Compatible avec Serial, BLEUart, etc.
- ✅ Testable avec un mock Print custom si nécessaire

---

### 2. **SettingsStore** (`SettingsStore.h`)
Abstraction du backend de persistance (comme un `Stream` abstrait le transport).
Le `SettingsManager` ne sait pas *où* les octets sont stockés, seulement comment lire/écrire un blob.

```cpp
class SettingsStore {
public:
    virtual bool init() = 0;
    virtual bool read(void* data, size_t len) = 0;
    virtual bool write(const void* data, size_t len) = 0;
};
```

Deux implémentations, choisies à la compilation via `SETTINGS_USE_EXTERNAL_FLASH` :
- `FlashSettingsStore` (SPI flash externe, ex. P25Q16H)
- `BondSettingsStore` (BLE bond store du SoftDevice)

Le backend actif est fourni par `getDefaultSettingsStore()`.

**Avantages:**
- ✅ Stockage isolé du modèle
- ✅ Testable avec un `FakeSettingsStore` (RAM)
- ✅ Ajout d'un backend sans toucher au reste

---

### 3. **SettingsManager** (`SettingsManager.h`)
Source unique de vérité des settings. Possède l'instance `settings_t` (le global
`g_settings` a été supprimé) et délègue la persistance à un `SettingsStore` **injecté**.

```cpp
SettingsManager settings(getDefaultSettingsStore());  // injection du backend

// Initialisation / persistance
settings.init();   // initialise le backend
settings.load();   // lit + valide (reset auto si invalide)
settings.save();   // écrit via le backend
settings.reset();  // valeurs par défaut (RAM)

// Accesseurs typés (idx 1..6 pour les gears)
uint16_t g1 = settings.getGear(1);
settings.setGear(1, 1500);

int16_t x, y, z;
settings.getImuOffsets(x, y, z);
settings.setImuOffsets(100, 200, 300);
```

**Avantages:**
- ✅ Plus de `g_settings` global, plus de double source de vérité
- ✅ Accesseurs typés (pas d'exposition brute de `settings_t`)
- ✅ Backend injecté → testable, pas de singleton

---

### 4. **SensorProvider / BanditController** (`ProtocolHandler.h`)
Interfaces qui découplent le protocole de l'application (modèle télémétrie).

```cpp
// Données courantes tirées par le protocole (cf. TelemetryProvider)
class SensorProvider {
public:
    virtual uint8_t  getCurrentGear() = 0;
    virtual uint16_t getCurrentRPM() = 0;
    virtual uint16_t getCurrentGearRaw() = 0;
};

// Actions déclenchées par le protocole (cf. TelemetryListener)
class BanditController {
public:
    virtual void onOverride(bool enabled, int rpm, int gear) = 0;
    virtual void onCalibrateIMU() = 0;
};
```

---

### 5. **ProtocolHandler** (`ProtocolHandler.h`)
Parser protocol découplé des I/O et des globals, par **injection de dépendances**.

```cpp
ProtocolHandler protocol(output, settings, sensors, controller);

// Traiter une commande complète
protocol.processCommand("$IMU=100,200,300");

// Envoyer un report (tire gear/rpm/adc via SensorProvider)
protocol.sendReport();
```

**Responsabilités:**
- ✅ Parser / valider les commandes
- ✅ Lire/écrire les settings via `SettingsManager&`
- ✅ Tirer les données via `SensorProvider&`, déclencher les actions via `BanditController&`
- ✅ Générer les réponses via `Print&`
- ❌ NE GÈRE PAS les I/O (lecture Serial/BLE) ni les globals

---

## Usage dans main.cpp

### Initialisation

```cpp
#include "SettingsStore.h"
#include "SettingsManager.h"
#include "ProtocolHandler.h"

// Instance unique (source de vérité), backend choisi à la compilation
SettingsManager settings(getDefaultSettingsStore());

// Façade appli implémentant les deux interfaces (cf. TelemetryManager)
class BanditApp : public SensorProvider, public BanditController {
  uint8_t  getCurrentGear() override    { return currentGear; }
  uint16_t getCurrentRPM() override     { return currentRPM; }
  uint16_t getCurrentGearRaw() override { return currentGearRaw; }
  void onOverride(bool en, int rpm, int g) override { setOverride(en, rpm, g); }
  void onCalibrateIMU() override        { calibrateIMU(); }
};
BanditApp banditApp;

// Dans setup()
settings.init();
if (!settings.load()) settings.save();  // défauts au 1er boot

initProtocol(settings, banditApp, banditApp);  // injection dans la couche I/O
```

### Traitement des commandes

```cpp
// Buffers pour accumulated input
static char serialBuffer[64];
static uint8_t serialCount = 0;

void processInput() {
  // Process Serial
  while (Serial.available()) {
    int c = Serial.read();
    if (c == '\n') {
      serialBuffer[serialCount] = 0;
      protocol.processCommand(serialBuffer);
      serialCount = 0;
    } else if (c != '\r' && c != ' ' && c != '\t') {
      if (serialCount < 63) {
        serialBuffer[serialCount++] = c;
      }
    }
  }
}

void loop() {
  processInput();
  
  // ... sensor reading (met à jour currentGear/currentRPM/currentGearRaw) ...
  
  // Les reports sont émis par processInput() qui tire les valeurs
  // via SensorProvider (banditApp) — aucun paramètre à passer.
}
```

---

## Avantages du découplage

| Aspect | Avant | Après |
|--------|-------|-------|
| **Responsabilités** | Mélangées | Séparées |
| **Testabilité** | Difficile | Mock-friendly |
| **Flexibilité** | Hard-codé | Composable |
| **Output** | IOutputStream custom | Print (Arduino standard) |
| **Dépendances** | Circulaires | Linéaires |

---

## Migration depuis ancien code

### Avant (tight coupling)
```cpp
// protocol.cpp accédait directement à g_settings et nrfuart
extern BLEUart nrfuart;
g_settings.gear1 = 1500;
OUTPUT_PRINT("OK");
```

### Après (loose coupling)
```cpp
// ProtocolHandler utilise SettingsManager& (injecté) + accesseurs typés
m_settings.setGear(1, 1500);
m_settings.save();

// Sortie via Print& passé au constructeur ; report tiré via SensorProvider
protocol.sendReport();
```

---

## Statut

Découplage **implémenté et intégré** :

- ✅ `SettingsStore` + backends (`FlashSettingsStore` / `BondSettingsStore`)
- ✅ `SettingsManager` source unique injectée (global `g_settings` supprimé)
- ✅ Interfaces `SensorProvider` / `BanditController`
- ✅ `ProtocolHandler` par injection (`Print&`, `SettingsManager&`, `SensorProvider&`, `BanditController&`)
- ✅ `protocol.cpp` réduit à la couche I/O (buffering Serial + BLE)
- ✅ `main.cpp` câblé via `BanditApp`
- ✅ Build validé (`pio run` : SUCCESS)

