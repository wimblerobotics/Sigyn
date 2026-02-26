# Electronics Design

Control board: **Teensy 4.1** with your custom expander board  
Max charge current: **20 A at 48 V** (960 W)  
AC switching: **25 A SSR on the 120 VAC charger input line**

---

## 1. System block diagram

```
                        120 VAC IN
                             │
                        ┌────┴────┐
                        │  GFCI   │  (outdoor only — indoor uses wall
                        │ outlet  │   GFCI at breaker panel)
                        └────┬────┘
                             │
                     ┌───────┴────────┐
                     │  Crydom D2425  │  25 A solid-state relay
                     │  (SSR)         │  Control: 3–32 VDC input
                     └───────┬────────┘
                             │ AC (switched)
                     ┌───────┴────────┐
                     │  48 V LiFePO₄ │  Charger module (20 A max)
                     │  Charger       │  See BOM for part suggestions
                     └───────┬────────┘
                             │ 48 VDC (+) and (−)
                             │
              ┌──────────────┼──────────────┐
              │              │              │
           Fuse 25 A    ┌────┴────┐     (to wiring loom
          (blade type)  │  Teensy │      to spring contacts)
                        │  4.1 +  │
                        │ expander│
                        └────┬────┘
                             │
              ┌──────────────┼───────────────────────────┐
              │              │                           │
       SENSE CIRCUIT   SSR CONTROL            STATUS LED / DISPLAY
       (see §3)        GPIO to SSR            / WiFi / Logging
```

---

## 2. Power supply for Teensy and control

The dock needs a low-power 5 V supply independent of the charger.

Options:

1. **Simple:** A small 5 V USB-C wall adapter plugged into the tower enclosure,
   powers the Teensy via USB.  Zero cost if you have a spare adapter.
2. **Integrated:** A buck converter module (e.g., Murata OKI-78SR-5/1.5-W36
   or Digikey ~$8 generic) taking 12 V from a small wall-wart, providing 5 V.
3. **From battery (when docked):** A small 48 V → 5 V buck module
   (e.g., Mean Well SD-15B-5) — **only** when robot is docked and battery is
   connected.  Not recommended as the sole supply; the dock needs to be alive
   before the robot arrives.

**Recommendation:** Use option 1 for prototyping.  Add option 2 in a clean
installation inside the NEMA 4X enclosure for the outdoor unit.

---

## 3. Sense circuit

### Schematic

```
Teensy 3.3 V ──── R1 (10 kΩ) ──┬──── SENSE_PIN_A (house-side pad)
                                │
                               ADC0 pin (e.g. Teensy pin A0)
                                │
                              R2 (10 kΩ)
                                │
                               GND

SENSE_PIN_B (house-side pad) ──── directly to GND (Teensy GND)


On the robot side:
  POGO_A ───── R_sense (100 Ω, 1/4 W) ───── POGO_B
  (both pogos are wired through a 100 Ω resistor on the robot PCB/harness)
```

### How it works

| Robot state | Circuit | Teensy ADC0 reads |
|-------------|---------|-------------------|
| Not docked | Open (both pads floating) | ~3.3 V (pulled to Vcc via R1) |
| Docked, contact good | R1 + R_sense + R2 divider: 10k + 100 + 10k | ~1.65 V |
| Docked, poor contact | R1 + R_contact (high) + R2 | 1.65–3.3 V (ADC can detect degraded contact) |
| Sense wire short / fault | Near 0 V | < 0.1 V → fault condition |

ADC threshold values at 3.3 V reference:
- `DOCKED_THRESHOLD_LOW  = 1.2 V` (ADC count ~1490)
- `DOCKED_THRESHOLD_HIGH = 2.1 V` (ADC count ~2605)
- Values between these two → robot is docked and contact is good → proceed
- Above HIGH → not docked
- Below LOW → fault (short) → do NOT enable relay

This provides **contact quality measurement on every docking** — log the
exact ADC value to see degradation trend over thousands of engagements.

---

## 4. SSR control circuit

```
Teensy GPIO (e.g. pin 2)  ───── R3 (470 Ω) ─────┬──── SSR Control (+)
                                                 │
                                                LED1 (3 mm green, forward
                                                 │     voltage ~2.2 V —
                                                GND    visible status that
                                                       relay is commanded ON)

SSR Control (−) ────────────────────────────── GND

SSR Load terminals: series with Line (L) of 120 VAC feed to charger.
Neutral (N) is connected directly; only switch one leg.
```

The Crydom D2425 has a 3–32 VDC control input (draws ~15 mA at 3.3 V).
The Teensy 3.3 V GPIO can source this directly, but it is healthier to buffer
through your expander board's driver if available.

---

## 5. Charger output voltage monitoring (optional but recommended)

Monitor the charger's 48 V output for logging and state detection.

```
Charger (+48 V) ──── R4 (150 kΩ, 1%) ──┬──── ADC1 (Teensy pin A1)
                                        │
                                       R5 (10 kΩ, 1%)
                                        │
                                       GND
```

Voltage divider: 10k / (10k + 150k) = 0.0625  
48 V × 0.0625 = **3.0 V** at full charge → just within Teensy 3.3 V ADC range.  
Resolution: ~3.0 V / 1024 counts ≈ 47 mV/count (10-bit) or 3 mV/count (12-bit —
Teensy 4.1 supports 12-bit ADC).  More than adequate for logging.

---

## 6. Status LED and buzzer (optional)

```
Teensy pin 3 ──── R6 (100 Ω) ──── LED2_RED (anode)   ──── GND
Teensy pin 4 ──── R7 (100 Ω) ──── LED2_GREEN (anode) ──── GND
Teensy pin 5 ──── R8 (470 Ω) ──── Piezo buzzer (5 V) ──── GND
                                  (use BJT driver: 2N2222 + 1 kΩ base resistor
                                   if buzzer draws >20 mA)
```

LED status codes (suggestions for firmware):

| LED state | Meaning |
|-----------|---------|
| Green solid | Robot docked, charging |
| Green slow blink (1 Hz) | Charge complete |
| Red blink (2 Hz) | Sense fault / poor contact |
| Red solid | SSR or charger error |
| Off | No robot present, standby |

---

## 7. Full ASCII schematic

```
                           120 VAC INPUT
    ┌──────────────────────────────────────────────────────────────┐
    │                                                              │
   L (Hot) ─────────── SSR D2425 (Load+) ─── SSR (Load−) ────── CHARGER AC IN (L)
   N (Neutral) ─────────────────────────────────────────────────── CHARGER AC IN (N)
   PE (Ground) ─────────────────────────────────────────────────── CHARGER PE
    │                                                              │
    │  SSR Control:                                               │
    │  Teensy pin 2 ── R3 470Ω ──┬── SSR Ctrl(+)                │
    │                            └── LED1 green ── GND           │
    │  GND ───────────────────────── SSR Ctrl(−)                 │
    └──────────────────────────────────────────────────────────────┘

                        CHARGER DC OUTPUT
    ┌──────────────────────────────────────────────────────────────┐
    │                                                              │
    │  Charger(+48V) ─── Fuse 25A ─── SPRING CONTACTS (+48V A,B) │
    │  Charger(GND)  ─────────────── SPRING CONTACTS (GND A,B)   │
    │                                                              │
    │  Charger(+48V) ─── R4 150kΩ ─┬─ ADC1 (Teensy A1)          │
    │                               └─ R5 10kΩ ─── GND           │
    └──────────────────────────────────────────────────────────────┘

                        SENSE CIRCUIT
    ┌──────────────────────────────────────────────────────────────┐
    │  3.3V ── R1 10kΩ ─┬─ SENSE_PAD_A (house side flat pad)     │
    │                   └─ ADC0 (Teensy A0)                       │
    │                                                              │
    │  SENSE_PAD_B ─────── GND                                    │
    │                                                              │
    │  [Robot side: POGO_A ── 100Ω ── POGO_B — bridged internally]│
    └──────────────────────────────────────────────────────────────┘

                        STATUS OUTPUT
    ┌──────────────────────────────────────────────────────────────┐
    │  pin 3 ── R6 100Ω ── LED_RED  ── GND                        │
    │  pin 4 ── R7 100Ω ── LED_GRN  ── GND                        │
    │  pin 5 ── R8 470Ω ── 2N2222(B) ── [Collector: Buzzer ─ 5V] │
    │                              └── [Emitter: GND]             │
    └──────────────────────────────────────────────────────────────┘
```

---

## 8. Teensy 4.1 firmware outline

```cpp
// Pin assignments
const int PIN_SSR        = 2;    // SSR control (OUTPUT)
const int PIN_ADC_SENSE  = A0;   // Sense circuit voltage
const int PIN_ADC_VBATT  = A1;   // Charger output voltage monitor
const int PIN_LED_RED    = 3;
const int PIN_LED_GREEN  = 4;
const int PIN_BUZZER     = 5;

// Thresholds (12-bit ADC, 3.3 V reference)
// Sense: 1.65 V nominal when docked → 1.65/3.3 * 4096 = 2048
const int SENSE_LOW_ADC  = 1489;  // 1.2 V
const int SENSE_HIGH_ADC = 2606;  // 2.1 V

// State machine
enum DockState { UNDOCKED, DEBOUNCE, VERIFYING, CHARGING, FAULT };
DockState state = UNDOCKED;

unsigned long debounceStart = 0;
const unsigned long DEBOUNCE_MS = 1500;

// Variables for logging (write to SD card or WiFi POST)
struct ChargeSession {
  unsigned long startTime;
  float peakVoltage;
  float finalVoltage;
  int   senseAdcAtDock;
  unsigned long durationMs;
};

void setup() {
  pinMode(PIN_SSR,       OUTPUT); digitalWrite(PIN_SSR, LOW);
  pinMode(PIN_LED_RED,   OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);
  analogReadResolution(12);  // Teensy 4.1: 12-bit ADC
  // Initialize SD, WiFi, display here
}

void loop() {
  int senseAdc = analogRead(PIN_ADC_SENSE);

  switch (state) {
    case UNDOCKED:
      setLed(OFF);
      if (senseAdc >= SENSE_LOW_ADC && senseAdc <= SENSE_HIGH_ADC) {
        debounceStart = millis();
        state = DEBOUNCE;
      }
      break;

    case DEBOUNCE:
      if (millis() - debounceStart >= DEBOUNCE_MS) {
        senseAdc = analogRead(PIN_ADC_SENSE);  // re-read after settling
        if (senseAdc >= SENSE_LOW_ADC && senseAdc <= SENSE_HIGH_ADC) {
          state = VERIFYING;
        } else {
          state = UNDOCKED;  // transient — robot moved
        }
      }
      break;

    case VERIFYING:
      // Sense ADC in window → enable charger
      logSession.senseAdcAtDock = senseAdc;
      logSession.startTime = millis();
      digitalWrite(PIN_SSR, HIGH);
      setLed(GREEN_SOLID);
      state = CHARGING;
      break;

    case CHARGING:
      // Monitor for undock or charge complete
      senseAdc = analogRead(PIN_ADC_SENSE);
      if (senseAdc < SENSE_LOW_ADC || senseAdc > SENSE_HIGH_ADC) {
        // Robot undocked
        digitalWrite(PIN_SSR, LOW);
        logSession.durationMs = millis() - logSession.startTime;
        writeLog(logSession);
        state = UNDOCKED;
      }
      if (senseAdc < 100) {  // fault — short circuit on sense
        state = FAULT;
      }
      // Optional: check ADC_VBATT for charge complete voltage
      break;

    case FAULT:
      digitalWrite(PIN_SSR, LOW);
      setLed(RED_SOLID);
      // Alert via WiFi; require manual reset
      break;
  }
}
```

> This is a sketch outline.  Add your display driver, SD logging, WiFi
> MQTT or HTTP POST, and ROS2 serial bridge (the Teensy can publish a
> `/dock_status` topic over serial to the robot's on-board computer).

---

## 9. Component list with Digikey part numbers

| Qty | Description | Mfr. Part | Digikey Part | Est. Cost |
|-----|-------------|-----------|--------------|-----------|
| 1 | SSR 25 A 24–280 VAC, 3–32 VDC ctrl | Crydom D2425 | 425-2590-ND | $25 |
| 1 | 48 V LiFePO₄ charger, 20 A, CC/CV | See note 1 | — | $60–100 |
| 1 | Teensy 4.1 | PJRC | 1568-DEV-16781-ND | $32 |
| 1 | 5 V / 1 A USB-C adapter | any | — | $8 |
| 1 | 10 kΩ 1/4 W 1% resistor × 5 pack | Vishay | — | $1 |
| 1 | 150 kΩ 1/4 W 1% resistor × 2 pack | Vishay | — | $1 |
| 1 | 470 Ω 1/4 W resistor × 2 pack | Vishay | — | $1 |
| 1 | 100 Ω 1/4 W resistor (robot side) | Vishay | — | $0.50 |
| 1 | 2N2222A BJT (NPN, TO-92) | onsemi | 2N2222A-ND | $0.50 |
| 2 | 3 mm LED green + red | Cree | — | $1 |
| 1 | Piezo buzzer 5 V passive | TDK | 445-2528-ND | $2 |
| 1 | 25 A blade fuse + inline holder | Littelfuse | F2551-ND | $4 |
| 2 | Mill-Max 0906 pogo pin (sense) | Mill-Max | 3321-0-15-15-47-27-10-0-ND | $4 each |
| 1 | Stabilant 22A 15 mL | Miller-Stephenson | — | $30 |
| 1 | M8 × 70 mm socket cap screw (guide pin) | A2 SS | — | $2 |
| 4 | M4 × 10 mm socket cap screw (pad mounting) | A2 SS | — | $3 pack |
| 2 | M3 × 6 mm pan head screw (sense pad) | A2 SS | — | $3 pack |
| 4 | M3 × 8 mm socket cap screw (spring base) | A2 SS | — | $3 pack |
| 2 | JST-XH 2-pin connector set (sense wiring) | JST | 455-2247-ND | $2 |
| 1 | 10 AWG silicone wire 1 m red + 1 m black | BNTECHGO | — | $8 |
| 4 | M4 ring terminal 10 AWG (power pads) | Panduit | DNF14-14LF-M-ND | $5 pack |
| 1 | C360 brass sq. rod 8 mm × 300 mm | — | McMaster 8984K272 | $12 |
| 1 | Phosphor-bronze Alloy 510 strip 0.5×25×250 mm | — | McMaster 9085K21 | $15 |
| 1 | 6061 Al round rod 18 mm Ø × 300 mm | — | McMaster 1600N122 | $10 |

**Note 1 — Charger:** For a 48 V 20 A LiFePO₄ charger, look for:
- **Mean Well ENC-120-48** (120 W, close but may need 2 in parallel for 20 A)
- **Genasun GVX-48V-20A** (specialty LiFePO₄ charger with proper CC/CV/BMS
  communication — best choice)
- **ICHARGER 4010 DUO** (bench supply with LiFePO₄ profile)
- Or source any 48 V / 20 A CCCV power supply and set the CV to the correct
  LiFePO₄ full-charge voltage for your pack (number of cells × 3.65 V).

**Total estimated cost (electronics only):** ~$200–280 depending on charger
choice.

---

## 10. Outdoor modifications

For the outdoor version:

1. House the entire control board (Teensy + SSR) inside a **NEMA 4X** (IP66)
   enclosure.  The SSR must be mounted on the metal enclosure wall (or a
   separate heat sink) — at 25 A it will dissipate up to 50 W.
2. Use **IP68 cable glands** (M20 for 10 AWG wiring loom, M12 for sense wires,
   M16 for AC feed).
3. **GFCI protection:** either a GFCI circuit breaker at the panel, or a
   GFCI outlet as the dock's inlet.  Leviton 7899 is a good outdoor-rated
   GFCI outlet with in-use cover.
4. Apply **conformal coat (MG Chemicals 422B)** to the Teensy and all PCBs.
5. Replace all A2 stainless hardware with **A4 (316) stainless**.
