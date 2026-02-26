# Sigyn Autonomous Docking / Charging Station

**Design version:** 1.0  
**Date:** 2026-02-25  
**Battery:** 48 V LiFePO₄, 50 Ah  
**Max charge current:** 20 A (960 W)  
**Expected engagement life:** 2,000+ cycles  
**Environments:** Indoor (v1), Outdoor GFCI (v2)

---

## Design summary

Sigyn drives into the station front-first, guided by April Tags for coarse
alignment and a physical funnel for final alignment.  A spring-loaded **floating
contact plate** on the house side accommodates the residual error after the
funnel.  A central **guide pin** (robot side, male) seats in a **guide socket**
(house side, female) to achieve precise contact registration.  Two dedicated
sense contacts verify seating before the charger is powered.

```
SIDE VIEW — ROBOT DOCKED
─────────────────────────────────────────────────────────────────────
                          Robot chassis (circular, aluminum extrusion)
                        ┌────────────────────────────────────────────
                        │
   Wall  ──────────┐    │   ┌──────────────────┐
                   │    │   │  Robot-side block │
   ╔═══════════╗   │    ╰───┤  100×100×50 mm   │
   ║  48 V     ║   │        │  (ASA printed)    │
   ║  Charger  ║◄──┤        ╞══════════════════╡ ← power pads touch
   ║           ║   │        │ ←── guide pin ──→ │   spring contacts
   ║  Teensy   ║   │        │                   │
   ║  control  ║   │ Tower  └──────────────────┘
   ╚═══════════╝   │ (fixed to wall)
   ────────────────┘
   Floor/wall anchor
─────────────────────────────────────────────────────────────────────
```

---

## Document index

| File | Contents |
|------|----------|
| [mechanical_design.md](mechanical_design.md) | Complete Fusion 360 modelling steps for every component, plus drawing/view instructions |
| [contact_fabrication.md](contact_fabrication.md) | How to make and finish the brass power pads, spring contacts, and sense pogo pins; wire sizing; screw specs |
| [electronics.md](electronics.md) | System block diagram, ASCII schematic, Teensy 4.1 control logic, specific Digikey part numbers |
| [bom.md](bom.md) | Full bill of materials (hardware, electronics, consumables), separate columns for indoor and outdoor variants |

---

## Key design decisions

### Contact topology

```
Robot face (front view, 100 × 100 mm)
┌─────────────────────────────────────────────┐
│                                             │
│  ┌──────────┐               ┌──────────┐   │
│  │ +48 V  A │               │  GND   A │   │
│  │ 8×20 mm  │               │ 8×20 mm  │   │
│  │  brass   │               │  brass   │   │
│  └──────────┘               └──────────┘   │
│                  ╔════════╗                 │
│   [S+]           ║ GUIDE  ║          [S−]  │
│  sense+          ║  PIN   ║         sense− │
│  (pogo)          ║ 18 mm Ø║         (pogo) │
│                  ╚════════╝                 │
│  ┌──────────┐               ┌──────────┐   │
│  │ +48 V  B │               │  GND   B │   │
│  │ 8×20 mm  │               │ 8×20 mm  │   │
│  └──────────┘               └──────────┘   │
└─────────────────────────────────────────────┘

  Pad centres:   ±35 mm X,  ±32 mm Y from block centre
  Sense pin centres: ±18 mm X, 0 Y
  Guide pin: (0, 0), 18 mm Ø, protrudes 40 mm
```

- **4 power pads** (2 × +48 V, 2 × GND) wired in parallel — each pair handles
  10 A nominal, 15 A peak with wide margin.
- **2 sense pogo pins** (robot side) **protrude 5 mm further** than the power pad
  faces so they make contact first and break contact last.
- **1 central guide pin** (aluminum, anodized) — mechanical alignment only,
  not an electrical contact.

### Alignment strategy

```
April Tags → Nav2 parks robot within ±80 mm of dock face
        ↓
  Fixed entry funnel: 220×180 mm entry tapers to 112×112 mm over 80 mm depth
        ↓
  Floating contact plate: ±5 mm spring-centered float (X and Y)
        ↓
  Guide pin chamfer (45°) + socket funnel (30°) achieve < 0.5 mm final error
```

### Connection verification sequence

```
1. Robot drives forward → funnel accepts block
2. Guide pin seats in guide socket (hard stop)
3. Sense pogo pins contact sense pads on robot first (+5 mm offset)
4. Teensy reads ADC on sense circuit:
     open circuit (~3.3 V)  →  not engaged
     ~1.65 V                →  sense circuit closed = robot present
     <0.3 V                 →  short / fault
5. Teensy waits 1.5 s (debounce / confirm robot stopped)
6. Teensy measures sense-circuit resistance via ADC; if < 50 Ω, enables relay
7. Relay closes → AC power to charger
8. Teensy monitors charger output voltage via voltage divider → logs charging session
9. Robot signals "charge complete" via ROS topic → Teensy opens relay
10. Robot reverses out → sense pins break → Teensy opens relay as backup
```

### Outdoor vs indoor variants

| Feature | Indoor | Outdoor |
|---------|--------|---------|
| Printed material | PETG (40 % infill) | ASA (40 % infill, UV-stable) |
| Hardware | A2 stainless | A4 (316) stainless |
| Charger enclosure | Open shelf / printed cover | NEMA 4X powder-coated steel |
| AC inlet | Standard NEMA 5-15 | GFCI NEMA 5-15R with in-use cover |
| Contact face sealing | None | EPDM gasket ring, compressed on dock |
| Cable entries | Open | IP68 cable glands |
| Contact treatment | Stabilant 22A every ~500 cycles | Silver-plated contacts + Stabilant 22A |
