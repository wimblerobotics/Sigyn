# Contact Fabrication Guide

Covers: power contact pads, leaf spring contacts, sense pogo pins,
wire sizing, and finishing treatments.

---

## 1. Power contact pads (robot side — 4 pieces)

### Material

Use **C360 free-machining brass** rod in **8 mm square cross-section**.  This
alloy (also sold as "360 brass" or "leaded free-machining brass") machines,
files, and taps cleanly, with good electrical conductivity
(~25 % IACS, resistivity ~68 nΩ·m) and adequate corrosion resistance for
indoor use.  For outdoor, upgrade to **C110 electrolytic tough-pitch copper**
which has 3× better conductivity and better corrosion resistance.

- McMaster-Carr: 8984K272 — C360 Brass, 8 mm Sq. × 300 mm, ~$12
- Stock needed: 4 × 20 mm = 80 mm + cut losses; one 300 mm stick is plenty.

### Cutting

1. Cut four pieces to **20 mm length** on a metal-cutting bandsaw or
   with a junior hacksaw and mitre block.  Burr the ends with a fine
   file — all surfaces should be square and smooth.

### Contact face (one 8 × 20 mm face)

This is the face that presses against the house-side spring contact.

1. File or sand this face flat on 220-grit wet/dry paper on a flat surface
   (glass plate is ideal).  Work through 320, 400, 600 grit.  The goal is
   a **flat, smooth surface** with a mirror-like finish on the high spots —
   this minimises contact resistance.
2. Lightly chamfer the four edges of the contact face: 0.5 mm × 45° with a
   fine file.  This prevents the spring contact from catching on a sharp edge
   during insertion.

### Wire attachment hole

Tap an **M4 × 0.7 blind hole** 8 mm deep into one of the **8 × 8 mm end
faces** (the short end).  This will receive an M4 socket cap screw + ring
terminal.

- Use a **M4 spiral-flute tap** (for brass — NOT a plug tap).
- Tap manually with cutting fluid (even WD-40 works for brass).
- Drill first: 3.3 mm drill bit, 8 mm deep.
- Thread: M4 × 0.7, 6 mm engagement minimum.  You will use an M4 × 10 mm
  socket cap screw to clamp the ring terminal.

> **Screw size rationale:** M4 socket cap screws are rated at roughly 2.5–3 kN
> clamping force at hand-tight torque (0.9 N·m).  A 10 AWG wire ring terminal
> (lug) under that force on a brass face gives well under 1 mΩ contact
> resistance — more than adequate for 20 A.

### Installing pads into the 3D-printed block

The pocket in the robot block is 9 × 21 mm; the pad is 8 × 20 mm — 0.5 mm
clearance each side.

1. Mix a small amount of **JB Weld SteelStik** or **Loctite EA 9462 epoxy**
   (or any high-gap-fill, electrically insulating, heat-resistant structural
   epoxy rated to at least 80 °C).
2. Apply a thin bead around three sides of the pad (not the contact face).
3. Press the pad into the pocket, contact face flush with the block face
   (or 0.2 mm recessed is acceptable).
4. Wipe off squeeze-out with IPA before it cures.
5. Cure 24 h at room temperature, or 1 h at 60 °C.

> Do **not** use CA glue.  It is too brittle and will crack under
> the spring contact load.

### Contact face treatment

Choose **one** of the following:

| Option | Effort | Life | Cost | Notes |
|--------|--------|------|------|-------|
| **Stabilant 22A** | Trivial | ~500 cycles | $30 / 15 mL | Re-apply every ~500 engagements; room temp is fine |
| Silver-plate (Caswell brush kit) | Moderate | 5000+ | $80 kit | Brush-plate only the contact face; 5–10 μm is sufficient |
| Gold-plate (Caswell) | Moderate | 10 000+ | $120 kit | Best for outdoor / humid environments |
| Tin-plate (Caswell) | Easy | 2000+ | $60 kit | Good all-rounder; slight contact resistance vs silver |

For the specified **2,000-cycle life** requirement, Stabilant 22A alone
(re-applied at 500-cycle intervals, ~4 re-applications) is sufficient and
requires no electroplating.  For a set-and-forget solution, silver-plate
and apply Stabilant 22A on top.

Stabilant 22A application: apply one drop to the contact face, spread with a
cotton bud.  It cures into a conductive polymer that fills micro-pores
while remaining active.

---

## 2. Power spring contacts (house side — 4 pieces)

### Material

**Phosphor-bronze Alloy 510 strip** (also called "spring bronze"):
- Thickness: **0.5 mm**
- Width: **15 mm**
- Per-piece length: **60 mm**
- McMaster-Carr: 9085K21 — 0.5 mm × 25 mm strip; or search "CuSn4 strip"
  on Digikey/Mouser.

The spring rate of this geometry (see below) is ~9 N/mm, giving ~9–13 N
contact force at 1–1.5 mm preload.  This is very comfortable for 2,000 cycles.

### Cutting

1. Cut four strips **60 mm × 15 mm** from the stock.
2. Deburr all four edges with a fine file.

### Forming the spring shape

```
TOP VIEW:                SIDE VIEW (as installed, protruding toward robot):

  base tab               fixed plate   floating plate face
  (M3 mount)                │                │
 ┌──────────┐               │  ┌────────────┘ ← spring base tab (flat, 25 mm)
 │ 25 mm    │               │  │
 │  flat    │               │  └────────────────
 └──────────┘               │               ↑ spring arm bends forward
       │                    │               │
       │ 90° bend           │               │ 35 mm arm
       │                    │               ↓
       │ spring arm         │  ────────────╮  ← contact tip (curved, 4 mm radius)
       │ (35 mm)            │              │
       │                    │              └── presses on robot brass pad
  ╭────╯ contact tip
  │ (4 mm radius convex)
```

1. Mark the bend line at **25 mm** from one end.
2. Clamp at the mark in a bench vise (smooth jaws — copper shims protect the
   surface) and bend to **90°** in one smooth motion.
3. The **25 mm base tab** mounts to the floating plate back.
4. The **35 mm spring arm** extends forward and contacts the robot-side pad.
5. **Contact tip radius:** bend the last 8 mm of the spring arm to a
   **4 mm convex radius**.  Use a 8 mm Ø steel rod clamped in the vise as a
   mandrel.  This ensures a consistent contact point regardless of small
   lateral offsets.

### Mounting holes in the base tab

1. Mark two hole centres along the 25 mm tab centreline: at 7 mm and 20 mm
   from the bend.
2. Centre-punch; drill 3.2 mm (M3 clearance).
3. Deburr both sides.

### Installing spring contacts

1. Offer the spring assembly up to the floating plate (contacts approach from
   the back face).
2. The base tab lies against the back face; two M3 × 8 mm socket cap screws
   (stainless) through the tab into the M3 tapped holes.  Torque: 0.3 N·m
   (finger-tight + ¼ turn).
3. With the tab screwed down, the spring arm passes forward through the
   slot in the floating plate and protrudes **3 mm** beyond the plate face at
   rest (no robot).  When the robot pad contacts it, the arm deflects back
   **~1 mm**, generating ~9 N preload force.

### Setting the correct preload

After mounting, check protrusion with a depth gauge or ruler.  Aim for
**3 ± 0.5 mm** protrusion.  Adjust by:
- Moving the base tab M3 screws to slightly different holes (if you make
  a slot instead of a point hole in the tab).
- Or bending the base tab angle slightly (a few degrees either way with
  a flat blade screwdriver under the tab edge).

### Contact face treatment

Same options as for the robot-side pads (Stabilant 22A is the minimum).
Apply to the **tip radius** area.

---

## 3. Sense contact pads (house side — 2 pieces)

The house side has two small **flat copper sense pads** that the robot-side
pogo pins contact.

### Material

Scrap PCB copper-clad FR4 works perfectly:
- Cut two pieces **8 × 10 mm** from a scrap FR4 panel.
- Alternatively: 0.5 mm thick copper sheet (McMaster 8964K17).

### Installation

1. The floating plate has two 8 × 12 mm × 3 mm recessed pockets for these
   shims (see mechanical_design.md §4.6).
2. Drill a 3.3 mm hole at the centre of each shim for the M3 mounting screw +
   ring terminal.
3. Press the shim into the pocket; it should be flush or 0.1 mm proud of the
   floating plate face.
4. Fasten with M3 × 6 mm pan-head screw + M3 ring terminal for the sense wire.
5. Apply Stabilant 22A to the top face.

---

## 4. Sense pogo pins (robot side — 2 pieces)

### Recommended part

**Mill-Max 0906-0-15-20-75-14-11-0**

- Body Ø: 5.08 mm (press-fit or can be retained with collar nut)
- Travel: 3.18 mm (spring travel)
- Current rating: 3 A (only milliamps needed for sense circuit; plenty of margin)
- Force: 0.74 N at full compression — adds to guide-in feel, negligible
- Tip style: flat (best for wiping against a flat copper pad)
- Digikey part: **3321-0-15-15-47-27-10-0-ND** or search Mill-Max 0906

### Installation into the 3D-printed block

The robot block has a **5.5 mm bore** through-hole for each pogo pin, with a
**8 mm × 5 mm counterbore** from the back for the retaining collar.

1. Press the pogo pin body into the 5.5 mm bore from the **back**.  The
   Mill-Max 0906 has a 5.08 mm body; in a 5.5 mm PETG bore it will be a
   snug-but-not-press fit.  Apply a small ring of **Loctite 638** retaining
   compound around the body before pressing in.  The collar seats in the
   counterbore, preventing forward ejection.
2. Cure 30 min.
3. **Protrusion check:** the tip of the pogo pin in its extended (unloaded)
   state should extend at least **5 mm beyond the face of the robot block** —
   more than the power pad surface.  This ensures the sense pins close the
   circuit before the power pads make contact.
4. Run wires: 24 AWG hookup wire to the back of each pogo pin.  Mill-Max pogo
   pins can be soldered — use a 350 °C iron, rosin flux, 63/37 solder.  Apply
   solder to the back of the pin barrel, then trim to 3 mm, slide on heat-
   shrink, attach wire.  Alternatively, use the solder-cup variant
   **Mill-Max 0906-0-15-20-75-14-11-0** which has a solder cup.

> **Alternative pogo pin (through-hole / solderable):**
> If you prefer a solderable pin with a defined PCB-style footprint, use
> **Harwin P70-1000045R** (2.5 mm Ø body, M2.5 thread, 3 A).  Thread it into
> an M2.5 heat-set insert pressed into the 3D-printed block.  This gives
> positive mechanical retention and easy replacement.  Digikey: P70-1000045R-ND.

---

## 5. Guide pin fabrication

### Material and source

Standard **6061-T6 aluminium round bar, 18 mm Ø** (or nearest metric size
available in your area — 18 mm is a clean metric standard).

- McMaster-Carr: 1600N122 — 18 mm OD × 300 mm Al 6061-T6 rod
- Cut one 70 mm piece.

### Machining steps

The pin can be made on a lathe, but with basic tools you can do it as follows
if you don't have a lathe:

1. **Cut to length:** 70 mm ± 0.5 mm.  A metalworking bandsaw or hacksaw
   with a mitre block.
2. **Face both ends:** File both ends dead square using a square as reference.
   A disc sander with a mitre gauge works well.
3. **Drill centre hole:** 8.5 mm through the full 70 mm length.  This requires
   a drill press.  Centre-punch exactly at the centre of each end face first.
   Use a 8.5 mm bit in three passes (pilot at 3 mm, intermediate at 6 mm,
   final at 8.5 mm).  Keep the rod perfectly vertical.  This through-hole
   accepts the M8 retaining bolt.
4. **Tip chamfer:** 45° chamfer on one end (the "entry" tip).  File at 45° by
   eye, checking with a protractor.  8 mm of material removed creates a
   clean leading edge.  Smooth with 220-grit emery cloth.
5. **Anodise (optional but recommended):** clear or black hard anodise.
   For a single-piece small quantity, send to a local anodising shop.  Hard
   anodise (Type III) gives a 20–60 μm layer with very high wear and
   corrosion resistance.  This is worth doing for the outdoor version.
   Indoor: bare aluminium with a wipe of oil is adequate.

### Installation into the robot block

1. Insert pin into the 18 mm bore (30 mm deep) from the front face.
2. From the back of the robot block, insert an **M8 × 70 mm socket cap
   screw** through the 8.5 mm bore, through the guide pin centre hole, and
   tighten into an **M8 hex nut captured in the hex pocket** on the back face.
3. The M8 bolt head bears against the inside back wall of the block; the nut
   is in the hex pocket.  Torque: 15 N·m.
4. The pin should be immovable after torquing.  If the fit in the 18 mm bore
   is loose (bore printed slightly over-size), apply thin CA glue around the
   pin circumference before inserting and torquing.

---

## 6. Wire sizing and crimp specifications

### Power wiring (48 V, 20 A max)

| Parameter | Value |
|-----------|-------|
| Wire gauge | **10 AWG** (5.26 mm² Cu) |
| Insulation | Silicone (rated 200 °C) — required for flexibility and heat tolerance |
| Conductor strand | Fine-stranded (Class K / 30 AWG strands) for flexibility |
| Ampacity at 10 AWG | 30 A in free air — 20 A is comfortable with plenty of margin |
| Wire colour | Red for +48 V, Black for GND (standard convention) |
| Crimp terminal | **M4 ring terminal, 10 AWG**, hexagonal crimped, tinned Cu |
| Recommended terminal | **Panduit DNF14-14LF-M** (M4, 10–14 AWG) or equivalent |
| Crimp tool | Ratchet crimper with M4 hex die (10–12 AWG) |

> Wire from each robot-side pad to the battery charger output (when docked)
> is approximately 300 mm.  Use a wiring loom: two +48 V wires bundled,
> two GND wires bundled, secured with nylon spiral wrap.

### Sense wiring (3.3 V, milliamps)

| Parameter | Value |
|-----------|-------|
| Wire gauge | **24 AWG** twisted pair |
| Insulation | PTFE or PVC, 24 AWG |
| Connector to robot harness | 2-pin JST-XH 2.54 mm pitch |
| Ring terminal at pad | M3 ring terminal for sense pad screw |

---

## 7. Maintenance schedule

| Interval | Task |
|----------|------|
| Every 500 engagements | Re-apply Stabilant 22A to robot power pads and spring contact tips |
| Every 500 engagements | Inspect pogo pins for tip wear; replace if tip is visibly worn or pitted |
| Every 1000 engagements | Inspect spring contacts for fatigue cracks; replace strip if cracked |
| Every 1000 engagements | Inspect guide pin tip chamfer and socket funnel for wear grooves |
| Every 2000 engagements | Full disassembly, clean all contacts with IPA, re-apply Stabilant |
| Outdoor only / annually | Check all sealed penetrations; replace any cracked gasketing |
