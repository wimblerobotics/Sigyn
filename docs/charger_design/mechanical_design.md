# Mechanical Design — Fusion 360 Step-by-Step Guide

This document covers every component of the docking station in modelling
order.  Steps reference the **Fusion 360 2024/2025 Design workspace** UI.
Keyboard shortcuts are shown in parentheses.

---

## 0. File and project organisation

1. Open Fusion 360.  In the **Data Panel** (top-left cloud icon), create a new
   project called `Sigyn_Docking_Station`.
2. Inside that project, create the main **assembly** file:
   `File → New Design`.  Immediately `Save (Ctrl+S / Cmd+S)` as
   `DockingStation_Assembly`.
3. In the **Browser** (left panel), rename `Component1` → `DockingStation`.
4. All individual parts will be created as **sub-components** of this assembly.
   That way you can generate drawings with multiple views from the assembly
   file later.

> **Tip — components vs bodies.**  Always create a new component
> (`Assemble → New Component`) before starting a new part.  This keeps the
> Browser clean and allows independent appearance, material, and joint
> assignment.

---

## 1. Robot-side contact block

### 1.1 Create the component

1. `Assemble → New Component` (or right-click on the root component in the
   Browser → `New Component`).  Name it `RobotBlock`.
2. Click the radio button next to `RobotBlock` in the Browser to activate it.
   Any sketches and features you create now belong to this component.

### 1.2 Outer body

1. `Solid → Sketch → Create Sketch`.  Select the **YZ plane** (the plane that
   will become the robot-facing contact face).
2. Press `R` (Rectangle).  Choose `Center Rectangle`.  Click the origin as
   centre.  Type `100` Tab `100` Enter.  You should have a 100 × 100 mm square
   centred on the origin.
3. Press `D` (Dimension) to confirm: add two smart dimensions confirming 100 mm
   width and 100 mm height.
4. `Finish Sketch` (green tick or `Shift+S`).
5. Press `E` (Extrude).  Select the square profile.  Direction: **one side**,
   distance `50` mm, direction: **negative X** (into the robot body).  `OK`.
6. Press `F` (Fillet).  Select all four back edges (the four edges of the
   50 mm-deep face furthest from the contact face).  Radius: `3` mm.  `OK`.

> The result is a 100 × 100 × 50 mm block with the contact face on the YZ plane.

### 1.3 Guide pin socket (centre of contact face)

The guide pin is on the **robot side** (male, protrudes outward).  The socket
is on the **house side**.  The robot block has a **mounting bore** to hold the
guide pin.

1. `Solid → Sketch → Create Sketch`, select the **front face** (the YZ-plane
   100 × 100 face).
2. Press `C` (Circle).  Click origin.  Diameter `18` mm.  This is the through-bore
   for the pin shaft retention section.
3. Finish Sketch.
4. `E` (Extrude).  Profile = 18 mm circle.  Operation = **Cut**.  Distance =
   `30` mm into the block (not all the way through).  `OK`.
5. Create another sketch on the same face.  Draw a circle, diameter `8` mm,
   concentric with origin.  Finish Sketch.
6. `E` → Cut → distance = **through all** (50 mm).  This 8 mm bore is the bolt
   clearance hole for the M8 retaining bolt that holds the guide pin.
7. On the **back face** of the block, create a sketch.  Draw a `14` mm
   diameter circle concentric with the 8 mm hole.  Finish Sketch.
8. `E` → Cut → distance = `8` mm into block.  This creates the counterbore for
   the M8 hex nut (wrench-flat to wrench-flat: standard M8 nut = 13 mm — make
   this pocket 14 mm Ø to give 0.5 mm clearance per side, or use a 14 mm ×
   12 mm hex pocket: see step below for hex pocket).
9. To make the nut pocket hex-shaped: on the back face, create a sketch.
   `Solid → Sketch → Create Sketch`.  Use `Inscribed Polygon` (6 sides,
   inscribed in a 14 mm circle).  Finish Sketch.  `E` → Cut → `8` mm.
   Delete the circle extrude from step 8 in the timeline if you already did it,
   or suppress it and replace with the hex pocket.

### 1.4 Power pad pockets (× 4)

Each pocket holds one 8 × 20 mm brass contact pad.  The pocket is slightly
larger to allow epoxy bedding.

Pad pocket dimensions: **9 × 21 mm**, depth **10 mm**.

Pad centre positions (from block centre, in the YZ contact face plane):

| Pad | Y offset | Z offset |
|-----|----------|----------|
| +48 V A | −35 mm | +32 mm |
| +48 V B | −35 mm | −32 mm |
| GND A   | +35 mm | +32 mm |
| GND B   | +35 mm | −32 mm |

1. `Solid → Sketch → Create Sketch` on the **contact face**.
2. Draw four **9 × 21 mm** centre-rectangles (`R` → `Center Rectangle`) at the
   four positions above.  Use `D` (Dimension) to place each centre point
   accurately from the origin.
3. Add `2` mm corner fillets to each rectangle (`F` while in sketch, or use the
   `Fillet` sketch tool).
4. Finish Sketch.
5. `E` → Cut → `10` mm.  Select all four profiles in one extrude operation by
   Shift-clicking all four rectangles.  `OK`.

### 1.5 Sense pogo pin through-holes (× 2)

The sense pins are pogo pins pressed into a 5.5 mm bore from the front.  They
protrude 5 mm further forward than the contact face (this is achieved by the
spring inside the pogo pin — see contact_fabrication.md).

1. Create sketch on the **contact face**.
2. Draw two circles, diameter `5.5` mm, at positions:
   - Left sense: Y = −18 mm, Z = 0
   - Right sense: Y = +18 mm, Z = 0
3. Finish Sketch.
4. `E` → Cut → **Through All** (50 mm).
5. On the back face, create counterbores for pogo pin retention collars:
   sketch two circles, `8` mm Ø, concentric with the sense holes.  Extrude
   Cut `5` mm.  The collar on the pogo pin seats in this counterbore.

### 1.6 Wire channel routing on back face

Wires exit from each pocket/bore through the back face.  Route channels connect
pads of the same polarity (+48 V A to +48 V B, GND A to GND B) and then
exit at the top edge.

1. Create sketch on the **back face** (50 mm in from contact face).
2. Draw two 8 × 8 mm square channels:
   - +48 V channel: connects both +48 V pad back-bores (left side of block),
     route a U-shape around the guide pin area, exit at top-left corner.
   - GND channel: mirrors it on the right side.
3. Use `Line (L)` to sketch the channel centreline, then `Offset Entities`
   4 mm each side to make the channel walls.  Close the ends.
4. Finish Sketch.  `E` → Cut → `8` mm deep.
5. These channels are 8 mm wide × 8 mm deep — sufficient for 10 AWG silicone
   wire (6.5 mm OD including insulation).

### 1.7 Mounting holes (× 4)

The block mounts to a 2020 extrusion platform on the robot via M5 screws.

1. Create sketch on any convenient side face (or on the contact face — it works
   either way).
2. Four circles, `5.3` mm Ø (M5 clearance), at:
   - (±40 mm Y, ±40 mm Z) — 20 mm in from each corner.
3. Finish Sketch.  `E` → Cut → Through All.
4. On the back face, counterbore each: 9 mm Ø × 5 mm deep (for M5 socket cap
   head, head height = 5 mm, OD = 8.5 mm).

### 1.8 Appearances / material for visualisation

1. In the Browser, right-click the `RobotBlock` body → `Appearance`.
2. Assign `Plastic > ABS (Black)` or `Generic Plastic` — this is the 3D-printed
   ASA body.
3. You will assign material to the guide pin component separately.

---

## 2. Guide pin (robot side, male)

### 2.1 Create the component

`Assemble → New Component`, name `GuidePin_Robot`.  Activate it.

### 2.2 Body of the guide pin

The pin is a two-diameter stepped aluminium rod:

- **Front shaft:** 18 mm Ø × 40 mm long (protrudes from robot block face)
- **Retention shaft:** 18 mm Ø × 30 mm long (sits inside the 18 mm bore in the block)
- **Tip chamfer:** 45° × 4 mm (on the front end)
- **Through bore:** 8.5 mm Ø (M8 clearance, through full length for retaining bolt)

Because the retention and front sections are the same diameter (18 mm), the
chamfer at the rear should be a shoulder — but since the bore in the block is
also 18 mm, the pin is retained purely by the M8 bolt from the back.  This
is intentional: it allows pin replacement.

1. Create sketch on the **XZ plane**.
2. Draw the pin profile (half-profile for revolve):
   - Start at origin (axis of rotation is the X axis)
   - Draw a 9 mm horizontal line upward (radius of 18 mm Ø) at X = 0
   - Line right (along X) 70 mm (full pin length)
   - Line down back to the axis (9 mm)
   - Close the profile
3. Draw the through-bore: centre rectangle 4.25 mm wide × 70 mm long
   (4.25 mm = 8.5/2) centred on the X axis.  This cuts the bore from the
   revolve.
4. Finish Sketch.
5. `Solid → Create → Revolve`.  Select the outer profile (excluding bore).  Axis:
   select the X axis.  Angle: 360°.  Operation: `New Body`.  `OK`.
6. `E` → Cut the bore profile (the rectangle) as a revolve (or just extrude
   cut a circle: create a sketch on the tip face, draw 8.5 mm Ø circle at
   centre, extrude cut through all).
7. **Tip chamfer:** `Solid → Modify → Chamfer`.  Select the front circular edge.
   Distance = `4` mm, Angle = `45°`.  `OK`.

### 2.3 Position the guide pin in the assembly

1. Activate the **root assembly** (click the radio button next to
   `DockingStation` in the Browser).
2. `Assemble → Joint`.  Select the cylindrical face of the guide pin retention
   shaft as one component, and the cylindrical bore inside the RobotBlock as
   the other.  Fusion will snap them concentric.  Set the offset along axis to
   `0` (pin shoulder flush with block face).  `OK`.

### 2.4 Appearance

Right-click `GuidePin_Robot` body → Appearance → `Metal > Aluminum 6061` (or
similar).  If you want it anodized: choose `Anodized Black Aluminum`.

---

## 3. House-side fixed back plate

This plate mounts directly to wall studs (or a plywood backing panel).

Dimensions: **220 × 220 mm face, 12 mm thick**.

### 3.1 Create the component

`Assemble → New Component`, name `BackPlate`.  Activate it.

1. Create sketch on XY plane.  Draw a 220 × 220 mm centred rectangle.
2. Extrude `12` mm.

### 3.2 Wall-mount slots (× 4)

Use slots (not holes) to allow stud spacing adjustment between 16" (406 mm)
and 24" (610 mm) centres.  The plate is smaller than this, so mount two screws
per side.

1. Create sketch on the front face.  Place four `8 × 14 mm` slots (use the
   `Slot (Centre Point)` sketch tool): at Y = ±80 mm, Z = ±80 mm.
2. Extrude Cut → Through All.  These accept M6 or #10 screws into studs.

### 3.3 Guide posts for floating plate (× 4)

Four M6 × 35 mm socket head shoulder bolts thread into the back plate and
protrude forward as guide rails.  The floating plate slides on them.

1. Create sketch on the back face.  Four circles, `5` mm Ø (M6 thread minor
   diameter for tapped hole), at (±55 mm Y, ±55 mm Z).
2. Extrude Cut `12` mm (through the plate) → tapped M6 blind hole.
3. In Fusion, mark these as threaded: `Solid → Create → Thread`, select each
   bore, choose `Metric (ISO)`, M6 × 1.0, `OK`.

### 3.4 Centering hub boss

A central 20 mm Ø × 15 mm tall boss on the back plate keeps the centering
spring concentric.

1. Create sketch on front face.  Draw a 20 mm Ø circle at origin.
2. Extrude `15` mm forward.  (This boss sits inside the hole in the floating
   plate's centre.)

### 3.5 Charger enclosure mounting pattern

Two M4 holes at the bottom of the back plate for bracket that supports the
charger below.  Position at Y = 0, Z = −90 mm ± 20 mm.  Extrude Cut M4 holes
(`3.3` mm drill, then `Solid → Create → Thread` M4 × 0.7).

---

## 4. House-side floating contact plate

This plate rides on the 4 guide post bolts with springs, can shift ±5 mm in
any direction to compensate residual robot alignment error.

Dimensions: **160 × 160 mm face, 18 mm thick**.

### 4.1 Create the component

`Assemble → New Component`, name `FloatingContactPlate`.  Activate it.

1. Create sketch on XY plane.  Draw a 160 × 160 mm centred rectangle.
2. Extrude `18` mm.
3. `F` → Fillet outer edges: `5` mm on all four vertical edge pairs.

### 4.2 Guide post through holes (× 4) — oversized for float

1. Create sketch on front face.  Four circles, `16` mm Ø, at (±55 mm Y,
   ±55 mm Z) — same XY positions as the guide posts in the back plate.
2. Extrude Cut → Through All.
3. Counterbore from the back: `26` mm Ø × `8` mm deep.  This counterbore
   accepts a 25 mm OD flat washer (or a printed flange) that retains the
   floating plate on the bolt.

> The `16` mm hole on a `6` mm guide post shoulder = **±5 mm float** in
> every direction.  This is the designed float range.

### 4.3 Central spring hub hole

1. Create sketch on back face.  Draw a `22` mm Ø circle at origin.
2. Extrude Cut `10` mm (not through-all; the hub sits in 10 mm of the plate).

### 4.4 Guide socket bore (for robot guide pin)

The socket accepts the 18 mm Ø guide pin.  It has a 30° entry funnel to
guide the pin in even when the floating plate is offset.

1. Create sketch on front face.  Draw two concentric circles:
   - Inner: `18.5` mm Ø (0.25 mm clearance on radius for smooth fit)
   - Outer: `35` mm Ø (entry funnel mouth)
2. Extrude Cut the 18.5 mm bore → `30` mm deep (full seating depth).
3. Create a **loft** for the funnel entry:
   - `Solid → Create → Loft`
   - Profile 1: the 35 mm Ø circle on the front face
   - Profile 2: the 18.5 mm Ø circle at 10 mm depth
   - Operation: **Cut**
   - This creates the 30° chamfered funnel entry, 10 mm deep.

> Alternatively use `Chamfer` on the front edge of the 18.5 mm bore:
> `Solid → Modify → Chamfer`, select the front circular edge, distance = `8` mm,
> angle = `30°`.  This is faster but gives a straight chamfer (fine for
> this use).

### 4.5 Spring contact pockets (× 4)

Each house-side spring contact is a phosphor-bronze leaf spring.  The base tab
is M3-screwed to the back face of the floating plate.  The contact arm
protrudes forward through a slot.

1. Create sketch on front face.  For each of the four pad positions
   (mirroring the robot-side pad layout):

   | Spring | Y | Z |
   |--------|---|---|
   | +48 V A | −35 mm | +32 mm |
   | +48 V B | −35 mm | −32 mm |
   | GND A   | +35 mm | +32 mm |
   | GND B   | +35 mm | −32 mm |

2. At each position draw a **10 × 22 mm** slot (slightly larger than pad
   face, 9 × 21 mm) to allow the spring arm to flex forward.
3. Extrude Cut `5` mm (only 5 mm; the spring arm tip will protrude 3 mm
   beyond the floating plate face to ensure contact pressure).
4. On the back face, add two M3 tapped holes per spring mount position
   (10 mm apart along the long axis of the slot, centred at the slot
   position).  Use `5` mm Ø × `10` mm deep for spring base tab.

### 4.6 Sense pogo socket bores (× 2)

The house side has pogo pin sockets that the robot-side pogo pins insert into.
Actually, the house side has **fixed** pogo pins that press against flat
pads on the robot side — or the other way around.  See README for the
chosen topology: **robot side has pogo pins that protrude**, house side has
**flat copper pads** (or simply a bridging pad shorting the two contacts).

Revised: since the robot side has the pogo pins, the house side needs only
flat copper sense pads at the two sense positions.

1. Create sketch on front face.  Two `8 × 12 mm` rectangles at
   Y = ±18 mm, Z = 0.  Extrude Cut `3` mm.  These hold a flat 8 × 10 mm
   copper or brass shim (sense pad).
2. Each shim has a single M3 hole at centre for a wire ring terminal.

---

## 5. Entry funnel (fixed to back plate)

The funnel is the large trumpet-shaped guide that narrows from 220 × 180 mm
to 112 × 112 mm over 80 mm depth.

### 5.1 Create the component

`Assemble → New Component`, name `EntryFunnel`.  Activate it.

### 5.2 Funnel body using Loft

The funnel is a hollow shell.  Build the outside solid first, shell it.

1. `Create Sketch` on the XY plane (the "back" wall-side face of the funnel).
   Draw a **220 × 180 mm** centred rectangle.
2. `Finish Sketch`.
3. `Create Sketch` at +80 mm offset on the Z axis:
   `Solid → Sketch → Create Sketch`, then click `Offset Plane` — offset from
   XY plane by `80` mm.  Draw a **112 × 112 mm** centred rectangle.
4. `Solid → Create → Loft`.  Profile 1: 220 × 180 rectangle on XY plane.
   Profile 2: 112 × 112 rectangle on offset plane.  Direction: `New Body`.
   `OK`.  This creates the solid tapering funnel shape.
5. `Solid → Modify → Shell`.  Select the large back face and the small front
   face.  Thickness: `4` mm.  `OK`.  This hollows the funnel to a
   4 mm-thick wall.

> Wall thickness: 4 mm at 40 % gyroid infill with ASA prints to about
> 180 MPa tensile strength — plenty for the guiding loads.

### 5.3 Attach to back plate

1. Add four M5 counterbored holes through the funnel back wall, aligned with
   M5 tapped holes in the back plate (position at ±80 mm from centre, clear
   of the guide post positions).
2. In the assembly, use a **Rigid Joint** (`Assemble → Joint → Rigid`) to
   mate the funnel back face to the back plate front face.

---

## 6. House-side tower enclosure

The tower houses the charger and Teensy control board.  It mounts above and
behind the back plate.

This component does not need to be modelled in Fusion if you are using a
commercial NEMA 4X enclosure (recommended for outdoor).  For indoor use,
model a simple printed cover.

### 6.1 Basic tower (indoor, printed)

`Assemble → New Component`, name `TowerCover`.

1. Sketch a 200 × 250 mm rectangle on a vertical plane aligned with the back
   plate.
2. Extrude `120` mm (depth of the charger + wiring space).
3. Shell it 3 mm.
4. Add a lid sketch and extrude a 3 mm lid.
5. Add ventilation slots on the sides (for charger heat): rows of 3 × 25 mm
   slots, 5 mm spacing, on both side panels.
6. Add a cutout on the bottom panel for the AC inlet (NEMA 5-15 outlet
   opening: 35 × 55 mm).
7. Add a cutout on the front panel (wall side) for an M20 cable gland (20.5 mm
   Ø hole) for the DC wiring loom from charger to contacts.

---

## 7. Assembly joints and constraints

Activate the root `DockingStation` component.

### 7.1 Float spring simulation

To represent the spring in Fusion (static position — springs can't be
simulated in free Design workspace, only in Simulation workspace):

1. Position the `FloatingContactPlate` centred on the `BackPlate` front face,
   offset forward by `35` mm (guide post length + washer thickness).
2. `Assemble → Joint` — use a **Planar Joint** to constrain the floating
   plate to translate only in the XY plane.  This represents the float.
3. Add a **Rigid Group** if you want to lock everything for rendering:
   `Assemble → Rigid Group`, select all components.  (Unlock for any
   animation work.)

### 7.2 Docked position simulation

To show the robot block inserted:

1. Activate root.
2. `Assemble → Joint` — Rigid joint between the guide pin front shaft
   centreline and the guide socket bore centreline.  Set offset to `0`
   (fully seated — pin shoulder flush with socket entry).
3. Confirm the power pads on the robot block face are coincident with the
   spring contact positions on the floating plate.

---

## 8. Creating the three requested views in a Drawing

### 8.1 Open Drawing workspace

1. In the assembly file: `File → New Drawing → From Design`.
2. Standard: `ISO` (or `ASME` — either is fine for this purpose).
   Sheet size: `A2` (594 × 420 mm) or `A3` — all three views should fit.
3. Scale: `1:4` for the assembly.

### 8.2 View 1 — Isometric assembly (right front top)

1. In the Drawing workspace toolbar: `Insert → Base View`.
2. Select the `DockingStation_Assembly` component.
3. In the `Orientation` dropdown, choose `Isometric — Top Right Front`.
4. Scale: `1:4`.  Style: `Shaded with Hidden Lines`.  Place in the top-right
   area of the sheet.
5. `OK`.

### 8.3 View 2 — Side cross-section (docked position)

1. Click the isometric view to select it.
2. In the toolbar: `Insert → Section View`.
3. Click once at the midpoint of the top edge of the isometric view, then
   click again at the midpoint of the bottom edge to define a vertical
   section cutting plane.
4. Move the section view to the left of the isometric view.  A side cross-
   section showing the charger, relay, guide pin, spring contacts, and robot
   block all in relationship will appear.
5. Add annotations (`Insert → Annotation → Leader Note`) labelling:
   - Guide pin
   - Spring contact
   - Robot-side pad
   - Guide socket funnel
   - Floating plate springs
   - Charger module

### 8.4 View 3 — Front exploded view

An exploded view works best from the Render or Animation workspace, but you
can approximate one in Drawing:

1. In the **Design** workspace (back in the main model), create a new
   **Position** under `Assemble → Positional Representations`.  Name it
   `Exploded`.
2. Manually offset each component in the exploded position:
   - Robot block: +150 mm in +X
   - Guide pin: same as robot block (+150 mm X)
   - Floating contact plate: +80 mm X
   - Entry funnel: 0 mm (stays at wall)
   - Back plate: 0 mm
3. In the Drawing, add another `Base View`, switch to the `Exploded` position.
   Orientation: `Front`.  Place below the isometric view.
4. Add leader notes identifying each component.
5. Add a title block annotation: `SIGYN DOCKING STATION — EXPLODED VIEW`.

### 8.5 Export

`Output → Export PDF` (full sheet, 1:1).  Also `Export DXF` for any laser-cut
plate work.

---

## 9. Dimension reference table

| Dimension | Value | Notes |
|-----------|-------|-------|
| Robot block W × H | 100 × 100 mm | Fixed by robot platform |
| Robot block depth | 50 mm | Into robot body |
| Guide pin Ø | 18 mm | 6061 Al, anodized |
| Guide pin protrusion | 40 mm | From robot block face |
| Guide pin total length | 70 mm | 40 mm out + 30 mm in block |
| Guide pin tip chamfer | 45°, 4 mm | |
| Guide socket bore Ø | 18.5 mm | 0.25 mm clearance |
| Guide socket funnel entry Ø | 35 mm | 30° taper, 8 mm deep |
| Guide socket depth | 30 mm | Full pin seating |
| Power pad (each) | 8 W × 20 L × 8 T mm | 8 mm sq. brass rod, 20 mm cut |
| Pad centre offset | ±35 mm Y, ±32 mm Z | From block centre |
| Sense pogo pin Ø | 5 mm body (see BOM) | Spring-loaded, 5 mm travel |
| Sense pin centre offset | ±18 mm Y, 0 Z | From block centre |
| Sense pin protrusion advantage | +5 mm | Contact before power pads |
| Floating plate | 160 × 160 × 18 mm | ASA printed |
| Floating plate float range | ±5 mm X/Y | Limited by guide post holes |
| Back plate | 220 × 220 × 12 mm | Al plate or 12 mm printed |
| Guide post hole Ø (floating plate) | 16 mm | On M6 post → ±5 mm float |
| Centering spring ID | 15 mm | Over 20 mm hub boss |
| Entry funnel entry aperture | 220 × 180 mm | |
| Entry funnel exit aperture | 112 × 112 mm | |
| Entry funnel depth | 80 mm | |
| Entry funnel wall thickness | 4 mm | |
| Tower cover W × H × D | 200 × 250 × 120 mm | |
