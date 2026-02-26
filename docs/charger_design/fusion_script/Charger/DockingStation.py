"""
Sigyn Autonomous Docking Station — Fusion 360 Python Script
============================================================
Builds the complete docking station assembly parametrically.

HOW TO RUN:
  1. Open Fusion 360 (any version, including free Personal Use).
  2. Press Shift+S  (or Tools → Scripts and Add-Ins).
  3. Click the "+" next to "My Scripts".
  4. Browse to this file's directory and select it.
  5. Click "Run".
  The script creates a new Design document with all components.

COMPONENTS CREATED:
  • RobotBlock          — 100×100×50 mm printed block, robot side
  • GuidePin            — 18 mm Ø aluminium guide pin (male)
  • BackPlate           — 220×220×12 mm wall-mount plate
  • FloatingContactPlate— 160×160×18 mm spring-floating contact plate
  • EntryFunnel         — tapered guide funnel, 220×180 → 112×112 mm

All dimensions are in mm throughout.  Parameters are defined in the
PARAMETERS section below — edit there to change the design.
"""

import adsk.core
import adsk.fusion
import traceback
import math


# ---------------------------------------------------------------------------
# PARAMETERS  — edit here to tune the design
# ---------------------------------------------------------------------------

# Robot-side block
BLOCK_W = 100.0          # Width  (Y axis)
BLOCK_H = 100.0          # Height (Z axis)
BLOCK_D = 50.0           # Depth  (X axis, into robot body)

# Guide pin (robot side, male)
PIN_DIA = 18.0           # Outer diameter
PIN_PROTRUDE = 40.0      # Length protruding from robot block face
PIN_INSET = 30.0         # Length inside the robot block bore
PIN_CHAMFER = 4.0        # Tip chamfer depth (45°)
PIN_BORE = 8.5           # Through-bore for M8 retaining bolt

# Power contact pads (brass, robot side)
PAD_W = 8.0              # Width  (narrow dimension)
PAD_L = 20.0             # Length (long dimension)
PAD_T = 10.0             # Pocket depth (pad is 8 mm thick, 2 mm epoxy behind)
PAD_POCKET_MARGIN = 0.5  # Extra clearance per side for epoxy
PAD_CENTRES_Y = 35.0     # ±Y offset of pad centres from block centre
PAD_CENTRES_Z = 32.0     # ±Z offset of pad centres from block centre

# Sense pogo pin holes (robot side)
POGO_BORE = 5.5          # Through-hole for pogo pin body
POGO_CB_DIA = 8.0        # Counterbore diameter (retaining collar)
POGO_CB_DEPTH = 5.0      # Counterbore depth
POGO_Y = 18.0            # ±Y offset from block centre (sense pins straddle guide)

# Back plate (house side, fixed to wall)
BP_W = 220.0
BP_H = 220.0
BP_T = 12.0
BP_BOSS_DIA = 20.0       # Centering spring hub boss diameter
BP_BOSS_H = 15.0         # Hub boss height
BP_POST_SPACING = 55.0   # ±X and ±Y of guide post hole centres

# Floating contact plate (house side, spring-mounted)
FP_W = 160.0
FP_H = 160.0
FP_T = 18.0
FP_CORNER_FILLET = 5.0
FP_POST_HOLE = 16.0      # Oversized hole for float (±5 mm on M6 post)
FP_POST_CB_DIA = 26.0    # Counterbore for retention washer
FP_POST_CB_DEPTH = 8.0
FP_SOCKET_BORE = 18.5    # Guide socket bore (0.25 mm clearance)
FP_SOCKET_FUNNEL_D = 35.0# Guide socket funnel entry diameter
FP_SOCKET_FUNNEL_D2 = 8.0# Chamfer / funnel depth
FP_SOCKET_DEPTH = 30.0   # Full seating depth of guide pin

# Entry funnel (house side, fixed)
FUNNEL_ENTRY_W = 220.0
FUNNEL_ENTRY_H = 180.0
FUNNEL_EXIT_W = 112.0
FUNNEL_EXIT_H = 112.0
FUNNEL_DEPTH = 80.0
FUNNEL_WALL = 4.0


# ---------------------------------------------------------------------------
# HELPERS
# ---------------------------------------------------------------------------

def cm(mm):
    """Convert mm to cm for the Fusion API (which works in cm)."""
    return mm / 10.0


def new_component(root, name):
    """Create a new sub-component under root and return (occurrence, comp)."""
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    return occ, comp


def sketch_on_plane(comp, plane):
    """Create a sketch on a construction plane or planar face."""
    sketches = comp.sketches
    sk = sketches.add(plane)
    return sk


def rect_centred(sketch, w, h):
    """Draw a centred rectangle (w wide, h tall) on sketch. Returns lines."""
    lines = sketch.sketchCurves.sketchLines
    pt1 = adsk.core.Point3D.create(-cm(w) / 2, -cm(h) / 2, 0)
    pt2 = adsk.core.Point3D.create( cm(w) / 2,  cm(h) / 2, 0)
    rect = lines.addTwoPointRectangle(pt1, pt2)
    return rect


def circle_at(sketch, x_mm, y_mm, dia_mm):
    """Draw a circle centred at (x_mm, y_mm) with given diameter."""
    circles = sketch.sketchCurves.sketchCircles
    centre = adsk.core.Point3D.create(cm(x_mm), cm(y_mm), 0)
    c = circles.addByCenterRadius(centre, cm(dia_mm) / 2)
    return c


def extrude_profile(comp, profile, dist_mm, operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation):
    """Extrude a profile by dist_mm. Returns the ExtrudeFeature."""
    feats = comp.features.extrudeFeatures
    inp = feats.createInput(profile, operation)
    dist = adsk.core.ValueInput.createByReal(cm(dist_mm))
    inp.setDistanceExtent(False, dist)
    return feats.add(inp)


def extrude_cut(comp, profile, dist_mm):
    """Extrude-cut a profile."""
    return extrude_profile(comp, profile, dist_mm,
                           adsk.fusion.FeatureOperations.CutFeatureOperation)


def get_face_at_z(body, z_cm, tolerance=0.01):
    """Return the first planar face at approximately the given Z (in cm)."""
    for face in body.faces:
        if face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
            normal = face.geometry.normal
            # flat face (normal parallel to Z) and at the right Z
            if abs(abs(normal.z) - 1.0) < 0.001:
                pt = face.pointOnFace
                if abs(pt.z - z_cm) < tolerance:
                    return face
    return None


def get_face_at_x(body, x_cm, tolerance=0.01):
    """Return the first planar face at approximately the given X (in cm)."""
    for face in body.faces:
        if face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
            normal = face.geometry.normal
            if abs(abs(normal.x) - 1.0) < 0.001:
                pt = face.pointOnFace
                if abs(pt.x - x_cm) < tolerance:
                    return face
    return None


def offset_plane(comp, base_plane, offset_mm):
    """Create an offset construction plane."""
    planes = comp.constructionPlanes
    inp = planes.createInput()
    dist = adsk.core.ValueInput.createByReal(cm(offset_mm))
    inp.setByOffset(base_plane, dist)
    return planes.add(inp)


# ---------------------------------------------------------------------------
# COMPONENT BUILDERS
# ---------------------------------------------------------------------------

def build_robot_block(root):
    """Build the robot-side contact block (100×100×50 mm)."""
    _, comp = new_component(root, "RobotBlock")
    planes = comp.constructionPlanes

    # ---- outer body ----
    # Contact face on the YZ plane (X=0); block extends in -X direction.
    sk = sketch_on_plane(comp, comp.yZConstructionPlane)
    rect_centred(sk, BLOCK_W, BLOCK_H)
    prof = sk.profiles.item(0)
    ext = extrude_profile(comp, prof, BLOCK_D,
                          adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = ext.bodies.item(0)

    # The block now extends from X=0 to X=+BLOCK_D (Fusion extrudes in +X by default
    # on YZ plane).  The face at X=0 is the contact face; X=BLOCK_D is the back.

    # ---- guide pin mounting bore (18 mm through 30 mm, centred) ----
    sk2 = sketch_on_plane(comp, comp.yZConstructionPlane)
    circle_at(sk2, 0, 0, PIN_DIA)
    # Use innermost profile (just the circle)
    pin_prof = None
    for i in range(sk2.profiles.count):
        p = sk2.profiles.item(i)
        bb = p.boundingBox
        w = bb.maxPoint.x - bb.minPoint.x
        if abs(w - cm(PIN_DIA)) < 0.001:
            pin_prof = p
            break
    if pin_prof is None:
        pin_prof = sk2.profiles.item(0)
    extrude_cut(comp, pin_prof, PIN_INSET)

    # ---- M8 bolt clearance through-bore (8.5 mm, full depth) ----
    sk3 = sketch_on_plane(comp, comp.yZConstructionPlane)
    circle_at(sk3, 0, 0, PIN_BORE)
    bolt_prof = sk3.profiles.item(0)
    extrude_cut(comp, bolt_prof, BLOCK_D)

    # ---- power pad pockets (×4) ----
    pw = PAD_W + 2 * PAD_POCKET_MARGIN
    pl = PAD_L + 2 * PAD_POCKET_MARGIN
    pad_positions = [
        (-PAD_CENTRES_Y,  PAD_CENTRES_Z),   # +48V A  (left,  top)
        (-PAD_CENTRES_Y, -PAD_CENTRES_Z),   # +48V B  (left,  bottom)
        ( PAD_CENTRES_Y,  PAD_CENTRES_Z),   # GND  A  (right, top)
        ( PAD_CENTRES_Y, -PAD_CENTRES_Z),   # GND  B  (right, bottom)
    ]
    sk4 = sketch_on_plane(comp, comp.yZConstructionPlane)
    for (py, pz) in pad_positions:
        lines = sk4.sketchCurves.sketchLines
        y0 = cm(py - pw / 2);  y1 = cm(py + pw / 2)
        z0 = cm(pz - pl / 2);  z1 = cm(pz + pl / 2)
        lines.addTwoPointRectangle(
            adsk.core.Point3D.create(y0, z0, 0),
            adsk.core.Point3D.create(y1, z1, 0))
    # Collect non-guide-pin profiles (4 small rectangles)
    pad_profs = adsk.core.ObjectCollection.create()
    for i in range(sk4.profiles.count):
        p = sk4.profiles.item(i)
        bb = p.boundingBox
        w = bb.maxPoint.x - bb.minPoint.x
        h = bb.maxPoint.y - bb.minPoint.y
        if abs(w - cm(pw)) < 0.005 and abs(h - cm(pl)) < 0.005:
            pad_profs.add(p)
    if pad_profs.count > 0:
        feats = comp.features.extrudeFeatures
        inp = feats.createInput(pad_profs,
                                adsk.fusion.FeatureOperations.CutFeatureOperation)
        dist = adsk.core.ValueInput.createByReal(cm(PAD_T))
        inp.setDistanceExtent(False, dist)
        feats.add(inp)

    # ---- sense pogo pin bores (×2, through-all) ----
    sk5 = sketch_on_plane(comp, comp.yZConstructionPlane)
    circle_at(sk5, -POGO_Y, 0, POGO_BORE)
    circle_at(sk5,  POGO_Y, 0, POGO_BORE)
    pogo_profs = adsk.core.ObjectCollection.create()
    for i in range(sk5.profiles.count):
        p = sk5.profiles.item(i)
        bb = p.boundingBox
        dia = bb.maxPoint.x - bb.minPoint.x
        if abs(dia - cm(POGO_BORE)) < 0.003:
            pogo_profs.add(p)
    if pogo_profs.count > 0:
        feats = comp.features.extrudeFeatures
        inp = feats.createInput(pogo_profs,
                                adsk.fusion.FeatureOperations.CutFeatureOperation)
        inp.setAllExtent(False)  # through-all
        feats.add(inp)

    # ---- pogo counterbores (×2, from back face) ----
    back_plane = offset_plane(comp, comp.yZConstructionPlane, BLOCK_D)
    sk6 = sketch_on_plane(comp, back_plane)
    circle_at(sk6, -POGO_Y, 0, POGO_CB_DIA)
    circle_at(sk6,  POGO_Y, 0, POGO_CB_DIA)
    cb_profs = adsk.core.ObjectCollection.create()
    for i in range(sk6.profiles.count):
        p = sk6.profiles.item(i)
        bb = p.boundingBox
        dia = bb.maxPoint.x - bb.minPoint.x
        if abs(dia - cm(POGO_CB_DIA)) < 0.003:
            cb_profs.add(p)
    if cb_profs.count > 0:
        feats = comp.features.extrudeFeatures
        inp = feats.createInput(cb_profs,
                                adsk.fusion.FeatureOperations.CutFeatureOperation)
        dist = adsk.core.ValueInput.createByReal(cm(POGO_CB_DEPTH))
        inp.setDistanceExtent(False, dist)
        feats.add(inp)

    # ---- mounting holes (×4, M5 clearance, corners) ----
    sk7 = sketch_on_plane(comp, comp.yZConstructionPlane)
    mount_r = 5.3 / 2  # M5 clearance
    for (my, mz) in [(-40, 40), (-40, -40), (40, 40), (40, -40)]:
        circle_at(sk7, my, mz, 5.3)
    mount_profs = adsk.core.ObjectCollection.create()
    for i in range(sk7.profiles.count):
        p = sk7.profiles.item(i)
        bb = p.boundingBox
        dia = bb.maxPoint.x - bb.minPoint.x
        if abs(dia - cm(5.3)) < 0.003:
            mount_profs.add(p)
    if mount_profs.count > 0:
        feats = comp.features.extrudeFeatures
        inp = feats.createInput(mount_profs,
                                adsk.fusion.FeatureOperations.CutFeatureOperation)
        inp.setAllExtent(False)
        feats.add(inp)

    return comp


# ---------------------------------------------------------------------------

def build_guide_pin(root):
    """Build the aluminium guide pin (robot side, male)."""
    _, comp = new_component(root, "GuidePin")

    # Revolve profile on XZ plane to create cylinder
    # Pin axis is along +X from origin.
    # Total length = PIN_PROTRUDE + PIN_INSET
    total = PIN_PROTRUDE + PIN_INSET

    sk = sketch_on_plane(comp, comp.xZConstructionPlane)
    lines = sk.sketchCurves.sketchLines

    r = cm(PIN_DIA / 2)
    L = cm(total)
    # Half-profile rectangle: from (0,0) to (L, r)
    lines.addTwoPointRectangle(
        adsk.core.Point3D.create(0, 0, 0),
        adsk.core.Point3D.create(L, r, 0))

    # Revolve around X axis
    prof = None
    for i in range(sk.profiles.count):
        prof = sk.profiles.item(i)
        break
    if prof is None:
        return comp

    revolves = comp.features.revolveFeatures
    ax_line = comp.xConstructionAxis
    inp = revolves.createInput(prof, ax_line,
                               adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))
    revolves.add(inp)

    # Through-bore for M8 bolt
    sk2 = sketch_on_plane(comp, comp.yZConstructionPlane)
    circle_at(sk2, 0, 0, PIN_BORE)
    bore_prof = sk2.profiles.item(0)
    extrude_cut(comp, bore_prof, total)

    # Chamfer on tip (front face at X=0, the protruding end)
    # We'll use a sketch chamfer approximation:
    # Get the circular edge at X=0 and apply chamfer
    body = comp.bRepBodies.item(0)
    chamfer_edges = adsk.core.ObjectCollection.create()
    for edge in body.edges:
        # Circular edge (loop) at X=0
        if hasattr(edge.geometry, 'center'):
            centre = edge.geometry.center
            if abs(centre.x) < 0.001:  # at the tip face
                chamfer_edges.add(edge)
                break
    if chamfer_edges.count > 0:
        chamfers = comp.features.chamferFeatures
        ch_inp = chamfers.createInput(chamfer_edges, True)
        ch_inp.setToEqualDistance(adsk.core.ValueInput.createByReal(cm(PIN_CHAMFER)))
        try:
            chamfers.add(ch_inp)
        except Exception:
            pass  # chamfer is cosmetic; don't fail the whole script

    return comp


# ---------------------------------------------------------------------------

def build_back_plate(root):
    """Build the house-side back plate (220×220×12 mm)."""
    _, comp = new_component(root, "BackPlate")

    # Main body — extrude in +X from YZ plane
    sk = sketch_on_plane(comp, comp.yZConstructionPlane)
    rect_centred(sk, BP_W, BP_H)
    prof = sk.profiles.item(0)
    extrude_profile(comp, prof, BP_T,
                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

    # Central hub boss (on front face at X = BP_T)
    front_plane = offset_plane(comp, comp.yZConstructionPlane, BP_T)
    sk2 = sketch_on_plane(comp, front_plane)
    circle_at(sk2, 0, 0, BP_BOSS_DIA)
    boss_prof = sk2.profiles.item(0)
    extrude_profile(comp, boss_prof, BP_BOSS_H,
                    adsk.fusion.FeatureOperations.JoinFeatureOperation)

    # Guide post M6 tapped holes (×4) — modelled as through-bores (5 mm Ø)
    sk3 = sketch_on_plane(comp, comp.yZConstructionPlane)
    for (py, pz) in [(-BP_POST_SPACING,  BP_POST_SPACING),
                     (-BP_POST_SPACING, -BP_POST_SPACING),
                     ( BP_POST_SPACING,  BP_POST_SPACING),
                     ( BP_POST_SPACING, -BP_POST_SPACING)]:
        circle_at(sk3, py, pz, 5.0)  # M6 tap drill ≈ 5 mm
    post_profs = adsk.core.ObjectCollection.create()
    for i in range(sk3.profiles.count):
        p = sk3.profiles.item(i)
        bb = p.boundingBox
        dia = bb.maxPoint.x - bb.minPoint.x
        if abs(dia - cm(5.0)) < 0.003:
            post_profs.add(p)
    if post_profs.count > 0:
        feats = comp.features.extrudeFeatures
        inp = feats.createInput(post_profs,
                                adsk.fusion.FeatureOperations.CutFeatureOperation)
        inp.setAllExtent(False)
        feats.add(inp)

    # Wall-mount slots (×4) — 8×14 mm, at ±80 mm
    sk4 = sketch_on_plane(comp, comp.yZConstructionPlane)
    slot_w = 8.0
    slot_l = 14.0
    for (sy, sz) in [(-80, 80), (-80, -80), (80, 80), (80, -80)]:
        lines = sk4.sketchCurves.sketchLines
        y0 = cm(sy - slot_w / 2);  y1 = cm(sy + slot_w / 2)
        z0 = cm(sz - slot_l / 2);  z1 = cm(sz + slot_l / 2)
        lines.addTwoPointRectangle(
            adsk.core.Point3D.create(y0, z0, 0),
            adsk.core.Point3D.create(y1, z1, 0))
    slot_profs = adsk.core.ObjectCollection.create()
    for i in range(sk4.profiles.count):
        p = sk4.profiles.item(i)
        bb = p.boundingBox
        w = bb.maxPoint.x - bb.minPoint.x
        h = bb.maxPoint.y - bb.minPoint.y
        if abs(w - cm(slot_w)) < 0.005 and abs(h - cm(slot_l)) < 0.005:
            slot_profs.add(p)
    if slot_profs.count > 0:
        feats = comp.features.extrudeFeatures
        inp = feats.createInput(slot_profs,
                                adsk.fusion.FeatureOperations.CutFeatureOperation)
        inp.setAllExtent(False)
        feats.add(inp)

    return comp


# ---------------------------------------------------------------------------

def build_floating_contact_plate(root):
    """Build the house-side floating contact plate (160×160×18 mm)."""
    _, comp = new_component(root, "FloatingContactPlate")

    # Main body
    sk = sketch_on_plane(comp, comp.yZConstructionPlane)
    rect_centred(sk, FP_W, FP_H)
    prof = sk.profiles.item(0)
    extrude_profile(comp, prof, FP_T,
                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

    # Guide post oversized holes (×4) — 16 mm Ø, through-all
    sk2 = sketch_on_plane(comp, comp.yZConstructionPlane)
    for (py, pz) in [(-BP_POST_SPACING,  BP_POST_SPACING),
                     (-BP_POST_SPACING, -BP_POST_SPACING),
                     ( BP_POST_SPACING,  BP_POST_SPACING),
                     ( BP_POST_SPACING, -BP_POST_SPACING)]:
        circle_at(sk2, py, pz, FP_POST_HOLE)
    post_profs = adsk.core.ObjectCollection.create()
    for i in range(sk2.profiles.count):
        p = sk2.profiles.item(i)
        bb = p.boundingBox
        dia = bb.maxPoint.x - bb.minPoint.x
        if abs(dia - cm(FP_POST_HOLE)) < 0.003:
            post_profs.add(p)
    if post_profs.count > 0:
        feats = comp.features.extrudeFeatures
        inp = feats.createInput(post_profs,
                                adsk.fusion.FeatureOperations.CutFeatureOperation)
        inp.setAllExtent(False)
        feats.add(inp)

    # Counterbores on back face for retention washers
    back_plane = offset_plane(comp, comp.yZConstructionPlane, FP_T)
    sk3 = sketch_on_plane(comp, back_plane)
    for (py, pz) in [(-BP_POST_SPACING,  BP_POST_SPACING),
                     (-BP_POST_SPACING, -BP_POST_SPACING),
                     ( BP_POST_SPACING,  BP_POST_SPACING),
                     ( BP_POST_SPACING, -BP_POST_SPACING)]:
        circle_at(sk3, py, pz, FP_POST_CB_DIA)
    cb_profs = adsk.core.ObjectCollection.create()
    for i in range(sk3.profiles.count):
        p = sk3.profiles.item(i)
        bb = p.boundingBox
        dia = bb.maxPoint.x - bb.minPoint.x
        if abs(dia - cm(FP_POST_CB_DIA)) < 0.003:
            cb_profs.add(p)
    if cb_profs.count > 0:
        feats = comp.features.extrudeFeatures
        inp = feats.createInput(cb_profs,
                                adsk.fusion.FeatureOperations.CutFeatureOperation)
        dist = adsk.core.ValueInput.createByReal(cm(FP_POST_CB_DEPTH))
        inp.setDistanceExtent(False, dist)
        feats.add(inp)

    # Guide socket bore (18.5 mm, 30 mm deep) — from contact face (X=0)
    sk4 = sketch_on_plane(comp, comp.yZConstructionPlane)
    circle_at(sk4, 0, 0, FP_SOCKET_BORE)
    socket_prof = None
    for i in range(sk4.profiles.count):
        p = sk4.profiles.item(i)
        bb = p.boundingBox
        dia = bb.maxPoint.x - bb.minPoint.x
        if abs(dia - cm(FP_SOCKET_BORE)) < 0.003:
            socket_prof = p
            break
    if socket_prof:
        extrude_cut(comp, socket_prof, FP_SOCKET_DEPTH)

    # Guide socket entry funnel — chamfer on front edge of socket bore
    # Implemented as a second larger bore from front, 8 mm deep
    sk5 = sketch_on_plane(comp, comp.yZConstructionPlane)
    circle_at(sk5, 0, 0, FP_SOCKET_FUNNEL_D)
    funnel_prof = None
    for i in range(sk5.profiles.count):
        p = sk5.profiles.item(i)
        bb = p.boundingBox
        dia = bb.maxPoint.x - bb.minPoint.x
        if abs(dia - cm(FP_SOCKET_FUNNEL_D)) < 0.005:
            funnel_prof = p
            break
    if funnel_prof:
        extrude_cut(comp, funnel_prof, FP_SOCKET_FUNNEL_D2)

    # Spring contact slots (×4) — 10×22 mm pockets, 5 mm deep, from contact face
    sk6 = sketch_on_plane(comp, comp.yZConstructionPlane)
    sp_w = PAD_W + 2.0   # 10 mm
    sp_l = PAD_L + 2.0   # 22 mm
    for (py, pz) in pad_positions_list():
        lines = sk6.sketchCurves.sketchLines
        y0 = cm(py - sp_w / 2);  y1 = cm(py + sp_w / 2)
        z0 = cm(pz - sp_l / 2);  z1 = cm(pz + sp_l / 2)
        lines.addTwoPointRectangle(
            adsk.core.Point3D.create(y0, z0, 0),
            adsk.core.Point3D.create(y1, z1, 0))
    spring_profs = adsk.core.ObjectCollection.create()
    for i in range(sk6.profiles.count):
        p = sk6.profiles.item(i)
        bb = p.boundingBox
        w = bb.maxPoint.x - bb.minPoint.x
        h = bb.maxPoint.y - bb.minPoint.y
        if abs(w - cm(sp_w)) < 0.005 and abs(h - cm(sp_l)) < 0.005:
            spring_profs.add(p)
    if spring_profs.count > 0:
        feats = comp.features.extrudeFeatures
        inp = feats.createInput(spring_profs,
                                adsk.fusion.FeatureOperations.CutFeatureOperation)
        dist = adsk.core.ValueInput.createByReal(cm(5.0))
        inp.setDistanceExtent(False, dist)
        feats.add(inp)

    # Sense pad recesses (×2) — 8×12 mm, 3 mm deep, from contact face
    sk7 = sketch_on_plane(comp, comp.yZConstructionPlane)
    for (sy, sz) in [(-POGO_Y, 0), (POGO_Y, 0)]:
        lines = sk7.sketchCurves.sketchLines
        y0 = cm(sy - 4);  y1 = cm(sy + 4)
        z0 = cm(sz - 6);  z1 = cm(sz + 6)
        lines.addTwoPointRectangle(
            adsk.core.Point3D.create(y0, z0, 0),
            adsk.core.Point3D.create(y1, z1, 0))
    sense_profs = adsk.core.ObjectCollection.create()
    for i in range(sk7.profiles.count):
        p = sk7.profiles.item(i)
        bb = p.boundingBox
        w = bb.maxPoint.x - bb.minPoint.x
        h = bb.maxPoint.y - bb.minPoint.y
        if abs(w - cm(8.0)) < 0.005 and abs(h - cm(12.0)) < 0.005:
            sense_profs.add(p)
    if sense_profs.count > 0:
        feats = comp.features.extrudeFeatures
        inp = feats.createInput(sense_profs,
                                adsk.fusion.FeatureOperations.CutFeatureOperation)
        dist = adsk.core.ValueInput.createByReal(cm(3.0))
        inp.setDistanceExtent(False, dist)
        feats.add(inp)

    return comp


def pad_positions_list():
    return [
        (-PAD_CENTRES_Y,  PAD_CENTRES_Z),
        (-PAD_CENTRES_Y, -PAD_CENTRES_Z),
        ( PAD_CENTRES_Y,  PAD_CENTRES_Z),
        ( PAD_CENTRES_Y, -PAD_CENTRES_Z),
    ]


# ---------------------------------------------------------------------------

def build_entry_funnel(root):
    """Build the tapered entry funnel using Loft."""
    _, comp = new_component(root, "EntryFunnel")

    # Profile 1 — large entry (on YZ plane, X=0)
    sk1 = sketch_on_plane(comp, comp.yZConstructionPlane)
    rect_centred(sk1, FUNNEL_ENTRY_W, FUNNEL_ENTRY_H)
    prof1 = sk1.profiles.item(0)

    # Profile 2 — small exit (offset plane at X = FUNNEL_DEPTH)
    exit_plane = offset_plane(comp, comp.yZConstructionPlane, FUNNEL_DEPTH)
    sk2 = sketch_on_plane(comp, exit_plane)
    rect_centred(sk2, FUNNEL_EXIT_W, FUNNEL_EXIT_H)
    prof2 = sk2.profiles.item(0)

    # Loft to create solid
    lofts = comp.features.loftFeatures
    inp = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.loftSections.add(prof1)
    inp.loftSections.add(prof2)
    loft = lofts.add(inp)

    # Shell to FUNNEL_WALL thickness — remove both end faces
    body = loft.bodies.item(0)
    shell_faces = adsk.core.ObjectCollection.create()
    for face in body.faces:
        if face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
            normal = face.geometry.normal
            if abs(abs(normal.x) - 1.0) < 0.01:  # planar faces perpendicular to X
                shell_faces.add(face)

    if shell_faces.count >= 2:
        shells = comp.features.shellFeatures
        sh_inp = shells.createInput(shell_faces, False)
        sh_inp.insideThickness = adsk.core.ValueInput.createByReal(cm(FUNNEL_WALL))
        shells.add(sh_inp)

    return comp


# ---------------------------------------------------------------------------
# POSITION COMPONENTS IN ASSEMBLY
# ---------------------------------------------------------------------------

def position_components(root_comp, app):
    """
    Translate each occurrence so the docked assembly makes spatial sense.

    Layout (X axis = depth, robot approaches from +X toward origin):
      RobotBlock:             X = +BLOCK_D  ..  0         (contact face at X=0)
      GuidePin:               X = -PIN_PROTRUDE .. +PIN_INSET (pin tip at X = -PIN_PROTRUDE)
      FloatingContactPlate:   X = 0 .. +FP_T              (contact face at X=0)
      EntryFunnel:            X = +FP_T .. +FP_T+FUNNEL_DEPTH
      BackPlate:              X = +FP_T+FUNNEL_DEPTH .. +FP_T+FUNNEL_DEPTH+BP_T
    """
    # The components were created centred on the origin of their own
    # component origin, but Fusion places occurrences at the root origin.
    # We set transform matrices on each occurrence.

    def translate_occ(occ, dx_mm, dy_mm, dz_mm):
        m = adsk.core.Matrix3D.create()
        m.translation = adsk.core.Vector3D.create(cm(dx_mm), cm(dy_mm), cm(dz_mm))
        occ.transform = m

    occs = root_comp.occurrences
    for i in range(occs.count):
        occ = occs.item(i)
        name = occ.component.name
        if name == "RobotBlock":
            translate_occ(occ, 0, 0, 0)          # contact face at X=0, extends to +BLOCK_D
        elif name == "GuidePin":
            translate_occ(occ, -PIN_PROTRUDE, 0, 0)  # tip at X=-PIN_PROTRUDE
        elif name == "FloatingContactPlate":
            translate_occ(occ, FP_T, 0, 0)       # plate behind X=0 face of funnel exit
        elif name == "EntryFunnel":
            translate_occ(occ, FP_T, 0, 0)       # funnel entry starts at back of plate
        elif name == "BackPlate":
            translate_occ(occ, FP_T + FUNNEL_DEPTH, 0, 0)


# ---------------------------------------------------------------------------
# APPEARANCE ASSIGNMENTS
# ---------------------------------------------------------------------------

def assign_appearances(app, root_comp):
    """Assign material appearances for easy visual identification."""
    lib = app.materialLibraries
    # Try to get a built-in appearance library
    appear_lib = None
    for i in range(lib.count):
        if "Fusion 360" in lib.item(i).name or "Appearance" in lib.item(i).name:
            appear_lib = lib.item(i)
            break

    if appear_lib is None:
        return  # skip if no library found

    def find_appearance(name_substr):
        appearances = appear_lib.appearances
        for i in range(appearances.count):
            a = appearances.item(i)
            if name_substr.lower() in a.name.lower():
                return a
        return None

    plastic = find_appearance("ABS")
    aluminum = find_appearance("Aluminum")

    occs = root_comp.occurrences
    for i in range(occs.count):
        occ = occs.item(i)
        comp = occ.component
        name = comp.name
        try:
            if name in ("RobotBlock", "FloatingContactPlate",
                        "BackPlate", "EntryFunnel"):
                if plastic:
                    for body in comp.bRepBodies:
                        body.appearance = plastic
            elif name == "GuidePin":
                if aluminum:
                    for body in comp.bRepBodies:
                        body.appearance = aluminum
        except Exception:
            pass  # appearance is cosmetic; never fail the build


# ---------------------------------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------------------------------

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        # Create a new document
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = app.activeProduct
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType

        root = design.rootComponent
        root.name = "DockingStation_Assembly"

        ui.messageBox("Building Sigyn Docking Station…\nThis may take 30–60 seconds.",
                      "Docking Station Script")

        # Build each component
        build_robot_block(root)
        build_guide_pin(root)
        build_back_plate(root)
        build_floating_contact_plate(root)
        build_entry_funnel(root)

        # Position in assembly space
        position_components(root, app)

        # Assign appearances
        assign_appearances(app, root)

        # Fit view
        app.activeViewport.fit()

        ui.messageBox(
            "✅  Docking Station assembly created successfully!\n\n"
            "Components built:\n"
            "  • RobotBlock (100×100×50 mm, ASA printed)\n"
            "  • GuidePin (18 mm Ø aluminium)\n"
            "  • BackPlate (220×220×12 mm)\n"
            "  • FloatingContactPlate (160×160×18 mm)\n"
            "  • EntryFunnel (220×180 → 112×112 mm)\n\n"
            "Suggested next steps:\n"
            "  1. File → Save  (name your design)\n"
            "  2. Inspect each component with Section Analysis\n"
            "  3. File → New Drawing → From Design for full drawings\n"
            "  4. Export each body as STL for printing\n\n"
            "See docs/charger_design/mechanical_design.md for full notes.",
            "Docking Station — Done")

    except Exception:
        if ui:
            ui.messageBox("Script error:\n" + traceback.format_exc(),
                          "Docking Station Script Error")
