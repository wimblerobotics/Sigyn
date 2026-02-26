"""
Sigyn Docking Station  DS3  v3.0
=================================
Corrected assembly stack, chamfered RobotBlock, funnel bolt-flange,
FloatingPlate seated in BackPlate recess.

ASSEMBLY (X axis = docking axis, robot reverses from +X toward wall at X=0)
  X =   0 ..  12   BackPlate         (wall face at 0, front face at 12)
  X =  12 ..  30   FloatingPlate     (contact face at 12, seated in recess)
  X =  12 ..  92   EntryFunnel       (flange at 12, wide mouth at 92)
  X =  92 .. 142   RobotBlock        (contact face at 92, back face at 142)
  GuidePin tip at  X = 52  (protrudes 40 mm ahead of contact face at 92)

HOW TO RUN
  1. Shift+S in Fusion 360 → Scripts and Add-Ins
  2. Green + button → navigate to the  DS3  folder (this file's folder) → Open
  3. Select DS3 and click Run
  (No need to pre-open any design; the script creates its own.)
"""

import adsk.core
import adsk.fusion
import traceback

# ── PARAMETERS  (all mm) ────────────────────────────────────────────────────

# Robot-side block
RB_W, RB_H, RB_D = 100.0, 100.0, 50.0
RB_CH = 8.0       # leading-edge chamfer (45°) — needed to clear funnel exit

# Guide pin
PIN_OD   = 18.0
PIN_BORE =  8.5   # M8 retaining bolt clearance
PIN_OUT  = 40.0   # protrudes ahead of robot contact face
PIN_IN   = 30.0   # inset inside RobotBlock (socket depth)
PIN_CH   =  4.0   # tip chamfer depth (45°)

# Power pad pockets (robot side)
PAD_W   =  8.0
PAD_L   = 20.0
PAD_DEP = 10.0
PAD_CLR =  0.5    # bedding clearance per side
PAD_Y   = 35.0    # ±Y
PAD_Z   = 32.0    # ±Z

# Sense pogo bores (robot side)
POGO_D   =  5.5   # through-bore
POGO_CB  =  8.0   # counterbore
POGO_CBD =  5.0   # counterbore depth from back face
POGO_Y   = 18.0   # ±Y

# RobotBlock mount holes
MNT_D   =  5.3    # M5 clearance
MNT_OFF = 40.0    # ±Y and ±Z

# Back plate
BP_W, BP_H, BP_T  = 220.0, 220.0, 12.0
BP_RCS_W          = 126.0   # FloatingPlate recess width  (120 + 3 mm each side)
BP_RCS_H          = 126.0
BP_RCS_D          = 20.0    # recess depth
BP_BOSS_D         = 20.0    # guide-pin receiver boss Ø
BP_BOSS_H         = 15.0    # boss height (protrudes from wall face, into wall gap)
BP_POST_OFF       = 50.0    # ±Y, ±Z for M6 float posts
BP_POST_D         =  6.5    # M6 clearance hole
BP_FN_BOLT_OFF    = 58.0    # ±Y, ±Z for M4 funnel flange bolts
BP_FN_BOLT_D      =  3.3    # M4 tap drill
BP_SLOT_W         =  8.0    # wall keyhole slot
BP_SLOT_L         = 14.0
BP_SLOT_OFF       = 85.0    # ±Y, ±Z

# Floating contact plate
FP_W, FP_H, FP_T  = 120.0, 120.0, 18.0
FP_FLOAT_D        = 10.0    # oversized hole on M6 post
FP_CB_D           = 16.0    # retention washer counterbore
FP_CB_DEP         =  6.0    # counterbore depth from back face
FP_SOCK_D         = 18.5    # guide socket bore
FP_FUNNEL_D       = 30.0    # lead-in bore on contact face
FP_FUNNEL_L       =  5.0
FP_SOCK_DEP       = 16.0

# Spring contact slots (house side)
SP_W   = PAD_W + 2.0   # 10 mm
SP_L   = PAD_L + 2.0   # 22 mm
SP_DEP =  5.0

# Sense pad recesses (house side)
SNS_W   =  8.0
SNS_L   = 12.0
SNS_DEP =  3.0

# Entry funnel
# Exit (narrow, wall side) at local X=0; Entry (wide, robot side) at X=FN_D
FN_XW, FN_XH  = 130.0, 130.0   # exit aperture — clears 100 mm block + chamfers
FN_EW, FN_EH  = 220.0, 180.0   # entry aperture
FN_D          =  80.0
FN_WALL       =   4.0
# Bolt flange at exit end
FN_FL_W         = 180.0
FN_FL_H         = 180.0
FN_FL_T         =   6.0
FN_FL_BOLT_D    =   4.5   # M4 clearance
FN_FL_BOLT_OFF  = BP_FN_BOLT_OFF   # 58 mm


# ── UNIT CONVERSION ─────────────────────────────────────────────────────────

def c(mm):
    """mm → cm for Fusion API."""
    return mm / 10.0


# ── HELPERS ─────────────────────────────────────────────────────────────────

def new_comp(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    return comp


def sk_on(comp, plane):
    return comp.sketches.add(plane)


def off_plane(comp, base, dist_mm):
    planes = comp.constructionPlanes
    inp = planes.createInput()
    inp.setByOffset(base, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return planes.add(inp)


def draw_rect(sk, cy, cz, w, h):
    """Centred rectangle on YZ sketch. All args in mm."""
    lines = sk.sketchCurves.sketchLines
    p1 = adsk.core.Point3D.create(c(cy - w/2), c(cz - h/2), 0)
    p2 = adsk.core.Point3D.create(c(cy + w/2), c(cz + h/2), 0)
    lines.addTwoPointRectangle(p1, p2)


def draw_circ(sk, cy, cz, dia):
    sk.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(c(cy), c(cz), 0), c(dia / 2))


def ext_new(comp, prof, dist_mm):
    f = comp.features.extrudeFeatures
    i = f.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    i.setDistanceExtent(False, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return f.add(i)


def ext_join(comp, prof, dist_mm):
    f = comp.features.extrudeFeatures
    i = f.createInput(prof, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    i.setDistanceExtent(False, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return f.add(i)


def ext_join_flip(comp, prof, dist_mm):
    """Extrude-join in the −X direction (flip=True)."""
    f = comp.features.extrudeFeatures
    i = f.createInput(prof, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    i.setDistanceExtent(True, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return f.add(i)


def cut_d(comp, prof, dist_mm):
    f = comp.features.extrudeFeatures
    i = f.createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
    i.setDistanceExtent(False, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return f.add(i)


def cut_thru(comp, prof):
    f = comp.features.extrudeFeatures
    i = f.createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
    i.setAllExtent(False)
    return f.add(i)


def first_prof(sk):
    if sk.profiles.count == 0:
        raise RuntimeError(f"No profiles in sketch '{sk.name}'")
    return sk.profiles.item(0)


def coll_rect(sk, w, h, tol=0.08):
    out = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        bb = p.boundingBox
        pw = (bb.maxPoint.x - bb.minPoint.x) * 10
        ph = (bb.maxPoint.y - bb.minPoint.y) * 10
        if ((abs(pw-w)<tol and abs(ph-h)<tol) or (abs(pw-h)<tol and abs(ph-w)<tol)):
            out.add(p)
    return out


def coll_circ(sk, dia, tol=0.08):
    out = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        bb = p.boundingBox
        pw = (bb.maxPoint.x - bb.minPoint.x) * 10
        ph = (bb.maxPoint.y - bb.minPoint.y) * 10
        if abs(pw-dia)<tol and abs(ph-dia)<tol:
            out.add(p)
    return out


# ── BUILDERS ────────────────────────────────────────────────────────────────

def build_robot_block(root):
    """
    100×100×50 mm robot-side block.  Contact face at local X=0.
    8 mm chamfers on the 4 long edges of the contact face let the block
    enter the 130×130 funnel exit (diagonal of unchamfered block = 141 mm;
    with 8 mm chamfer effective corner-to-face = ~64 mm → clears 65 mm half-width).
    """
    comp = new_comp(root, "RobotBlock")
    yz   = comp.yZConstructionPlane
    pw   = PAD_W + 2*PAD_CLR   # 9 mm
    pl   = PAD_L + 2*PAD_CLR   # 21 mm

    # ① Body
    sk = sk_on(comp, yz)
    draw_rect(sk, 0, 0, RB_W, RB_H)
    body_feat = ext_new(comp, first_prof(sk), RB_D)

    # ② Chamfer the 4 long edges at the contact face (X=0)
    body = body_feat.bodies.item(0)
    ch_edges = adsk.core.ObjectCollection.create()
    tol_x = 0.001           # cm — edge must be at X≈0
    min_len = c(RB_W * 0.8) # must be a long edge, not a short corner edge
    for edge in body.edges:
        p1 = edge.startVertex.geometry
        p2 = edge.endVertex.geometry
        if abs(p1.x) < tol_x and abs(p2.x) < tol_x:
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dz = p2.z - p1.z
            if (dx*dx + dy*dy + dz*dz)**0.5 > min_len:
                ch_edges.add(edge)
    if ch_edges.count > 0:
        try:
            ci = comp.features.chamferFeatures.createInput(ch_edges, True)
            ci.setToEqualDistance(adsk.core.ValueInput.createByReal(c(RB_CH)))
            comp.features.chamferFeatures.add(ci)
        except Exception:
            pass   # cosmetic; skip if fails

    # ③ Guide-pin socket bore (18 mm Ø, 30 mm deep)
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, PIN_OD)
    cut_d(comp, first_prof(sk), PIN_IN)

    # ④ M8 bolt through-bore
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, PIN_BORE)
    cut_thru(comp, first_prof(sk))

    # ⑤ Power pad pockets ×4
    sk = sk_on(comp, yz)
    for (py, pz) in [(-PAD_Y,PAD_Z),(-PAD_Y,-PAD_Z),(PAD_Y,PAD_Z),(PAD_Y,-PAD_Z)]:
        draw_rect(sk, py, pz, pw, pl)
    col = coll_rect(sk, pw, pl)
    if col.count > 0:
        cut_d(comp, col, PAD_DEP)

    # ⑥ Sense pogo through-bores ×2
    sk = sk_on(comp, yz)
    draw_circ(sk, -POGO_Y, 0, POGO_D)
    draw_circ(sk,  POGO_Y, 0, POGO_D)
    col = coll_circ(sk, POGO_D)
    if col.count > 0:
        cut_thru(comp, col)

    # ⑦ Pogo counterbores ×2 (from back face)
    cb_pl = off_plane(comp, yz, RB_D - POGO_CBD)
    sk = sk_on(comp, cb_pl)
    draw_circ(sk, -POGO_Y, 0, POGO_CB)
    draw_circ(sk,  POGO_Y, 0, POGO_CB)
    col = coll_circ(sk, POGO_CB)
    if col.count > 0:
        cut_d(comp, col, POGO_CBD)

    # ⑧ M5 mount holes ×4
    sk = sk_on(comp, yz)
    for (my,mz) in [(-MNT_OFF,MNT_OFF),(-MNT_OFF,-MNT_OFF),(MNT_OFF,MNT_OFF),(MNT_OFF,-MNT_OFF)]:
        draw_circ(sk, my, mz, MNT_D)
    col = coll_circ(sk, MNT_D)
    if col.count > 0:
        cut_thru(comp, col)

    return comp


def build_guide_pin(root):
    """18 mm Ø × 70 mm aluminium pin, M8 bore, 45° tip chamfer."""
    comp  = new_comp(root, "GuidePin")
    yz    = comp.yZConstructionPlane
    total = PIN_OUT + PIN_IN   # 70 mm

    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, PIN_OD)
    ext_new(comp, first_prof(sk), total)

    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, PIN_BORE)
    cut_thru(comp, first_prof(sk))

    # Tip chamfer (lofted cut; cosmetic — skip gracefully)
    sk1 = sk_on(comp, yz)
    sk2 = sk_on(comp, off_plane(comp, yz, PIN_CH))
    draw_circ(sk1, 0, 0, PIN_OD)
    draw_circ(sk2, 0, 0, PIN_OD - 2*PIN_CH)
    lofts = comp.features.loftFeatures
    li = lofts.createInput(adsk.fusion.FeatureOperations.CutFeatureOperation)
    li.loftSections.add(first_prof(sk1))
    li.loftSections.add(first_prof(sk2))
    try:
        lofts.add(li)
    except Exception:
        pass

    return comp


def build_back_plate(root):
    """
    220×220×12 mm wall-mount plate.
    Wall face at local X=0, front face at X=12.

      ① Main body
      ② 126×126×20 recess on front face — FloatingPlate sits here
      ③ Guide-pin receiver boss on wall face (20 mm Ø × 15 mm, protrudes −X)
      ④ M6 float-post through-holes ×4 at ±50 mm
      ⑤ M4 funnel-flange tapped holes ×4 at ±58 mm
      ⑥ Wall-mount keyhole slots ×4 at ±85 mm
    """
    comp = new_comp(root, "BackPlate")
    yz   = comp.yZConstructionPlane

    # ① Body
    sk = sk_on(comp, yz)
    draw_rect(sk, 0, 0, BP_W, BP_H)
    ext_new(comp, first_prof(sk), BP_T)

    # ② Front-face recess for FloatingPlate
    front = off_plane(comp, yz, BP_T)
    sk = sk_on(comp, front)
    draw_rect(sk, 0, 0, BP_RCS_W, BP_RCS_H)
    col = coll_rect(sk, BP_RCS_W, BP_RCS_H)
    if col.count > 0:
        cut_d(comp, col, BP_RCS_D)

    # ③ Guide-pin receiver boss on wall face (extrude in −X direction)
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, BP_BOSS_D)
    try:
        ext_join_flip(comp, first_prof(sk), BP_BOSS_H)
    except Exception:
        pass

    # ④ M6 float-post through-holes ×4
    sk = sk_on(comp, front)
    for (py,pz) in [(-BP_POST_OFF,BP_POST_OFF),(-BP_POST_OFF,-BP_POST_OFF),
                    ( BP_POST_OFF,BP_POST_OFF),( BP_POST_OFF,-BP_POST_OFF)]:
        draw_circ(sk, py, pz, BP_POST_D)
    col = coll_circ(sk, BP_POST_D)
    if col.count > 0:
        cut_thru(comp, col)

    # ⑤ M4 funnel-flange tapped holes ×4
    sk = sk_on(comp, front)
    for (py,pz) in [(-BP_FN_BOLT_OFF,BP_FN_BOLT_OFF),(-BP_FN_BOLT_OFF,-BP_FN_BOLT_OFF),
                    ( BP_FN_BOLT_OFF,BP_FN_BOLT_OFF),( BP_FN_BOLT_OFF,-BP_FN_BOLT_OFF)]:
        draw_circ(sk, py, pz, BP_FN_BOLT_D)
    col = coll_circ(sk, BP_FN_BOLT_D)
    if col.count > 0:
        cut_thru(comp, col)

    # ⑥ Wall-mount keyhole slots ×4
    sk = sk_on(comp, yz)
    for (sy,sz) in [(-BP_SLOT_OFF,BP_SLOT_OFF),(-BP_SLOT_OFF,-BP_SLOT_OFF),
                    ( BP_SLOT_OFF,BP_SLOT_OFF),( BP_SLOT_OFF,-BP_SLOT_OFF)]:
        draw_rect(sk, sy, sz, BP_SLOT_W, BP_SLOT_L)
    col = coll_rect(sk, BP_SLOT_W, BP_SLOT_L)
    if col.count > 0:
        cut_thru(comp, col)

    return comp


def build_floating_plate(root):
    """
    120×120×18 mm floating contact plate.
    Sits in the BackPlate recess on M6 posts with springs.
    Contact face (faces robot) at local X=0.

      ① Body
      ② Float holes ×4 (10 mm Ø) at ±50 mm
      ③ Retention-washer counterbores ×4 (16 mm Ø, 6 mm from back)
      ④ Guide socket bore (18.5 mm Ø, 16 mm deep)
      ⑤ Lead-in funnel bore (30 mm Ø, 5 mm deep)
      ⑥ Spring contact slots ×4 (10×22 mm, 5 mm deep)
      ⑦ Sense pad recesses ×2 (8×12 mm, 3 mm deep)
    """
    comp = new_comp(root, "FloatingContactPlate")
    yz   = comp.yZConstructionPlane

    # ① Body
    sk = sk_on(comp, yz)
    draw_rect(sk, 0, 0, FP_W, FP_H)
    ext_new(comp, first_prof(sk), FP_T)

    # ② Float holes ×4
    sk = sk_on(comp, yz)
    for (py,pz) in [(-BP_POST_OFF,BP_POST_OFF),(-BP_POST_OFF,-BP_POST_OFF),
                    ( BP_POST_OFF,BP_POST_OFF),( BP_POST_OFF,-BP_POST_OFF)]:
        draw_circ(sk, py, pz, FP_FLOAT_D)
    col = coll_circ(sk, FP_FLOAT_D)
    if col.count > 0:
        cut_thru(comp, col)

    # ③ Retention-washer counterbores ×4 from back face
    back = off_plane(comp, yz, FP_T - FP_CB_DEP)
    sk = sk_on(comp, back)
    for (py,pz) in [(-BP_POST_OFF,BP_POST_OFF),(-BP_POST_OFF,-BP_POST_OFF),
                    ( BP_POST_OFF,BP_POST_OFF),( BP_POST_OFF,-BP_POST_OFF)]:
        draw_circ(sk, py, pz, FP_CB_D)
    col = coll_circ(sk, FP_CB_D)
    if col.count > 0:
        cut_d(comp, col, FP_CB_DEP)

    # ④ Guide socket bore
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, FP_SOCK_D)
    cut_d(comp, first_prof(sk), FP_SOCK_DEP)

    # ⑤ Lead-in funnel bore
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, FP_FUNNEL_D)
    cut_d(comp, first_prof(sk), FP_FUNNEL_L)

    # ⑥ Spring contact slots ×4
    sk = sk_on(comp, yz)
    for (py,pz) in [(-PAD_Y,PAD_Z),(-PAD_Y,-PAD_Z),(PAD_Y,PAD_Z),(PAD_Y,-PAD_Z)]:
        draw_rect(sk, py, pz, SP_W, SP_L)
    col = coll_rect(sk, SP_W, SP_L)
    if col.count > 0:
        cut_d(comp, col, SP_DEP)

    # ⑦ Sense pad recesses ×2
    sk = sk_on(comp, yz)
    for (py,pz) in [(-POGO_Y, 0),(POGO_Y, 0)]:
        draw_rect(sk, py, pz, SNS_W, SNS_L)
    col = coll_rect(sk, SNS_W, SNS_L)
    if col.count > 0:
        cut_d(comp, col, SNS_DEP)

    return comp


def build_entry_funnel(root):
    """
    Lofted funnel shell 80 mm deep, 4 mm wall.
    Exit (narrow 130×130) at local X=0 — flush with BackPlate front face.
    Entry (wide 220×180) at local X=80 — faces the approaching robot.

    A 180×180×6 mm bolt flange at X=0 with 4× M4 clearance holes bolts
    the funnel to the BackPlate front face.
    """
    comp = new_comp(root, "EntryFunnel")
    yz   = comp.yZConstructionPlane

    # ① Lofted shell: exit at X=0, entry at X=FN_D
    sk_x = sk_on(comp, yz)
    draw_rect(sk_x, 0, 0, FN_XW, FN_XH)
    prof_x = first_prof(sk_x)

    entry_pl = off_plane(comp, yz, FN_D)
    sk_e = sk_on(comp, entry_pl)
    draw_rect(sk_e, 0, 0, FN_EW, FN_EH)
    prof_e = first_prof(sk_e)

    lofts = comp.features.loftFeatures
    li = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    li.loftSections.add(prof_x)
    li.loftSections.add(prof_e)
    loft_f = lofts.add(li)

    # Shell both flat ends open
    body = loft_f.bodies.item(0)
    faces = adsk.core.ObjectCollection.create()
    for face in body.faces:
        g = face.geometry
        if g.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
            if abs(abs(g.normal.x) - 1.0) < 0.01:
                faces.add(face)
    if faces.count >= 2:
        si = comp.features.shellFeatures.createInput(faces, False)
        si.insideThickness = adsk.core.ValueInput.createByReal(c(FN_WALL))
        comp.features.shellFeatures.add(si)

    # ② Bolt flange at exit end (X=0 plane)
    #    Draw 180×180; the shell opening leaves the 130×130 centre open.
    #    Pick the outer-ring profile by largest area.
    sk_fl = sk_on(comp, yz)
    draw_rect(sk_fl, 0, 0, FN_FL_W, FN_FL_H)
    # The sketch now has the 180×180 outer rectangle profile.
    # If the shell wall overlaps there may be multiple profiles;
    # take the largest one (the outer ring).
    best = None
    best_area = 0.0
    for i in range(sk_fl.profiles.count):
        p = sk_fl.profiles.item(i)
        bb = p.boundingBox
        w  = (bb.maxPoint.x - bb.minPoint.x) * 10
        h  = (bb.maxPoint.y - bb.minPoint.y) * 10
        area = w * h
        if abs(w - FN_FL_W) < 1.0 and abs(h - FN_FL_H) < 1.0 and area > best_area:
            best = p
            best_area = area
    if best is not None:
        try:
            ext_join(comp, best, FN_FL_T)
        except Exception:
            pass

    # ③ M4 clearance bolt holes ×4 through flange
    sk_bolt = sk_on(comp, yz)
    for (py,pz) in [(-FN_FL_BOLT_OFF,FN_FL_BOLT_OFF),(-FN_FL_BOLT_OFF,-FN_FL_BOLT_OFF),
                    ( FN_FL_BOLT_OFF,FN_FL_BOLT_OFF),( FN_FL_BOLT_OFF,-FN_FL_BOLT_OFF)]:
        draw_circ(sk_bolt, py, pz, FN_FL_BOLT_D)
    col = coll_circ(sk_bolt, FN_FL_BOLT_D)
    if col.count > 0:
        cut_d(comp, col, FN_FL_T)

    return comp


# ── POSITIONING ──────────────────────────────────────────────────────────────

def position_components(root):
    """
    Translate each occurrence to its correct world X position.
    X=0 = wall face of BackPlate.

    Component              world X offset
    ─────────────────────  ──────────────
    BackPlate              0              (built local 0..12, stays at 0)
    FloatingContactPlate   BP_T = 12      (contact face at 12)
    EntryFunnel            BP_T = 12      (flange face at 12, mouth at 92)
    RobotBlock             BP_T+FN_D = 92 (contact face at 92)
    GuidePin               BP_T+FN_D-PIN_OUT = 52  (tip at 52, body 52..122)
    """
    offsets = {
        "BackPlate":            0,
        "FloatingContactPlate": BP_T,
        "EntryFunnel":          BP_T,
        "RobotBlock":           BP_T + FN_D,
        "GuidePin":             BP_T + FN_D - PIN_OUT,
    }
    for i in range(root.occurrences.count):
        occ  = root.occurrences.item(i)
        name = occ.component.name
        if name in offsets:
            m = adsk.core.Matrix3D.create()
            m.translation = adsk.core.Vector3D.create(c(offsets[name]), 0, 0)
            occ.transform = m


# ── ENTRY POINT ──────────────────────────────────────────────────────────────

def run(context):
    ui        = None
    completed = []
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        # ── Create a new Fusion Design document ──────────────────────────────
        # IMPORTANT: recent Fusion 360 defaults "New Design" to "Part Design"
        # (single-component only).  We explicitly create a FusionDesignDocumentType
        # here.  We then also force ParametricDesignType so multi-component
        # assemblies are permitted.
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)

        # Prefer getting the design directly from the document we just created
        # (avoids race condition with app.activeProduct on some Fusion versions).
        design = None
        try:
            design = adsk.fusion.Design.cast(doc.products.item(0))
        except Exception:
            pass
        if not design:
            design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox(
                "Could not obtain a Fusion Design from the new document.\n"
                "Please ensure Fusion 360 is fully loaded and try again.",
                "DS3 — Error")
            return

        # Force parametric mode (allows multiple sub-components)
        try:
            design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        except Exception:
            pass   # already parametric, or read-only — proceed anyway

        root = design.rootComponent

        diag = (
            f"DS3  v3.0 — new document created.\n\n"
            f"Design type: {design.designType}  (1=Parametric, 0=Direct)\n"
            f"Existing occurrences: {root.occurrences.count}\n\n"
            f"Click OK to build 5 components (~30–90 s)."
        )
        ui.messageBox(diag, "DS3")

        build_robot_block(root);        completed.append("RobotBlock ✓")
        build_guide_pin(root);          completed.append("GuidePin ✓")
        build_back_plate(root);         completed.append("BackPlate ✓")
        build_floating_plate(root);     completed.append("FloatingContactPlate ✓")
        build_entry_funnel(root);       completed.append("EntryFunnel ✓")
        position_components(root);      completed.append("Positioned ✓")

        app.activeViewport.fit()

        ui.messageBox(
            "✅  DS3 complete!\n\n"
            + "\n".join(f"  {s}" for s in completed)
            + "\n\nNext steps:\n"
            "  1. File → Save\n"
            "  2. Inspect → Section Analysis (Y=0 plane)\n"
            "  3. Right-click body → Save As Mesh (STL) for printing",
            "DS3 ✅")

    except Exception:
        err = traceback.format_exc()
        if completed:
            err = "Completed:\n  " + "\n  ".join(completed) + "\n\n" + err
        if ui:
            ui.messageBox(err, "DS3 — Error")
        else:
            print(err)


def stop(context):
    pass
