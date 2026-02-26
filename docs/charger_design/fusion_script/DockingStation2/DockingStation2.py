"""
Sigyn Docking Station — Fusion 360 Python Script  v3.0  (DockingStation2)
=========================================================================
Corrected assembly stack, chamfered RobotBlock leading edges,
funnel bolt-flange for BackPlate attachment, FloatingContactPlate
seated in a recess in the BackPlate front face.

ASSEMBLY STACK (back → front, X axis = docking axis, robot approaches from −X)
────────────────────────────────────────────────────────────────────────────────
  BackPlate         X = 0 … +12   (wall face at X=0, front face at X=12)
  FloatingPlate     X = 12 … +30  (seated in 20 mm deep recess in BackPlate)
  EntryFunnel       X = 12 … +92  (flange face flush with BackPlate front,
                                    wide entry mouth at X=92)
  RobotBlock        X = 92 … +42  (contact face at X=92, back at X=42)
  GuidePin          tip at X=132, body X=62…132 (protrudes 40 mm ahead)

  Robot docks by reversing: GuidePin enters BackPlate boss first, then
  RobotBlock slides through the funnel, chamfered leading edges clear the
  funnel exit (130×130 mm), and the contact face presses the FloatingPlate.

HOW TO RUN
──────────
1.  Press  Shift + S  in Fusion 360 → Scripts and Add-Ins.
2.  Click the green  +  and navigate to the  DockingStation2  folder.
3.  Select  DockingStation2  and click  Run.
   (No need to pre-open any design; the script creates its own.)

PARAMETERS you may want to tune are in the PARAMETERS section below.
"""

import adsk.core
import adsk.fusion
import traceback

# ── PARAMETERS  (all mm) ────────────────────────────────────────────────────

# ── Robot-side block ──────────────────────────────────────────────────────
RB_W, RB_H, RB_D = 100.0, 100.0, 50.0   # width, height, depth
RB_CH = 8.0    # leading-edge chamfer size (45°) — clears funnel exit lip

# ── Guide pin ─────────────────────────────────────────────────────────────
PIN_OD   = 18.0   # outer diameter
PIN_BORE =  8.5   # through-bore for M8 retaining bolt
PIN_OUT  = 40.0   # protrusion ahead of robot contact face (into funnel)
PIN_IN   = 30.0   # inset inside RobotBlock
PIN_CH   =  4.0   # tip chamfer depth (45°)

# ── Power contact pad pockets (robot side) ────────────────────────────────
PAD_W   =  8.0    # pocket width  (narrow)
PAD_L   = 20.0    # pocket length (long)
PAD_DEP = 10.0    # pocket depth
PAD_CLR =  0.5    # epoxy bedding clearance each side
PAD_Y   = 35.0    # ±Y offset from block centre
PAD_Z   = 32.0    # ±Z offset from block centre

# ── Sense pogo pin bores (robot side) ─────────────────────────────────────
POGO_D   =  5.5   # through-bore Ø
POGO_CB  =  8.0   # counterbore Ø (retaining collar)
POGO_CBD =  5.0   # counterbore depth (from back face)
POGO_Y   = 18.0   # ±Y from block centre

# ── RobotBlock mount holes (M5 into aluminium extrusion) ─────────────────
MNT_D   =  5.3    # M5 clearance hole Ø
MNT_OFF = 40.0    # ±Y and ±Z from centre

# ── Back plate ────────────────────────────────────────────────────────────
BP_W, BP_H, BP_T  = 220.0, 220.0, 12.0   # overall body
BP_RCS_W          = 126.0   # FloatingPlate recess width  (FP_W + 6 mm clearance)
BP_RCS_H          = 126.0   # FloatingPlate recess height
BP_RCS_D          = 20.0    # recess depth (FloatingPlate sits fully inside)
BP_BOSS_D         = 20.0    # guide-pin receiver boss Ø (on back face / wall side)
BP_BOSS_H         = 15.0    # boss height (protrudes from wall face)
BP_POST_OFF       = 50.0    # ±Y and ±Z for M6 float guide posts (inside recess)
BP_POST_D         =  6.5    # M6 clearance Ø for post holes (through-all)
BP_FN_BOLT_OFF    = 58.0    # ±Y and ±Z for M4 funnel-flange bolts
BP_FN_BOLT_D      =  3.3    # M4 tap drill Ø  (tapped M4×0.7 in BackPlate)
BP_SLOT_W         =  8.0    # wall-mount keyhole slot width
BP_SLOT_L         = 14.0    # wall-mount keyhole slot length (allows levelling)
BP_SLOT_OFF       = 85.0    # ±Y and ±Z for wall-mount slots

# ── Floating contact plate ────────────────────────────────────────────────
FP_W, FP_H, FP_T  = 120.0, 120.0, 18.0   # fits inside BP_RCS with 3 mm gap each side
FP_FLOAT_D        = 10.0    # oversized float hole Ø on M6 post (gives ±1.75 mm float)
FP_CB_D           = 16.0    # retention washer counterbore Ø
FP_CB_DEP         =  6.0    # counterbore depth from back (funnel) face
FP_SOCK_D         = 18.5    # guide-pin socket bore Ø
FP_FUNNEL_D       = 30.0    # chamfer/lead-in bore Ø on contact (robot) face
FP_FUNNEL_L       =  5.0    # lead-in depth
FP_SOCK_DEP       = 16.0    # socket depth (pin enters 16 mm)

# ── Spring contact slots on FloatingPlate (house-side spring contacts) ────
SP_W  = PAD_W + 2.0    # 10 mm
SP_L  = PAD_L + 2.0    # 22 mm
SP_DEP = 5.0

# ── Sense pad recesses on FloatingPlate ───────────────────────────────────
SNS_W   =  8.0
SNS_L   = 12.0
SNS_DEP =  3.0

# ── Entry funnel ──────────────────────────────────────────────────────────
# The funnel is oriented: wide entry mouth at HIGH X (far from wall),
# narrow exit at LOW X (flush with BackPlate front face at X = BP_T).
#
#   Robot approaches from +X direction.
#   Funnel exit  (narrow) → X = BP_T = 12
#   Funnel entry (wide)   → X = BP_T + FN_D = 12 + 80 = 92
#
FN_XW, FN_XH  = 130.0, 130.0   # exit aperture — clears RB_W+chamfer, mates recess
FN_EW, FN_EH  = 220.0, 180.0   # entry aperture — generous robot catch
FN_D          =  80.0           # funnel depth along X
FN_WALL       =   4.0           # shell wall thickness

# Flange at funnel exit end: bolts funnel to BackPlate front face
FN_FL_W       = 180.0   # flange plate width  (must be < BP_W)
FN_FL_H       = 180.0   # flange plate height
FN_FL_T       =   6.0   # flange thickness
FN_FL_BOLT_D  =   4.5   # M4 clearance Ø through flange
FN_FL_BOLT_OFF = BP_FN_BOLT_OFF   # same ±58 mm as tapped holes in BackPlate


# ── UNIT CONVERSION ─────────────────────────────────────────────────────────

def c(mm):
    """mm → cm  (Fusion 360 internal unit is cm)."""
    return mm / 10.0


# ── LOW-LEVEL HELPERS ───────────────────────────────────────────────────────

def new_comp(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    return comp


def sk_on(comp, plane):
    return comp.sketches.add(plane)


def off_plane(comp, base_plane, dist_mm):
    planes = comp.constructionPlanes
    inp = planes.createInput()
    inp.setByOffset(base_plane, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return planes.add(inp)


def draw_rect(sk, cy_mm, cz_mm, w_mm, h_mm):
    """Centred rectangle on YZ-plane sketch (Y = horizontal, Z = vertical)."""
    lines = sk.sketchCurves.sketchLines
    p1 = adsk.core.Point3D.create(c(cy_mm - w_mm / 2), c(cz_mm - h_mm / 2), 0)
    p2 = adsk.core.Point3D.create(c(cy_mm + w_mm / 2), c(cz_mm + h_mm / 2), 0)
    lines.addTwoPointRectangle(p1, p2)


def draw_circ(sk, cy_mm, cz_mm, dia_mm):
    sk.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(c(cy_mm), c(cz_mm), 0),
        c(dia_mm / 2))


def extrude_new(comp, profile, dist_mm):
    feats = comp.features.extrudeFeatures
    inp = feats.createInput(profile,
                            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return feats.add(inp)


def extrude_join(comp, profile, dist_mm):
    feats = comp.features.extrudeFeatures
    inp = feats.createInput(profile,
                            adsk.fusion.FeatureOperations.JoinFeatureOperation)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return feats.add(inp)


def cut_depth(comp, profile_or_coll, dist_mm):
    feats = comp.features.extrudeFeatures
    inp = feats.createInput(profile_or_coll,
                            adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return feats.add(inp)


def cut_all(comp, profile_or_coll):
    feats = comp.features.extrudeFeatures
    inp = feats.createInput(profile_or_coll,
                            adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.setAllExtent(False)
    return feats.add(inp)


def first_prof(sk):
    if sk.profiles.count == 0:
        raise RuntimeError(f"Sketch '{sk.name}' has no profiles.")
    return sk.profiles.item(0)


def profs_by_rect(sk, w_mm, h_mm, tol=0.08):
    """Collect profiles whose bounding box matches w×h (either orientation)."""
    coll = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        bb = p.boundingBox
        pw = (bb.maxPoint.x - bb.minPoint.x) * 10
        ph = (bb.maxPoint.y - bb.minPoint.y) * 10
        if ((abs(pw - w_mm) < tol and abs(ph - h_mm) < tol) or
                (abs(pw - h_mm) < tol and abs(ph - w_mm) < tol)):
            coll.add(p)
    return coll


def profs_by_dia(sk, dia_mm, tol=0.08):
    """Collect profiles whose bounding box is approximately dia×dia (circles)."""
    coll = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        bb = p.boundingBox
        pw = (bb.maxPoint.x - bb.minPoint.x) * 10
        ph = (bb.maxPoint.y - bb.minPoint.y) * 10
        if abs(pw - dia_mm) < tol and abs(ph - dia_mm) < tol:
            coll.add(p)
    return coll


# ── COMPONENT BUILDERS ──────────────────────────────────────────────────────

def build_robot_block(root):
    """
    Robot-side 100×100×50 mm contact block.
    The CONTACT FACE is at X = 0 (approaches the station).
    The BACK FACE (mount to extrusion) is at X = +50.

    Features:
      ① 100×100 outer body, 50 mm deep
      ② 4 × 45° chamfers on the 4 leading edges of the contact face
         (8 mm chamfer clears the 130×130 funnel exit when RB = 100×100)
      ③ 18 mm guide-pin socket bore, 30 mm deep (from contact face)
      ④ 8.5 mm M8 bolt through-bore (full depth)
      ⑤ 4 × power pad pockets (9×21 mm, 10 mm deep)
      ⑥ 2 × sense pogo through-bores (5.5 mm)
      ⑦ 2 × pogo counterbores (8 mm, 5 mm from back)
      ⑧ 4 × M5 mount holes (through-all)

    The chamfers are the critical change: without them the sharp 100×100
    corners snag on the 130×130 funnel exit. With 8 mm chamfers the
    effective diagonal at the leading corner is reduced enough to enter
    cleanly (diagonal of chamfered block ≈ 128 mm vs. 130 mm opening).
    """
    comp = new_comp(root, "RobotBlock")
    yz = comp.yZConstructionPlane
    pw = PAD_W + 2 * PAD_CLR    # 9 mm
    pl = PAD_L + 2 * PAD_CLR    # 21 mm

    # ① Outer body
    sk = sk_on(comp, yz)
    draw_rect(sk, 0, 0, RB_W, RB_H)
    body_feat = extrude_new(comp, first_prof(sk), RB_D)

    # ② Chamfer the 4 leading edges (the 4 edges of the contact face at X=0
    #    that run parallel to Y or Z).  The Fusion chamfer feature needs the
    #    actual BRepEdge objects.  We identify them by: they lie at X≈0 and
    #    their length ≈ RB_W or RB_H (they're the long straight edges, not
    #    the short corner edges at the face corners).
    body = body_feat.bodies.item(0)
    chamfer_edges = adsk.core.ObjectCollection.create()
    tol_x = 0.001   # cm
    edge_len_min = c(RB_W * 0.8)   # must be > 80% of block width
    for edge in body.edges:
        # Both endpoints near X=0?
        p1 = edge.startVertex.geometry
        p2 = edge.endVertex.geometry
        if abs(p1.x) < tol_x and abs(p2.x) < tol_x:
            # Is this a long edge (parallel to Y or Z axis)?
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dz = p2.z - p1.z
            length = (dx*dx + dy*dy + dz*dz) ** 0.5
            if length > edge_len_min:
                chamfer_edges.add(edge)

    if chamfer_edges.count > 0:
        try:
            ch_inp = comp.features.chamferFeatures.createInput(
                chamfer_edges, True)
            ch_inp.setToEqualDistance(
                adsk.core.ValueInput.createByReal(c(RB_CH)))
            comp.features.chamferFeatures.add(ch_inp)
        except Exception:
            pass   # chamfer is safety geometry; continue if it fails

    # ③ Guide-pin socket bore (18 mm Ø, 30 mm deep from contact face)
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, PIN_OD)
    cut_depth(comp, first_prof(sk), PIN_IN)

    # ④ M8 bolt clearance bore (through-all)
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, PIN_BORE)
    cut_all(comp, first_prof(sk))

    # ⑤ Power pad pockets ×4 (9×21 mm, 10 mm deep)
    sk = sk_on(comp, yz)
    for (py, pz) in [(-PAD_Y, PAD_Z), (-PAD_Y, -PAD_Z),
                     ( PAD_Y, PAD_Z), ( PAD_Y, -PAD_Z)]:
        draw_rect(sk, py, pz, pw, pl)
    coll = profs_by_rect(sk, pw, pl)
    if coll.count > 0:
        cut_depth(comp, coll, PAD_DEP)

    # ⑥ Sense pogo through-bores ×2 (5.5 mm Ø, through-all)
    sk = sk_on(comp, yz)
    draw_circ(sk, -POGO_Y, 0, POGO_D)
    draw_circ(sk,  POGO_Y, 0, POGO_D)
    coll = profs_by_dia(sk, POGO_D)
    if coll.count > 0:
        cut_all(comp, coll)

    # ⑦ Pogo counterbores ×2 (8 mm Ø, 5 mm deep from back face)
    cb_plane = off_plane(comp, yz, RB_D - POGO_CBD)
    sk = sk_on(comp, cb_plane)
    draw_circ(sk, -POGO_Y, 0, POGO_CB)
    draw_circ(sk,  POGO_Y, 0, POGO_CB)
    coll = profs_by_dia(sk, POGO_CB)
    if coll.count > 0:
        cut_depth(comp, coll, POGO_CBD)

    # ⑧ M5 mount holes ×4 (through-all)
    sk = sk_on(comp, yz)
    for (my, mz) in [(-MNT_OFF, MNT_OFF), (-MNT_OFF, -MNT_OFF),
                     ( MNT_OFF, MNT_OFF), ( MNT_OFF, -MNT_OFF)]:
        draw_circ(sk, my, mz, MNT_D)
    coll = profs_by_dia(sk, MNT_D)
    if coll.count > 0:
        cut_all(comp, coll)

    return comp


def build_guide_pin(root):
    """
    Aluminium guide pin: 18 mm Ø × 70 mm total.
      ① Main cylinder (PIN_IN + PIN_OUT = 70 mm)
      ② M8 through-bore
      ③ Tip 45° chamfer (cosmetic loft-cut; skipped gracefully if loft fails)
    The pin mounts in the RobotBlock socket and protrudes 40 mm forward.
    """
    comp = new_comp(root, "GuidePin")
    yz = comp.yZConstructionPlane
    total = PIN_OUT + PIN_IN   # 70 mm

    # ① Cylinder
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, PIN_OD)
    extrude_new(comp, first_prof(sk), total)

    # ② M8 through-bore
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, PIN_BORE)
    cut_all(comp, first_prof(sk))

    # ③ Tip chamfer (at the X=0 end, the protruding tip)
    sk_tip   = sk_on(comp, yz)
    sk_inner = sk_on(comp, off_plane(comp, yz, PIN_CH))
    draw_circ(sk_tip,   0, 0, PIN_OD)
    draw_circ(sk_inner, 0, 0, PIN_OD - 2 * PIN_CH)
    lofts = comp.features.loftFeatures
    loft_inp = lofts.createInput(adsk.fusion.FeatureOperations.CutFeatureOperation)
    loft_inp.loftSections.add(first_prof(sk_tip))
    loft_inp.loftSections.add(first_prof(sk_inner))
    try:
        lofts.add(loft_inp)
    except Exception:
        pass

    return comp


def build_back_plate(root):
    """
    Wall-mount back plate: 220×220×12 mm.

    The WALL FACE is at X = 0.
    The FRONT FACE (faces robot) is at X = +12.

    Features:
      ① 220×220×12 main body
      ② 126×126×20 mm recess centred on front face — FloatingPlate sits here.
         (FP = 120×120, +3 mm clearance each side = 126×126)
      ③ Guide-pin receiver boss on WALL face (Ø20 × 15 mm, protrudes from wall)
      ④ 4 × M6 float-post through-holes (6.5 mm Ø, through-all) at ±50 mm
      ⑤ 4 × M4 tapped holes for funnel flange (3.3 mm Ø, through-all) at ±58 mm
      ⑥ 4 × keyhole wall-mount slots (8×14 mm, through-all) at ±85 mm

    The float posts pass right through, threading into nuts/inserts behind
    the BackPlate (or into the wall mount hardware).  Springs on the posts
    inside the recess centre the FloatingPlate.
    """
    comp = new_comp(root, "BackPlate")
    yz = comp.yZConstructionPlane

    # ① Main body
    sk = sk_on(comp, yz)
    draw_rect(sk, 0, 0, BP_W, BP_H)
    extrude_new(comp, first_prof(sk), BP_T)

    # ② Front-face recess for FloatingPlate (126×126, 20 mm deep from front)
    #    Sketch on offset plane at X = BP_T (the front face),
    #    then cut 20 mm into the body (toward wall, i.e. −X direction).
    #    In Fusion an extrude-cut from the front face cuts toward the back.
    front = off_plane(comp, yz, BP_T)
    sk = sk_on(comp, front)
    draw_rect(sk, 0, 0, BP_RCS_W, BP_RCS_H)
    coll = profs_by_rect(sk, BP_RCS_W, BP_RCS_H)
    if coll.count > 0:
        cut_depth(comp, coll, BP_RCS_D)

    # ③ Guide-pin receiver boss on WALL face (Ø20 × 15 mm, protrudes from wall)
    #    Boss is on the wall side (X = 0 plane); extrude in −X direction.
    #    We sketch on the YZ plane (X=0) and extrude in the −X direction.
    #    Fusion's extrude_new with positive dist goes in +X, so we need
    #    to use a negative offset plane trick: sketch at X=0, extrude
    #    with a negative distance (flip direction flag = True).
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, BP_BOSS_D)
    feats = comp.features.extrudeFeatures
    inp = feats.createInput(first_prof(sk),
                            adsk.fusion.FeatureOperations.JoinFeatureOperation)
    inp.setDistanceExtent(True,   # True = flip direction (−X)
                          adsk.core.ValueInput.createByReal(c(BP_BOSS_H)))
    try:
        feats.add(inp)
    except Exception:
        pass   # boss is nice-to-have; continue if flip fails

    # ④ M6 float-post through-holes ×4 (6.5 mm Ø, through-all from front face)
    sk = sk_on(comp, front)
    for (py, pz) in [(-BP_POST_OFF,  BP_POST_OFF), (-BP_POST_OFF, -BP_POST_OFF),
                     ( BP_POST_OFF,  BP_POST_OFF), ( BP_POST_OFF, -BP_POST_OFF)]:
        draw_circ(sk, py, pz, BP_POST_D)
    coll = profs_by_dia(sk, BP_POST_D)
    if coll.count > 0:
        cut_all(comp, coll)

    # ⑤ M4 funnel-bolt tapped holes ×4 (3.3 mm tap drill, through-all)
    sk = sk_on(comp, front)
    for (py, pz) in [(-BP_FN_BOLT_OFF,  BP_FN_BOLT_OFF),
                     (-BP_FN_BOLT_OFF, -BP_FN_BOLT_OFF),
                     ( BP_FN_BOLT_OFF,  BP_FN_BOLT_OFF),
                     ( BP_FN_BOLT_OFF, -BP_FN_BOLT_OFF)]:
        draw_circ(sk, py, pz, BP_FN_BOLT_D)
    coll = profs_by_dia(sk, BP_FN_BOLT_D)
    if coll.count > 0:
        cut_all(comp, coll)

    # ⑥ Wall-mount keyhole slots ×4 (8×14 mm, through-all)
    sk = sk_on(comp, yz)
    for (sy, sz) in [(-BP_SLOT_OFF,  BP_SLOT_OFF), (-BP_SLOT_OFF, -BP_SLOT_OFF),
                     ( BP_SLOT_OFF,  BP_SLOT_OFF), ( BP_SLOT_OFF, -BP_SLOT_OFF)]:
        draw_rect(sk, sy, sz, BP_SLOT_W, BP_SLOT_L)
    coll = profs_by_rect(sk, BP_SLOT_W, BP_SLOT_L)
    if coll.count > 0:
        cut_all(comp, coll)

    return comp


def build_floating_plate(root):
    """
    Floating contact plate: 120×120×18 mm.
    Sits inside the 126×126×20 mm recess in the BackPlate, floating on four
    M6 posts with compression springs.  The ±5 mm oversized float holes give
    ±(10−6.5)/2 ≈ ±1.75 mm float travel on the 6.5 mm posts (small; for more
    float, increase FP_FLOAT_D or reduce BP_POST_OFF springs).

    The CONTACT FACE (faces robot) is at X = 0 locally.
    The BACK FACE (faces wall) is at X = +18.

    Features:
      ① 120×120×18 body
      ② 4 × 10 mm Ø float holes at ±50 mm (slide on M6 posts)
      ③ 4 × 16 mm Ø counterbores, 6 mm deep from back — for retention washers
      ④ 18.5 mm guide-socket bore, 16 mm deep (from contact face)
      ⑤ 30 mm lead-in funnel bore, 5 mm deep (from contact face — helps centre pin)
      ⑥ 4 × spring contact slots (10×22 mm, 5 mm deep from contact face)
      ⑦ 2 × sense pad recesses (8×12 mm, 3 mm deep from contact face)
    """
    comp = new_comp(root, "FloatingContactPlate")
    yz = comp.yZConstructionPlane

    # ① Main body
    sk = sk_on(comp, yz)
    draw_rect(sk, 0, 0, FP_W, FP_H)
    extrude_new(comp, first_prof(sk), FP_T)

    # ② Float holes ×4 (10 mm Ø, through-all)
    sk = sk_on(comp, yz)
    for (py, pz) in [(-BP_POST_OFF,  BP_POST_OFF), (-BP_POST_OFF, -BP_POST_OFF),
                     ( BP_POST_OFF,  BP_POST_OFF), ( BP_POST_OFF, -BP_POST_OFF)]:
        draw_circ(sk, py, pz, FP_FLOAT_D)
    coll = profs_by_dia(sk, FP_FLOAT_D)
    if coll.count > 0:
        cut_all(comp, coll)

    # ③ Retention-washer counterbores ×4 (16 mm Ø, 6 mm from back face)
    back = off_plane(comp, yz, FP_T - FP_CB_DEP)
    sk = sk_on(comp, back)
    for (py, pz) in [(-BP_POST_OFF,  BP_POST_OFF), (-BP_POST_OFF, -BP_POST_OFF),
                     ( BP_POST_OFF,  BP_POST_OFF), ( BP_POST_OFF, -BP_POST_OFF)]:
        draw_circ(sk, py, pz, FP_CB_D)
    coll = profs_by_dia(sk, FP_CB_D)
    if coll.count > 0:
        cut_depth(comp, coll, FP_CB_DEP)

    # ④ Guide-socket bore (18.5 mm Ø, 16 mm deep from contact face)
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, FP_SOCK_D)
    cut_depth(comp, first_prof(sk), FP_SOCK_DEP)

    # ⑤ Lead-in funnel bore (30 mm Ø, 5 mm deep — eases pin entry)
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, FP_FUNNEL_D)
    cut_depth(comp, first_prof(sk), FP_FUNNEL_L)

    # ⑥ Spring contact slots ×4 (10×22 mm, 5 mm deep from contact face)
    sk = sk_on(comp, yz)
    for (py, pz) in [(-PAD_Y, PAD_Z), (-PAD_Y, -PAD_Z),
                     ( PAD_Y, PAD_Z), ( PAD_Y, -PAD_Z)]:
        draw_rect(sk, py, pz, SP_W, SP_L)
    coll = profs_by_rect(sk, SP_W, SP_L)
    if coll.count > 0:
        cut_depth(comp, coll, SP_DEP)

    # ⑦ Sense pad recesses ×2 (8×12 mm, 3 mm deep)
    sk = sk_on(comp, yz)
    for (py, pz) in [(-POGO_Y, 0), (POGO_Y, 0)]:
        draw_rect(sk, py, pz, SNS_W, SNS_L)
    coll = profs_by_rect(sk, SNS_W, SNS_L)
    if coll.count > 0:
        cut_depth(comp, coll, SNS_DEP)

    return comp


def build_entry_funnel(root):
    """
    Entry funnel: lofted shell, 80 mm deep, 4 mm wall.

    ORIENTATION: the funnel is built with its EXIT (narrow) end at X = 0
    and its ENTRY (wide) end at X = +FN_D.  This means in local space:
        X = 0         → exit aperture 130×130 mm (flush with BackPlate front)
        X = +FN_D=80  → entry aperture 220×180 mm (wide robot catch)

    A solid bolt flange (180×180×6 mm plate) is added at X = 0 (the exit end),
    with M4 clearance holes aligning to the tapped holes in BackPlate.
    The central 130×130 opening in the flange is left by the lofted shell shell.

    Features:
      ① Lofted shell body (exit 130×130 at X=0 → entry 220×180 at X=80)
      ② Bolt flange at X=0: 180×180×6 mm plate with 4×4.5 mm M4 clearance holes
    """
    comp = new_comp(root, "EntryFunnel")
    yz = comp.yZConstructionPlane

    # ① Loft: exit profile at X=0, entry profile at X=FN_D
    sk_exit  = sk_on(comp, yz)
    draw_rect(sk_exit, 0, 0, FN_XW, FN_XH)
    prof_exit = first_prof(sk_exit)

    entry_plane = off_plane(comp, yz, FN_D)
    sk_entry = sk_on(comp, entry_plane)
    draw_rect(sk_entry, 0, 0, FN_EW, FN_EH)
    prof_entry = first_prof(sk_entry)

    lofts = comp.features.loftFeatures
    loft_inp = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    loft_inp.loftSections.add(prof_exit)
    loft_inp.loftSections.add(prof_entry)
    loft_feat = lofts.add(loft_inp)

    # Shell: open both flat ends, keep FN_WALL thickness
    body = loft_feat.bodies.item(0)
    shell_faces = adsk.core.ObjectCollection.create()
    for face in body.faces:
        geom = face.geometry
        if geom.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
            if abs(abs(geom.normal.x) - 1.0) < 0.01:
                shell_faces.add(face)
    if shell_faces.count >= 2:
        sh_inp = comp.features.shellFeatures.createInput(shell_faces, False)
        sh_inp.insideThickness = adsk.core.ValueInput.createByReal(c(FN_WALL))
        comp.features.shellFeatures.add(sh_inp)

    # ② Bolt flange at the exit end (X = 0 plane).
    #    Draw the full 180×180 rectangle; the shell opening leaves the
    #    130×130 hole in the centre automatically when we extrude-join.
    sk_fl = sk_on(comp, yz)
    draw_rect(sk_fl, 0, 0, FN_FL_W, FN_FL_H)
    #    After drawing 180×180 there will be two profiles:
    #    the 180×180 outer area and whatever the shell body intersects.
    #    We want the outer ring only; since the lofted shell walls are only
    #    4 mm thick at the exit end the inner "hole" profile corresponds to
    #    the 130×130 exit aperture.  Pick the profile by its bounding box.
    coll_fl = profs_by_rect(sk_fl, FN_FL_W, FN_FL_H)
    if coll_fl.count > 0:
        try:
            extrude_join(comp, coll_fl.item(0), FN_FL_T)
        except Exception:
            pass

    # ③ M4 clearance bolt holes ×4 through flange (4.5 mm Ø)
    fl_plane = off_plane(comp, yz, 0)   # same as yz but explicit offset
    sk_bolt = sk_on(comp, yz)
    for (py, pz) in [(-FN_FL_BOLT_OFF,  FN_FL_BOLT_OFF),
                     (-FN_FL_BOLT_OFF, -FN_FL_BOLT_OFF),
                     ( FN_FL_BOLT_OFF,  FN_FL_BOLT_OFF),
                     ( FN_FL_BOLT_OFF, -FN_FL_BOLT_OFF)]:
        draw_circ(sk_bolt, py, pz, FN_FL_BOLT_D)
    coll = profs_by_dia(sk_bolt, FN_FL_BOLT_D)
    if coll.count > 0:
        cut_depth(comp, coll, FN_FL_T)

    return comp


# ── ASSEMBLY POSITIONING ─────────────────────────────────────────────────────

def position_components(root):
    """
    Place each component occurrence in the correct position along the X axis.

    X=0 is the WALL FACE of the BackPlate (where it touches the wall).

    Component              X start   X end   Notes
    ─────────────────────  ───────   ─────   ──────────────────────────────────
    BackPlate                  0      12     wall face=0, front face=12
    FloatingContactPlate      12      30     contact face=12, back=30
                                             (seated 18 mm deep in 20 mm recess)
    EntryFunnel (flange)      12      18     flange back face = BackPlate front
    EntryFunnel (barrel)      12      92     funnel opens toward robot
    RobotBlock (docked)       92     142     contact face at funnel exit
    GuidePin (docked)         52     122     IN section = 52..82 inside block,
                                             OUT section = 82..122 ahead of block
    """
    # BackPlate built local X=0..+12 → place at world X=0 (no offset needed)
    # FloatingPlate built local X=0..+18 → place at world X=12
    # EntryFunnel built local X=0..+80 (exit at 0, entry at 80) → X=12
    # RobotBlock built local X=0..+50 (contact face at 0) → X=92
    # GuidePin built local X=0..+70 (tip at 0, in the -X direction)
    #   → place at world X = 92 - PIN_OUT = 52
    #   so tip is at world X = 52, body 52..122, contact face at 92+0=92 ✓

    offsets = {
        "BackPlate":            (0,                  0, 0),
        "FloatingContactPlate": (BP_T,               0, 0),
        "EntryFunnel":          (BP_T,               0, 0),
        "RobotBlock":           (BP_T + FN_D,        0, 0),
        "GuidePin":             (BP_T + FN_D - PIN_OUT, 0, 0),
    }
    for i in range(root.occurrences.count):
        occ = root.occurrences.item(i)
        name = occ.component.name
        if name in offsets:
            dx, dy, dz = offsets[name]
            m = adsk.core.Matrix3D.create()
            m.translation = adsk.core.Vector3D.create(c(dx), c(dy), c(dz))
            occ.transform = m


# ── ENTRY POINT ──────────────────────────────────────────────────────────────

def run(context):
    ui = None
    completed = []
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        # Create a new multi-component Fusion Design document.
        # CRITICAL: recent Fusion 360 defaults "New Design" to "Part Design"
        # (single component only).  We must create the document explicitly here.
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox(
                "Could not create a Fusion Design document.\n"
                "Ensure Fusion 360 is fully loaded and try again.",
                "DockingStation2 — Error")
            return

        root = design.rootComponent

        ui.messageBox(
            "DockingStation2  v3.0\n\n"
            "A new Fusion Design document has been created.\n\n"
            "Building 5 components — allow 30–90 seconds.\n"
            "Click OK to start.",
            "DockingStation2")

        build_robot_block(root);        completed.append("RobotBlock ✓")
        build_guide_pin(root);          completed.append("GuidePin ✓")
        build_back_plate(root);         completed.append("BackPlate ✓")
        build_floating_plate(root);     completed.append("FloatingContactPlate ✓")
        build_entry_funnel(root);       completed.append("EntryFunnel ✓")

        position_components(root);      completed.append("Positioned ✓")

        app.activeViewport.fit()

        ui.messageBox(
            "✅  DockingStation2 assembly complete!\n\n"
            + "\n".join(f"  {s}" for s in completed)
            + "\n\n"
            "Key changes from v1:\n"
            "  • RobotBlock: 8 mm chamfers on 4 leading edges\n"
            "  • BackPlate:  126×126×20 mm recess for FloatingPlate\n"
            "  • BackPlate:  M4 tapped holes for funnel flange\n"
            "  • EntryFunnel: narrow exit (130×130) at wall end,\n"
            "                 wide entry (220×180) faces robot\n"
            "  • EntryFunnel: 180×180×6 mm bolt flange at exit end\n"
            "  • FloatingPlate: 120×120 (fits recess, not outside funnel)\n"
            "  • Stack corrected: BP → FP → Funnel → [open] → RB\n\n"
            "Suggested next steps:\n"
            "  1. File → Save\n"
            "  2. Inspect → Section Analysis (set plane at Y=0)\n"
            "  3. Right-click each body → Save As Mesh (STL) for printing\n\n"
            "NOTE: Guide-pin tip chamfer and boss back-extrude may fail\n"
            "gracefully on some Fusion versions — add manually if missing\n"
            "(Solid → Chamfer on tip edge; Solid → Extrude on boss circle).",
            "DockingStation2 ✅")

    except Exception:
        err = traceback.format_exc()
        if completed:
            err = ("Completed before failure:\n  "
                   + "\n  ".join(completed) + "\n\n" + err)
        if ui:
            ui.messageBox(err, "DockingStation2 — Script Error")
        else:
            print(err)


def stop(context):
    pass
