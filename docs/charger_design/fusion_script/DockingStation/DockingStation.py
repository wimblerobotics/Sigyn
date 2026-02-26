"""
Sigyn Docking Station — Fusion 360 Python Script  v2.0
=======================================================
Builds the full docking station assembly in whatever design is currently
open.  Create a new empty design first, then run this script.

HOW TO RUN
──────────
1.  In Fusion 360, create a new design  (File → New Design).
2.  Press  Shift + S  to open Scripts and Add-Ins.
3.  Under "My Scripts", click the green  +  button.
4.  In the file browser, navigate to the  DockingStation  *folder*
    (the folder that contains this file) and click  Open / Select.
5.  "DockingStation" will appear in the list.  Select it and click  Run.
6.  A progress dialog appears; click OK to start.  Allow 30–90 s.

COMPONENTS CREATED (all in the active design)
──────────────────────────────────────────────
  RobotBlock            100 × 100 × 50 mm  — robot-side printed block
  GuidePin               18 mm Ø × 70 mm  — aluminium alignment pin
  BackPlate             220 × 220 × 12 mm  — wall-mount plate
  FloatingContactPlate  160 × 160 × 18 mm  — spring-floating contact plate
  EntryFunnel           220×180 → 112×112, 80 mm deep — tapered entry guide

All dimensions are in mm.  Edit the PARAMETERS section below to change
the design; then re-run.
"""

import adsk.core
import adsk.fusion
import traceback

# ── PARAMETERS  (all mm) ────────────────────────────────────────────────────

# Robot-side block
B_W, B_H, B_D = 100.0, 100.0, 50.0

# Guide pin
PIN_OD   = 18.0   # outer diameter
PIN_BORE =  8.5   # through-bore for M8 retaining bolt
PIN_OUT  = 40.0   # protrusion from robot block face
PIN_IN   = 30.0   # inset inside block
PIN_CH   =  4.0   # tip chamfer depth (45°)

# Power contact pad pockets (robot side)
PAD_W   =  8.0    # narrow dimension
PAD_L   = 20.0    # long dimension
PAD_DEP = 10.0    # pocket depth
PAD_CLR =  0.5    # epoxy bedding clearance per side
PAD_Y   = 35.0    # ±Y from block centre
PAD_Z   = 32.0    # ±Z from block centre

# Sense pogo pin bores (robot side)
POGO_D   =  5.5   # through-bore Ø
POGO_CB  =  8.0   # counterbore Ø (retaining collar)
POGO_CBD =  5.0   # counterbore depth (from back face)
POGO_Y   = 18.0   # ±Y from block centre

# Robot block mount holes (M5)
MNT_D   =  5.3    # M5 clearance
MNT_OFF = 40.0    # ±Y and ±Z from centre

# Back plate (house side)
BP_W, BP_H, BP_T = 220.0, 220.0, 12.0
BP_BOSS_D   = 20.0  # centering spring boss Ø
BP_BOSS_H   = 15.0  # centering spring boss height
BP_POST_OFF = 55.0  # ±Y and ±Z for M6 guide posts
BP_SLOT_W   =  8.0  # wall-mount slot width
BP_SLOT_L   = 14.0  # wall-mount slot length
BP_SLOT_OFF = 80.0  # ±Y and ±Z for wall-mount slots

# Floating contact plate (house side)
FP_W, FP_H, FP_T = 160.0, 160.0, 18.0
FP_FLOAT_D  = 16.0  # oversized float hole Ø (±5 mm float)
FP_CB_D     = 26.0  # retention-washer counterbore Ø
FP_CB_DEP   =  8.0  # counterbore depth (from back face)
FP_SOCK_D   = 18.5  # guide socket bore Ø
FP_FUNNEL_D = 35.0  # funnel entry Ø
FP_FUNNEL_L =  8.0  # funnel depth
FP_SOCK_DEP = 30.0  # guide socket seating depth

# Spring contact slots (on floating plate)
SP_W  = PAD_W + 2.0   # 10 mm
SP_L  = PAD_L + 2.0   # 22 mm
SP_DEP = 5.0

# Sense pad recesses (on floating plate)
SNS_W   =  8.0
SNS_L   = 12.0
SNS_DEP =  3.0

# Entry funnel
FN_EW, FN_EH = 220.0, 180.0   # entry aperture
FN_XW, FN_XH = 112.0, 112.0   # exit aperture
FN_D         =  80.0            # funnel depth
FN_WALL      =   4.0            # shell wall thickness


# ── UNIT CONVERSION ─────────────────────────────────────────────────────────

def c(mm):
    """Convert mm to cm (Fusion 360 API works in cm)."""
    return mm / 10.0


# ── LOW-LEVEL HELPERS ───────────────────────────────────────────────────────

def new_comp(root, name):
    """Add a new sub-component to root, return the component."""
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    return comp


def sk_on(comp, plane):
    """Return a new sketch on the given construction plane or face."""
    return comp.sketches.add(plane)


def off_plane(comp, base_plane, dist_mm):
    """Return a construction plane offset dist_mm from base_plane."""
    planes = comp.constructionPlanes
    inp = planes.createInput()
    inp.setByOffset(base_plane, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return planes.add(inp)


def draw_rect(sk, cy_mm, cz_mm, w_mm, h_mm):
    """Draw a centred rectangle on a YZ-plane sketch (Y=horiz, Z=vert)."""
    lines = sk.sketchCurves.sketchLines
    p1 = adsk.core.Point3D.create(c(cy_mm - w_mm / 2), c(cz_mm - h_mm / 2), 0)
    p2 = adsk.core.Point3D.create(c(cy_mm + w_mm / 2), c(cz_mm + h_mm / 2), 0)
    lines.addTwoPointRectangle(p1, p2)


def draw_circ(sk, cy_mm, cz_mm, dia_mm):
    """Draw a circle on a YZ-plane sketch."""
    sk.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(c(cy_mm), c(cz_mm), 0),
        c(dia_mm / 2))


def extrude_new(comp, profile, dist_mm):
    """Extrude profile dist_mm, creating a new body."""
    feats = comp.features.extrudeFeatures
    inp = feats.createInput(profile,
                            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return feats.add(inp)


def extrude_join(comp, profile, dist_mm):
    """Extrude profile dist_mm, joining to the existing body."""
    feats = comp.features.extrudeFeatures
    inp = feats.createInput(profile,
                            adsk.fusion.FeatureOperations.JoinFeatureOperation)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return feats.add(inp)


def cut_depth(comp, profile_or_coll, dist_mm):
    """Extrude-cut to a specific depth (positive = away from sketch normal)."""
    feats = comp.features.extrudeFeatures
    inp = feats.createInput(profile_or_coll,
                            adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(c(dist_mm)))
    return feats.add(inp)


def cut_all(comp, profile_or_coll):
    """Extrude-cut through all (positive normal direction)."""
    feats = comp.features.extrudeFeatures
    inp = feats.createInput(profile_or_coll,
                            adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.setAllExtent(False)
    return feats.add(inp)


def first_prof(sk):
    """Return the first (and usually only) profile in a sketch."""
    if sk.profiles.count == 0:
        raise RuntimeError(f"Sketch '{sk.name}' has no profiles.")
    return sk.profiles.item(0)


def profs_by_rect(sk, w_mm, h_mm, tol=0.08):
    """
    Return an ObjectCollection of profiles whose bounding box matches
    w_mm × h_mm (either orientation).
    """
    coll = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        bb = p.boundingBox
        pw = (bb.maxPoint.x - bb.minPoint.x) * 10   # cm → mm
        ph = (bb.maxPoint.y - bb.minPoint.y) * 10
        if ((abs(pw - w_mm) < tol and abs(ph - h_mm) < tol) or
                (abs(pw - h_mm) < tol and abs(ph - w_mm) < tol)):
            coll.add(p)
    return coll


def profs_by_dia(sk, dia_mm, tol=0.08):
    """
    Return an ObjectCollection of profiles whose bounding box is
    approximately dia_mm × dia_mm (circular profiles).
    """
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
    Features (all sketched on the YZ plane = contact face):
      ① 100×100 body extruded 50 mm in +X
      ② 18 mm guide-pin bore, 30 mm deep
      ③ 8.5 mm M8 bolt through-bore
      ④ 4 × pad pockets  (9×21 mm, 10 mm deep)
      ⑤ 2 × sense pogo bores (5.5 mm, through-all)
      ⑥ 2 × pogo counterbores (8 mm, 5 mm from back)
      ⑦ 4 × M5 mount holes (5.3 mm, through-all)
    """
    comp = new_comp(root, "RobotBlock")
    yz = comp.yZConstructionPlane
    pw = PAD_W + 2 * PAD_CLR   # pad pocket width  = 9 mm
    pl = PAD_L + 2 * PAD_CLR   # pad pocket length = 21 mm

    # ① Outer body
    sk = sk_on(comp, yz)
    draw_rect(sk, 0, 0, B_W, B_H)
    extrude_new(comp, first_prof(sk), B_D)

    # ② Guide-pin mounting bore (18 mm Ø, 30 mm deep from contact face)
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, PIN_OD)
    cut_depth(comp, first_prof(sk), PIN_IN)

    # ③ M8 bolt clearance bore (8.5 mm, through-all)
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, PIN_BORE)
    cut_all(comp, first_prof(sk))

    # ④ Power pad pockets ×4 (9×21 mm, 10 mm deep)
    sk = sk_on(comp, yz)
    for (py, pz) in [(-PAD_Y, PAD_Z), (-PAD_Y, -PAD_Z),
                     ( PAD_Y, PAD_Z), ( PAD_Y, -PAD_Z)]:
        draw_rect(sk, py, pz, pw, pl)
    coll = profs_by_rect(sk, pw, pl)
    if coll.count > 0:
        cut_depth(comp, coll, PAD_DEP)

    # ⑤ Sense pogo through-bores ×2 (5.5 mm, through-all)
    sk = sk_on(comp, yz)
    draw_circ(sk, -POGO_Y, 0, POGO_D)
    draw_circ(sk,  POGO_Y, 0, POGO_D)
    coll = profs_by_dia(sk, POGO_D)
    if coll.count > 0:
        cut_all(comp, coll)

    # ⑥ Pogo counterbores ×2 (8 mm Ø, 5 mm deep from back face)
    #    Strategy: sketch on a plane at X = (B_D - POGO_CBD) = 45 mm,
    #    then cut 5 mm in +X direction → pocket from X=45 to X=50 (back).
    cb_plane = off_plane(comp, yz, B_D - POGO_CBD)
    sk = sk_on(comp, cb_plane)
    draw_circ(sk, -POGO_Y, 0, POGO_CB)
    draw_circ(sk,  POGO_Y, 0, POGO_CB)
    coll = profs_by_dia(sk, POGO_CB)
    if coll.count > 0:
        cut_depth(comp, coll, POGO_CBD)

    # ⑦ M5 mount holes ×4 (through-all)
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
    Aluminium guide pin: 18 mm Ø × 70 mm total, with 8.5 mm through-bore
    and a tip chamfer (approximated as a cut cone).
    The pin is a simple cylinder; the chamfer is done as a cone frustum cut.
    """
    comp = new_comp(root, "GuidePin")
    yz = comp.yZConstructionPlane
    total = PIN_OUT + PIN_IN   # 70 mm

    # ① Main cylinder
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, PIN_OD)
    extrude_new(comp, first_prof(sk), total)

    # ② M8 bolt through-bore
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, PIN_BORE)
    cut_all(comp, first_prof(sk))

    # ③ Tip chamfer — cone frustum cut at X=0 (the protruding tip end).
    #    A plane at X=0 has a full-diameter circle (18 mm).
    #    A plane at X=PIN_CH (4 mm) has a reduced circle = PIN_OD - 2*PIN_CH = 10 mm.
    #    Loft between them then cut → creates the 45° chamfer cone.
    #    We use two profiles: outer circle at X=0, reduced circle at X=4.
    sk_tip   = sk_on(comp, yz)                          # at X=0
    sk_inner = sk_on(comp, off_plane(comp, yz, PIN_CH)) # at X=4 mm

    inner_dia = PIN_OD - 2 * PIN_CH   # 10 mm
    draw_circ(sk_tip,   0, 0, PIN_OD)
    draw_circ(sk_inner, 0, 0, inner_dia)

    prof_tip   = first_prof(sk_tip)
    prof_inner = first_prof(sk_inner)

    lofts = comp.features.loftFeatures
    loft_inp = lofts.createInput(adsk.fusion.FeatureOperations.CutFeatureOperation)
    loft_inp.loftSections.add(prof_tip)
    loft_inp.loftSections.add(prof_inner)
    try:
        lofts.add(loft_inp)
    except Exception:
        pass   # chamfer is cosmetic; continue if loft fails

    return comp


def build_back_plate(root):
    """
    House-side back plate: 220×220×12 mm with:
      ① 220×220 body, 12 mm thick
      ② 20 mm Ø × 15 mm boss (centring spring hub) on front face
      ③ 4 × M6 guide post holes (5 mm Ø, through)
      ④ 4 × wall-mount slots (8×14 mm, through)
    """
    comp = new_comp(root, "BackPlate")
    yz = comp.yZConstructionPlane

    # ① Main body
    sk = sk_on(comp, yz)
    draw_rect(sk, 0, 0, BP_W, BP_H)
    extrude_new(comp, first_prof(sk), BP_T)

    # ② Centering boss on front face (offset plane at X=BP_T)
    front = off_plane(comp, yz, BP_T)
    sk = sk_on(comp, front)
    draw_circ(sk, 0, 0, BP_BOSS_D)
    extrude_join(comp, first_prof(sk), BP_BOSS_H)

    # ③ Guide post M6 holes ×4 (5 mm Ø, through-all from contact face)
    sk = sk_on(comp, yz)
    for (py, pz) in [(-BP_POST_OFF,  BP_POST_OFF), (-BP_POST_OFF, -BP_POST_OFF),
                     ( BP_POST_OFF,  BP_POST_OFF), ( BP_POST_OFF, -BP_POST_OFF)]:
        draw_circ(sk, py, pz, 5.0)
    coll = profs_by_dia(sk, 5.0)
    if coll.count > 0:
        cut_all(comp, coll)

    # ④ Wall-mount slots ×4 (8×14 mm, through-all)
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
    House-side floating contact plate: 160×160×18 mm with:
      ① 160×160 body, 18 mm thick
      ② 4 × 16 mm float holes (±5 mm play on M6 post)
      ③ 4 × 26 mm counterbores, 8 mm deep (from back, retention washers)
      ④ 18.5 mm guide socket bore, 30 mm deep
      ⑤ 35 mm funnel entry bore, 8 mm deep
      ⑥ 4 × spring contact slots (10×22 mm, 5 mm deep)
      ⑦ 2 × sense pad recesses (8×12 mm, 3 mm deep)
    """
    comp = new_comp(root, "FloatingContactPlate")
    yz = comp.yZConstructionPlane

    # ① Main body
    sk = sk_on(comp, yz)
    draw_rect(sk, 0, 0, FP_W, FP_H)
    extrude_new(comp, first_prof(sk), FP_T)

    # ② Float holes ×4 (16 mm Ø, through-all)
    sk = sk_on(comp, yz)
    for (py, pz) in [(-BP_POST_OFF,  BP_POST_OFF), (-BP_POST_OFF, -BP_POST_OFF),
                     ( BP_POST_OFF,  BP_POST_OFF), ( BP_POST_OFF, -BP_POST_OFF)]:
        draw_circ(sk, py, pz, FP_FLOAT_D)
    coll = profs_by_dia(sk, FP_FLOAT_D)
    if coll.count > 0:
        cut_all(comp, coll)

    # ③ Retention-washer counterbores ×4 (26 mm Ø, 8 mm from back face)
    #    Sketch at X = FP_T - FP_CB_DEP = 10 mm; cut 8 mm in +X → X=10 to X=18
    cb_plane = off_plane(comp, yz, FP_T - FP_CB_DEP)
    sk = sk_on(comp, cb_plane)
    for (py, pz) in [(-BP_POST_OFF,  BP_POST_OFF), (-BP_POST_OFF, -BP_POST_OFF),
                     ( BP_POST_OFF,  BP_POST_OFF), ( BP_POST_OFF, -BP_POST_OFF)]:
        draw_circ(sk, py, pz, FP_CB_D)
    coll = profs_by_dia(sk, FP_CB_D)
    if coll.count > 0:
        cut_depth(comp, coll, FP_CB_DEP)

    # ④ Guide socket bore (18.5 mm Ø, 30 mm deep from contact face)
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, FP_SOCK_D)
    cut_depth(comp, first_prof(sk), FP_SOCK_DEP)

    # ⑤ Funnel entry bore (35 mm Ø, 8 mm deep — creates chamfered entry)
    sk = sk_on(comp, yz)
    draw_circ(sk, 0, 0, FP_FUNNEL_D)
    cut_depth(comp, first_prof(sk), FP_FUNNEL_L)

    # ⑥ Spring contact slots ×4 (10×22 mm, 5 mm deep)
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
    Entry funnel: lofted shell from 220×180 (entry) to 112×112 (exit),
    80 mm deep, 4 mm wall thickness.
    """
    comp = new_comp(root, "EntryFunnel")
    yz = comp.yZConstructionPlane

    # ① Entry profile at X=0
    sk1 = sk_on(comp, yz)
    draw_rect(sk1, 0, 0, FN_EW, FN_EH)
    prof1 = first_prof(sk1)

    # ② Exit profile at X = FN_D
    exit_plane = off_plane(comp, yz, FN_D)
    sk2 = sk_on(comp, exit_plane)
    draw_rect(sk2, 0, 0, FN_XW, FN_XH)
    prof2 = first_prof(sk2)

    # ③ Loft
    lofts = comp.features.loftFeatures
    loft_inp = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    loft_inp.loftSections.add(prof1)
    loft_inp.loftSections.add(prof2)
    loft_feat = lofts.add(loft_inp)

    # ④ Shell: remove both flat end faces, keep FN_WALL thickness
    body = loft_feat.bodies.item(0)
    shell_faces = adsk.core.ObjectCollection.create()
    for face in body.faces:
        geom = face.geometry
        if geom.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
            normal = geom.normal
            if abs(abs(normal.x) - 1.0) < 0.01:   # flat face ⊥ to X axis
                shell_faces.add(face)

    if shell_faces.count >= 2:
        shell_inp = comp.features.shellFeatures.createInput(shell_faces, False)
        shell_inp.insideThickness = adsk.core.ValueInput.createByReal(c(FN_WALL))
        comp.features.shellFeatures.add(shell_inp)

    return comp


# ── ASSEMBLY POSITIONING ─────────────────────────────────────────────────────

def position_components(root):
    """
    Translate each occurrence so the docked assembly reads correctly in 3D.

    X axis = approach depth (robot comes from +X, contact face at X = 0).

    Component         | front face X | back face X
    ──────────────────|──────────────|────────────
    RobotBlock        |       0      |    +50
    GuidePin          |     -40      |    +30    (tip at X=-40)
    FloatingPlate     |       0      |    +18    (contact face flush with funnel exit)
    EntryFunnel       |      +18     |    +98    (funnel entry at X=+98)
    BackPlate         |      +98     |   +110
    """
    offsets = {
        "RobotBlock":           (    0, 0, 0),
        "GuidePin":             (-PIN_OUT, 0, 0),
        "FloatingContactPlate": ( FP_T,  0, 0),
        "EntryFunnel":          ( FP_T,  0, 0),
        "BackPlate":            ( FP_T + FN_D, 0, 0),
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

        # Create a new standard Fusion Design document.
        # This is critical: "Part Design" documents (the default in recent
        # Fusion versions when you do File → New Design) only support a single
        # component and will error on addNewComponent().  Creating the document
        # here with FusionDesignDocumentType guarantees a multi-component
        # assembly-capable design regardless of what was open before.
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox(
                "Could not create a new Fusion Design document.\n"
                "Please check that Fusion 360 is fully loaded and try again.",
                "Docking Station — Error")
            return

        root = design.rootComponent

        ui.messageBox(
            "A new Fusion Design document has been created.\n\n"
            "About to build the Sigyn Docking Station assembly (5 components).\n"
            "Allow 30–90 seconds — click OK to start.",
            "Docking Station")

        build_robot_block(root);        completed.append("RobotBlock ✓")
        build_guide_pin(root);          completed.append("GuidePin ✓")
        build_back_plate(root);         completed.append("BackPlate ✓")
        build_floating_plate(root);     completed.append("FloatingContactPlate ✓")
        build_entry_funnel(root);       completed.append("EntryFunnel ✓")

        position_components(root);      completed.append("positioned ✓")

        app.activeViewport.fit()

        ui.messageBox(
            "✅  Docking Station assembly complete!\n\n" +
            "\n".join(f"  {s}" for s in completed) +
            "\n\nSuggested next steps:\n"
            "  1. File → Save  (give the design a name)\n"
            "  2. Inspect → Section Analysis to see cross-sections\n"
            "  3. File → New Drawing → From Design for full drawings\n"
            "  4. Right-click each body → Save As Mesh (STL) for printing\n\n"
            "NOTE: the guide pin tip chamfer is a lofted cut — if it\n"
            "failed silently, add it manually with Solid → Chamfer\n"
            "(select the tip edge, 4 mm × 45°).",
            "Docking Station ✅")

    except Exception:
        err = traceback.format_exc()
        if completed:
            err = "Completed before failure:\n  " + "\n  ".join(completed) + "\n\n" + err
        if ui:
            ui.messageBox(err, "Docking Station — Script Error")
        else:
            print(err)


def stop(context):
    pass
