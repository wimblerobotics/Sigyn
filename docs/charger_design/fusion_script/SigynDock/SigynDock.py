"""
SigynDock  v1.0
===============
Sigyn docking station assembly builder.
Reports progress after every major step so failures are easy to pinpoint.

Assembly (X = docking axis, robot reverses from +X toward wall at X=0):
  X =  0..12   BackPlate        (wall face=0, front face=12)
  X = 12..30   FloatingPlate    (contact face=12, seated in recess)
  X = 12..92   EntryFunnel      (flange at 12, wide mouth at 92)
  X = 92..142  RobotBlock       (contact face=92)
  GuidePin tip at X=52, body 52..122

HOW TO RUN
  Shift+S → green + → navigate to SigynDock folder → Open → Run
  (script creates its own document; no need to open one first)
"""

import adsk.core
import adsk.fusion
import traceback

# ── PARAMETERS (mm) ─────────────────────────────────────────────────────────

RB_W, RB_H, RB_D = 100.0, 100.0, 50.0
RB_CH             =   8.0   # leading-edge chamfer

PIN_OD   = 18.0
PIN_BORE =  8.5
PIN_OUT  = 40.0
PIN_IN   = 30.0
PIN_CH   =  4.0

PAD_W, PAD_L, PAD_DEP = 8.0, 20.0, 10.0
PAD_CLR                = 0.5
PAD_Y, PAD_Z           = 35.0, 32.0

POGO_D, POGO_CB, POGO_CBD = 5.5, 8.0, 5.0
POGO_Y                     = 18.0

MNT_D, MNT_OFF = 5.3, 40.0

BP_W, BP_H, BP_T     = 220.0, 220.0, 12.0
BP_RCS_W, BP_RCS_H   = 126.0, 126.0
BP_RCS_D             =  20.0
BP_BOSS_D, BP_BOSS_H =  20.0, 15.0
BP_POST_OFF          =  50.0
BP_POST_D            =   6.5
BP_FN_BOLT_OFF       =  58.0
BP_FN_BOLT_D         =   3.3
BP_SLOT_W, BP_SLOT_L =   8.0, 14.0
BP_SLOT_OFF          =  85.0

FP_W, FP_H, FP_T       = 120.0, 120.0, 18.0
FP_FLOAT_D             =  10.0
FP_CB_D, FP_CB_DEP     =  16.0,  6.0
FP_SOCK_D, FP_SOCK_DEP =  18.5, 16.0
FP_FUNNEL_D, FP_FUNNEL_L = 30.0, 5.0

SP_W, SP_L, SP_DEP   = 10.0, 22.0, 5.0
SNS_W, SNS_L, SNS_DEP =  8.0, 12.0, 3.0

FN_XW, FN_XH = 130.0, 130.0
FN_EW, FN_EH = 220.0, 180.0
FN_D, FN_WALL =  80.0,   4.0
FN_FL_W, FN_FL_H, FN_FL_T = 180.0, 180.0, 6.0
FN_FL_BOLT_D               =   4.5
FN_FL_BOLT_OFF             =  BP_FN_BOLT_OFF


# ── UNIT ────────────────────────────────────────────────────────────────────
def cm(mm): return mm / 10.0


# ── GEOMETRY HELPERS ────────────────────────────────────────────────────────

def new_occ(root):
    """Add a blank new-component occurrence to root and return it."""
    return root.occurrences.addNewComponent(adsk.core.Matrix3D.create())


def sk_on(comp, plane):
    return comp.sketches.add(plane)


def offset_plane(comp, base, dist_mm):
    pi = comp.constructionPlanes.createInput()
    pi.setByOffset(base, adsk.core.ValueInput.createByReal(cm(dist_mm)))
    return comp.constructionPlanes.add(pi)


def rect(sk, cy, cz, w, h):
    ln = sk.sketchCurves.sketchLines
    ln.addTwoPointRectangle(
        adsk.core.Point3D.create(cm(cy - w/2), cm(cz - h/2), 0),
        adsk.core.Point3D.create(cm(cy + w/2), cm(cz + h/2), 0))


def circ(sk, cy, cz, dia):
    sk.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(cm(cy), cm(cz), 0), cm(dia / 2))


def first_prof(sk):
    if sk.profiles.count == 0:
        raise RuntimeError(f"No profiles in sketch '{sk.name}'.")
    return sk.profiles.item(0)


def profs_matching_rect(sk, w, h, tol=0.15):
    out = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        p  = sk.profiles.item(i)
        bb = p.boundingBox
        pw = (bb.maxPoint.x - bb.minPoint.x) * 10.0
        ph = (bb.maxPoint.y - bb.minPoint.y) * 10.0
        if ((abs(pw-w) < tol and abs(ph-h) < tol) or
                (abs(pw-h) < tol and abs(ph-w) < tol)):
            out.add(p)
    return out


def profs_matching_circ(sk, dia, tol=0.15):
    out = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        p  = sk.profiles.item(i)
        bb = p.boundingBox
        pw = (bb.maxPoint.x - bb.minPoint.x) * 10.0
        ph = (bb.maxPoint.y - bb.minPoint.y) * 10.0
        if abs(pw - dia) < tol and abs(ph - dia) < tol:
            out.add(p)
    return out


def extrude(comp, prof, dist_mm, op):
    fi = comp.features.extrudeFeatures.createInput(prof, op)
    fi.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(dist_mm)))
    return comp.features.extrudeFeatures.add(fi)


def extrude_flip(comp, prof, dist_mm, op):
    fi = comp.features.extrudeFeatures.createInput(prof, op)
    fi.setDistanceExtent(True, adsk.core.ValueInput.createByReal(cm(dist_mm)))
    return comp.features.extrudeFeatures.add(fi)


def extrude_all(comp, prof, op):
    fi = comp.features.extrudeFeatures.createInput(prof, op)
    fi.setAllExtent(False)
    return comp.features.extrudeFeatures.add(fi)


OP_NEW  = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
OP_JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
OP_CUT  = adsk.fusion.FeatureOperations.CutFeatureOperation


# ── COMPONENT BUILDERS ──────────────────────────────────────────────────────

def make_robot_block(root, log):
    occ  = new_occ(root)
    comp = occ.component
    comp.name = "RobotBlock"
    yz   = comp.yZConstructionPlane
    pw   = PAD_W + 2*PAD_CLR
    pl   = PAD_L + 2*PAD_CLR

    sk = sk_on(comp, yz); rect(sk, 0, 0, RB_W, RB_H)
    bf = extrude(comp, first_prof(sk), RB_D, OP_NEW)
    log.append("  RobotBlock: body OK")

    # Chamfer 4 long edges at contact face (X=0)
    body   = bf.bodies.item(0)
    cedges = adsk.core.ObjectCollection.create()
    for e in body.edges:
        p1, p2 = e.startVertex.geometry, e.endVertex.geometry
        if abs(p1.x) < 0.001 and abs(p2.x) < 0.001:
            dx, dy, dz = p2.x-p1.x, p2.y-p1.y, p2.z-p1.z
            if (dx*dx+dy*dy+dz*dz)**0.5 > cm(RB_W*0.8):
                cedges.add(e)
    if cedges.count > 0:
        try:
            ci = comp.features.chamferFeatures.createInput(cedges, True)
            ci.setToEqualDistance(adsk.core.ValueInput.createByReal(cm(RB_CH)))
            comp.features.chamferFeatures.add(ci)
            log.append(f"  RobotBlock: {cedges.count} chamfers OK")
        except Exception as ex:
            log.append(f"  RobotBlock: chamfer skipped ({ex})")

    sk = sk_on(comp, yz); circ(sk, 0, 0, PIN_OD)
    extrude(comp, first_prof(sk), PIN_IN, OP_CUT)
    log.append("  RobotBlock: pin socket OK")

    sk = sk_on(comp, yz); circ(sk, 0, 0, PIN_BORE)
    extrude_all(comp, first_prof(sk), OP_CUT)

    sk = sk_on(comp, yz)
    for py, pz in [(-PAD_Y,PAD_Z),(-PAD_Y,-PAD_Z),(PAD_Y,PAD_Z),(PAD_Y,-PAD_Z)]:
        rect(sk, py, pz, pw, pl)
    col = profs_matching_rect(sk, pw, pl)
    if col.count > 0: extrude(comp, col, PAD_DEP, OP_CUT)
    log.append(f"  RobotBlock: {col.count} pad pockets OK")

    sk = sk_on(comp, yz)
    circ(sk, -POGO_Y, 0, POGO_D); circ(sk, POGO_Y, 0, POGO_D)
    col = profs_matching_circ(sk, POGO_D)
    if col.count > 0: extrude_all(comp, col, OP_CUT)

    cb = offset_plane(comp, yz, RB_D - POGO_CBD)
    sk = sk_on(comp, cb)
    circ(sk, -POGO_Y, 0, POGO_CB); circ(sk, POGO_Y, 0, POGO_CB)
    col = profs_matching_circ(sk, POGO_CB)
    if col.count > 0: extrude(comp, col, POGO_CBD, OP_CUT)

    sk = sk_on(comp, yz)
    for my, mz in [(-MNT_OFF,MNT_OFF),(-MNT_OFF,-MNT_OFF),(MNT_OFF,MNT_OFF),(MNT_OFF,-MNT_OFF)]:
        circ(sk, my, mz, MNT_D)
    col = profs_matching_circ(sk, MNT_D)
    if col.count > 0: extrude_all(comp, col, OP_CUT)
    log.append("  RobotBlock: all holes OK")

    return comp


def make_guide_pin(root, log):
    occ  = new_occ(root)
    comp = occ.component
    comp.name = "GuidePin"
    yz   = comp.yZConstructionPlane

    sk = sk_on(comp, yz); circ(sk, 0, 0, PIN_OD)
    extrude(comp, first_prof(sk), PIN_OUT + PIN_IN, OP_NEW)

    sk = sk_on(comp, yz); circ(sk, 0, 0, PIN_BORE)
    extrude_all(comp, first_prof(sk), OP_CUT)
    log.append("  GuidePin: cylinder + bore OK")

    sk1 = sk_on(comp, yz)
    sk2 = sk_on(comp, offset_plane(comp, yz, PIN_CH))
    circ(sk1, 0, 0, PIN_OD); circ(sk2, 0, 0, PIN_OD - 2*PIN_CH)
    li = comp.features.loftFeatures.createInput(OP_CUT)
    li.loftSections.add(first_prof(sk1)); li.loftSections.add(first_prof(sk2))
    try:
        comp.features.loftFeatures.add(li)
        log.append("  GuidePin: tip chamfer OK")
    except Exception as ex:
        log.append(f"  GuidePin: tip chamfer skipped ({ex})")

    return comp


def make_back_plate(root, log):
    occ  = new_occ(root)
    comp = occ.component
    comp.name = "BackPlate"
    yz   = comp.yZConstructionPlane

    sk = sk_on(comp, yz); rect(sk, 0, 0, BP_W, BP_H)
    extrude(comp, first_prof(sk), BP_T, OP_NEW)
    log.append("  BackPlate: body OK")

    front = offset_plane(comp, yz, BP_T)
    sk = sk_on(comp, front); rect(sk, 0, 0, BP_RCS_W, BP_RCS_H)
    col = profs_matching_rect(sk, BP_RCS_W, BP_RCS_H)
    if col.count > 0: extrude(comp, col, BP_RCS_D, OP_CUT)
    log.append("  BackPlate: FloatingPlate recess OK")

    sk = sk_on(comp, yz); circ(sk, 0, 0, BP_BOSS_D)
    try:
        extrude_flip(comp, first_prof(sk), BP_BOSS_H, OP_JOIN)
        log.append("  BackPlate: wall boss OK")
    except Exception as ex:
        log.append(f"  BackPlate: wall boss skipped ({ex})")

    sk = sk_on(comp, front)
    for py, pz in [(-BP_POST_OFF,BP_POST_OFF),(-BP_POST_OFF,-BP_POST_OFF),
                   ( BP_POST_OFF,BP_POST_OFF),( BP_POST_OFF,-BP_POST_OFF)]:
        circ(sk, py, pz, BP_POST_D)
    col = profs_matching_circ(sk, BP_POST_D)
    if col.count > 0: extrude_all(comp, col, OP_CUT)
    log.append("  BackPlate: post holes OK")

    sk = sk_on(comp, front)
    for py, pz in [(-BP_FN_BOLT_OFF,BP_FN_BOLT_OFF),(-BP_FN_BOLT_OFF,-BP_FN_BOLT_OFF),
                   ( BP_FN_BOLT_OFF,BP_FN_BOLT_OFF),( BP_FN_BOLT_OFF,-BP_FN_BOLT_OFF)]:
        circ(sk, py, pz, BP_FN_BOLT_D)
    col = profs_matching_circ(sk, BP_FN_BOLT_D)
    if col.count > 0: extrude_all(comp, col, OP_CUT)
    log.append("  BackPlate: funnel bolt holes OK")

    sk = sk_on(comp, yz)
    for sy, sz in [(-BP_SLOT_OFF,BP_SLOT_OFF),(-BP_SLOT_OFF,-BP_SLOT_OFF),
                   ( BP_SLOT_OFF,BP_SLOT_OFF),( BP_SLOT_OFF,-BP_SLOT_OFF)]:
        rect(sk, sy, sz, BP_SLOT_W, BP_SLOT_L)
    col = profs_matching_rect(sk, BP_SLOT_W, BP_SLOT_L)
    if col.count > 0: extrude_all(comp, col, OP_CUT)
    log.append("  BackPlate: wall slots OK")

    return comp


def make_floating_plate(root, log):
    occ  = new_occ(root)
    comp = occ.component
    comp.name = "FloatingContactPlate"
    yz   = comp.yZConstructionPlane

    sk = sk_on(comp, yz); rect(sk, 0, 0, FP_W, FP_H)
    extrude(comp, first_prof(sk), FP_T, OP_NEW)
    log.append("  FloatingPlate: body OK")

    sk = sk_on(comp, yz)
    for py, pz in [(-BP_POST_OFF,BP_POST_OFF),(-BP_POST_OFF,-BP_POST_OFF),
                   ( BP_POST_OFF,BP_POST_OFF),( BP_POST_OFF,-BP_POST_OFF)]:
        circ(sk, py, pz, FP_FLOAT_D)
    col = profs_matching_circ(sk, FP_FLOAT_D)
    if col.count > 0: extrude_all(comp, col, OP_CUT)
    log.append("  FloatingPlate: float holes OK")

    back = offset_plane(comp, yz, FP_T - FP_CB_DEP)
    sk = sk_on(comp, back)
    for py, pz in [(-BP_POST_OFF,BP_POST_OFF),(-BP_POST_OFF,-BP_POST_OFF),
                   ( BP_POST_OFF,BP_POST_OFF),( BP_POST_OFF,-BP_POST_OFF)]:
        circ(sk, py, pz, FP_CB_D)
    col = profs_matching_circ(sk, FP_CB_D)
    if col.count > 0: extrude(comp, col, FP_CB_DEP, OP_CUT)
    log.append("  FloatingPlate: counterbores OK")

    sk = sk_on(comp, yz); circ(sk, 0, 0, FP_SOCK_D)
    extrude(comp, first_prof(sk), FP_SOCK_DEP, OP_CUT)

    sk = sk_on(comp, yz); circ(sk, 0, 0, FP_FUNNEL_D)
    extrude(comp, first_prof(sk), FP_FUNNEL_L, OP_CUT)
    log.append("  FloatingPlate: guide socket + lead-in OK")

    sk = sk_on(comp, yz)
    for py, pz in [(-PAD_Y,PAD_Z),(-PAD_Y,-PAD_Z),(PAD_Y,PAD_Z),(PAD_Y,-PAD_Z)]:
        rect(sk, py, pz, SP_W, SP_L)
    col = profs_matching_rect(sk, SP_W, SP_L)
    if col.count > 0: extrude(comp, col, SP_DEP, OP_CUT)
    log.append("  FloatingPlate: spring slots OK")

    sk = sk_on(comp, yz)
    for py, pz in [(-POGO_Y, 0),(POGO_Y, 0)]:
        rect(sk, py, pz, SNS_W, SNS_L)
    col = profs_matching_rect(sk, SNS_W, SNS_L)
    if col.count > 0: extrude(comp, col, SNS_DEP, OP_CUT)
    log.append("  FloatingPlate: sense recesses OK")

    return comp


def make_entry_funnel(root, log):
    occ  = new_occ(root)
    comp = occ.component
    comp.name = "EntryFunnel"
    yz   = comp.yZConstructionPlane

    # Loft: exit (narrow) at X=0, entry (wide) at X=FN_D
    sk_x = sk_on(comp, yz);                       rect(sk_x, 0, 0, FN_XW, FN_XH)
    sk_e = sk_on(comp, offset_plane(comp,yz,FN_D)); rect(sk_e, 0, 0, FN_EW, FN_EH)
    li = comp.features.loftFeatures.createInput(OP_NEW)
    li.loftSections.add(first_prof(sk_x))
    li.loftSections.add(first_prof(sk_e))
    lf = comp.features.loftFeatures.add(li)
    log.append("  EntryFunnel: loft OK")

    body   = lf.bodies.item(0)
    sfaces = adsk.core.ObjectCollection.create()
    for face in body.faces:
        g = face.geometry
        if (g.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType and
                abs(abs(g.normal.x) - 1.0) < 0.01):
            sfaces.add(face)
    if sfaces.count >= 2:
        si = comp.features.shellFeatures.createInput(sfaces, False)
        si.insideThickness = adsk.core.ValueInput.createByReal(cm(FN_WALL))
        comp.features.shellFeatures.add(si)
        log.append("  EntryFunnel: shell OK")
    else:
        log.append(f"  EntryFunnel: shell skipped (found {sfaces.count} flat faces)")

    # Bolt flange at exit end (X=0): pick largest profile matching FL_W×FL_H
    sk_fl = sk_on(comp, yz); rect(sk_fl, 0, 0, FN_FL_W, FN_FL_H)
    best = None; best_area = 0.0
    for i in range(sk_fl.profiles.count):
        p  = sk_fl.profiles.item(i)
        bb = p.boundingBox
        pw = (bb.maxPoint.x - bb.minPoint.x) * 10.0
        ph = (bb.maxPoint.y - bb.minPoint.y) * 10.0
        area = pw * ph
        if abs(pw - FN_FL_W) < 1.0 and abs(ph - FN_FL_H) < 1.0 and area > best_area:
            best = p; best_area = area
    if best is not None:
        try:
            extrude(comp, best, FN_FL_T, OP_JOIN)
            log.append("  EntryFunnel: flange OK")
        except Exception as ex:
            log.append(f"  EntryFunnel: flange skipped ({ex})")
    else:
        log.append(f"  EntryFunnel: flange profile not found "
                   f"(sketch has {sk_fl.profiles.count} profiles)")

    sk_bolt = sk_on(comp, yz)
    for py, pz in [(-FN_FL_BOLT_OFF,FN_FL_BOLT_OFF),(-FN_FL_BOLT_OFF,-FN_FL_BOLT_OFF),
                   ( FN_FL_BOLT_OFF,FN_FL_BOLT_OFF),( FN_FL_BOLT_OFF,-FN_FL_BOLT_OFF)]:
        circ(sk_bolt, py, pz, FN_FL_BOLT_D)
    col = profs_matching_circ(sk_bolt, FN_FL_BOLT_D)
    if col.count > 0:
        extrude(comp, col, FN_FL_T, OP_CUT)
    log.append("  EntryFunnel: bolt holes OK")

    return comp


def position_all(root, log):
    offsets = {
        "BackPlate":            0.0,
        "FloatingContactPlate": BP_T,
        "EntryFunnel":          BP_T,
        "RobotBlock":           BP_T + FN_D,
        "GuidePin":             BP_T + FN_D - PIN_OUT,
    }
    moved = []
    for i in range(root.occurrences.count):
        occ  = root.occurrences.item(i)
        name = occ.component.name
        if name in offsets:
            m = adsk.core.Matrix3D.create()
            m.translation = adsk.core.Vector3D.create(cm(offsets[name]), 0, 0)
            occ.transform = m
            moved.append(name)
    log.append(f"  Positioned: {moved}")


# ── ENTRY POINT ──────────────────────────────────────────────────────────────

def run(context):
    ui  = None
    log = []
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        # Create a fresh Fusion Design document
        doc    = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(doc.products.item(0))
        if not design:
            design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Could not obtain a Fusion Design.", "SigynDock")
            return

        # Force parametric (multi-component) mode
        try:
            design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        except Exception:
            pass

        root = design.rootComponent
        log.append(f"Document OK — designType={design.designType}  "
                   f"occurrences={root.occurrences.count}")

        ui.messageBox("\n".join(log) +
                      "\n\nClick OK to build RobotBlock.", "SigynDock")

        # ── Build each component, showing progress dialog after each ─────────

        log.append("Building RobotBlock...")
        make_robot_block(root, log)
        ui.messageBox("\n".join(log) +
                      "\n\nClick OK to build GuidePin.", "SigynDock")

        log.append("Building GuidePin...")
        make_guide_pin(root, log)
        ui.messageBox("\n".join(log) +
                      "\n\nClick OK to build BackPlate.", "SigynDock")

        log.append("Building BackPlate...")
        make_back_plate(root, log)
        ui.messageBox("\n".join(log) +
                      "\n\nClick OK to build FloatingContactPlate.", "SigynDock")

        log.append("Building FloatingContactPlate...")
        make_floating_plate(root, log)
        ui.messageBox("\n".join(log) +
                      "\n\nClick OK to build EntryFunnel.", "SigynDock")

        log.append("Building EntryFunnel...")
        make_entry_funnel(root, log)
        ui.messageBox("\n".join(log) +
                      "\n\nClick OK to position all components.", "SigynDock")

        log.append("Positioning...")
        position_all(root, log)
        app.activeViewport.fit()

        ui.messageBox("✅  SigynDock complete!\n\n" + "\n".join(log), "SigynDock ✅")

    except Exception:
        err = traceback.format_exc()
        msg = "ERROR\n\nLog so far:\n" + "\n".join(log) + "\n\n" + err
        if ui:
            ui.messageBox(msg, "SigynDock — Error")
        else:
            print(msg)


def stop(context):
    pass
