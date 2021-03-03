import math

import adsk.cam
import adsk.core
import adsk.fusion
import traceback


def sketch_splines(sketch, points):
    pts = adsk.core.ObjectCollection.create()
    for x, y in points:
        pt = adsk.core.Point3D.create(x, y, 0)
        pts.add(pt)
    curve = sketch.sketchCurves.sketchFittedSplines.add(pts)
    return curve


def pin_path_on_epi_cycloid(radians, ra, rb, rc, e=None, x_offset=0, y_offset=0):
    """

    :param radians:         angle
    :param ra:              radius of the base circle
    :param rb:              radius of the rolling circle
    :param rc:              radius of the pin circle
    :param e:               eccentricity, defaults to ra if not passed
    :param x_offset:
    :param y_offset:
    :return:
    """
    e = rb if e is None else e
    n = ra / rb                                 # number of lobes
    tt = radians                                # base circle total angle
    r = n * radians                             # rolling circle angle
    rt = r + radians                            # rolling circle total angle [ (n + 1) * radians ]
    p = abs((math.pi - r) / 2)                  # pin circle angle at rolling point

    if e == rb:
        pe = p
    else:
        l1 = 2 * rb * math.cos(p)               # line FB
        l2 = (rb - e) * math.cos(p)             # line GB
        l3 = l1 - l2                            # line FG
        a1 = (math.pi / 2) - p                  # angle GAB - this can be simplified to r / 2
        l4 = (rb - e) * math.sin(p)             # line AG
        # angle FAG
        if l4 == 0:
            a2 = math.pi / 2
        else:
            a2 = math.atan(l3 / l4)
        pe = math.pi - (a1 + a2)                # pin circle angle at eccentric point [ angle EAF ]

    # pin circle total angle at eccentric point
    if r > math.pi:
        pet = rt - pe
    else:
        pet = rt + pe

    x = x_offset + ((ra + rb) * math.cos(tt)) - (e * math.cos(rt)) + (rc * math.cos(pet))
    y = y_offset + ((ra + rb) * math.sin(tt)) - (e * math.sin(rt)) + (rc * math.sin(pet))

    return x, y


def epi_cycloid_c(radians, ra, rb, e=None, x_offset=0, y_offset=0):
    """

    :param radians:         angle
    :param ra:              radius of the base circle
    :param rb:              radius of the rolling circle
    :param e:               eccentricity, defaults to ra if not passed
    :param x_offset:
    :param y_offset:
    :return:
    """

    e = rb if e is None else e
    n = ra / rb             # number of lobes
    t = radians             # base circle angle
    r = (n + 1) * radians   # rolling circle angle NOTE: n + 1 !!

    x = x_offset + ((ra + rb) * math.cos(t)) - (e * math.cos(r))
    y = y_offset + ((ra + rb) * math.sin(t)) - (e * math.sin(r))

    return x, y


def gen_pts_from_rads(func, radians, *args):
    return [func(i, *args) for i in radians]


def simp_gen_rads(slice_start_radians, slice_end_radians, nbr_of_steps):
    total_radians = slice_end_radians - slice_start_radians
    radians_per_step = total_radians / (nbr_of_steps - 1)
    values = [slice_start_radians + (step_nbr * radians_per_step) for step_nbr in range(nbr_of_steps)]
    return values


def contracted_cycloid(extrudes, sketches, xy_plane, z, D, d, sigma, pr, e, NBR, nbr, tol=0.025, spln_pts=33):
    extrude_distance = adsk.core.ValueInput.createByReal(z)

    offset = D - d
    offset = 0
    sketch1 = sketches.add(xy_plane)
    sketch1.name = f'contracted-cycloid'

    rads = simp_gen_rads(0, 2 * math.pi / nbr, spln_pts)

    # sketch the points that make up the contracted cycloid shape
    pts = gen_pts_from_rads(epi_cycloid_c, rads, d, sigma, e, offset)
    curve = sketch_splines(sketch1, pts)

    start_point = curve.startSketchPoint
    end_point = curve.endSketchPoint
    lines = sketch1.sketchCurves.sketchLines
    line = lines.addByTwoPoints(adsk.core.Point3D.create(offset, 0, 0), start_point)
    lines.addByTwoPoints(line.startSketchPoint, end_point)
    circles = sketch1.sketchCurves.sketchCircles
    # circle = circles.addByCenterRadius(start_point, pr + (tol / 2))
    # circles.addByCenterRadius(end_point, pr + (tol / 2))

    extrudes.addSimple(sketch1.profiles.item(0), extrude_distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

    for i, (x, y) in enumerate(pts, 1):
        # ui.messageBox(f'after point {i}')
        sk_obj = sketches.add(xy_plane)
        sc_obj = sk_obj.sketchCurves.sketchCircles
        pt_obj = adsk.core.Point3D.create(x, y, 0)
        sc_obj.addByCenterRadius(pt_obj, pr)
        extrudes.addSimple(sk_obj.profiles.item(0), extrude_distance,
                           adsk.fusion.FeatureOperations.CutFeatureOperation)


    # sketch the points that make up the path the pin circle follows
    sketch2 = sketches.add(xy_plane)
    sketch2.name = f'pin-edge-path'

    # pts = gen_pts_from_rads(pin_path_on_epi_cycloid, rads[1:-1], d, sigma, pr, e, offset)
    pts = gen_pts_from_rads(pin_path_on_epi_cycloid, rads, d, sigma, pr, e, offset)
    sketch_splines(sketch2, pts)

    # start_point = curve.startSketchPoint
    # end_point = curve.endSketchPoint
    # lines = sketch2.sketchCurves.sketchLines
    # line = lines.addByTwoPoints(adsk.core.Point3D.create(offset, 0, 0), start_point)
    # lines.addByTwoPoints(line.startSketchPoint, end_point)

ui = None
def run(context):
    global ui
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        design = app.activeProduct

        # Get the root component of the active design.
        root_comp = design.rootComponent
        extrudes = root_comp.features.extrudeFeatures

        # Create a new sketch on the xy plane.
        sketches = root_comp.sketches
        xy_plane = root_comp.xYConstructionPlane

        contracted_cycloid(extrudes, sketches, xy_plane, 1.0, 11.0 / 2, 10.0 / 2, 1.0 / 2, 1.5 / 2, 0.47, 11, 10, 0.0, 129)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
