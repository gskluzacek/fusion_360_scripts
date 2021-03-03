import adsk.core, adsk.fusion, adsk.cam, traceback


import math
import logging
from datetime import datetime

dts = datetime.now().strftime('%Y%m%d_%H%M%S')

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
ch = logging.FileHandler(f'/Users/gskluzacek/f360_test_log_{dts}.txt')
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s- %(levelname)s - %(message)s')
ch.setFormatter(formatter)
logger.addHandler(ch)

logger.info('Logger setup complete')


def sketch_splines(sketch, points):
    pts = adsk.core.ObjectCollection.create()
    for x, y in points:
        pt = adsk.core.Point3D.create(x, y, 0)
        pts.add(pt)
    curve = sketch.sketchCurves.sketchFittedSplines.add(pts)
    return curve


def epi_cycloid_c(radians, ra, rb, e=None, x_offset=0, y_offset=0):
    """

    :param radians:         angle
    :param ra:              radius of the base circle
    :param rb:              radius of the rolling circle
    :param e:               ecentricity, defaults to ra if not passed
    :param x_offset:
    :param y_offset:
    :return:
    """

    e = rb if e is None else e
    n = ra / rb  # number of lobes
    t = radians  # base circle angle
    r = (n + 1) * radians  # rolling circle angle

    x = x_offset + ((ra + rb) * math.cos(t)) - (e * math.cos(r))
    y = y_offset + ((ra + rb) * math.sin(t)) - (e * math.sin(r))

    return x, y


def gen_pts_from_rads(func, radians, *args):
    return [func(i, *args) for i in radians]


def xform_for_tolerance(points_list, x0, y0, t):
    xfrm_points_list = []
    for points in points_list:
        xfrm_points = [xform_point_for_tol(x0, y0, x1, y1, t) for x1, y1 in points]
        xfrm_points_list.append(xfrm_points)
    return xfrm_points_list


def xform_point_for_tol(x0, y0, x1, y1, t):
    a1 = x1 - x0        # calc side a1 from x coordinates
    b1 = y1 - y0        # calc side b1 from y coordinates
    c1 = math.sqrt(math.pow(a1, 2) + math.pow(b1, 2))   # calc c1 -- a^2 + b^2 = c^2
    angle = math.atan(b1 / a1)      # calc common angle -- tan(angle) = opposite / adjacent
    c2 = c1 - t                     # calc c2 -- length of c1 less the tolerance t
    a2 = math.cos(angle) * c2       # calc a2 -- cos(angle) = adjacent / hypotenuse
    b2 = math.sin(angle) * c2       # calc b2 -- sin(angle) = opposite / hypotenuse
    x2 = a2 + x0        # calc x2 -- a2 = x2 + x0
    y2 = b2 + y0        # calc y2 -- b2 = y2 + y0
    return x2, y2


def gen_radians(nbr_of_slices, nbr_of_steps, every_nth_slice=1, slice_phase=1, slices=None, slice_list=None):
    radians_per_slice = (2 * math.pi) / nbr_of_slices
    radians_per_step = radians_per_slice / (nbr_of_steps - 1)

    if slice_list:
        slice_range = slice_list
    elif slices:
        whole_range = range(nbr_of_slices)
        slice_range = []
        for start, stop in slices:
            slice_range.extend(whole_range[start:stop])
    else:
        slice_range = range(slice_phase - 1, nbr_of_slices, every_nth_slice)

    list_of_values = []
    for slice_nbr in slice_range:
        slice_start_radians = slice_nbr * radians_per_slice
        values = [slice_start_radians + (step_nbr * radians_per_step) for step_nbr in range(nbr_of_steps)]
        list_of_values.append(values)
    return list_of_values


def simp_gen_rads(slice_start_radians, slice_end_radians, nbr_of_steps):
    total_radians = slice_end_radians - slice_start_radians
    radians_per_step = total_radians / (nbr_of_steps - 1)
    values = [slice_start_radians + (step_nbr * radians_per_step) for step_nbr in range(nbr_of_steps)]
    return values


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

    logger.info('tt: %s degrees', round(math.degrees(tt), 5))
    logger.info('r:  %s degrees', round(math.degrees(r), 5))
    logger.info('rt: %s degrees', round(math.degrees(rt), 5))
    logger.info('p:  %s degrees', round(math.degrees(p), 5))

    if e == rb:
        pe = p
        l1 = l2 = l3 = a1 = a2 = l4 = 0
    else:
        l1 = 2 * rb * math.cos(p)                   # line FB
        l2 = (rb - e) * math.cos(p)                 # line GB
        l3 = l1 - l2                                # line FG
        # angle GAB - this is equal to [ angle r / 2 ] because line AG is parallel to line EH
        a1 = (math.pi / 2) - p                  # this can be simplified to r / 2
        l4 = (rb - e) * math.sin(p)                 # line AG
        # angle FAG
        if l4 == 0:
            a2 = math.pi / 2
        else:
            a2 = math.atan(l3 / l4)
        pe = math.pi - (a1 + a2)                    # pin circle angle at eccentric point [ angle EAF ]
    # pin circle total angle at eccentric point
    if r > math.pi:
        logger.info('r > 180: %s degrees', round(math.degrees(radians), 5))
        pet = rt - pe
    else:
        pet = rt + pe

    logger.info('l1: %s mm', round(l1 * 10, 5))
    logger.info('l2: %s mm', round(l2 * 10, 5))
    logger.info('l3: %s mm', round(l3 * 10, 5))
    logger.info('a1: %s degrees', round(math.degrees(a1), 5))
    logger.info('l4: %s mm', round(l4 * 10, 5))
    logger.info('a2: %s degrees', round(math.degrees(a2), 5))
    logger.info('pe: %s degrees', round(math.degrees(pe), 5))
    logger.info('pet: %s degrees', round(math.degrees(pet), 5))

    x1 = (ra + rb) * math.cos(tt)
    x2 = -e * math.cos(rt)
    x3 = rc * math.cos(pet)
    y1 = (ra + rb) * math.sin(tt)
    y2 = -e * math.sin(rt)
    y3 = rc * math.sin(pet)

    # SAVE POINT 4 start
    # x = x_offset + ((ra + rb) * math.cos(tt)) - (e * math.cos(rt)) + (m * rc * math.cos(pet))
    # y = y_offset + ((ra + rb) * math.sin(tt)) - (e * math.sin(rt)) + (m * rc * math.sin(pet))
    # SAVE POINT 4 end
    # NEW CODE 4 start
    x = x_offset + ((ra + rb) * math.cos(tt)) - (e * math.cos(rt)) + (rc * math.cos(pet))
    y = y_offset + ((ra + rb) * math.sin(tt)) - (e * math.sin(rt)) + (rc * math.sin(pet))
    # NEW CODE 4 end

    logger.info('%s', '-' * 50)
    logger.info('x1: %s mm', round(x1 * 10, 6))
    logger.info('x2: %s mm', round(x2 * 10, 6))
    logger.info('x3: %s mm', round(x3 * 10, 6))
    logger.info('x: %s mm', round(x * 10, 6))
    logger.info('y1: %s mm', round(y1 * 10, 6))
    logger.info('y2: %s mm', round(y2 * 10, 6))
    logger.info('y3: %s mm', round(y3 * 10, 6))
    logger.info('y: %s mm', round(y * 10, 6))
    logger.info('%s', '-' * 50)

    return x, y


def contracted_cycloid(sketches, xy_plane, D, d, sigma, pr, e, NBR, nbr, tol=0.025, spln_pts=33):

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
    circles.addByCenterRadius(start_point, pr + (tol / 2))
    circles.addByCenterRadius(end_point, pr + (tol / 2))

    logger.info('e:  %s mm', round(e * 10, 5))
    logger.info('ra: %s mm', round(d * 10, 5))
    logger.info('rb: %s mm', round(sigma * 10, 5))
    logger.info('rc: %s mm', round(pr * 10, 5))
    logger.info('n:  %s', round(nbr, 5))
    logger.info('%s', '=' * 50)

    # sketch the points that make up the path the pin circle follows
    # sketch1 = sketches.add(xy_plane)
    # sketch1.name = f'bads'

    # pts = gen_pts_from_rads(pin_path_on_epi_cycloid, rads[1:-1], d, sigma, pr, e, offset)
    pts = gen_pts_from_rads(pin_path_on_epi_cycloid, rads, d, sigma, pr, e, offset)
    curve = sketch_splines(sketch1, pts)

    # start_point = curve.startSketchPoint
    # end_point = curve.endSketchPoint
    # lines = sketch1.sketchCurves.sketchLines
    # line = lines.addByTwoPoints(adsk.core.Point3D.create(offset, 0, 0), start_point)
    # lines.addByTwoPoints(line.startSketchPoint, end_point)


def run(context):
    ui = None
    try:
        logger.info('Program execution starting...')
        app = adsk.core.Application.get()
        ui = app.userInterface
        # ui.messageBox('Hello script')

        # get the design object for the CURRENT document
        design = app.activeProduct

        # Get the root component of the active design.
        root_comp = design.rootComponent

        # Create a new sketch on the xy plane.
        sketches = root_comp.sketches
        xy_plane = root_comp.xYConstructionPlane

        # contracted_cycloid(sketches, xy_plane, 11.0 / 2, 10.0 / 2, 1.0 / 2, 1.5 / 2, 0.4325, 11, 10, 0.0, 217)
        contracted_cycloid(sketches, xy_plane, 11.0 / 2, 10.0 / 2, 1.0 / 2, 1.5 / 2, 0.35, 11, 10, 0.0, 37)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
