# Author-greg
# Description-just testing

"""
Rb - is the radius of the small circle that is used to make the lobes

Ra1 - is the radius of the larger cycloidal gear
N1  - is the number of lobes of the larger cycloidal gear

Ra2 - is the radius of the larger cycloidal gear
N2  - is the number of lobes of the larger cycloidal gear

N1 and N2 must be integers
N2 = N1 - 1

Ra2 = Ra1 * (N1 - 1) / N1
Rb = Ra1 / 2 / N1
or
Rb = Ra2 / 2 / (N1 -1)

"""


import adsk.cam
import adsk.core
import adsk.fusion
import traceback

import math


def hypo_cycloid_2(degrees, ra, rb, x_offset=0, y_offset=0):
    radians = math.radians(degrees)
    x = x_offset + (((ra - rb) * math.cos(radians)) + rb * math.cos(((ra - rb) / rb) * radians))
    y = y_offset + (((ra - rb) * math.sin(radians)) - rb * math.sin(((ra - rb) / rb) * radians))
    return adsk.core.Point3D.create(x, y, 0)


def epi_cycloid_2(degrees, ra, rb, x_offset=0, y_offset=0):
    radians = math.radians(degrees)
    x = x_offset + ((ra + rb) * math.cos(radians) - rb * math.cos(((ra + rb) / rb) * radians))
    y = y_offset + ((ra + rb) * math.sin(radians) - rb * math.sin(((ra + rb) / rb) * radians))
    return adsk.core.Point3D.create(x, y, 0)


def circle_2(degrees, radius, x_offset=0, y_offset=0):
    radians = math.radians(degrees)
    x = x_offset + (radius * math.cos(radians))
    y = y_offset + (radius * math.sin(radians))
    return adsk.core.Point3D.create(x, y, 0)


def all_in_tooth_flank(radians, base_radius, nbr_fixed_pins, fixed_pin_radius, eccentricity):
    x = (
        (base_radius * math.cos(radians)) - (
            fixed_pin_radius * math.cos(
                radians + math.atan(
                    math.sin((1 - nbr_fixed_pins) * radians) / (
                        (base_radius / eccentricity * nbr_fixed_pins) -
                        math.cos((1 - nbr_fixed_pins) * radians)
                    )
                )
            )
        ) - (eccentricity * math.cos(nbr_fixed_pins * radians))
    )
    y = (
        (-base_radius * math.sin(radians)) + (
            fixed_pin_radius * math.sin(
                radians + math.atan(
                    math.sin((1 - nbr_fixed_pins) * radians) / (
                        (base_radius / eccentricity * nbr_fixed_pins) -
                        math.cos((1 - nbr_fixed_pins) * radians)
                    )
                )
            )
        ) + (eccentricity * math.sin(nbr_fixed_pins * radians))
    )
    return x, y


def hypo_cycloid(radians, ra, rb, x_offset=0, y_offset=0):
    x = x_offset + (((ra - rb) * math.cos(radians)) + rb * math.cos(((ra - rb) / rb) * radians))
    y = y_offset + (((ra - rb) * math.sin(radians)) - rb * math.sin(((ra - rb) / rb) * radians))
    return x, y


def epi_cycloid(radians, ra, rb, x_offset=0, y_offset=0):
    """

    :param radians:
    :param ra:              radius of the base circle
    :param rb:              radius of the rolling circle
    :param x_offset:
    :param y_offset:
    :return:
    """
    x = x_offset + (
        (
            (ra + rb) * math.cos(radians)
        ) - (
            rb * math.cos(
                (
                    (ra + rb) / rb
                ) * radians
            )
        )
    )
    y = y_offset + ((ra + rb) * math.sin(radians) - rb * math.sin(((ra + rb) / rb) * radians))
    return x, y


def circle(radians, radius, x_offset=0, y_offset=0):
    x = x_offset + (radius * math.cos(radians))
    y = y_offset + (radius * math.sin(radians))
    return x, y


def sketch_splines2(sketch, func, degrees_list, *args):
    curves = []
    for degrees in degrees_list:
        points = adsk.core.ObjectCollection.create()
        for i in degrees:
            points.add(func(i, *args))
        curve = sketch.sketchCurves.sketchFittedSplines.add(points)
        curves.append(curve)
    return curves


def sketch_splines(sketch, points_list):
    curves = []
    for points in points_list:
        pts = adsk.core.ObjectCollection.create()
        for x, y in points:
            pt = adsk.core.Point3D.create(x, y, 0)
            pts.add(pt)
        curve = sketch.sketchCurves.sketchFittedSplines.add(pts)
        curves.append(curve)
    return curves


def gen_pts_from_rads(func, radians_list, *args):
    points_list = []
    for radians in radians_list:
        points = [func(i, *args) for i in radians]
        points_list.append(points)
    return points_list


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


def sketch_point(sketch, x, y):
    sketch.sketchPoints.add(adsk.core.Point3D.create(x, y, 0))


def gen_degrees(nbr_of_slices, nbr_of_steps, every_nth_slice=1, slice_phase=1, slices=None, slice_list=None):
    degrees_per_slice = 360 / nbr_of_slices
    degrees_per_step = degrees_per_slice / (nbr_of_steps - 1)

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
        slice_start_degrees = slice_nbr * degrees_per_slice
        values = [slice_start_degrees + (step_nbr * degrees_per_step) for step_nbr in range(nbr_of_steps)]
        list_of_values.append(values)
    return list_of_values


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


def test_1(sketches, xy_plane):
    sketch2 = sketches.add(xy_plane)
    sketch2.name = f'circ3'

    d11 = gen_degrees(20, 21, slice_list=[1])
    d12 = gen_degrees(20, 21, slice_list=[0])
    sketch_splines2(sketch2, epi_cycloid2, d11, 1.65, 0.0825)
    sketch_splines2(sketch2, hypo_cycloid2, d12, 1.65, 0.0825)
    sketch_point(sketch2, 0, 0)


def lobe_20_19_old(sketches, xy_plane):
    sketch1 = sketches.add(xy_plane)
    sketch1.name = f'circ1'

    d11 = gen_degrees(40, 21, slice_list=[0])
    d12 = gen_degrees(40, 21, slice_list=[1])
    curves_1 = sketch_splines2(sketch1, epi_cycloid2, d11, 6.0, 0.15)
    curves_2 = sketch_splines2(sketch1, hypo_cycloid2, d12, 6.0, 0.15)

    lines = sketch1.sketchCurves.sketchLines
    line = lines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0), curves_1[0].startSketchPoint)
    lines.addByTwoPoints(line.startSketchPoint, curves_2[0].endSketchPoint)

    sketch2 = sketches.add(xy_plane)
    sketch2.name = f'circ2'

    d21 = gen_degrees(38, 21, slice_list=[0])
    d22 = gen_degrees(38, 21, slice_list=[1])
    curves_1 = sketch_splines2(sketch2, epi_cycloid2, d21, 5.7, 0.15, 0.3)
    curves_2 = sketch_splines2(sketch2, hypo_cycloid2, d22, 5.7, 0.15, 0.3)

    lines = sketch2.sketchCurves.sketchLines
    line = lines.addByTwoPoints(adsk.core.Point3D.create(0.3, 0, 0), curves_1[0].startSketchPoint)
    lines.addByTwoPoints(line.startSketchPoint, curves_2[0].endSketchPoint)


def lobe_20_19(sketches, xy_plane):
    sketch1 = sketches.add(xy_plane)
    sketch1.name = f'circ1'

    r11 = gen_radians(40, 21, slice_list=[0])
    r12 = gen_radians(40, 21, slice_list=[1])
    p11 = gen_pts_from_rads(epi_cycloid, r11, 6.0, 0.15)
    p12 = gen_pts_from_rads(hypo_cycloid, r12, 6.0, 0.15)
    curves_1 = sketch_splines(sketch1, p11)
    curves_2 = sketch_splines(sketch1, p12)

    lines = sketch1.sketchCurves.sketchLines
    line = lines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0), curves_1[0].startSketchPoint)
    lines.addByTwoPoints(line.startSketchPoint, curves_2[0].endSketchPoint)

    sketch2 = sketches.add(xy_plane)
    sketch2.name = f'circ2'

    r21 = gen_radians(38, 21, slice_list=[0])
    r22 = gen_radians(38, 21, slice_list=[1])
    tp21 = gen_pts_from_rads(epi_cycloid, r21, 5.7, 0.15, 0.3)
    tp22 = gen_pts_from_rads(hypo_cycloid, r22, 5.7, 0.15, 0.3)
    p21 = xform_for_tolerance(tp21, 0.3, 0, 0.025)
    p22 = xform_for_tolerance(tp22, 0.3, 0, 0.025)
    curves_1 = sketch_splines(sketch2, p21)
    curves_2 = sketch_splines(sketch2, p22)

    lines = sketch2.sketchCurves.sketchLines
    line = lines.addByTwoPoints(adsk.core.Point3D.create(0.3, 0, 0), curves_1[0].startSketchPoint)
    lines.addByTwoPoints(line.startSketchPoint, curves_2[0].endSketchPoint)


# calc_pt_uc1(math.radians(5), 8, 0, 8, 10, 9, 72)
def calc_pt_uc1(rad, x0, y0, rs, rp, n, b):
    roll_pt_angle = rad * n                                 # roll point angle = radians * number-of-lobes
    p1 = math.sin(roll_pt_angle) * rs
    p2 = math.cos(roll_pt_angle) * rs                       # rs (hy), p1 (op), p2 (ad) form a right triangle
    p3 = math.sqrt(math.pow(rp, 2) - math.pow(p1, 2))       # rp (c), p1 (a), p3 (b) form a right triangle: solve for b
    p4 = rs - p2
    p5 = p3 - p4
    p6 = b - p5
    opp = math.sin(rad) * p6
    adj = math.cos(rad) * p6                                # p6 (hy), opp (op), adj (ad) form a right triangle
    xp = x0 + adj
    yp = y0 + opp
    return xp, yp


def calc_cycloid_tooth_pt(angle_g, coord_xb, coord_yb, radius_1, radius_3, nbr_of_lobes, radius_4):
    """For the given angle G passed, calculate the corresponding point which lies on the cycloid tooth path.

    The function should be called multiple times passing in the values for each Angle G (radians) that make up
    arc that defines the cycloid tooth.

    :param angle_g:         the given angle of the base circle (variable)
    :param coord_xb:        the x coordinate of the center point of the base circle
    :param coord_yb:        the y coordinate of the center point of the base circle
    :param radius_1:        the radius of the rolling circle (radius_2 = radius_1) - isosceles triangle
    :param radius_3:        the radius of the stationary pin circle
    :param nbr_of_lobes:    the number of lobes on the cycloidal disc
    :param radius_4:        the radius of the base circle.
    :return:                the x and y coordinates corresponding to given angle_g

    see: https://docs.google.com/document/d/1WOi9ffHHctRW7r76wrddw6mHHZkYbwIL0w1kgEwKnSk/edit?usp=sharing
    for documentation of the various calculations below
    """
    # since line_t3 is parallel to line_a, angle_a1 and angle_g are the same value
    angle_a1 = angle_g
    angle_c = angle_g * nbr_of_lobes
    angle_b = (math.pi - angle_c) / 2
    # since the triangle is an isosceles triangle angle_a0 is the same as angle_b
    angle_a0 = angle_b
    angle_a2 = angle_a0 - angle_a1

    line_t0 = math.cos(angle_b) * radius_1
    line_t2 = (2 * line_t0) - radius_3
    line_t3 = math.cos(angle_a2) * line_t2
    line_c = line_t3 / math.cos(angle_a1)

    line_b1 = math.tan(angle_a1) * line_t3
    line_b2 = math.tan(angle_a2) * line_t3

    line_ct = line_c + radius_4
    line_bt = math.sin(angle_g) * line_ct
    line_a = math.cos(angle_g) * line_ct
    line_b = line_bt - (line_b1 + line_b2)

    coord_xp = coord_xb + line_a
    coord_yp = coord_yb + line_b

    return coord_xp, coord_yp


def cycloidal_tooth_flank_small(sketches, xy_plane):
    D = 8.0 / 2        # radius of the pins reference circle
    d = 7.6 / 2        # radius of the epi-cycloid base circle
    offset = D - d      # offset of the 2 circle's center points
    sigma = 0.4 / 2     # radius of the epi-cycloid rolling circle
    pr = 0.5 / 2         # radius of the pin circle
    nbr = 19             # number of lobes
    NBR = 20            # number of pins
    spln_pts = 43       # number of points on the spline sketch

    rads = gen_radians(nbr, spln_pts, slice_list=[0])

    sketch1 = sketches.add(xy_plane)
    sketch1.name = f'cycloid_gear_inner'

    pts_tmp = gen_pts_from_rads(epi_cycloid, rads, d, sigma, offset)
    pts = xform_for_tolerance(pts_tmp, offset, 0.0, 0.025)
    sketch_splines(sketch1, pts)

    pts_tmp = gen_pts_from_rads(calc_cycloid_tooth_pt, rads, offset, 0, sigma, pr, nbr, d)
    pts = xform_for_tolerance(pts_tmp, offset, 0.0, 0.025)
    sketch_splines(sketch1, pts)

    D = 8.0 / 2        # radius of the pins reference circle
    d = 8.0 / 2        # radius of the epi-cycloid base circle
    offset = D - d      # offset of the 2 circle's center points
    sigma = 0.4 / 2     # radius of the epi-cycloid rolling circle
    pr = 0.5 / 2         # radius of the pin circle
    nbr = 20             # number of lobes
    NBR = 20            # number of pins
    spln_pts = 43       # number of points on the spline sketch

    rads = gen_radians(nbr, spln_pts, slice_list=[0])

    sketch1 = sketches.add(xy_plane)
    sketch1.name = f'cycloid_gear_outer'

    pts = gen_pts_from_rads(epi_cycloid, rads, d, sigma, offset)
    sketch_splines(sketch1, pts)

    pts = gen_pts_from_rads(calc_cycloid_tooth_pt, rads, offset, 0, sigma, pr, nbr, d)
    sketch_splines(sketch1, pts)


def cycloidal_actuator_experimental(sketches, xy_plane, base_radius, nbr_fixed_pins, fixed_pin_radius, eccentricity, spln_pts=11):
    sketch1 = sketches.add(xy_plane)
    sketch1.name = f'cycloid_gear_inner'
    rads = gen_radians(nbr_fixed_pins - 1, spln_pts)
    pts = gen_pts_from_rads(all_in_tooth_flank, rads, base_radius, nbr_fixed_pins, fixed_pin_radius, eccentricity)
    curves = sketch_splines(sketch1, pts)
    start_point = curves[0].startSketchPoint
    end_point = curves[0].endSketchPoint
    lines = sketch1.sketchCurves.sketchLines
    line = lines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0), start_point)
    lines.addByTwoPoints(line.startSketchPoint, end_point)


def cycloidal_actuator(sketches, xy_plane, D, d, sigma, pr, NBR, nbr, tol=0.025, spln_pts=43):
    offset = D - d

    sketch1 = sketches.add(xy_plane)
    sketch1.name = f'cycloid_gear_inner'

    rads = gen_radians(nbr, spln_pts, slice_list=[0])
    pts_tmp = gen_pts_from_rads(epi_cycloid, rads, d, sigma, offset)
    pts = xform_for_tolerance(pts_tmp, offset, 0.0, tol)
    curves = sketch_splines(sketch1, pts)

    pts_tmp = gen_pts_from_rads(calc_cycloid_tooth_pt, rads, offset, 0, sigma, pr, nbr, d)
    pts = xform_for_tolerance(pts_tmp, offset, 0.0, tol)
    sketch_splines(sketch1, pts)

    start_point = curves[0].startSketchPoint
    end_point = curves[0].endSketchPoint
    lines = sketch1.sketchCurves.sketchLines
    line = lines.addByTwoPoints(adsk.core.Point3D.create(offset, 0, 0), start_point)
    lines.addByTwoPoints(line.startSketchPoint, end_point)
    circles = sketch1.sketchCurves.sketchCircles
    circles.addByCenterRadius(start_point, pr + (tol / 2))
    circles.addByCenterRadius(end_point, pr + (tol / 2))

    sketch1 = sketches.add(xy_plane)
    sketch1.name = f'cycloid_gear_outer'

    rads = gen_radians(NBR, spln_pts, slice_list=[0])
    pts = gen_pts_from_rads(epi_cycloid, rads, D, sigma)
    curves = sketch_splines(sketch1, pts)

    pts = gen_pts_from_rads(calc_cycloid_tooth_pt, rads, 0, 0, sigma, pr, NBR, D)
    sketch_splines(sketch1, pts)

    start_point = curves[0].startSketchPoint
    end_point = curves[0].endSketchPoint
    lines = sketch1.sketchCurves.sketchLines
    line = lines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0), start_point)
    lines.addByTwoPoints(line.startSketchPoint, end_point)
    circles = sketch1.sketchCurves.sketchCircles
    circles.addByCenterRadius(start_point, pr)
    circles.addByCenterRadius(end_point, pr)


def cycloidal_tooth_flank(sketches, xy_plane):
    D = 16.0 / 2        # radius of the pins reference circle
    d = 14.4 / 2        # radius of the epi-cycloid base circle
    offset = D - d      # offset of the 2 circle's center points
    sigma = 1.6 / 2     # radius of the epi-cycloid rolling circle
    pr = 2.0 / 2         # radius of the pin circle
    nbr = 9             # number of lobes
    NBR = 10            # number of pins
    spln_pts = 43       # number of points on the spline sketch

    rads = gen_radians(nbr, spln_pts, slice_list=[0])

    sketch1 = sketches.add(xy_plane)
    sketch1.name = f'epi-cycloid_wt'

    pts_tmp = gen_pts_from_rads(epi_cycloid, rads, d, sigma, offset)
    pts = xform_for_tolerance(pts_tmp, offset, 0.0, 0.025)
    sketch_splines(sketch1, pts)

    pts_tmp = gen_pts_from_rads(calc_cycloid_tooth_pt, rads, offset, 0, sigma, pr, nbr, d)
    pts = xform_for_tolerance(pts_tmp, offset, 0.0, 0.025)
    sketch_splines(sketch1, pts)


def cycloidal_tooth_flank_no_t(sketches, xy_plane):
    D = 16.0 / 2        # radius of the pins reference circle
    d = 14.4 / 2        # radius of the epi-cycloid base circle
    offset = D - d      # offset of the 2 circle's center points
    sigma = 1.6 / 2     # radius of the epi-cycloid rolling circle
    pr = 2.0 / 2         # radius of the pin circle
    nbr = 9             # number of lobes
    NBR = 10            # number of pins
    spln_pts = 43       # number of points on the spline sketch

    rads = gen_radians(nbr, spln_pts, slice_list=[0])

    sketch1 = sketches.add(xy_plane)
    sketch1.name = f'epi-cycloid'

    pts = gen_pts_from_rads(epi_cycloid, rads, d, sigma, offset)
    sketch_splines(sketch1, pts)

    pts = gen_pts_from_rads(calc_cycloid_tooth_pt, rads, offset, 0, sigma, pr, nbr, d)
    sketch_splines(sketch1, pts)


def cycloidal_tooth_flank1(sketches, xy_plane):
    D = 16.0 / 2        # radius of the pins reference circle
    d = 16.0 / 2        # radius of the epi-cycloid base circle
    offset = D - d      # offset of the 2 circle's center points
    sigma = 1.6 / 2     # radius of the epi-cycloid rolling circle
    pr = 2.0 / 2         # radius of the pin circle
    nbr = 10             # number of lobes
    NBR = 10            # number of pins
    spln_pts = 43       # number of points on the spline sketch

    rads = gen_radians(nbr, spln_pts, slice_list=[0])

    sketch1 = sketches.add(xy_plane)
    sketch1.name = f'epi-cycloid-2'

    pts = gen_pts_from_rads(epi_cycloid, rads, d, sigma, offset)
    sketch_splines(sketch1, pts)

    pts = gen_pts_from_rads(calc_cycloid_tooth_pt, rads, offset, 0, sigma, pr, nbr, d)
    sketch_splines(sketch1, pts)


def cycloidal_tooth_flank2(sketches, xy_plane):
    D = 6.0        # radius of the pins reference circle
    d = 5.7        # radius of the epi-cycloid base circle
    offset = D - d      # offset of the 2 circle's center points
    sigma = 0.30     # radius of the epi-cycloid rolling circle
    pr = 0.475         # radius of the pin circle
    nbr = 19             # number of lobes
    NBR = 20            # number of pins
    spln_pts = 21       # number of points on the spline sketch

    rads = gen_radians(nbr, spln_pts, slice_list=[0])

    sketch1 = sketches.add(xy_plane)
    sketch1.name = f'epi-cycloid'

    pts = gen_pts_from_rads(epi_cycloid, rads, d, sigma, offset)
    sketch_splines(sketch1, pts)

    pts = gen_pts_from_rads(calc_cycloid_tooth_pt, rads, offset, 0, sigma, pr, nbr, d)
    sketch_splines(sketch1, pts)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        # get the design object for the CURRENT document
        design = app.activeProduct

        # Get the root component of the active design.
        root_comp = design.rootComponent

        # Create a new sketch on the xy plane.
        sketches = root_comp.sketches
        xy_plane = root_comp.xYConstructionPlane

        # lobe_20_19(sketches, xy_plane)                # was used to do prototype cycloidal gear
        # cycloidal_tooth_flank(sketches, xy_plane)     # was used for true cycloidal gear
        # cycloidal_tooth_flank_no_t(sketches, xy_plane)
        # cycloidal_tooth_flank2(sketches, xy_plane)    # was used in attempt to get the lobe size & pin size equal
        # cycloidal_tooth_flank1(sketches, xy_plane)
        # cycloidal_tooth_flank_small(sketches, xy_plane)

        # cycloidal_actuator(sketches, xy_plane, 5.8 / 2, 5.51 / 2, 0.29 / 2, 0.4 / 2, 20, 19)
        # cycloidal_actuator(sketches, xy_plane, 6.0 / 2, 5.6 / 2, 0.4 / 2, 0.5720 / 2, 15, 14, .0125)

        # cycloidal_actuator_experimental(sketches, xy_plane, 5.6 / 2, 15, 0.5720 / 2, .2)
        cycloidal_actuator_experimental(sketches, xy_plane, 2.5, 21, 0.0, .1)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
