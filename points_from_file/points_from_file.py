#Author-
#Description-

import adsk.core, adsk.fusion, adsk.cam, traceback

from collections import namedtuple
Rec = namedtuple('Rec', ['ln_nbr', 'label', 'x', 'y'])


fin = '/Users/gskluzacek/fine_grained_striped_very.txt'


class PointRec:
    def __init__(self, line):
        row = line.strip().split('\t')
        self.ln_nbr = int(row[0])
        self.label = row[1]
        self.x = float(row[2])
        self.y = float(row[3])


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        # get the design object for the CURRENT document
        design = app.activeProduct

        # Get the root component of the active design.
        root_comp = design.rootComponent

        # Create a new sketch on the xy plane.
        sketches = root_comp.sketches
        xy_plane = root_comp.xYConstructionPlane

        sketch1 = sketches.add(xy_plane)
        sketch1.name = f'just some points'

        circles = sketch1.sketchCurves.sketchCircles
        circles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), 50)

        with open(fin) as fi:
            for line in fi:
                point = PointRec(line)
                sketch1.sketchPoints.add(adsk.core.Point3D.create(point.x, point.y, 0))

        ui.messageBox('Where is the shit')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
