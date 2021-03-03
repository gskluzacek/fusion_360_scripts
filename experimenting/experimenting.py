#Author-greg skluzacek
#Description-just trying to learn how to write a python script in fusion 360

import adsk.core, adsk.fusion, adsk.cam, traceback

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        # ui.messageBox('Hello script')
        design = adsk.fusion.Design.cast(app.activeProduct)

        root = design.rootComponent

        rotor_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        rotor = rotor_occ.component
        rotor.name = 'rotor'

        points = adsk.core.ObjectCollection.create()
        points.add(adsk.core.Point3D.create(0, 0, 0))     # x, y, z?
        points.add(adsk.core.Point3D.create(0, 40, 0))
        points.add(adsk.core.Point3D.create(40, 40, 0))
        points.add(adsk.core.Point3D.create(40, 0, 0))

        sketch = rotor.sketches.add(root.xYConstructionPlane)
        crv = sketch.sketchCurves.sketchFittedSplines.add(points)

        lines = sketch.sketchCurves.sketchLines
        line1 = lines.addByTwoPoints(crv.startSketchPoint, crv.endSketchPoint)

        prof = sketch.profiles.item(0)
        distance = adsk.core.ValueInput.createByReal(1.5)   # in CM ???

        extrudes = rotor.features.extrudeFeatures
        extrude1 = extrudes.addSimple(prof, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        body1 = extrude1.bodies.item(0)
        body1.name = "rotor"

        input_entities = adsk.core.ObjectCollection.create()
        input_entities.add(body1)

        return

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
