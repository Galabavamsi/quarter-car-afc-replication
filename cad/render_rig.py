"""Render / animate the FreeCAD rig in Blender.

    blender --background --python render_rig.py -- --still rig.png
    blender --background --python render_rig.py -- --csv ../animation/anim_bump.csv --ctrl afc_aw --out rig_bump.mp4

Imports the STL parts written by quarter_car_rig.FCMacro (folder export/),
colours them, and optionally drives the moving parts with a trajectory CSV
from export_animation_data.m: road platform <- z_r, carrier/tyre/rim and
lower strut parts <- z_u, body + sensors <- z_s, spring stretched between
them. Motion is exaggerated (--exag, default 4) and slowed (--slow, 2).
"""
import csv
import math
import os
import sys

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
OPTS = {"still": "", "out": "", "csv": "", "ctrl": "afc_aw", "exag": 4.0, "slow": 2.0, "samples": 12, "engine": "", "resx": 1280, "resy": 960}
if "--" in sys.argv:
    a = sys.argv[sys.argv.index("--") + 1:]
    for i in range(0, len(a) - 1, 2):
        k = a[i].lstrip("-")
        OPTS[k] = type(OPTS[k])(a[i + 1]) if k in OPTS else a[i + 1]
for k in ("still", "out", "csv"):
    if OPTS[k] and not os.path.isabs(OPTS[k]):
        OPTS[k] = os.path.join(HERE, OPTS[k])
EXP = os.path.join(HERE, "export")
COL = {"Frame": (0.55, 0.58, 0.62), "Base": (0.30, 0.32, 0.35), "PneumaticCylinder": (0.70, 0.72, 0.75),
       "RoadPlatform": (0.42, 0.42, 0.42), "Tyre": (0.06, 0.06, 0.06), "Rim": (0.78, 0.78, 0.80),
       "Carrier": (0.30, 0.45, 0.60), "Body": (0.00, 0.42, 0.52), "Spring": (0.85, 0.70, 0.18),
       "Damper": (0.22, 0.22, 0.25), "HydraulicCylinder": (0.72, 0.18, 0.12), "ServoValve": (0.15, 0.18, 0.55),
       "Sensors": (0.10, 0.60, 0.30)}
MOVE_U = ("Carrier", "Tyre", "Rim", "Damper", "HydraulicCylinder", "ServoValve")
MOVE_S = ("Body", "Sensors")


def import_stl(path):
    before = set(bpy.data.objects)
    if hasattr(bpy.ops.wm, "stl_import"):
        bpy.ops.wm.stl_import(filepath=path, global_scale=0.001)
    else:
        bpy.ops.import_mesh.stl(filepath=path, global_scale=0.001)
    new = [o for o in bpy.data.objects if o not in before]
    return new[0]


def mat(name, rgb):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*rgb, 1)
    b.inputs["Roughness"].default_value = 0.4
    b.inputs["Metallic"].default_value = 0.3 if name in ("Frame", "Rim", "PneumaticCylinder") else 0.0
    return m


def main():
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    objs = {}
    for f in sorted(os.listdir(EXP)):
        if f.endswith(".stl"):
            name = f[:-4]
            o = import_stl(os.path.join(EXP, f))
            o.name = name
            o.data.materials.clear()
            o.data.materials.append(mat(name, COL.get(name, (0.6, 0.6, 0.6))))
            bpy.ops.object.select_all(action="DESELECT")
            o.select_set(True)
            bpy.context.view_layer.objects.active = o
            # some importers put the mm->m factor in the object scale; bake it into the mesh
            if tuple(o.scale) != (1.0, 1.0, 1.0):
                from mathutils import Matrix
                o.data = o.data.copy()
                o.data.transform(Matrix.Diagonal((*o.scale, 1.0)))
                o.scale = (1.0, 1.0, 1.0)
            if name in ("Tyre", "Spring"):
                bpy.ops.object.shade_smooth()
            else:
                bpy.ops.object.shade_flat()
            objs[name] = o
    sc = bpy.context.scene
    # camera, light, world
    cam = bpy.data.objects.new("Camera", bpy.data.cameras.new("Camera"))
    sc.collection.objects.link(cam)
    cam.location = (2.1, -2.5, 1.45)
    cam.data.lens = 46
    target = bpy.data.objects.new("CameraTarget", None)
    sc.collection.objects.link(target)
    target.location = (0, 0, 0.82)
    tc = cam.constraints.new("TRACK_TO")
    tc.target = target
    tc.track_axis = "TRACK_NEGATIVE_Z"
    tc.up_axis = "UP_Y"
    sc.camera = cam
    sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", "SUN"))
    sc.collection.objects.link(sun)
    sun.rotation_euler = (math.radians(50), math.radians(10), math.radians(-30))
    sun.data.energy = 3.5
    if sc.world is None:
        sc.world = bpy.data.worlds.new("World")
    sc.world.use_nodes = True
    sc.world.node_tree.nodes["Background"].inputs[0].default_value = (0.93, 0.95, 0.97, 1)
    sc.world.node_tree.nodes["Background"].inputs[1].default_value = 0.8
    try:
        sc.view_settings.view_transform = "Standard"
    except TypeError:
        pass
    engines = [e.identifier for e in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items]
    eng = OPTS["engine"] or ("CYCLES" if bpy.app.background else ("BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in engines else "BLENDER_EEVEE"))
    sc.render.engine = eng
    if eng == "CYCLES":
        sc.cycles.samples = int(OPTS["samples"])
        sc.cycles.device = "CPU"
        if hasattr(sc.cycles, "use_denoising"):
            sc.cycles.use_denoising = False
    sc.render.resolution_x, sc.render.resolution_y = int(OPTS["resx"]), int(OPTS["resy"])

    if OPTS["csv"]:
        with open(OPTS["csv"], newline="") as fh:
            rows = list(csv.reader(fh))
        hd = rows[0]
        D = {h: [float(r[i]) for r in rows[1:]] for i, h in enumerate(hd)}
        E = float(OPTS["exag"])
        dt = D["t"][1] - D["t"][0]
        step = max(1, int(round(1.0 / (25 * dt * float(OPTS["slow"])))))
        idx = list(range(0, len(D["t"]), step))
        sc.render.fps = 25
        sc.frame_start, sc.frame_end = 1, len(idx)
        base = {n: o.location.z for n, o in objs.items()}
        spring = objs["Spring"]
        spring_z0 = min(v.co.z for v in spring.data.vertices) * spring.scale.z
        span0 = (max(v.co.z for v in spring.data.vertices) - min(v.co.z for v in spring.data.vertices))
        # move the spring origin to its bottom so scaling stretches it upwards
        for v in spring.data.vertices:
            v.co.z -= spring_z0
        spring.location.z += spring_z0
        pneu = objs["PneumaticCylinder"]
        pz0 = min(v.co.z for v in pneu.data.vertices)
        pspan = max(v.co.z for v in pneu.data.vertices) - pz0
        for v in pneu.data.vertices:
            v.co.z -= pz0
        pneu.location.z += pz0
        c = OPTS["ctrl"]
        for f, k in enumerate(idx, start=1):
            zr, zu, zs = E * D["zr"][k], E * D["zu_" + c][k], E * D["zs_" + c][k]
            objs["RoadPlatform"].location.z = base["RoadPlatform"] + zr
            objs["RoadPlatform"].keyframe_insert("location", frame=f)
            pneu.scale.z = max(0.2, (pspan + zr) / pspan)   # rod extends with the road
            pneu.keyframe_insert("scale", frame=f)
            for n in MOVE_U:
                objs[n].location.z = base[n] + zu
                objs[n].keyframe_insert("location", frame=f)
            for n in MOVE_S:
                objs[n].location.z = base[n] + zs
                objs[n].keyframe_insert("location", frame=f)
            spring.location.z = base["Spring"] + spring_z0 + zu
            spring.scale.z = max(0.2, (span0 + zs - zu) / span0)
            spring.keyframe_insert("location", frame=f)
            spring.keyframe_insert("scale", frame=f)
    if OPTS["still"]:
        sc.frame_set(max(1, sc.frame_end // 3))
        sc.render.image_settings.file_format = "PNG"
        sc.render.filepath = OPTS["still"]
        bpy.ops.render.render(write_still=True)
    if OPTS["out"]:
        sc.render.image_settings.file_format = "FFMPEG"
        sc.render.ffmpeg.format = "MPEG4"
        sc.render.ffmpeg.codec = "H264"
        sc.render.filepath = OPTS["out"]
        bpy.ops.render.render(animation=True)


main()
