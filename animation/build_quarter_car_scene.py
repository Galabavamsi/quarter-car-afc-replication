"""Blender scene: three quarter cars driven by the simulated trajectories.

Rigs, left to right: Passive | AFC (paper, Case A) | AFC-AW (our extension).
Vertical motion is exaggerated (default x4) so centimetre-level motion is
visible; the prescribed bound +/-mu1(t) is drawn as red bars beside the
two AFC bodies.

Run inside Blender's Scripting tab (edit CSV_NAME below), or headless:
    blender --background --python build_quarter_car_scene.py -- \
        --csv anim_bump.csv --out quarter_car_bump.mp4 [--exag 4] [--slow 2] [--engine CYCLES]
The CSV files come from export_animation_data.m (MATLAB or Octave).
"""
import csv
import math
import os
import sys

import bpy

# ----------------------------------------------------------------- settings
HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
CSV_NAME = "anim_bump.csv"
OPTS = {"csv": os.path.join(HERE, CSV_NAME), "out": "", "exag": 4.0, "slow": 2.0,
        "engine": "", "fps": 25, "still": "", "title": "", "samples": 12}
if "--" in sys.argv:
    args = sys.argv[sys.argv.index("--") + 1:]
    for i in range(0, len(args) - 1, 2):
        key = args[i].lstrip("-")
        OPTS[key] = type(OPTS.get(key, ""))(args[i + 1]) if key in OPTS else args[i + 1]
for key in ("csv", "out", "still"):
    if OPTS[key] and not os.path.isabs(OPTS[key]):
        OPTS[key] = os.path.join(HERE, OPTS[key])

RIGS = [("passive", "Passive", False), ("afc", "AFC (paper, Case A)", True),
        ("afc_aw", "AFC-AW (extension)", True)]
SPACING = 3.0
R_TIRE = 0.32
BODY_Z0 = R_TIRE + 1.05
E = float(OPTS["exag"])

COL = {"body": (0.0, 0.40, 0.50, 1), "tire": (0.05, 0.05, 0.05, 1), "hub": (0.6, 0.6, 0.62, 1),
       "road": (0.35, 0.35, 0.35, 1), "spring": (0.85, 0.75, 0.25, 1), "damper": (0.25, 0.25, 0.28, 1),
       "act": (0.75, 0.15, 0.12, 1), "bound": (0.85, 0.05, 0.05, 1), "text": (0.05, 0.05, 0.05, 1),
       "ground": (0.92, 0.92, 0.92, 1)}


# ----------------------------------------------------------------- helpers
def read_csv(path):
    with open(path, newline="") as fh:
        rows = list(csv.reader(fh))
    head = rows[0]
    cols = {h: [] for h in head}
    for r in rows[1:]:
        for h, v in zip(head, r):
            cols[h].append(float(v) if v != "" else float("nan"))
    return cols


def material(name, rgba, emission=0.0):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = rgba
    bsdf.inputs["Roughness"].default_value = 0.45
    if emission > 0:
        for key in ("Emission Color", "Emission"):
            if key in bsdf.inputs:
                bsdf.inputs[key].default_value = rgba
                break
        if "Emission Strength" in bsdf.inputs:
            bsdf.inputs["Emission Strength"].default_value = emission
    m.diffuse_color = rgba
    return m


def add_box(name, size, loc, mat):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = bpy.context.active_object
    o.name = name
    o.scale = size
    o.data.materials.append(mat)
    return o


def add_cyl(name, radius, depth, loc, mat, rot=(0, 0, 0), verts=32):
    bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=radius, depth=depth, location=loc, rotation=rot)
    o = bpy.context.active_object
    o.name = name
    o.data.materials.append(mat)
    return o


def add_spring(name, radius, turns, wire, loc, mat):
    """Helix of unit height (z from 0 to 1); animate scale.z = length."""
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    sp = cu.splines.new("POLY")
    n = int(turns * 24)
    sp.points.add(n)
    for i in range(n + 1):
        a = 2 * math.pi * turns * i / n
        sp.points[i].co = (radius * math.cos(a), radius * math.sin(a), i / n, 1)
    cu.bevel_depth = wire
    cu.bevel_resolution = 3
    o = bpy.data.objects.new(name, cu)
    bpy.context.collection.objects.link(o)
    o.location = loc
    o.data.materials.append(mat)
    return o


def add_text(name, body, loc, size, mat):
    cu = bpy.data.curves.new(name, "FONT")
    cu.body = body
    cu.size = size
    cu.align_x = "CENTER"
    o = bpy.data.objects.new(name, cu)
    bpy.context.collection.objects.link(o)
    o.location = loc
    o.rotation_euler = (math.pi / 2, 0, 0)
    o.data.materials.append(mat)
    return o


def key(o, attr, frame):
    o.keyframe_insert(data_path=attr, frame=frame)


# ----------------------------------------------------------------- scene
def build():
    data = read_csv(OPTS["csv"])
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    sc = bpy.context.scene
    fps = int(OPTS["fps"])
    sc.render.fps = fps
    t = data["t"]
    dt = t[1] - t[0]
    step = max(1, int(round(1.0 / (fps * dt * OPTS["slow"]))))
    idx = list(range(0, len(t), step))
    sc.frame_start = 1
    sc.frame_end = len(idx)

    mats = {k: material("M_" + k, v, 3.0 if k == "bound" else 0.0) for k, v in COL.items()}
    add_box("Ground", (3 * SPACING + 2, 4, 0.02), (0, 0, -0.32), mats["ground"])
    rigs = []
    for j, (tag, label, has_bound) in enumerate(RIGS):
        x0 = (j - 1) * SPACING
        road = add_box(f"Road_{tag}", (1.2, 0.9, 0.3), (x0, 0, -0.15), mats["road"])
        tire = add_cyl(f"Tire_{tag}", R_TIRE, 0.22, (x0, 0, R_TIRE), mats["tire"], rot=(math.pi / 2, 0, 0))
        hub = add_box(f"Hub_{tag}", (0.9, 0.3, 0.12), (x0, 0, R_TIRE), mats["hub"])
        body = add_box(f"Body_{tag}", (1.4, 0.9, 0.38), (x0, 0, BODY_Z0), mats["body"])
        spring = add_spring(f"Spring_{tag}", 0.08, 6, 0.012, (x0 - 0.3, 0, R_TIRE + 0.06), mats["spring"])
        dmp_lo = add_cyl(f"DamperTube_{tag}", 0.05, 0.4, (x0, 0, R_TIRE + 0.26), mats["damper"])
        dmp_hi = add_cyl(f"DamperRod_{tag}", 0.018, 0.5, (x0, 0, BODY_Z0 - 0.44), mats["hub"])
        act_lo = add_cyl(f"Cylinder_{tag}", 0.06, 0.42, (x0 + 0.3, 0, R_TIRE + 0.27), mats["act"])
        act_hi = add_cyl(f"PistonRod_{tag}", 0.02, 0.5, (x0 + 0.3, 0, BODY_Z0 - 0.44), mats["hub"])
        add_text(f"Label_{tag}", label, (x0, -0.6, BODY_Z0 + 0.75), 0.22, mats["text"])
        bounds = []
        pointer = add_box(f"Pointer_{tag}", (0.18, 0.05, 0.035), (x0 + 0.78, -0.3, BODY_Z0), mats["text"])
        pointer.parent = body
        pointer.location = (0.78 / 1.4, -0.3 / 0.9, 0)
        pointer.scale = (0.18 / 1.4, 0.05 / 0.9, 0.035 / 0.38)
        if has_bound:
            for s in (1, -1):
                bounds.append(add_box(f"Bound_{tag}_{s}", (0.32, 0.05, 0.035), (x0 + 0.95, -0.3, BODY_Z0), mats["bound"]))
        rigs.append(dict(tag=tag, x0=x0, road=road, tire=tire, hub=hub, body=body, spring=spring,
                         dmp_lo=dmp_lo, dmp_hi=dmp_hi, act_lo=act_lo, act_hi=act_hi, bounds=bounds))

    info = add_text("Info", "", (0, -0.6, -0.72), 0.17, mats["text"])
    title = OPTS.get("title") or os.path.splitext(os.path.basename(OPTS["csv"]))[0]
    add_text("Title", title, (0, -0.6, BODY_Z0 + 1.25), 0.2, mats["text"])
    add_text("Legend", "red bars: prescribed bound +/- mu1(t); black tick: body centre (both exaggerated)",
             (0, -0.6, -1.0), 0.13, mats["text"])

    for f, k in enumerate(idx, start=1):
        zr = E * data["zr"][k]
        mu = E * data["mu1"][k]
        for rg in rigs:
            zs = E * data["zs_" + rg["tag"]][k]
            zu = E * data["zu_" + rg["tag"]][k]
            x0 = rg["x0"]
            rg["road"].location = (x0, 0, -0.15 + zr); key(rg["road"], "location", f)
            for name in ("tire", "hub"):
                rg[name].location = (x0, 0, R_TIRE + zu); key(rg[name], "location", f)
            rg["body"].location = (x0, 0, BODY_Z0 + zs); key(rg["body"], "location", f)
            bottom = R_TIRE + 0.06 + zu
            top = BODY_Z0 - 0.19 + zs
            rg["spring"].location = (x0 - 0.3, 0, bottom); key(rg["spring"], "location", f)
            rg["spring"].scale = (1, 1, max(top - bottom, 0.05)); key(rg["spring"], "scale", f)
            for lo, hi, dx in ((rg["dmp_lo"], rg["dmp_hi"], 0.0), (rg["act_lo"], rg["act_hi"], 0.3)):
                lo.location = (x0 + dx, 0, R_TIRE + 0.27 + zu); key(lo, "location", f)
                hi.location = (x0 + dx, 0, BODY_Z0 - 0.44 + zs); key(hi, "location", f)
            for s, b in zip((1, -1), rg["bounds"]):
                b.location = (x0 + 0.95, -0.3, BODY_Z0 + s * mu); key(b, "location", f)

    texts = [f"t = {t[k]:5.2f} s   road {1e3 * data['zr'][k]:+5.1f} mm   "
             f"u(AFC) {data['u_afc'][k]:+4.1f} V   u(AFC-AW) {data['u_afc_aw'][k]:+4.1f} V   "
             f"(motion x{E:g}, {OPTS['slow']:g}x slow)" for k in idx]

    def update_info(scene, *_):
        i = min(max(scene.frame_current - 1, 0), len(texts) - 1)
        info.data.body = texts[i]

    bpy.app.handlers.frame_change_pre.clear()
    bpy.app.handlers.frame_change_pre.append(update_info)
    update_info(sc)

    # camera and light
    cam_data = bpy.data.cameras.new("Cam")
    cam_data.type = "ORTHO"
    cam_data.ortho_scale = 3 * SPACING + 0.6
    cam = bpy.data.objects.new("Camera", cam_data)
    bpy.context.collection.objects.link(cam)
    cam.location = (0, -12, 0.95)
    cam.rotation_euler = (math.pi / 2, 0, 0)
    sc.camera = cam
    sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", "SUN"))
    bpy.context.collection.objects.link(sun)
    sun.rotation_euler = (math.radians(55), 0, math.radians(-25))
    sun.data.energy = 3.0
    if sc.world is None:
        sc.world = bpy.data.worlds.new("World")
    sc.world.color = (1, 1, 1)
    if sc.world.use_nodes and "Background" in sc.world.node_tree.nodes:
        sc.world.node_tree.nodes["Background"].inputs[0].default_value = (1, 1, 1, 1)
        sc.world.node_tree.nodes["Background"].inputs[1].default_value = 0.6
    try:
        sc.view_settings.view_transform = "Standard"
    except TypeError:
        pass

    # render settings
    engines = [e.identifier for e in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items]
    eng = OPTS["engine"] or ("CYCLES" if bpy.app.background else
                             ("BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in engines else "BLENDER_EEVEE"))
    sc.render.engine = eng
    if eng == "CYCLES":
        sc.cycles.samples = int(OPTS.get("samples", 12))
        sc.cycles.device = "CPU"
        if hasattr(sc.cycles, "use_denoising"):
            sc.cycles.use_denoising = False
    sc.render.resolution_x = 1280
    sc.render.resolution_y = 720
    sc.render.film_transparent = False
    return sc


def render(sc):
    if OPTS["still"]:
        sc.frame_set(max(1, sc.frame_end // 3))
        sc.render.image_settings.file_format = "PNG"
        sc.render.filepath = OPTS["still"]
        bpy.ops.render.render(write_still=True)
    if OPTS["out"]:
        sc.render.image_settings.file_format = "FFMPEG"
        sc.render.ffmpeg.format = "MPEG4"
        sc.render.ffmpeg.codec = "H264"
        sc.render.ffmpeg.constant_rate_factor = "MEDIUM"
        sc.render.filepath = OPTS["out"]
        bpy.ops.render.render(animation=True)


scene = build()
render(scene)
print("quarter-car scene ready:", scene.frame_end, "frames")
