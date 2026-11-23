"""Retarget Mixamo poses onto another armature.

Three corrections, chosen from the Mixamo bone name:

  track
      Upper arm, forearm, hand. The bone points the same way Mixamo's
      does. Only an X/Z roll flip in the rest pose is removed. An A-pose
      is not added on top of a T-pose, so the arm angle stays accurate.

  offset
      Hips and shoulders. The target keeps its own rest direction.
      Only Mixamo's change from his rest is applied. Shoulders use
      this, so a T-pose source does not lift an A-pose shoulder.
      Thighs, calves, and feet track. Adding the rest stance on top
      of a lifted run thigh folded the foot into the chest.

  spine
      Spine, neck, head. Same aim rule as track. The spine is planted
      on the hip's aim, not on the hip bone's raw axis, so a hip that
      points forward does not pitch the chest.

Split deform bones that are not parented to each other (Rigify DEF
bones hanging off ORG) are chained rest-tail to rest-head. Other deform
bones, such as fingers, keep their rest curl and ride the mapped bone
above them.

  blender --background --python tools/mixamo_retarget.py -- \\
      --target models/characters/grapple_girl.blend \\
      --map tools/retarget_maps/grapple_girl.json \\
      --rest \\
      --out /tmp/girl_rest.png

  blender --background --python tools/mixamo_retarget.py -- \\
      --target models/characters/grapple_girl.blend \\
      --map tools/retarget_maps/grapple_girl.json \\
      --export /tmp/grapple_girl.glb \\
      --clip path/movement.blend\\|Idle Active\\|FightStance
"""
import argparse
import json
import math
import os
import sys

import bmesh
import bpy
from mathutils import Matrix, Vector

MIXAMO = "/workspace/games/snaiel-combat/animations/movement_animations_1/movement_animations_1.blend"

_prepared = set()
_prev_quat = {}


def role_of(mixamo_name):
    name = mixamo_name.split(":")[-1]
    if name == "Hips" or name.endswith("Shoulder"):
        return "offset"
    if name.startswith("Spine") or name in ("Neck", "Head"):
        return "spine"
    return "track"


def ang(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, a.normalized().dot(b.normalized())))))


def from_down(v):
    return ang(v, Vector((0, 0, -1)))


def rest_roll(src_rest, dst_rest):
    """Degrees around local +Y that turns Mixamo's rest X toward the target's."""
    rel = src_rest.inverted() @ dst_rest
    y = Vector((0, 1, 0))
    gx = rel @ Vector((1, 0, 0))
    plane = gx - y * gx.dot(y)
    if plane.length < 1e-8:
        return 0.0
    plane.normalize()
    x = Vector((1, 0, 0))
    return math.degrees(math.atan2(x.cross(plane).dot(y), x.dot(plane)))


def rolled(src_pose, src_rest, dst_rest):
    roll = rest_roll(src_rest, dst_rest)
    return src_pose @ Matrix.Rotation(math.radians(roll), 3, "Y")


def aligned(src_pose, src_rest, dst_rest):
    return src_pose @ src_rest.inverted() @ dst_rest


def update():
    bpy.context.view_layer.update()


def load_map(path):
    with open(path) as fh:
        data = json.load(fh)
    return data["armature"], data["mesh"], data["bones"]


def load_mixamo(action, frame, use_rest, source=None):
    bpy.ops.wm.open_mainfile(filepath=source or MIXAMO)
    arm = bpy.data.objects["Armature"]
    if use_rest:
        if arm.animation_data:
            arm.animation_data_clear()
        arm.data.pose_position = "POSE"
        for pb in arm.pose.bones:
            pb.rotation_mode = "QUATERNION"
            pb.rotation_quaternion = (1, 0, 0, 0)
            pb.location = (0, 0, 0)
            pb.scale = (1, 1, 1)
    else:
        act = bpy.data.actions[action]
        ad = arm.animation_data_create()
        ad.action = act
        if getattr(act, "slots", None) and act.slots:
            ad.action_slot = act.slots[0]
        bpy.context.scene.frame_set(frame)
    update()
    mats = {b.name: arm.pose.bones[b.name].matrix.copy() for b in arm.data.bones}
    rests = {b.name: b.matrix_local.to_3x3() for b in arm.data.bones}
    return arm, mats, rests


def mute(arm):
    arm.data.pose_position = "POSE"
    if arm.animation_data:
        arm.animation_data_clear()
    for pb in arm.pose.bones:
        while pb.constraints:
            pb.constraints.remove(pb.constraints[0])
    update()


def disconnect(arm, names):
    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.object.select_all(action="DESELECT")
    arm.select_set(True)
    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode="EDIT")
    for name in names:
        bone = arm.data.edit_bones.get(name)
        if bone is not None:
            bone.use_connect = False
    bpy.ops.object.mode_set(mode="POSE")


def bone_length(bone):
    return (bone.tail_local - bone.head_local).length


def climb_mapped(arm, name, mapping):
    """Nearest mapped bone above this one, including a Rigify ORG to DEF twin."""
    parent = arm.data.bones[name].parent
    while parent is not None:
        if parent.name in mapping:
            return parent.name
        if parent.name.startswith("ORG-"):
            twin = "DEF-" + parent.name[4:]
            if twin in mapping:
                return twin
        parent = parent.parent
    return None


def chain_anchor(arm, name, mapping):
    """Bone this one should pivot from.

    A mapped parent wins, so a normal FK chain is unchanged. Otherwise
    the mapped bone whose rest tail meets this head, which is how split
    DEF twist bones connect. Otherwise the ORG twin above.
    """
    bone = arm.data.bones[name]
    parent = bone.parent
    if parent is not None and parent.name in mapping:
        return parent.name
    best = None
    best_d = 1e9
    for other in mapping:
        if other == name:
            continue
        other_bone = arm.data.bones[other]
        dist = (other_bone.tail_local - bone.head_local).length
        limit = max(0.025, 0.5 * max(bone_length(other_bone), 1e-8))
        if dist <= limit and dist < best_d:
            best, best_d = other, dist
    if best is not None:
        return best
    return climb_mapped(arm, name, mapping)


def place_order(anchors):
    pending = dict(anchors)
    order = []
    while pending:
        ready = [name for name, anchor in pending.items() if anchor not in pending]
        if not ready:
            name = sorted(pending)[0]
            pending[name] = None
            continue
        for name in sorted(ready):
            order.append(name)
            del pending[name]
    return order


def ensure_prepared(arm):
    """Mute constraints and disconnect deform bones once per armature."""
    if arm.as_pointer() in _prepared:
        return
    mute(arm)
    disconnect(arm, [bone.name for bone in arm.data.bones if bone.use_deform])
    _prepared.add(arm.as_pointer())


def apply_pose(arm, mapping, mats, rests, quiet=False, key_frame=None):
    """Pose arm. mapping is target bone -> Mixamo bone, parents before children."""
    ensure_prepared(arm)
    anchors = {name: chain_anchor(arm, name, mapping) for name in mapping}
    order = place_order(anchors)
    hip_name = next((n for n in order if mapping[n].endswith(":Hips") or mapping[n].endswith("Hips")), None)
    report = []
    if not quiet:
        print("anchors:")
        for name in order:
            print(f"  {name} -> {anchors[name]}")

    def rotation(name):
        src = mapping[name]
        src_pose = mats[src].to_3x3()
        src_rest = rests[src]
        dst_rest = arm.pose.bones[name].bone.matrix_local.to_3x3()
        kind = role_of(src)
        if kind == "offset":
            rot = aligned(src_pose, src_rest, dst_rest)
        else:
            rot = rolled(src_pose, src_rest, dst_rest)
        return kind, rot, src, src_pose, src_rest, dst_rest

    hip_pb = None
    hip_aim = None
    for name in order:
        pb = arm.pose.bones[name]
        kind, rot, src, src_pose, src_rest, dst_rest = rotation(name)
        anchor_name = anchors[name]
        use_hip_aim = (
            hip_pb is not None
            and anchor_name == hip_name
            and kind == "spine"
            and hip_aim is not None
        )
        if use_hip_aim:
            basis = Matrix.LocRotScale(hip_pb.head.copy(), hip_aim.to_quaternion(), Vector((1, 1, 1)))
            head = basis @ (hip_pb.bone.matrix_local.inverted() @ pb.bone.head_local)
        elif anchor_name is not None:
            anchor_pb = arm.pose.bones[anchor_name]
            head = anchor_pb.matrix @ (anchor_pb.bone.matrix_local.inverted() @ pb.bone.head_local)
        elif pb.parent is not None:
            head = pb.parent.matrix @ (pb.parent.bone.matrix_local.inverted() @ pb.bone.head_local)
        else:
            head = pb.bone.head_local.copy()
        pb.rotation_mode = "QUATERNION"
        pb.matrix = Matrix.LocRotScale(head, rot.to_quaternion(), Vector((1, 1, 1)))
        update()
        if name == hip_name:
            hip_pb = pb
            hip_aim = rolled(src_pose, src_rest, dst_rest)
        report.append((name, kind, src, src_pose, dst_rest, src_rest))

    rigid = 0
    keyed = [arm.pose.bones[name] for name in order]
    for bone in arm.data.bones:
        if bone.name in mapping or not bone.use_deform:
            continue
        anchor_name = climb_mapped(arm, bone.name, mapping)
        if anchor_name is None:
            continue
        anchor_pb = arm.pose.bones[anchor_name]
        delta = anchor_pb.matrix.to_3x3() @ anchor_pb.bone.matrix_local.to_3x3().inverted()
        rest = bone.matrix_local.to_3x3()
        head = anchor_pb.matrix @ (anchor_pb.bone.matrix_local.inverted() @ bone.head_local)
        pb = arm.pose.bones[bone.name]
        pb.rotation_mode = "QUATERNION"
        pb.matrix = Matrix.LocRotScale(head, (delta @ rest).to_quaternion(), Vector((1, 1, 1)))
        update()
        keyed.append(pb)
        rigid += 1
    if not quiet:
        print(f"rigid follow {rigid} deform bones")
    if key_frame is not None:
        prev = _prev_quat.setdefault(arm.as_pointer(), {})
        for pb in keyed:
            quat = pb.rotation_quaternion.copy()
            if pb.name in prev and quat.dot(prev[pb.name]) < 0.0:
                quat.negate()
                pb.rotation_quaternion = quat
            prev[pb.name] = quat.copy()
            pb.keyframe_insert(data_path="location", frame=key_frame)
            pb.keyframe_insert(data_path="rotation_quaternion", frame=key_frame)
            pb.keyframe_insert(data_path="scale", frame=key_frame)
    return report


def print_report(arm, report):
    print(f"{'bone':24} {'role':7} {'roll':7} {'restY':7} {'posed':7} {'mixamo':7} {'hx':7} {'hy':7} {'hz':7}")
    for name, kind, src, src_pose, dst_rest, src_rest in report:
        roll = rest_roll(src_rest, dst_rest)
        rest_y = ang(src_rest.col[1], dst_rest.col[1])
        posed = from_down(arm.pose.bones[name].y_axis)
        source = from_down(src_pose.col[1])
        head = arm.pose.bones[name].head
        print(f"{name:24} {kind:7} {roll:7.1f} {rest_y:7.1f} {posed:7.1f} {source:7.1f} {head.x:7.2f} {head.y:7.2f} {head.z:7.2f}")


def sticks(arm):
    height = 1.8
    radius = height * 0.012
    groups = {"C": [], "L": [], "R": []}
    for bone in arm.data.bones:
        if not bone.use_deform:
            continue
        pb = arm.pose.bones[bone.name]
        key = "C"
        low = bone.name.lower()
        if "left" in low or low.endswith(".l") or ".l." in low:
            key = "L"
        elif "right" in low or low.endswith(".r") or ".r." in low:
            key = "R"
        groups[key].append((pb.head.copy(), pb.tail.copy()))
    colors = {"C": (0.92, 0.90, 0.84, 1), "L": (0.25, 0.75, 0.95, 1), "R": (0.95, 0.45, 0.25, 1)}
    objects = []
    for key, segs in groups.items():
        bm = bmesh.new()
        for head, tail in segs:
            direction = tail - head
            if direction.length < 0.01:
                continue
            direction.normalize()
            created = bmesh.ops.create_cone(
                bm, cap_ends=True, segments=8,
                radius1=radius, radius2=radius * 0.45, depth=(tail - head).length,
            )
            rot = Vector((0, 0, 1)).rotation_difference(direction).to_matrix().to_4x4()
            mid = (head + tail) * 0.5
            bmesh.ops.transform(bm, verts=created["verts"], matrix=Matrix.Translation(mid) @ rot)
        me = bpy.data.meshes.new("Skel" + key)
        bm.to_mesh(me)
        bm.free()
        obj = bpy.data.objects.new("Skel" + key, me)
        bpy.context.scene.collection.objects.link(obj)
        mat = bpy.data.materials.new("Bone" + key)
        mat.use_nodes = True
        bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        bsdf.inputs["Base Color"].default_value = colors[key]
        bsdf.inputs["Emission Color"].default_value = colors[key]
        bsdf.inputs["Emission Strength"].default_value = 0.45
        obj.data.materials.append(mat)
        objects.append(obj)
    return objects


def shoot(path, objects, view="front"):
    pts = []
    for obj in objects:
        for vert in obj.data.vertices:
            pts.append(obj.matrix_world @ vert.co)
    scene = bpy.context.scene
    world = scene.world or bpy.data.worlds.new("World")
    scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background") or world.node_tree.nodes.new("ShaderNodeBackground")
    bg.inputs[0].default_value = (0.22, 0.23, 0.25, 1)
    bg.inputs[1].default_value = 1.0
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 32
    scene.cycles.use_denoising = True
    scene.view_settings.view_transform = "Standard"
    scene.view_settings.exposure = 1.2
    scene.render.resolution_percentage = 100
    sun = bpy.data.lights.new("Key", "SUN")
    sun.energy = 5.0
    key = bpy.data.objects.new("Key", sun)
    key.rotation_euler = (0.9, 0.2, 0.6)
    scene.collection.objects.link(key)
    fill = bpy.data.lights.new("Fill", "SUN")
    fill.energy = 2.5
    fill_obj = bpy.data.objects.new("Fill", fill)
    fill_obj.rotation_euler = (1.4, 0.0, 3.1)
    scene.collection.objects.link(fill_obj)
    cam_data = bpy.data.cameras.new("Cam")
    cam_data.type = "ORTHO"
    cam = bpy.data.objects.new("Cam", cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam
    minx, maxx = min(p.x for p in pts), max(p.x for p in pts)
    miny, maxy = min(p.y for p in pts), max(p.y for p in pts)
    minz, maxz = min(p.z for p in pts), max(p.z for p in pts)
    span = max(maxx - minx, maxz - minz, maxy - miny, 0.4)
    cam_data.ortho_scale = span * 1.25
    if view == "side":
        center = Vector((minx, (miny + maxy) * 0.5, (minz + maxz) * 0.5))
        cam.location = Vector((minx - span * 4.0, center.y, center.z))
    else:
        center = Vector(((minx + maxx) * 0.5, miny, (minz + maxz) * 0.5))
        cam.location = Vector((center.x, miny - span * 4.0, center.z))
    cam.rotation_euler = (center - cam.location).to_track_quat("-Z", "Y").to_euler()
    scene.render.resolution_x = 900
    scene.render.resolution_y = 1100
    scene.render.filepath = path
    bpy.ops.render.render(write_still=True)
    print("SHOT", path, os.path.getsize(path))


def evaluated_mesh(name):
    src = bpy.data.objects[name]
    for mod in src.modifiers:
        if mod.type == "ARMATURE" and mod.object is None:
            mod.show_viewport = False
            mod.show_render = False
    update()
    deps = bpy.context.evaluated_depsgraph_get()
    mesh = bpy.data.meshes.new_from_object(src.evaluated_get(deps), depsgraph=deps)
    vis = bpy.data.objects.new(name + "_vis", mesh)
    bpy.context.scene.collection.objects.link(vis)
    for mat in src.data.materials:
        vis.data.materials.append(mat)
    color = next((im for im in bpy.data.images if "color" in im.name.lower() and im.size[0]), None)
    if color is not None:
        mat = bpy.data.materials.new("RetargetColor")
        mat.use_nodes = True
        bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        tex = mat.node_tree.nodes.new("ShaderNodeTexImage")
        tex.image = color
        mat.node_tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
        vis.data.materials.clear()
        vis.data.materials.append(mat)
    bm = bmesh.new()
    bm.from_mesh(vis.data)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(vis.data)
    bm.free()
    pts = [vis.matrix_world @ Vector(c) for c in vis.bound_box]
    height = max(p.z for p in pts) - min(p.z for p in pts)
    vis.scale = (1.75 / max(height, 1e-6),) * 3
    update()
    vis.data.transform(vis.matrix_world)
    vis.matrix_world = Matrix.Identity(4)
    update()
    pts = [vis.matrix_world @ Vector(c) for c in vis.bound_box]
    shift = Vector((
        (min(p.x for p in pts) + max(p.x for p in pts)) * 0.5,
        (min(p.y for p in pts) + max(p.y for p in pts)) * 0.5,
        min(p.z for p in pts),
    ))
    for vert in vis.data.vertices:
        vert.co -= shift
    vis.data.update()
    return vis


def sample_clip(source, action_name):
    """Every frame of one Mixamo action, as armature-space matrices."""
    bpy.ops.wm.open_mainfile(filepath=source)
    arm = bpy.data.objects["Armature"]
    act = bpy.data.actions[action_name]
    start = int(round(act.frame_range[0]))
    end = int(round(act.frame_range[1]))
    limit = int(os.environ.get("RETARGET_MAX_FRAMES", "0"))
    if limit > 0:
        end = min(end, start + limit - 1)
    ad = arm.animation_data_create()
    ad.action = act
    if getattr(act, "slots", None) and act.slots:
        ad.action_slot = act.slots[0]
    scene = bpy.context.scene
    frames = []
    for frame in range(start, end + 1):
        scene.frame_set(frame)
        update()
        frames.append({b.name: arm.pose.bones[b.name].matrix.copy() for b in arm.data.bones})
    rests = {b.name: b.matrix_local.to_3x3() for b in arm.data.bones}
    print(f"SAMPLED {action_name} {len(frames)} frames from {source}")
    return frames, rests


def new_action(arm, name):
    action = bpy.data.actions.new(name)
    ad = arm.animation_data_create()
    ad.action = action
    if hasattr(action, "slots") and hasattr(ad, "action_slot"):
        slot = action.slots.new(arm.id_type, name)
        ad.action_slot = slot
    _prev_quat.pop(arm.as_pointer(), None)
    return action


def link_unconnected_color(mesh_obj):
    """Demon materials ship with an unused color map. Connect it if needed."""
    if not mesh_obj.data.materials:
        return
    mat = mesh_obj.data.materials[0]
    if mat is None or not mat.use_nodes:
        return
    bsdf = next((n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
    if bsdf is None:
        return
    base = bsdf.inputs["Base Color"]
    if base.is_linked:
        return
    color = next((im for im in bpy.data.images if "color" in im.name.lower() and im.size[0]), None)
    if color is None:
        return
    tex = mat.node_tree.nodes.new("ShaderNodeTexImage")
    tex.image = color
    mat.node_tree.links.new(tex.outputs["Color"], base)


def hand_bone(mapping):
    for target, source in mapping.items():
        if source.endswith("LeftHand"):
            return target
    return None


def export_glb(arm, mesh_obj, path):
    for obj in bpy.context.scene.objects:
        obj.select_set(False)
    arm.select_set(True)
    mesh_obj.select_set(True)
    bpy.context.view_layer.objects.active = arm
    pts = [mesh_obj.matrix_world @ Vector(corner) for corner in mesh_obj.bound_box]
    print(
        "BOUNDS",
        "x", round(min(p.x for p in pts), 3), round(max(p.x for p in pts), 3),
        "y", round(min(p.y for p in pts), 3), round(max(p.y for p in pts), 3),
        "z", round(min(p.z for p in pts), 3), round(max(p.z for p in pts), 3),
    )
    bpy.ops.export_scene.gltf(
        filepath=path,
        export_format="GLB",
        use_selection=True,
        export_animations=True,
        export_animation_mode="ACTIONS",
        export_skins=True,
        export_yup=True,
        export_apply=False,
    )
    print("EXPORT", path, os.path.getsize(path))


def bake_export(args, mapping):
    clips = []
    for spec in args.clip:
        source, action_name, export_name = spec.split("|")
        frames, rests = sample_clip(source, action_name)
        clips.append((export_name, frames, rests))

    global _prepared
    _prepared = set()
    bpy.ops.wm.open_mainfile(filepath=args.target)
    arm = bpy.data.objects[args.arm_name]
    mesh_obj = bpy.data.objects[args.mesh_name]
    for act in list(bpy.data.actions):
        bpy.data.actions.remove(act)
    for obj in list(bpy.data.objects):
        if obj is not arm and obj is not mesh_obj:
            bpy.data.objects.remove(obj, do_unlink=True)
    for mod in mesh_obj.modifiers:
        if mod.type == "ARMATURE" and mod.object is None:
            mod.show_viewport = False
            mod.show_render = False
        if mod.type in ("SUBSURF", "EDGE_SPLIT", "MIRROR"):
            mod.show_viewport = False
            mod.show_render = False
    world = mesh_obj.matrix_world.copy()
    mesh_obj.parent = arm
    mesh_obj.matrix_world = world
    link_unconnected_color(mesh_obj)
    bpy.context.scene.render.fps = 30
    bpy.context.preferences.edit.keyframe_new_interpolation_type = "LINEAR"
    ensure_prepared(arm)
    hand = hand_bone(mapping)
    for export_name, frames, rests in clips:
        action = new_action(arm, export_name)
        print(f"BAKE {export_name} {len(frames)} frames")
        for index, mats in enumerate(frames):
            frame = index + 1
            apply_pose(arm, mapping, mats, rests, quiet=True, key_frame=frame)
            if hand and export_name == "Punch" and index % 5 == 0:
                head = arm.pose.bones[hand].head
                print(f"PUNCH {frame} hand {head.x:.3f} {head.y:.3f} {head.z:.3f}")
        if hasattr(action, "frame_start"):
            action.frame_start = 1
            action.frame_end = len(frames)
        for layer in getattr(action, "layers", []):
            for strip in layer.strips:
                bags = getattr(strip, "channelbags", [])
                for bag in bags:
                    for curve in bag.fcurves:
                        for key in curve.keyframe_points:
                            key.interpolation = "LINEAR"
        print(f"BAKED {export_name} keys")
    export_glb(arm, mesh_obj, args.export)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Retarget a Mixamo pose or bake clips")
    parser.add_argument("--target", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--action", default="Idle Inactive")
    parser.add_argument("--frame", type=int, default=0)
    parser.add_argument("--rest", action="store_true")
    parser.add_argument("--out", default="")
    parser.add_argument("--side", action="store_true")
    parser.add_argument("--report-only", action="store_true")
    parser.add_argument("--export", default="", help="GLB path. Requires one or more --clip.")
    parser.add_argument("--clip", action="append", default=[], help="source.blend|Action Name|ExportName")
    args = parser.parse_args(argv)
    if not args.export and not args.out and not args.report_only:
        parser.error("--out is required unless --export or --report-only is set")
    if args.export and not args.clip:
        parser.error("--export needs at least one --clip")
    return args


def main(argv):
    args = parse_args(argv)
    arm_name, mesh_name, mapping = load_map(args.map)
    args.arm_name = arm_name
    args.mesh_name = mesh_name
    if args.export:
        bake_export(args, mapping)
        return
    src, mats, rests = load_mixamo(args.action, args.frame, args.rest)
    if not args.report_only:
        stick_objs = sticks(src)
        for obj in bpy.context.scene.objects:
            if obj not in stick_objs and obj.type != "LIGHT":
                obj.hide_render = True
        root, ext = os.path.splitext(args.out)
        shoot(root + "_mixamo" + ext, stick_objs)
        if args.side:
            shoot(root + "_mixamo_side" + ext, stick_objs, view="side")

    global _prepared
    _prepared = set()
    bpy.ops.wm.open_mainfile(filepath=args.target)
    arm = bpy.data.objects[arm_name]
    report = apply_pose(arm, mapping, mats, rests)
    print_report(arm, report)
    if args.report_only:
        return
    vis = evaluated_mesh(mesh_name)
    for obj in bpy.context.scene.objects:
        obj.hide_render = obj is not vis
    shoot(args.out, [vis])
    if args.side:
        root, ext = os.path.splitext(args.out)
        shoot(root + "_side" + ext, [vis], view="side")


if __name__ == "__main__":
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []
    main(argv)
