# vis_utils.py

from __future__ import annotations
import numpy as np
import sapien.core as sapien

def make_material(scene: "sapien.Scene", color=(1,0,0,1)):
    mat = None
    if hasattr(scene, "create_material"):
        try: mat = scene.create_material()
        except Exception: mat = None
    if mat is None:
        try:
            from sapien.core import render
            if hasattr(render, "RenderMaterial"):
                mat = render.RenderMaterial()
        except Exception:
            mat = None
    if mat is not None and hasattr(mat, "set_base_color"):
        try:   mat.set_base_color(color)
        except: mat.set_base_color(color[:3])
    if mat is not None and hasattr(mat, "set_unlit"):
        mat.set_unlit(True)
    return mat

def spawn_dot(scene: "sapien.Scene", position, radius: float = 0.01,
              color=(1,0,0,1), name="dot"):
    p = np.asarray(position, dtype=np.float32).reshape(3)
    pose = sapien.Pose(p, [1,0,0,0])
    mat  = make_material(scene, color)
    builder = scene.create_actor_builder()
    try:
        builder.add_sphere_visual(radius=radius, material=mat)
    except TypeError:
        builder.add_sphere_visual(radius)
    actor = builder.build_static(name=name)
    actor.set_pose(pose)
    return actor

def spawn_world_axes(scene: "sapien.Scene", length=0.3, thickness=0.005):
    def make_axis(color, half_size, pose, name):
        builder = scene.create_actor_builder()
        mat = make_material(scene, color=color)
        builder.add_box_visual(half_size=half_size, material=mat)
        actor = builder.build_static(name=name)
        actor.set_pose(pose)
        return actor

    half_x = [length/2, thickness/2, thickness/2]
    half_y = [thickness/2, length/2, thickness/2]
    half_z = [thickness/2, thickness/2, length/2]
    make_axis((1,0,0,1), half_x, sapien.Pose([length/2, 0, 0]), "axis_x")
    make_axis((0,1,0,1), half_y, sapien.Pose([0, length/2, 0]), "axis_y")
    make_axis((0,0,1,1), half_z, sapien.Pose([0, 0, length/2]), "axis_z")
