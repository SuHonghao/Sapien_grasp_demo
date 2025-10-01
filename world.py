# world.py

from __future__ import annotations
import numpy as _np
import sapien.core as sapien

def create_world(timestep: float = 1 / 100.0,
                 with_viewer: bool = True,
                 ambient=(0.5, 0.5, 0.5),
                 sun_dir=(0, 1, -1),
                 sun_rgb=(0.5, 0.5, 0.5)):
    """兜底世界创建器（带实体地板）"""
    scene = sapien.Scene()
    scene.set_timestep(timestep)

    # --- 实体地面 (10x10 m, 厚度0.1m) ---
    builder = scene.create_actor_builder()
    half_size = [5.0, 5.0, 0.05]
    phys_mat = scene.create_physical_material(0.8, 0.8, 0.0)
    builder.add_box_collision(half_size=half_size, material=phys_mat)
    builder.add_box_visual(half_size=half_size)
    ground = builder.build_static(name="ground")
    ground.set_pose(sapien.Pose([0, 0, -0.05]))  # 顶面在 z=0

    # --- 光照 ---
    scene.set_ambient_light(list(ambient))
    scene.add_directional_light(list(sun_dir), list(sun_rgb))

    # --- Viewer ---
    viewer = None
    if with_viewer:
        viewer = scene.create_viewer()
        viewer.set_camera_xyz(x=-2.0, y=0.0, z=1.2)
        viewer.set_camera_rpy(r=0, p=-_np.arctan2(1.2, 2.0), y=0)
        try:
            viewer.window.set_camera_parameters(near=0.02, far=200, fovy=1.0)
        except Exception:
            pass
    return scene, viewer
