# physx_utils.py

from __future__ import annotations
import numpy as np

def setup_physx_defaults(gravity_z: float = -9.8,
                         static_mu: float = 0.3,
                         dynamic_mu: float = 0.8,
                         restitution: float = 0.0):
    """设置全局 PhysX 默认参数（跨版本做 try/except 保守处理）"""
    try:
        import importlib
        _sapien = importlib.import_module("sapien")
        scene_config = _sapien.physx.PhysxSceneConfig()
        scene_config.gravity = np.array([0.0, 0.0, gravity_z], dtype=np.float32)
        _sapien.physx.set_scene_config(scene_config)
        _sapien.physx.set_default_material(static_friction=static_mu,
                                           dynamic_friction=dynamic_mu,
                                           restitution=restitution)
    except Exception as e:
        print("[WARN] setup_physx_defaults failed:", e)

def _get_rigidbody_component(entity):
    try:
        import importlib
        _sapien = importlib.import_module("sapien")
        return entity.find_component_by_type(_sapien.physx.PhysxRigidBodyComponent)
    except Exception:
        return None

def set_damping_if_dynamic(entity, linear_damping: float = 0.0, angular_damping: float = 0.0):
    rb = _get_rigidbody_component(entity)
    if rb is not None:
        try:
            rb.set_linear_damping(float(linear_damping))
            rb.set_angular_damping(float(angular_damping))
        except Exception:
            pass
