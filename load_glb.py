# load_glb.py

import numpy as np
import sapien.core as sapien


def load_my_object(
    scene: "sapien.Scene",
    model_path: str,
    *,
    scale=(1.0, 1.0, 1.0),
    density: float = 100.0,
    pose: "sapien.Pose" = None,
    name: str = "my_glb_model",
    add_collision: bool = True,
    build_dynamic: bool = False,   # True → 动态刚体；False → 静态刚体
    use_convex: bool = False,      # 动态刚体推荐用凸包
    friction: float = 1,         # 摩擦系数 (静/动一致)，np.inf → 1e6
    restitution: float = 0       # 弹性系数（反弹系数）
):
    """
    从 glb/gltf 文件加载模型 (SAPIEN 3.x)

    参数
    ----
    scene : sapien.Scene
        目标场景
    model_path : str
        GLB/GLTF 文件路径
    scale : tuple[float, float, float]
        缩放比例
    density : float
        碰撞体密度（仅对动态刚体有效）
    pose : sapien.Pose
        初始位姿（默认原点）
    name : str
        Actor 名称
    add_collision : bool
        是否添加碰撞体
    build_dynamic : bool
        True → 构建动态刚体；False → 构建静态刚体
    use_convex : bool
        动态刚体是否使用凸碰撞体
    friction : float
        摩擦系数（np.inf 会映射为 1e6）
    restitution : float
        弹性系数（0 = 完全无反弹）

    返回
    ----
    actor : sapien.Entity
        构建好的刚体
    """

    # ===== 基础设置 =====
    scale_np = np.array(scale, dtype=np.float32)
    builder = scene.create_actor_builder()

    # ===== 可视外观 =====
    builder.add_visual_from_file(model_path, scale=scale_np)

    # ===== 碰撞体 =====
    if add_collision:
        mu = 10 if friction == np.inf else float(friction)
        phys_mat = sapien.physx.PhysxMaterial(
            static_friction=mu,
            dynamic_friction=mu,
            restitution=float(restitution),
        )

        if build_dynamic:
            if use_convex:
                builder.add_convex_collision_from_file(
                    model_path, scale=scale_np, density=density, material=phys_mat
                )
                print(f"[INFO] 动态刚体：使用 CONVEX 碰撞体，摩擦={mu}, 弹性={restitution}")
            else:
                builder.add_nonconvex_collision_from_file(
                    model_path, scale=scale_np, density=density, material=phys_mat
                )
                print(f"[WARN] 动态刚体：使用 NONCONVEX 碰撞体，摩擦={mu}, 弹性={restitution}")
        else:
            builder.add_nonconvex_collision_from_file(
                model_path, scale=scale_np, density=density, material=phys_mat
            )
            print(f"[INFO] 静态刚体：使用 NONCONVEX 碰撞体，摩擦={mu}, 弹性={restitution}")

    # ===== 构建 Actor =====
    actor = builder.build(name=name) if build_dynamic else builder.build_static(name=name)

    # ===== 初始位姿 =====
    if pose is None:
        pose = sapien.Pose([0, 0, 0], [1, 0, 0, 0])
    actor.set_pose(pose)

    return actor
