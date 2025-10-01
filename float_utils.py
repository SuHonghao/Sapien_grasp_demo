# float_utils.py

import sapien.core as sapien


def make_float(entity, height: float = None):
    """
    让已有的 Actor 或 Articulation 悬浮（禁用重力）。
    
    参数
    ----
    entity : SAPIEN 对象
        可以是 Actor (PhysxRigidDynamicActor) 或 Articulation (PhysxArticulation)
    height : float, optional
        指定悬浮高度（z 坐标）。如果为 None，则保持原高度。
    """

    # --- 情况 1: Actor ---
    if hasattr(entity, "find_component_by_type"):
        rigid = entity.find_component_by_type(sapien.physx.PhysxRigidBodyComponent)
        if rigid is None:
            print(f"[WARN] {entity.get_name()} 没有刚体组件，无法设置为悬浮。")
            return
        rigid.set_disable_gravity(True)

        pose = entity.get_pose()
        pos, quat = pose.p, pose.q
        if height is not None:
            pos[2] = height
            entity.set_pose(sapien.Pose(pos, quat))


    # --- 情况 2: Articulation ---
    elif hasattr(entity, "get_links"):
        root_link = entity.get_links()[0]  # PhysxArticulationLinkComponent
        # 直接禁用重力
        root_link.set_disable_gravity(True)

        pose = entity.get_root_pose()
        pos, quat = pose.p, pose.q
        if height is not None:
            pos[2] = height
            entity.set_root_pose(sapien.Pose(pos, quat))

    else:
        print(f"[ERROR] 不支持的类型: {type(entity)}")
