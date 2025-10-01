# test_main.py — 使用 Gripper + 力反馈 (每个 proposal 重建场景)
from __future__ import annotations
import argparse
import numpy as np
import sapien.core as sapien
import trimesh
import yaml
import os

# === 自定义模块 ===
from load_glb import load_my_object
from math_utils import world_to_object
from float_utils import make_float
from world import create_world
from physx_utils import setup_physx_defaults, set_damping_if_dynamic
from gripper_demo import Gripper

# transforms3d
from transforms3d.quaternions import axangle2quat, qmult, qinverse

# === 参数 ===
OFFSET = 0.5
MOVE_SPEED = 0.25
ROT_SPEED = 1.0
SCALE_OBJ = 1
TASK_FILE = "grasp/task/task.yml"


# ------------------- 计算抓取位姿 -------------------
def compute_pose_in_obj(gripper, robot, actor):
    """计算 TCP 和 gripper 姿态在物体系下的表达"""
    tcp_world = gripper.get_tcp_between_fingers()
    obj_pose = actor.get_pose()
    tcp_in_obj = world_to_object(obj_pose, tcp_world)

    # root pose 四元数 (xyzw → wxyz)
    quat_xyzw = robot.get_root_pose().q
    quat_wxyz = np.array([quat_xyzw[3], *quat_xyzw[:3]])

    # 去掉 90° 补偿
    rot_local_z90 = axangle2quat([0, 0, 1], np.deg2rad(90))
    quat_wxyz_no90 = qmult(quat_wxyz, qinverse(rot_local_z90))

    # 转到物体系
    obj_q_xyzw = obj_pose.q
    obj_q_wxyz = np.array([obj_q_xyzw[3], *obj_q_xyzw[:3]])
    quat_in_obj = qmult(qinverse(obj_q_wxyz), quat_wxyz_no90)

    return tcp_in_obj, quat_in_obj


# ------------------- Panda 手爪加载 -------------------
def setup_robot(scene, tcp_world, quat_new):
    """加载 Panda 手爪并放置到 proposal 位置"""
    urdf_loader = scene.create_urdf_loader()
    urdf_loader.fix_root_link = False
    robot = urdf_loader.load("grasp/panda/panda_hand.urdf")

    make_float(robot, height=OFFSET)
    for link in robot.get_links():
        link.set_disable_gravity(True)

    # 初始位置
    robot.set_root_pose(sapien.Pose([0, 0, OFFSET], quat_new))

    # 平移 tcp_link 到 proposal
    tcp_link = [l for l in robot.get_links() if l.get_name() == "tcp"][0]
    delta = tcp_world + np.array([0, 0, OFFSET]) - tcp_link.get_entity_pose().p
    root_pose = robot.get_root_pose()
    root_pose.set_p(root_pose.p + delta)
    robot.set_root_pose(root_pose)

    return robot


# ------------------- 单个 proposal 测试 -------------------
def run_single_proposal(glb_path, proposal, with_viewer=True):
    tcp, quat, key = proposal

    # 创建新场景
    scene, viewer = create_world(with_viewer=with_viewer)
    setup_physx_defaults(gravity_z=-9.8, static_mu=0.3, dynamic_mu=0.8, restitution=0.3)

    trimesh.load(glb_path, force="mesh")
    base_pose = sapien.Pose([0, 0, OFFSET], [1, 0, 0, 0])
    actor = load_my_object(
        scene, glb_path,
        scale=(SCALE_OBJ,) * 3,
        pose=base_pose,
        build_dynamic=True,
        friction=10,
        restitution=0.3,
    )
    if actor:
        make_float(actor, height=OFFSET)
        set_damping_if_dynamic(actor, linear_damping=0.1, angular_damping=0.1)

    robot = setup_robot(scene, tcp, quat)
    gripper = Gripper(robot, scene)

    print(f"[INFO] ▶️ 开始测试 proposal {key}")

    # 夹取判定逻辑
    grabbed, true_count, fail_count = False, 0, 0
    required_frames, max_fail_frames = 10, 30

    while with_viewer and not viewer.closed:
        gripper.control("close", actor)
        scene.step()
        scene.update_render()
        viewer.render()

        status = gripper.is_grasping(actor)
        if status is True:
            true_count += 1
            fail_count = 0
            if true_count >= required_frames:
                print(f"[INFO] Proposal {key} ✅ 初步成功")
                grabbed = True
                break
        elif status is False:
            fail_count += 1
            true_count = 0
            if fail_count >= max_fail_frames:
                print(f"[INFO] Proposal {key} ❌ 失败（未夹住）")
                return key, {"best_key": key, "status": "failed", "tcp_in_obj": None, "gripper_quat_in_obj": None}
        else:
            true_count = fail_count = 0

    if not grabbed:
        return key, {"best_key": key, "status": "failed", "tcp_in_obj": None, "gripper_quat_in_obj": None}

    # 动作稳定性检测
    tcp_world = gripper.get_tcp_between_fingers()
    obj_center = actor.get_pose().p
    last_dist = np.linalg.norm(tcp_world - obj_center)
    motions = [(0, 0, 0.1), (0.1, 0, 0), (0, 0.1, 0)]
    threshold = 0.005

    for i, move in enumerate(motions):
        vx, vy, vz = move
        robot.set_root_linear_velocity([vx, vy, vz])
        for _ in range(200):
            gripper.control("stop")
            scene.step()
            scene.update_render()
            if with_viewer:
                viewer.render()
        robot.set_root_linear_velocity([0, 0, 0])

        tcp_world = gripper.get_tcp_between_fingers()
        obj_center = actor.get_pose().p
        curr_dist = np.linalg.norm(tcp_world - obj_center)
        delta = abs(curr_dist - last_dist)
        print(f"[INFO] Motion {i+1}: Δ={delta:.6f}")
        if delta > threshold:
            print(f"[INFO] Proposal {key} ❌ 滑动失败")
            return key, {"best_key": key, "status": "failed", "tcp_in_obj": None, "gripper_quat_in_obj": None}
        last_dist = curr_dist

    # 最终成功
    tcp_in_obj, quat_in_obj = compute_pose_in_obj(gripper, robot, actor)
    result = {
        "format": "isaac_grasp",
        "format_version": "1.0",
        "grasps": {
            key: {
                "confidence": 1.0,
                "position": [float(x) for x in tcp],
                "orientation": {"w": float(quat_in_obj[0]), "xyz": [float(quat_in_obj[1]), float(quat_in_obj[2]), float(quat_in_obj[3])]},
                "tcp_position": [float(x) for x in tcp_in_obj],
                "score": 0.0
            }
        },
        "ranking": [key]
    }
    print(f"[INFO] Proposal {key} ✅ 最终成功")
    return key, result


# ------------------- 主函数 -------------------
def main(cfg_path: str, glb_path: str, task_name: str | None, with_viewer=True):
    # 载入 proposals
    with open(cfg_path, "r", encoding="utf-8") as f:
        g = yaml.safe_load(f)

    proposals = []
    if g.get("format") == "isaac_grasp":
        for k in g["ranking"]:
            grasp = g["grasps"][k]
            tcp = np.array(grasp["tcp_position"], dtype=np.float32)
            quat = [grasp["orientation"]["w"], *grasp["orientation"]["xyz"]]
            quat = qmult(quat, axangle2quat([0, 0, 1], np.deg2rad(90)))
            proposals.append((tcp, quat, k))
    else:
        raise ValueError("目前只支持 isaac_grasp 格式")

    # 执行 proposals
    results = {}
    for idx, proposal in enumerate(proposals):
        key, result = run_single_proposal(glb_path, proposal, with_viewer=with_viewer)
        results[key] = result
        print(f"[INFO] Proposal {key} 完成 ({idx+1}/{len(proposals)})")

    # 保存结果
    if task_name:
        with open(TASK_FILE, "r", encoding="utf-8") as f:
            tasks = yaml.safe_load(f).get("tasks", {})
        parts = task_name.split(".")
        if len(parts) == 1:
            cfg_path = tasks[parts[0]]["config"]
        elif len(parts) == 2:
            cfg_path = tasks[parts[0]][parts[1]]["config"]
        else:
            raise ValueError(f"任务名称格式不正确: {task_name}")
        task_dir = os.path.dirname(cfg_path)
        out_path = os.path.join(task_dir, f"batch_res_{task_name}.yml")
        with open(out_path, "w", encoding="utf-8") as f:
            yaml.dump(results, f, sort_keys=False, allow_unicode=True)
        print(f"[INFO] 🚩 已保存到 {out_path}")


# ------------------- 程序入口 -------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, help="抓取配置文件路径")
    parser.add_argument("--glb", type=str, help="GLB 模型路径")
    parser.add_argument("--task", type=str, help="任务名称 (支持: bag / bag.001)")
    parser.add_argument("--all", action="store_true", help="运行所有任务")
    parser.add_argument("--viewer", action="store_true", help="是否启用可视化")
    args = parser.parse_args()

    with open(TASK_FILE, "r", encoding="utf-8") as f:
        tasks = yaml.safe_load(f).get("tasks", {})

    if args.all:
        # 跑所有任务
        all_jobs = []
        for tname, tval in tasks.items():
            if "config" in tval and "model" in tval:
                all_jobs.append((tname, tval["config"], tval["model"]))
            else:
                for sub, sub_cfg in tval.items():
                    task_name = f"{tname}.{sub}"
                    all_jobs.append((task_name, sub_cfg["config"], sub_cfg["model"]))
        for idx, (task_name, cfg, glb) in enumerate(all_jobs, 1):
            print(f"\n[PROGRESS] [{idx}/{len(all_jobs)}] {task_name}")
            main(cfg, glb, task_name, with_viewer=args.viewer)

    elif args.task:
        # 跑单个任务
        parts = args.task.split(".")
        if len(parts) == 1:
            sub_jobs = list(tasks[parts[0]].items())
            for idx, (sub, sub_cfg) in enumerate(sub_jobs, 1):
                task_name = f"{parts[0]}.{sub}"
                print(f"\n[PROGRESS] [{idx}/{len(sub_jobs)}] {task_name}")
                main(sub_cfg["config"], sub_cfg["model"], task_name, with_viewer=args.viewer)
        elif len(parts) == 2:
            task_cfg = tasks[parts[0]][parts[1]]
            print(f"\n[PROGRESS] [1/1] {args.task}")
            main(task_cfg["config"], task_cfg["model"], args.task, with_viewer=args.viewer)
        else:
            raise ValueError(f"任务名称格式不正确: {args.task}")

    else:
        # 自定义输入
        cfg = args.cfg
        glb = args.glb
        task_name = None
        print(f"\n[PROGRESS] [1/1] 自定义任务")
        main(cfg, glb, task_name, with_viewer=args.viewer)
