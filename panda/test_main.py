# test_main.py — 使用 Gripper + 力反馈 (配置从 grasp.yml 读取)
from __future__ import annotations
import argparse
import numpy as np
import sapien.core as sapien
import trimesh
import yaml
import time
import os

# === 自定义模块 ===
from load_glb import load_my_object
from float_utils import make_float
from world import create_world
from math_utils import quat_wxyz_to_R
from vis_utils import spawn_dot, spawn_world_axes
from physx_utils import (
    setup_physx_defaults,
    zero_velocities_if_dynamic,
    set_damping_if_dynamic,
)
from gripper_demo import Gripper

# transforms3d
import transforms3d.quaternions as tq
from transforms3d.quaternions import axangle2quat, qmult, qinverse

# === 参数 ===
OPEN_WIDTH     = 1.0
CLOSE_WIDTH    = 0.0
DOT_RADIUS     = 0.012
SCALE_OBJ      = 1
WORLD_AXIS_LEN = 10
OFFSET         = 0.5
STEP_SIZE      = 0.2
MOVE_SPEED     = 0.25
ROT_SPEED      = 1.0

GRASP_GLB_PATH      = "grasp/task/teapot/teapot.glb"
GRASP_PROPOSAL_PATH = "grasp/task/teapot/teapot.yml"
TASK_FILE           = "grasp/task/task.yml"


# ------------------- 工具函数 -------------------
def load_task(task_name: str, task_file=TASK_FILE):
    with open(task_file, "r", encoding="utf-8") as f:
        tasks = yaml.safe_load(f)["tasks"]
    if task_name not in tasks:
        raise ValueError(f"Task {task_name} not found in {task_file}")
    return tasks[task_name]["config"], tasks[task_name]["model"]


def world_to_object(obj_pose: sapien.Pose, point_world: np.ndarray):
    q = [obj_pose.q[3], obj_pose.q[0], obj_pose.q[1], obj_pose.q[2]]  # [w,x,y,z]
    R = tq.quat2mat(q)
    t = obj_pose.p
    return R.T @ (point_world - t)


def save_result_yaml(task_name: str, tcp_in_obj, quat_in_obj, task_file=TASK_FILE):
    with open(task_file, "r", encoding="utf-8") as f:
        tasks = yaml.safe_load(f)["tasks"]
    if task_name not in tasks:
        raise ValueError(f"Task {task_name} not found in {task_file}")

    cfg_path = tasks[task_name]["config"]
    task_dir = os.path.dirname(cfg_path)
    out_path = os.path.join(task_dir, f"cor_res_{task_name}.yml")

    data = {
        "tcp_in_obj": [float(x) for x in tcp_in_obj],
        "gripper_quat_in_obj": [float(x) for x in quat_in_obj]
    }
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, sort_keys=False)

    print(f"[INFO] 保存结果到 {out_path}")


# ------------------- 主函数 -------------------
def main(cfg_path: str, glb_path: str, task_name: str | None):
    # === 0) 读取配置 ===
    with open(cfg_path, "r", encoding="utf-8") as f:
        g = yaml.safe_load(f)

    # === 1) 场景 ===
    scene, viewer = create_world(with_viewer=True)
    setup_physx_defaults(gravity_z=-9.8, static_mu=0.3, dynamic_mu=0.8, restitution=0.3)

    # === 2) 加载物体 ===
    model_path = glb_path
    trimesh.load(model_path, force="mesh")

    scale = float(SCALE_OBJ)
    shift = np.array([0.0, 0.0, OFFSET], dtype=float)
    base_pose = sapien.Pose(shift, [1, 0, 0, 0])

    actor = load_my_object(
        scene, model_path,
        scale=(scale, scale, scale),
        pose=base_pose,
        build_dynamic=True,
        friction=10,
        restitution=0.3,
    )
    if actor:
        make_float(actor, height=OFFSET)
        set_damping_if_dynamic(actor, linear_damping=0.1, angular_damping=0.1)

    # === 3) 抓取点姿态 (输入时加 90°) ===
    tcp_world = np.array(g["tcp_position"], dtype=np.float32)
    quat = [g["orientation"]["w"], *g["orientation"]["xyz"]]
    quat_new = qmult(quat, axangle2quat([0, 0, 1], np.deg2rad(90)))  # 输入要补偿 90°

    # 可视化
    spawn_dot(scene, tcp_world + np.array([0, 0, OFFSET]), radius=DOT_RADIUS, color=(1, 0, 0, 1))
    spawn_world_axes(scene, length=WORLD_AXIS_LEN, thickness=0.01)

    # === 4) 初始化夹爪 ===
    urdf_loader = scene.create_urdf_loader()
    urdf_loader.fix_root_link = False
    robot = urdf_loader.load("grasp/panda/panda_hand.urdf")

    make_float(robot, height=OFFSET)
    for link in robot.get_links():
        link.set_disable_gravity(True)

    robot.set_root_pose(sapien.Pose([0, 0, OFFSET], quat_new))

    tcp_link = [l for l in robot.get_links() if l.get_name() == "tcp"][0]
    tcp_pose = tcp_link.get_entity_pose()
    delta = tcp_world + np.array([0, 0, OFFSET]) - tcp_pose.p
    root_pose = robot.get_root_pose()
    root_pose.set_p(root_pose.p + delta)
    robot.set_root_pose(root_pose)

    # === 初始化 Gripper ===
    gripper = Gripper(robot, scene)
    last_print_time = time.time()

    # === 5) 主循环 ===
    while not viewer.closed:
        if viewer.window.key_down("x"):
            gripper.control("close", actor)
        elif gripper.is_grasping(actor):
            gripper.control("stop")
        elif viewer.window.key_down("c"):
            gripper.control("open")
        else:
            gripper.control("stop")

        if gripper.is_grasping(actor):
            if not getattr(gripper, "_has_grasped", False):
                print("[INFO] Object grasped!")
                gripper._has_grasped = True

                # --- 输出 ===
                tcp_world = gripper.get_tcp_between_fingers()
                obj_pose = actor.get_pose()
                tcp_in_obj = world_to_object(obj_pose, tcp_world)

                root_pose = robot.get_root_pose()
                quat_xyzw = root_pose.q
                quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

                # 去掉 90° 补偿
                rot_local_z90 = axangle2quat([0, 0, 1], np.deg2rad(90))
                quat_wxyz_no90 = qmult(quat_wxyz, qinverse(rot_local_z90))

                # 转到物体系
                obj_q_xyzw = obj_pose.q
                obj_q_wxyz = np.array([obj_q_xyzw[3], obj_q_xyzw[0], obj_q_xyzw[1], obj_q_xyzw[2]])
                quat_in_obj = qmult(qinverse(obj_q_wxyz), quat_wxyz_no90)

                print("[TCP] Object-frame position =", [float(x) for x in tcp_in_obj])
                print("[Gripper] orientation in object (quat, wxyz) =", [float(x) for x in quat_in_obj])

                f1, f2 = gripper.get_finger_forces(actor)
                print(f"[FORCE] finger1={f1:.3f} N, finger2={f2:.3f} N")

                if task_name:
                    save_result_yaml(task_name, tcp_in_obj, quat_in_obj)

        # 键盘控制
        lin = np.zeros(3, dtype=np.float32)
        if viewer.window.key_down("i"): lin[1] += MOVE_SPEED
        if viewer.window.key_down("k"): lin[1] -= MOVE_SPEED
        if viewer.window.key_down("j"): lin[0] -= MOVE_SPEED
        if viewer.window.key_down("l"): lin[0] += MOVE_SPEED
        if viewer.window.key_down("u"): lin[2] += MOVE_SPEED
        if viewer.window.key_down("o"): lin[2] -= MOVE_SPEED

        ang = np.zeros(3, dtype=np.float32)
        if viewer.window.key_down("1"): ang[0] = ROT_SPEED
        if viewer.window.key_down("2"): ang[1] = ROT_SPEED
        if viewer.window.key_down("3"): ang[2] = ROT_SPEED

        robot.set_root_linear_velocity(lin)
        robot.set_root_angular_velocity(ang)

        now = time.time()
        if now - last_print_time >= 3.0:
            obj_pos = actor.get_pose().p
            gripper_pos = robot.get_pose().p
            dist = np.linalg.norm(obj_pos - gripper_pos)
            print(f"[DIST] distance between object and gripper = {float(dist):.4f}")
            last_print_time = now

        scene.step()
        scene.update_render()
        viewer.render()


# ------------------- 程序入口 -------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, help="抓取配置文件路径")
    parser.add_argument("--glb", type=str, help="GLB 模型路径")
    parser.add_argument("--task", type=str, help="任务名称 (优先级最高)")
    args = parser.parse_args()

    if args.task:
        cfg, glb = load_task(args.task)
        task_name = args.task
    else:
        cfg = args.cfg or GRASP_PROPOSAL_PATH
        glb = args.glb or GRASP_GLB_PATH
        task_name = None

    main(cfg, glb, task_name)
