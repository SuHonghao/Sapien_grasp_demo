import sapien.core as sapien
from sapien.core import Pose
import numpy as np
from collections import deque

class Gripper:
    def __init__(self, robot, scene):
        self.robot = robot
        self.scene = scene
        self.joints = robot.get_active_joints()
        if len(self.joints) < 1:
            raise RuntimeError("Gripper has no active joints!")

        # 找手指 link
        links = robot.get_links()
        self.finger1_link = links[1]  # 左指
        self.finger2_link = links[2]  # 右指

        # 速度模式（刚度/阻尼参数）
        for j in self.joints:
            j.set_drive_property(stiffness=500, damping=500, force_limit=5)
            j.set_drive_target(0.1)   # 初始张开
            j.set_drive_velocity_target(0.0)

        # 初始化关节位置
        qpos = self.robot.get_qpos()
        qpos[:] = 0.1
        self.robot.set_qpos(qpos)

        self.closing_speed = -0.01
        self.opening_speed = 0.01
        self.hold_speed = 0.0

        # === 最近20帧的力缓存 ===
        self.l_force_history = deque(maxlen=20)
        self.r_force_history = deque(maxlen=20)

    def is_grasping(self, obj, finger_thresh=1e-3):
        """
        检测抓取状态 (三态返回)
        返回值:
            True  -> 抓住了 (左右手指都与物体接触)
            False -> 左右两个手指互相接触 (空夹)
            None  -> 其他情况 (不判定)
        """
        contacts = self.scene.get_contacts()
        l_contact, r_contact = False, False
        finger_contact = False

        for c in contacts:
            bodies = [b.get_name() for b in c.bodies]
            for p in c.points:
                if p.separation <= 0:
                    if self.finger1_link.get_name() in bodies and obj.get_name() in bodies:
                        l_contact = True
                    if self.finger2_link.get_name() in bodies and obj.get_name() in bodies:
                        r_contact = True
                    # 检查 finger1 和 finger2 直接接触
                    if (self.finger1_link.get_name() in bodies 
                        and self.finger2_link.get_name() in bodies):
                        finger_contact = True

        if l_contact and r_contact:
            return True   # 抓住物体
        if finger_contact:
            return False  # 夹爪互相碰到，没夹住
        return None        # 其他情况（不判定）



    def get_tcp_between_fingers(self):
        """返回两个 finger link 之间的中点 (世界坐标系下)"""
        p1 = self.finger1_link.get_pose().p
        p2 = self.finger2_link.get_pose().p
        return 0.5 * (p1 + p2)

    def control(self, command: str, obj=None):
        """简单开/合/停控制"""
        if command == "open":
            for j in self.joints:
                j.set_drive_target(0.1)
        elif command == "close":
            if obj is not None and self.is_grasping(obj):
                for j in self.joints:
                    j.set_drive_target(j.get_drive_target())  # 保持不动
            else:
                for j in self.joints:
                    j.set_drive_target(0.0)
        else:  # stop
            for j in self.joints:
                j.set_drive_target(j.get_drive_target())  # 保持不动


    def get_finger_forces(self, obj):
        """
        返回 (finger1_force, finger2_force)，单位 N
        使用最近20帧的最大值，避免瞬时为0
        """
        dt = self.scene.get_timestep()
        contacts = self.scene.get_contacts()
        l_force, r_force = 0.0, 0.0

        for c in contacts:
            bodies = [b.get_name() for b in c.bodies]
            for p in c.points:
                if p.separation <= 0:
                    # 用冲量模长近似力
                    force_mag = np.linalg.norm(p.impulse) / max(dt, 1e-6)

                    if self.finger1_link.get_name() in bodies and obj.get_name() in bodies:
                        l_force = max(l_force, force_mag)

                    if self.finger2_link.get_name() in bodies and obj.get_name() in bodies:
                        r_force = max(r_force, force_mag)

        # 推入缓存
        self.l_force_history.append(l_force)
        self.r_force_history.append(r_force)

        # 返回最近20帧的最大值
        l_max = max(self.l_force_history) if self.l_force_history else 0.0
        r_max = max(self.r_force_history) if self.r_force_history else 0.0

        return l_max, r_max
