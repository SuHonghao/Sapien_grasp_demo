# math_utils.py

from __future__ import annotations
import math
import numpy as np
import sapien.core as sapien
import transforms3d.quaternions as tq

def rpy_to_R(r, p, y):
    """roll/pitch/yaw (rad) -> Rz(yaw) @ Ry(pitch) @ Rx(roll)"""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]], dtype=np.float32)
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]], dtype=np.float32)
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]], dtype=np.float32)
    return Rz @ Ry @ Rx

def mat_to_pose(M: np.ndarray) -> sapien.Pose:
    R, p = M[:3,:3], M[:3,3]
    tr = float(np.trace(R))
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S; x = (R[2,1]-R[1,2]) / S; y = (R[0,2]-R[2,0]) / S; z = (R[1,0]-R[0,1]) / S
    else:
        if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1]-R[1,2])/S; x = 0.25*S; y = (R[0,1]+R[1,0])/S; z = (R[0,2]+R[2,0])/S
        elif R[1,1] > R[2,2]:
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2]-R[2,0])/S; x = (R[0,1]+R[1,0])/S; y = 0.25*S; z = (R[1,2]+R[2,1])/S
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0]-R[0,1])/S; x = (R[0,2]+R[2,0])/S; y = (R[1,2]+R[2,1])/S; z = 0.25*S
    return sapien.Pose(p, np.array([w, x, y, z], dtype=np.float32))

def world_to_object(obj_pose: sapien.Pose, point_world: np.ndarray):
    """Convert a world-space point to object(local)-space given the object's pose."""
    # SAPIEN uses xyzw; convert to wxyz
    q_wxyz = [obj_pose.q[3], obj_pose.q[0], obj_pose.q[1], obj_pose.q[2]]
    R = tq.quat2mat(q_wxyz)
    t = obj_pose.p
    return R.T @ (point_world - t)
