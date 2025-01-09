from carla import VehicleControl

from .controllers.pid_lat_controller import PidLatController
from .controllers.pid_lon_controller import PidLonController

from .planners.cubic_spline_planner import Spline2D

import numpy as np
import math

def get_driver():
    return "BaselineDriver"

def is_same_direction(ego_yaw, npc_yaw, tolerance=10.0):
    ego_yaw = ego_yaw % 360
    npc_yaw = npc_yaw % 360
    angle_diff = abs(ego_yaw - npc_yaw)
    if angle_diff > 180:
        angle_diff = 360 - angle_diff
    return angle_diff <= tolerance

def is_ahead(ego_location, ego_yaw, npc_location):
    dx = npc_location.x - ego_location.x
    dy = npc_location.y - ego_location.y
    position_vector = np.array([dx, dy])
    ego_yaw_rad = np.deg2rad(ego_yaw)
    heading_vector = np.array([np.cos(ego_yaw_rad), np.sin(ego_yaw_rad)])
    position_vector_norm = position_vector / np.linalg.norm(position_vector)
    heading_vector_norm = heading_vector / np.linalg.norm(heading_vector)
    cos_theta = np.dot(position_vector_norm, heading_vector_norm)
    return cos_theta > 0

class BaselineDriver():
    def __init__(self, router):
        self.router = router
        self.pid_lat = PidLatController()
        self.pid_lon = PidLonController()

    def run(
        self, ego_pose, ego_dimension, ego_dynamics,
        npcs, timestamp, horizon=30
    ):
        print( npcs )
        print("--------------------------------------")
        print( ego_pose )
        print("--------------------------------------")
        print( ego_dynamics )
        print("--------------------------------------")
        print( ego_dimension )

        brakeing = 0.
        ref_path = self.router.get_reference_path(
            ego_pose, ego_dynamics, timestamp, horizon)

        seed_trajectory = self.plan_trajectory(ref_path)
        target_waypoint = seed_trajectory[0]
        
        steering = self.pid_lat.pid_control(ego_pose, target_waypoint)
        target_speed = 15
        if self.is_curve(ref_path):
            target_speed = 5
            print("--------------------------------------")
            print("Curve&&&&&&&&&&&")
        for i in npcs:
            if (is_same_direction(ego_pose.rotation.yaw,i.state.rotation.yaw) 
                and is_ahead(ego_pose.location,ego_pose.rotation.yaw,i.state.location) ):
                denta_dis = math.sqrt((ego_pose.location.x-i.state.location.x)**2
                                     +(ego_pose.location.y-i.state.location.y)**2
                                     +(ego_pose.location.z-i.state.location.z)**2)
                v_of_npc = math.sqrt(i.dynamics.velocity.x**2+i.dynamics.velocity.y**2+i.dynamics.velocity.z**2)
                if denta_dis <= 30 and denta_dis > 25:
                    target_speed = 10
                elif denta_dis <= 25 and denta_dis > 20:
                    target_speed = 12
                elif denta_dis <= 20 :
                    if ego_dynamics.get_speed() >= 5:
                        brakeing = 1
                    else:
                        if v_of_npc<0.4:
                            target_speed = 0
                            brakeing = 1
                        else :
                            target_speed = v_of_npc 
                            brakeing = 0.1
        throt = self.pid_lon.pid_control(
            target_speed,
            current_speed=ego_dynamics.get_speed()
        )

        return VehicleControl(steer=steering, throttle=throt, brake=brakeing)
    def is_curve(self, path):
        for i in range(len(path) - 11):
            p1 = path[i].transform.location
            p2 = path[i + 1].transform.location
            p3 = path[i + 2].transform.location

            v1 = np.array([p2.x - p1.x, p2.y - p1.y])
            v2 = np.array([p3.x - p2.x, p3.y - p2.y])

            angle = self.calculate_angle(v1, v2)
            if angle > 0.1:
                return True
        return False

    def calculate_angle(self, v1, v2):
        cos_theta = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)) 
        angle = np.arccos(cos_theta) 
        return angle
    def plan_trajectory(self, ref_path, ds=1.):
        # NOTE: This is NOT a real planner
        traj = []
        sp = Spline2D(
            [wp.transform.location.x for wp in ref_path],
            [wp.transform.location.y for wp in ref_path]
        )
        s = np.arange(0, sp.s[-1], ds)
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            iyaw = sp.calc_yaw(i_s)
            traj.append([ix, iy, iyaw])

        return traj