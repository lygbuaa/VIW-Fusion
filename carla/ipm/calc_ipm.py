#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, time
import math
import cv2
import numpy as np
#pip install numba scipy numpy-quaternion
import quaternion

# using carla body coordinate, calc homography from svc to bev
# X-forward, Y-right, Z-up,  euler angles in right-hand, pitch: +Y, roll: +X, yaw: -Z.
class CarlaIPM(object):
    def __init__(self):
        # camera original point in body-frame, in meter
        self.X0_SVC = 2.58
        self.Y0_SVC = 0.0
        self.Z0_SVC = 0.73
        # hight of virtual-bev-camera in body-frame, could be same as svc-camera
        self.Z0_BEV = self.Z0_SVC
        # bev image params
        self.H_BEV_PIXEL = 960
        self.W_BEV_PIXEL = 1280
        self.XMAX_BODY_M = 48
        # self.YMAX_BODY_M = 32*2

        # rotation matrix from camera to body, could not be ignored
        self.R0_CAM_TO_BODY = np.array([
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0]
        ], dtype=np.float32)
        self.R0_BODY_TO_CAM = np.linalg.inv(self.R0_CAM_TO_BODY)

        self.init_svc_camera_params()
        self.init_bev_camera_params()
        # self.calc_ipm_demo()
        self.calc_ipm()

    # https://answers.opencv.org/question/174548/inverse-perspective-mapping-ipm-on-the-capture-from-camera-feed/
    def calc_ipm_demo(self):
        K_cam = self.K_svc_cam
        T = np.array([
            [0.0],
            [0.0],
            [100.0]
        ], dtype=np.float32)
        R = self.rot_mat_from_euler(pitch=0.0, roll=-70.0, yaw=0.0)
        RT = np.concatenate((np.concatenate((R, T), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

        K_bev = np.array([
            [1.0, 0.0, -960, 0.0],
            [0.0, 1.0, -540, 0.0],
            [0.0, 0.0, 0.0,  0.0],
            [0.0, 0.0, 1.0,  1.0]
        ], dtype=np.float32)
        self.H_bev_to_svc = (K_cam @ (RT @ K_bev))[0:3, 0:3]
        self.H_svc_to_bev = np.linalg.inv(self.H_bev_to_svc)
        print("H_bev_to_svc: {}, \n H_svc_to_bev: {}".format(self.H_bev_to_svc, self.H_svc_to_bev))

    def calc_ipm(self):
        # to be confirmed
        # self.inv_K_bev_cam[2, 2] = 0.0
        # self.inv_K_bev_cam[3, 2] = 1.0
        # self.inv_K_bev_cam[3, 1] = 1.0
        # self.inv_K_bev_cam[3, 0] = 1.0
        self.H_bev_to_svc = (self.K_svc_cam @ (self.Ess_svc_body_to_cam @ (self.Ess_bev_cam_to_body @ self.inv_K_bev_cam)))[0:3, 0:3]
        self.H_svc_to_bev = np.linalg.inv(self.H_bev_to_svc)
        print("H_bev_to_svc: {}, \n H_svc_to_bev: {}".format(self.H_bev_to_svc, self.H_svc_to_bev))

    def init_svc_camera_params(self, orientation=0.0):
        self.K_svc_cam = np.array([
            [554.256, 0.0, 960.0, 0.0],
            [0.0, 554.256, 540.0, 0.0],
            [0.0,     0.0,   1.0, 0.0],
            [0.0,     0.0,   0.0, 1.0]
        ], dtype=np.float32)
        self.inv_K_svc_cam = np.linalg.inv(self.K_svc_cam)
        print("K_svc_cam: {}, \n inv_K_svc_cam: {}".format(self.K_svc_cam, self.inv_K_svc_cam))
        # from camera to body, pitch(+Y, right) is 20
        self.R_svc_cam_to_body = self.rot_mat_from_euler(pitch=20.0, roll=0.0, yaw=0.0) @ self.R0_CAM_TO_BODY
        self.T_svc_cam_to_body = np.array([[self.X0_SVC], [self.Y0_SVC], [self.Z0_SVC]], dtype=np.float32)
        self.Ess_svc_cam_to_body = np.concatenate((np.concatenate((self.R_svc_cam_to_body, self.T_svc_cam_to_body), axis=1), np.array([[0, 0, 0, 1]])), axis=0)
        self.Ess_svc_body_to_cam = np.linalg.inv(self.Ess_svc_cam_to_body)
        print("Ess_svc_cam_to_body: {}, \n Ess_svc_body_to_cam: {}".format(self.Ess_svc_cam_to_body, self.Ess_svc_body_to_cam))

    def init_bev_camera_params(self):
        self.f_bev_cam = self.Z0_BEV * self.H_BEV_PIXEL / self.XMAX_BODY_M
        print("f_bev_cam: {}".format(self.f_bev_cam))
        self.K_bev_cam = np.array([
            [self.f_bev_cam, 0.0, self.W_BEV_PIXEL/2.0, 0.0],
            [0.0, self.f_bev_cam, self.H_BEV_PIXEL/2.0, 0.0],
            [0.0,     0.0,                   1.0,       0.0],
            [0.0,     0.0,                   0.0,       1.0]
        ], dtype=np.float32)
        # intrinsic normalization
        self.K_bev_cam[0:3, 0:3] = self.K_bev_cam[0:3, 0:3]/self.Z0_BEV
        self.inv_K_bev_cam = np.linalg.inv(self.K_bev_cam)
        print("K_bev_cam: {}, \n inv_K_bev_cam: {}".format(self.K_bev_cam, self.inv_K_bev_cam))
        # from camera to body, pitch(+Y, right) is 90
        self.R_bev_cam_to_body = self.rot_mat_from_euler(pitch=90.0, roll=0.0, yaw=0.0) @ self.R0_CAM_TO_BODY
        # T make no difference to homography at all, needs debug
        self.T_bev_cam_to_body = np.array([
            [self.X0_SVC+self.XMAX_BODY_M/2],
            [self.Y0_SVC],
            [self.Z0_BEV]
        ], dtype=np.float32)
        self.Ess_bev_cam_to_body = np.concatenate((np.concatenate((self.R_bev_cam_to_body, self.T_bev_cam_to_body), axis=1), np.array([[0, 0, 0, 1]])), axis=0)
        self.Ess_bev_body_to_cam = np.linalg.inv(self.Ess_bev_cam_to_body)
        print("Ess_bev_cam_to_body: {}, \n Ess_bev_body_to_cam: {}".format(self.Ess_bev_cam_to_body, self.Ess_bev_body_to_cam))

    # pitch, roll, yaw in degree
    # np.quaternion(qw, qx, qy, qz)
    def rot_mat_from_euler(self, pitch=0.0, roll=0.0, yaw=0.0):
        pitch = np.deg2rad(pitch)
        roll = np.deg2rad(roll)
        yaw = np.deg2rad(yaw)
        # roll: +X
        q_roll = np.quaternion(np.cos(roll/2), np.sin(roll/2), 0, 0)
        # pitch: +Y
        q_pitch = np.quaternion(np.cos(pitch/2), 0, np.sin(pitch/2), 0)
        # yaw: -Z
        q_yaw = np.quaternion(np.cos(-yaw/2), 0, 0, np.sin(-yaw/2))
        q = q_pitch * q_roll * q_yaw
        rot = quaternion.as_rotation_matrix(q)
        print("pitch: {:.3f}, roll: {:.3f}, yaw: {:.3f}, \nq: {}, \nrot: {}".format(pitch, roll, yaw, q, rot))
        return rot

    def rot_mat_from_quat(self, q):
        return quaternion.as_rotation_matrix(q)

    def valid_H(self):
        # H_bev_to_svc = np.array([
        #     [ 7.12538216e+01,  0.00000000e+00, -2.34211150e+04],
        #     [ 5.07434023e+01,  5.54255990e+01, -2.96748554e+04],
        #     [ 9.39692635e-02,  0.00000000e+00, -3.03198368e+01]
        # ], dtype=np.float32)
        # H_svc_to_bev = np.array([
        #     [-7.49365324e-01,  0.00000000e+00,  5.78861014e+02],
        #     [-5.57395928e-01,  1.80422047e-02,  4.12912326e+02],
        #     [-2.32248307e-03,  0.00000000e+00,  1.76106301e+00]
        # ], dtype=np.float32)
        print("test H@invH: {}".format(self.H_bev_to_svc@self.H_svc_to_bev))

    def test_H(self):
        image_path = "image/front_camera_view.png"
        img_src = cv2.imread(image_path)
        # img_dst = cv2.warpPerspective(img_src, self.H_svc_to_bev, (img_src.shape[1], img_src.shape[0]))
        H = self.H_svc_to_bev
        # H = np.array([
        #     [ 0.0,  1.0, 0.0],
        #     [ 1.0,  0.0, 0.0],
        #     [ 0.0,  0.0, 1.0]
        # ], dtype=np.float32)
        img_dst = cv2.warpPerspective(img_src, H, (int(self.W_BEV_PIXEL), int(self.H_BEV_PIXEL/2)))
        cv2.imwrite(image_path + ".ipm.jpg", img_dst)
        cv2.namedWindow("ipm", cv2.WINDOW_NORMAL)
        cv2.imshow("ipm", img_dst)
        cv2.waitKey(-1)


if __name__ == "__main__":
    ipm = CarlaIPM()
    ipm.test_H()
