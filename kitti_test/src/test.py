#!/usr/bin/python2
# coding=UTF-8

import cv2
import os
import rospy
import numpy as np
import kitti_data_utils


def angle_in_limit(angle):
    # To limit the angle in -pi/2 - pi/2
    limit_degree = 5
    while angle >= np.pi / 2:
        angle -= np.pi
    while angle < -np.pi / 2:
        angle += np.pi
    if abs(angle + np.pi / 2) < limit_degree / 180 * np.pi:
        angle = np.pi / 2
    return angle

if __name__ == '__main__':


    kitti_3d_label = "/home/vip/000000.txt"
    calib = kitti_data_utils.Calibration('fix')

    boxes = open(kitti_3d_label, 'r')
    f = open('m_000000.txt', 'w')

    for box in boxes:

        obj = box.strip().split(" ")
        convert_info = [float(i) for i in obj[1:]]
        h, w, l, x, y, z, rz = convert_info
        center = np.array((x, y, z), dtype=np.float32).reshape(-1,3)

        # camera_h = l 
        # camera_w = h
        # camera_l = w

        ref = calib.project_velo_to_ref(center)
        recf = calib.project_velo_to_rect(ref)

     
        x, y, z = recf[0,0:1], recf[0,1:2], recf[0,2:3]
        ry = -rz - np.pi / 2
        ry = angle_in_limit(ry)
        
        f.write(obj[0] + " ")
        f.write("0 ")
        f.write("0 ")
        f.write("0 ")
        f.write("0 ")
        f.write("0 ")
        f.write("0 ")
        f.write("0 ")
        f.write(str(h) + " ")
        f.write(str(w) + " ")
        f.write(str(l) + " ")

        f.write(str(x).strip("[,]") + " ")
        f.write(str(y).strip("[,]") + " ")
        f.write(str(z).strip("[,]") + " ")

        f.write(str(ry) + "\n")
