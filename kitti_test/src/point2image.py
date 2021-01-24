#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



import os
import numpy as np
import cv2
CAM = 2
import matplotlib.pyplot as plt


h_min, h_max = -180, 180
v_min, v_max = -24.9, 2.0
v_fov, h_fov = (-24.9, 2.0), (-90, 90)
x_range, y_range, z_range = None, None, None




def load_calib(calib_dir):

    # P2 * R0_rect * Tr_velo_to_cam * y

    cam2cam = "calib_cam_to_cam.txt"
    velo2cam = "calib_velo_to_cam.txt"

    #inrinsic
    Intrinsic = open(os.path.join(calib_dir, cam2cam),"r")
    I_lines = Intrinsic.readlines()
    for line in I_lines:
        info = line.strip("\n").split(" ")
        if info[0] == "P_rect_02:":
            P2 = np.array(info[1:], dtype=np.float).reshape(3,4)
            break
    P2 = np.concatenate( (  P2, np.array( [[0,0,0,0]] )  ), 0  )        

    #extrinsic
    Extrinsic = open(os.path.join(calib_dir, velo2cam),"r")
    E_lines = Extrinsic.readlines()
    T_vc = np.array(E_lines[2].strip("\n").split(" ")[1:], dtype= np.float).reshape(3, 1)
    R_vc = np.array(E_lines[1].strip("\n").split(" ")[1:], dtype = np.float).reshape(3,3)
    RT_ = np.concatenate((R_vc, T_vc), axis=1)
    RT_ = np.concatenate(  [ RT_, np.array([0,0,0,1]).reshape(1,4)  ])

    #recified
    R_rect = np.eye(4)

    for line in I_lines:
        info  = line.strip("\n").split(" ")
        if info[0] == "R_rect_02:":
            R_cam_to_rect = np.array(info[1:], dtype=np.float).reshape(3,3)
            break

    return P2, RT_, R_rect

def points_filter(points):

    """
    filter points based on h,v FOV and x,y,z distance range.
    x,y,z direction is based on velodyne coordinates
    1. azimuth & elevation angle limit check
    2. x,y,z distance limit
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    d = np.sqrt(x ** 2 + y ** 2 + z ** 2)

    if h_fov is not None and v_fov is not None:
        if h_fov[1] == h_max and h_fov[0] == h_min and \
                    v_fov[1] == v_max and v_fov[0] == v_min:
            pass
        elif h_fov[1] == h_max and h_fov[0] == h_min:
            con = hv_in_range(d, z, v_fov, fov_type='v')
            lim_x, lim_y, lim_z, lim_d = x[con], y[con], z[con], d[con]
            x, y, z,d = lim_x, lim_y, lim_z, lim_d
        elif v_fov[1] == v_max and v_fov[0] == v_min:
            con = hv_in_range(x, y, h_fov, fov_type='h')
            lim_x, lim_y, lim_z, lim_d = x[con], y[con], z[con], d[con]
            x, y, z, d = lim_x, lim_y, lim_z, lim_d
        else:
            h_points = hv_in_range(x, y, h_fov, fov_type='h')
            v_points = hv_in_range(d, z, v_fov, fov_type='v')
            con = np.logical_and(h_points, v_points)
            lim_x, lim_y, lim_z, lim_d = x[con], y[con], z[con], d[con]
            x, y, z, d = lim_x, lim_y, lim_z, lim_d
    else:
        pass

    if x_range is None and y_range is None and z_range is None:
        pass
    elif x_range is not None and y_range is not None and z_range is not None:
        # extract in-range points
        temp_x, temp_y = __3d_in_range(x), __3d_in_range(y)
        temp_z, temp_d = __3d_in_range(z), __3d_in_range(d)
        x, y, z, d = temp_x, temp_y, temp_z, temp_d
    else:
        raise ValueError("Please input x,y,z's min, max range(m) based on velodyne coordinates. ")

    return x, y, z, d

def hv_in_range( m, n, fov, fov_type='h'):
    """ extract filtered in-range velodyne coordinates based on azimuth & elevation angle limit
    horizontal limit = azimuth angle limit
    vertical limit = elevation angle limit
    """

    if fov_type == 'h':
        return np.logical_and(np.arctan2(n, m) > (-fov[1] * np.pi / 180), \
            np.arctan2(n, m) < (-fov[0] * np.pi / 180))
    elif fov_type == 'v':
        return np.logical_and(np.arctan2(n, m) < (fov[1] * np.pi / 180), \
            np.arctan2(n, m) > (fov[0] * np.pi / 180))
    else:
        raise NameError("fov type must be set between 'h' and 'v' ")

def __3d_in_range(points):
    """ extract filtered in-range velodyne coordinates based on x,y,z limit """
    return points[np.logical_and.reduce((x > x_range[0], x < x_range[1], \
    y > y_range[0], y < y_range[1], \
    z > z_range[0], z < z_range[1]))]

def normalize_data(val, min, max, scale, depth=False, clip=False):
    """ Return normalized data """
    if clip:
        # limit the values in an array
        np.clip(val, min, max, out=val)
    if depth:
        """
        print 'normalized depth value'
        normalize values to (0 - scale) & close distance value has high value. (similar to stereo vision's disparity map)
        """
        return (((max - val) / (max - min)) * scale).astype(np.uint8)
    else:
        """
        print 'normalized value'
        normalize values to (0 - scale) & close distance value has low value.
        """
        return (((val - min) / (max - min)) * scale).astype(np.uint8)

def print_projection_cv2(points, color, image):
    """ project converted velodyne points into camera image """
    
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    for i in range(points.shape[1]):
        cv2.circle(hsv_image, (np.int32(points[0][i]),np.int32(points[1][i])),2, (int(color[i]),255,255),-1)

    return cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)





def pc2img(point_cloud, img, RT_, R_rect, P2):

    x, y, z, dis = points_filter(point_cloud)


    xyz_ = np.hstack((x[:, None], y[:, None], z[:, None]))
    xyz_ = xyz_.T
    one_mat = np.full((1, xyz_.shape[1]), 1)
    xyz_ = np.concatenate((xyz_, one_mat), axis=0)

    color = normalize_data(dis, min=1, max=70, scale=120, clip=True)



    xyz_r = np.dot(RT_, xyz_)

   
    xyz_undistorted = np.dot(R_rect, xyz_r)

 
    xyz_img = np.dot(P2,xyz_undistorted)
    xyz_img = np.delete(xyz_img, 3, axis=0)

    xy_img = xyz_img[::] / xyz_img[::][2]
    ans = np.delete(xy_img, 2, axis=0)

   
    result = print_projection_cv2(ans, color, img)

    return result


if __name__ == "__main__":

    calib_dir = "/home/s5620/2011_09_26/"
    P2, RT_, R_rect = load_calib(calib_dir)

    point_cloud_dir = "/home/s5620/2011_09_26/velodyne_points/data/"
    img_dir = "/home/s5620/2011_09_26/image_02/data/"

   
    # 
    # cv2.imwrite("aa44.jpg",result)

    frame = 0
    rospy.init_node("kitti_node", anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    bridge = CvBridge()


    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        point_cloud = np.fromfile(os.path.join(point_cloud_dir, '%010d.bin'%frame), dtype=np.float32).reshape(-1,4)
        img = cv2.imread(os.path.join(img_dir, '%010d.png'%frame))
        print(img)
        result = pc2img(point_cloud, img, RT_, R_rect, P2)
        cam_pub.publish(bridge.cv2_to_imgmsg(result, "bgr8"))
        rospy.loginfo("camera image published")
        rate.sleep()
        frame += 1
        frame %= 313