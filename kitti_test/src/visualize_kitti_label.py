#!/usr/bin/env python2

#turn off catkin_extand
import rospy
import cv2
import os
import glob
from cv_bridge import CvBridge
from utils import *
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2
import kitti_data_utils
import kitti_config as cnf
from visualization_msgs.msg import MarkerArray, Marker

CAM =2
detection_class = ['Car', 'Pedestrian']
gt_class = ['Car', 'Van', 'Pedestrian']

def get_3dboxes_corner(objects, calib):

    ''' Show all LiDAR points.
        Draw 3d box in LiDAR point cloud (in velo coord system) '''

    box3d_corners = []
    object_types = []

    for obj in objects:
        
        if obj.type not in  detection_class: 
            continue
        # Draw 3d bounding box
  
        box3d_pts_2d, box3d_pts_3d = kitti_data_utils.compute_box_3d(obj, calib.P)
        box3d_pts_3d_velo = calib.project_rect_to_velo(box3d_pts_3d)
       
        box3d_corners.append(box3d_pts_3d_velo)
        object_types.append(obj.type)
      
    return np.array(box3d_corners).reshape(-1, 8, 3), object_types

def get_gt_3dboxes_corner(objects, calib):

    ''' Show all LiDAR points.
        Draw 3d box in LiDAR point cloud (in velo coord system) '''

    box3d_corners = []
    object_types = []

    for obj in objects:
        if obj.type not in  gt_class: 
            continue
        # Draw 3d bounding box
        box3d_pts_2d, box3d_pts_3d = kitti_data_utils.compute_box_3d(obj, calib.P)
        box3d_pts_3d_velo = calib.project_rect_to_velo(box3d_pts_3d)
        box3d_corners.append(box3d_pts_3d_velo)
        object_types.append(obj.type)
      
    return np.array(box3d_corners).reshape(-1, 8, 3), object_types

def show_image_with_boxes(img, objects, gt_objects, calib):
    ''' Show image with 2D bounding boxes '''

    img2 = np.copy(img)  # for 3d bbox
    for obj in objects:
        if obj.type not in detection_class: continue
        box3d_pts_2d, box3d_pts_3d = kitti_data_utils.compute_box_3d(obj, calib.P)
        if box3d_pts_2d is not None:
            img2 = kitti_data_utils.draw_projected_box3d(img2, box3d_pts_2d, cnf.colors[obj.cls_id])

    for gt_obj in gt_objects:
        if gt_obj.type not in gt_class: continue
        box3d_pts_2d, box3d_pts_3d = kitti_data_utils.compute_box_3d(gt_obj, calib.P)
        if box3d_pts_2d is not None:
            img2 = kitti_data_utils.draw_projected_box3d(img2, box3d_pts_2d, (255, 0, 0))

    return img2
        


if __name__ == "__main__":

    
    """
    requirement:
    
    1. label
    2. image
    3. testing velodyne
    4. calibration file, fixed or different 

    tracking label transformer in utils.py
    tracking2label()
    """


    # label_dir = "/home/vip/Kitti_test/2011_09_26/2011_09_26_drive_0014_sync/label_0014/"
    # image_dir = "/home/vip/Kitti_test/2011_09_26/2011_09_26_drive_0014_sync/image_02/data"
    # velo_dir = "/home/vip/Kitti_test/2011_09_26/2011_09_26_drive_0014_sync/velodyne_points/data"
    # calib_dir = "/home/vip/Kitti_test/2011_09_26/"

    gt_dir    = "/home/vip/kitti_format"
    # label_dir = "/home/vip/conference/frustum-pointnets/dataset/KITTI/object/training/detection"
    # label_dir = "/home/vip/Second/second.pytorch/second/pointpillar_output/eval_results/step_296960/label"
    image_dir = "/home/vip/image_2"
    velo_dir = "/home/vip/velodyne"
    # real_velo_dir = "/home/vip/conference/frustum-pointnets/dataset/KITTI/object/training/original_velodyne"
    # calib_dir = "/home/vip/Downloads/kitti_second/training/calib"
    frame = 2


    rospy.init_node("kitti_node", anonymous=True)

    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    lidar_pub = rospy.Publisher('kitti_lidar', PointCloud2, queue_size=10)
    box3d_pub = rospy.Publisher('box', MarkerArray, queue_size=10)
    gt_box3d_pub = rospy.Publisher('gt', MarkerArray, queue_size=10)
    car_model = rospy.Publisher('car_model', Marker, queue_size=10)
    
    bridge = CvBridge()   
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        objects=[]
        gt_objects=[]

        index = '%006d' % frame
        

        # pred_path = os.path.join(label_dir, index + ".txt")
        gt_path = os.path.join(gt_dir, index + ".txt")
        # calib_path = os.path.join(calib_dir, index + ".txt")


        image_path = os.path.join(image_dir, index + ".png")
        velo_path = os.path.join(velo_dir, index + ".bin")


        # pred_label = open(pred_path, 'r').readlines()
        gt_label = open(gt_path, 'r').readlines()



        img = read_img(image_path)
        pc  = read_point_cloud(velo_path)
        # real_pc  = read_point_cloud(real_velo_path)
        calib = kitti_data_utils.Calibration('fix') 

        # for pred_line in pred_label:
        #     objects = np.append(objects ,kitti_data_utils.Object3d(pred_line.strip()))

        for gt_line in gt_label:
            gt_objects = np.append(gt_objects ,kitti_data_utils.Object3d(gt_line.strip()))

        img = show_image_with_boxes(img, objects, gt_objects, calib)


        # coners_3d, object_types = get_3dboxes_corner(objects, calib)
        gt_coners_3d, gt_object_types = get_gt_3dboxes_corner(gt_objects, calib)
       
      

        publish_car_model(car_model)
        publish_camera(cam_pub, bridge, img)
        publish_point_cloud(lidar_pub, pc)
        # publish_3dbox(box3d_pub, coners_3d, object_types)
        publish_gt_3dbox(gt_box3d_pub, gt_coners_3d)
        # publish_point_cloud(real_lidar_pub, real_pc)


        rate.sleep()
        # frame = frame + 1
        # if frame % 4 == 0:
        #     frame = 0