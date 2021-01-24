import kitti_data_utils
import os
import argparse
import numpy as np
from utils import  *


parser = argparse.ArgumentParser(description="arg parser")
parser.add_argument('--config', type=str, default="/home/vip/config", help="specify intric and extric paramters")
parser.add_argument('--json', type=str, default="/home/vip/json", help="specify the annotation file")
parser.add_argument('--kitti', type=str, default="/home/vip/kitti_format", help="specify name of the KITTI-formatted file")
args = parser.parse_args()
calib = kitti_data_utils.Calibration('fix', args.config)

json_dir = args.json
kitti_dir = args.kitti

if __name__ == "__main__":
    
    json_files = os.listdir(json_dir)
    json_files.sort(key=lambda x:int(x[:-5]))
    
    for file_name in json_files:

        json_file = os.path.join(json_dir, file_name)
        kitti_file = os.path.join(kitti_dir, file_name[:-5] + ".txt")
        
        lidar_boxes3d = json_to_kitti_lidar(json_file)
        boxes  = lidar_boxes3d[:,1:]
        cls = lidar_boxes3d[:,0:1]
        camera_bboxes = lidar_to_camera3d(boxes, cls, calib.V2C, calib.R0)
        camera_boxes2label(camera_bboxes, kitti_file)
    
    print("done!")