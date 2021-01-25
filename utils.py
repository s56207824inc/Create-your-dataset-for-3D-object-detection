import os 
import cv2
import numpy as np
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pcl2
import rospy
from geometry_msgs.msg import Point
import tf
import json
FRAME_ID = 'map'
Detection_color_dict = {'Car':(255 , 0, 0), 'Pedestrian':(0, 226, 255), 'Cyclist':(141, 40, 255)}
LIFETIME_gt = 0.1
LIFETIME_pred = 0.1
id = { 0 : 'Car'}
LINES  = [[0, 1], [1, 2], [2, 3], [3, 0]]
LINES += [[4, 5], [5, 6], [6, 7], [7, 4]]
LINES += [[4, 0], [5, 1], [6, 2], [7, 3]]
# LINES += [[4, 1], [5, 0]] 
def read_img(path):
    img = cv2.imread(path)
    return img 

def read_point_cloud(path):
    pc = np.fromfile(path, dtype=np.float32).reshape(-1,4)
    return pc[:, 0:3]

def publish_camera(cam_pub, bridge, image):

    rospy.loginfo("camera image published")
    cam_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))


def publish_point_cloud(pcl_pub, point_cloud):

    rospy.loginfo("point cloud published")
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))

# def publish_3dbox(box_pub, coners_3d_velos, gt_coners_3d_velos, object_types):

#     rospy.loginfo("bbox published")

#     marker_array = MarkerArray()
    
#     for i , coners_3d_velo in enumerate(coners_3d_velos):

#         marker = Marker()
#         marker.header.frame_id = FRAME_ID
#         marker.header.stamp = rospy.Time.now()

     

#         marker.id = i
#         marker.action = Marker.ADD
#         marker.lifetime = rospy.Duration(LIFETIME)
#         marker.type = Marker.LINE_LIST

#         r, g, b = Detection_color_dict[object_types[i]]

#         marker.color.r = r/255.0
#         marker.color.g = g/255.0
#         marker.color.b = b/255.0

#         marker.color.a = 1.0
#         marker.scale.x = 0.1

#         marker.points = []

#         for l in  LINES:
#             p1 = coners_3d_velo[l[0]]
#             marker.points.append(Point(p1[0], p1[1], p1[2]))
#             p2 = coners_3d_velo[l[1]]
#             marker.points.append(Point(p2[0], p2[1], p2[2]))

#         marker_array.markers.append(marker)

#     #gt
#     for i , gt_coners_3d_velo in enumerate(gt_coners_3d_velos):
#         print("a")
#         marker = Marker()
#         marker.header.frame_id = FRAME_ID
#         marker.header.stamp = rospy.Time.now()

     

#         marker.id = i
#         marker.action = Marker.ADD
#         marker.lifetime = rospy.Duration(LIFETIME)
#         marker.type = Marker.LINE_LIST

#         # r, g, b = Detection_color_dict[object_types[i]]

#         marker.color.r = 0
#         marker.color.g = 0
#         marker.color.b = 1

#         marker.color.a = 1.0
#         marker.scale.x = 0.1

#         marker.points = []

#         for l in  LINES:
#             p1 = gt_coners_3d_velo[l[0]]
#             marker.points.append(Point(p1[0], p1[1], p1[2]))
#             p2 = gt_coners_3d_velo[l[1]]
#             marker.points.append(Point(p2[0], p2[1], p2[2]))

#         marker_array.markers.append(marker)

#     box_pub.publish(marker_array)

def publish_3dbox(box_pub, coners_3d_velos, object_types):

    rospy.loginfo("bbox published")
    marker_array = MarkerArray()

    i = 0

    for coners_3d_velo in coners_3d_velos:

        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()
 
        marker.id = i
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(LIFETIME_pred)
        marker.type = Marker.LINE_LIST
      

        r, g, b = Detection_color_dict[object_types[i]]

        marker.color.r = r/255.0
        marker.color.g = g/255.0
        marker.color.b = b/255.0

        marker.color.a = 1.0
        marker.scale.x = 0.1
        marker.points = []
   

        for l in  LINES:
            p1 = coners_3d_velo[l[0]]
            marker.points.append(Point(p1[0], p1[1], p1[2]))

            p2 = coners_3d_velo[l[1]]
            marker.points.append(Point(p2[0], p2[1], p2[2]))
     

        marker_array.markers.append(marker)

        i=i+1

    box_pub.publish(marker_array)


def publish_gt_3dbox(gt_box3d_pub ,gt_coners_3d_velos):

    rospy.loginfo("gt_bbox published")
    marker_array_2 = MarkerArray()
    i = 100

    for gt_coners_3d_velo in  gt_coners_3d_velos:

     
        marker_2 = Marker()
        marker_2.header.frame_id = FRAME_ID
        marker_2.header.stamp = rospy.Time.now()

    
        marker_2.id = i+1
        marker_2.action = Marker.ADD
        marker_2.lifetime = rospy.Duration(LIFETIME_gt)
        marker_2.type = Marker.LINE_LIST




        marker_2.color.r = 0
        marker_2.color.g = 0
        marker_2.color.b = 1


        marker_2.color.a = 1.0
        marker_2.scale.x = 0.1

        marker_2.points = []

        for l in  LINES:
       
            p1 = gt_coners_3d_velo[l[0]]
            marker_2.points.append(Point(p1[0], p1[1], p1[2]))
            p2 = gt_coners_3d_velo[l[1]]
            marker_2.points.append(Point(p2[0], p2[1], p2[2]))
        marker_array_2.markers.append(marker_2)

 
        i=i+1
    gt_box3d_pub.publish(marker_array_2)

def publish_car_model(model_pub):

    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()

    marker.id = 100 
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration()
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = "package://kitti_test/car_model/car_model.dae"

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = -1.73
    
    q = tf.transformations.quaternion_from_euler(0, 0, -np.pi)
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    model_pub.publish(marker)

def camera_boxes2label(camera_boxes, save_name):
    # (x, y, z, h, w, l, r ) camera_box3d
    kitti_format = open(save_name, 'w')
    for box in camera_boxes:

        cls = box[0]
        x,y,z,h,w,l,r = [float(info) for info in box[1:]]
        
        kitti_format.write(id[cls] + " ")
        kitti_format.write("0 ")
        kitti_format.write("0 ")
        kitti_format.write("0 ")
        kitti_format.write("0 ")
        kitti_format.write("0 ")
        kitti_format.write("0 ")
        kitti_format.write("0 ")
        kitti_format.write(str(h)+" ")
        kitti_format.write(str(w)+" ")
        kitti_format.write(str(l)+" ")
        kitti_format.write(str(x)+" ")
        kitti_format.write(str(y)+" ")
        kitti_format.write(str(z)+" ")
        kitti_format.write(str(r)+"\n")
         


def lidar_to_camera3d(lidar_boxes,cls, T_VELO_2_CAM=None, R_RECT_0=None):
    camera_boxes = []
    for cls, box in zip(cls, lidar_boxes):
        one_camera_box3d = lidar_to_camera_box(
                box[np.newaxis, :].astype(np.float32), T_VELO_2_CAM, R_RECT_0)[0]
        one_camera_box3d = np.insert(one_camera_box3d, 0, cls)
        camera_boxes.append(one_camera_box3d)
    
    return np.array(camera_boxes, dtype=np.float32).reshape(-1,8)

def lidar_to_camera(x, y, z, T_VELO_2_CAM=None, R_RECT_0=None):
  
    p = np.array([x, y, z, 1])
    p = np.matmul(T_VELO_2_CAM, p)
    p = np.matmul(R_RECT_0, p)
    p = p[0:3]

    return tuple(p)

def lidar_to_camera_box(boxes, T_VELO_2_CAM=None, R_RECT_0=None):
    # (N, 7) -> (N, 7) x,y,z,h,w,l,r
    ret = []
    for box in boxes:
        h, w, l,x,y,z,rz = box
        z=z-h/2
        (x, y, z), h, w, l, ry = lidar_to_camera(
            x, y, z, T_VELO_2_CAM, R_RECT_0), h, w, l, -rz - np.pi / 2
        ry = angle_in_limit(ry)
        ret.append([x, y, z,h, w, l, ry])
    return np.array(ret).reshape(-1, 7)

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

def json_to_kitti_lidar(annotation_path):

    classes = {"car":0}
    json_data = None
    boxes = []

    with open(annotation_path) as annotation:
        json_data = json.load(annotation)
    box_list = json_data["frame"]["bounding_boxes"]

    for box in box_list:
        clas, x, y, z, rz, w, l, h = box["kitti_class"], box["kitti_x"], box["kitti_y"], box["kitti_z"],\
                                        box["kitti_theta"], box["kitti_w"], box["kitti_l"], box["kitti_h"]
        one_box = [classes[clas], h, w, l, x, y, z, rz]
        one_box = np.array(one_box, dtype=np.float32)
        
        boxes.append(one_box)

    return np.array(boxes).reshape(-1, 8)


# def tracking2label(path):
#     """
#     set your input and output
#     """
#     output = "/home/vip/Kitti_test/output"

#     lines = open(path, 'r').readlines()
#     current_idx = 0

#     for line in lines:

#         info = line.strip("\n").split(" ")
#         idx = int(info[0])
  
    
#         required_line = ""
#         for i in info[2:]:
#             required_line = required_line + i + " " 
    

#         if idx == current_idx:

#             save_name = '%006d.txt' % idx
#             current_txt = open(os.path.join(output, save_name),'a')
#             current_txt.write(required_line)
#             current_txt.write("\n")
#         else:
#             current_idx = current_idx + 1
