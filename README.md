# Abstract
---
### 1. To verify your visualization is correct
### 2. Pocessing raw data from rosbag files
### 3. Annotate data
### 4. Convert format of annotated data into KITTI format
### 5. Visualize result


# Visualization Tool
----
* To use visualization tool to verify you can correctly show the sence(images, point clouds) and ground truth box based on KITTI dataset cause we finally would convert our json file into KITTI label format

* [Detail in my another repo](https://github.com/s56207824inc/self-driving_visualization_ROS/tree/master)

# Extract raw data from rosbag
---

#### only testing on Velodyne HDL-32E, Ouster-128
* Velodyne point cloud should be reshape into (-1, 4)
* Ouster point cloud should be reshape inot (-1,9)

### Step 1. To observe your bag information
#### Open a terminal and type this command find which topic is your targeted signal

```
rosbag info <your bagfile.bag>
```

#### EX . in ouster-128 if my targeted signal is point cloud
#### Its topic would be /points_raw



### Step 2. To extract your targeted signal
#### modify pt_decoder subfunction in below script
#### you might need to set your output path and signal topic name
```
python rosbag_deconder.py
```

# Annotated data
---
### [Label toolbox](https://github.com/ziliHarvey/smart-annotation-pointrcnn)

#### only can load 50 files a time to label
#### format of result file is json


# Convert format into KITTI label
---
```
python json2kitti_camera.py --config /PATH/to/your/intrics and extrics dir --json PATH/to/json_dir --kitti PATH/to/output_dir

```

# To see your label result is correct or not
---

