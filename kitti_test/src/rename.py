import os

path = "/home/vip/Second/second.pytorch/second/data/kitti/gt"

path_dir = os.listdir(path)
path_dir = sorted(path_dir)

frame = 7481
for i in path_dir:

    name = "%006d.txt" % frame
    old_path = os.path.join(path, i)
    new_path = os.path.join(path, name)
    os.rename(old_path, new_path)
    frame+=1