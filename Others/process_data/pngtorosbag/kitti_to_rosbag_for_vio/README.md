# kitti_to_rosbag_for_vio
A modified version based on [ethz-asl/kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) and [PetWorm/kitti_to_rosbag_for_vio](https://github.com/PetWorm/kitti_to_rosbag_for_vio) to change the Image.png to rosbag format for running vios

## How to run
~~~
rosrun kitti_to_rosbag kitti_rosbag_converter /home/yourname/Datasets/KITTI/dataset/zurich_city_04_b/ /home/yourname/Datasets/KITTI/dataset/zurich_city_04_b/zurich_city_04_b_drive_0001_sync/ /home/yourname/Datasets/KITTI/zurich_city_04_b_image.bag
~~~