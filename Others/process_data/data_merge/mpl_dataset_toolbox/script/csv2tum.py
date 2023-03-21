import argparse
import numpy as np
import rosbag

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert OptiTrack raw csv file to txt in tum format')
    parser.add_argument('csv_path', type=str, help='csv file path')
    parser.add_argument('bag_path', type=str, help='ROS bag path')
    args = parser.parse_args()

    csv_path = args.csv_path
    bag_path = args.bag_path

    tum_path = csv_path[:-3] + "tum"
    bag_name = csv_path[:-4].split("/")[-1]
    bag = rosbag.Bag(bag_path, "r")
    ts_offset = bag.get_start_time()

    with open(csv_path, encoding = "utf-8") as gt:
        data = np.loadtxt(gt, delimiter=',', skiprows=7, usecols=[0,6,7,8,2,3,4,5])
        data[:,0] = data[:,0] / 120 + ts_offset
        # data[:,4:8] = data[:,4:8]jnp.linalg.norm(data[:,4:8], axis=1)[:,np.newaxis]

    np.savetxt(tum_path, data, fmt = "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f",
            header = "ground truth for " + bag_name + "\ntimestamp tx ty tz qx qy qz qw")
