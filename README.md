
# An Event-based Dataset for VO/VIO/SLAM in Robotics
## Author: [Guan Weipeng](https://scholar.google.com/citations?hl=zh-CN&user=fUU5Cv0AAAAJ), [Chen Peiyu](https://github.com/cpymaple)






## Dataset for Event-based Stereo Visual-inertial Odometry
### Acquisition Platform

### Data Sequence
Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag
--|:--|:--:|--:|--:|--:
gate_01|2021-07-31|16.4g|172s|dark,around gate|[Rosbag](https://sjtueducn-my.sharepoint.com/:u:/g/personal/594666_sjtu_edu_cn/ET3mU1rvdTpEl8VYvC25q7YB5pmPQlwru0jBbQ9iu0oAMA?e=LrKUpJ)
gate_02|2021-07-31|27.3g|327s|dark,loop back|[Rosbag](https://sjtueducn-my.sharepoint.com/:u:/g/personal/594666_sjtu_edu_cn/EY7fHSh4NnxBvemze1JS8TEBy5beLh_xlJ6mdi2IYmeY9w?e=xIcvDe)
gate_03|2021-08-04|21.9g|283s|day|[Rosbag](https://sjtueducn-my.sharepoint.com/:u:/g/personal/594666_sjtu_edu_cn/EUthdjvVIVdFmFxR82jzVqUBQTpsfvvb26pYtb0yj-_hlw?e=6MWwmJ)






## Dataset for Event-based Monocular Visual-inertial Odometry
### Acquisition Platform

<center>
<img src="https://github.com/arclab-hku/Datasequence_Event_based_SLAM/blob/main/IROS2022/sensor_setup.png" width="100%">
</center>

<p align="center"> The Platform for Data Collection </p>

* The configuration files is in [link](https://github.com/arclab-hku/Datasequence_Event_based_SLAM/tree/main/IROS2022)

 

### Data Sequence
Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag
--|:--|:--:|--:|--:|--:
vicon_aggressive_hdr|---|23.0g|---|HDR, Aggressive Motion|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/ESxBPJlRT4FApeMZgwvAo4YBuAhoOT5tcb_A9dAvPSEeeg?e=CRDVrD)
vicon_dark1|---|10.5g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EaY7bfm8ZytGvlFP3v1TNHgBXMubjQvjiuoiZVqqEmA2jA?e=OyZyyU)
vicon_dark2|---|16.6g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/Ed1hZLF4mOJJlz8nuk92evYByN9PkbrJE_xS8yuKy14ZUg?e=gYqWbg)
vicon_darktolight1|---|17.2g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EQwioJi0GqlKmc7j4BcDyQEB-YrX6HSk_FsEavKFYoihYw?e=24ZYdR)
vicon_darktolight2|---|14.4g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EWLO58HfLOxNpFdEQzJgZaoBq4Mo74ceZGcgUYlMLhUJbg?e=JNjn1x)
vicon_hdr1|---|13.7g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EfGW22iMVwZEoVCOdZ9cuHYB2_ZUXR0VA4QJBrRZMftzjA?e=BSuYih)
vicon_hdr2|---|16.9g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EVUTYAGK1a9HslLSffS3y9gBMQZYoZVxWPwaQUGLXzqVHQ?e=9N2zxZ)
vicon_hdr3|---|11.0g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/Eafi0sYdsrpBrkbDt06gqf4BDAj8_MvzTETE1Kx8E6dpSA?e=3GC44d)
vicon_hdr4|---|19.6g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EXt_PrUjWgxNimNDCH9oM2gBcypymHdVrMh5r0hQf1AdAA?e=cUfNMA)
vicon_lighttodark1|---|17.0g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EfOYBysbkRtApSy6-qaMHVEBO7z92UZiQRRhYWnzCW-M1Q?e=sdvcV4)
vicon_lighttodark2|---|12.0g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EXjiHBhmoMlOvtP_T-WP2sgBhJKu9oL9ZpMUIOq-trG4ww?e=rAnaKQ)

 


## Our Works in Event-based Vision
1. IROS2022
~~~
@inproceedings{GWPHKU:EVIO,
  title={Monocular Event Visual Inertial Odometry based on Event-corner using Sliding Windows Graph-based Optimization},
  author={Guan, Weipeng and Lu, Peng},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={******},
  year={2022},
  organization={IEEE}
}
~~~
2. PL-EVIO
~~~
@article{PL-EVIO,
  title={PL-EVIO: Robust Monocular Event-based Visual Inertial Odometry with Point and Line Features},
  author={Guan, Weipeng and Chen, Peiyu and Xie, Yuhan and Lu, Peng},
  journal={arXiv preprint arXiv:2209.12160},
  year={2022}
}
~~~
3. 


### If you have any suggestions or questions, do not hesitate to propose an issue. And if you find our dataset helpful in your research, a simple star or citation are the best affirmation for us.

## Other Event-based Dataset
* [M2DGR](https://github.com/SJTU-ViSYS/M2DGR)
*

## ACKNOWLEGEMENT

## Reference
* [Event Camera Calibration using dv-gui](https://blog.csdn.net/gwplovekimi/article/details/121637241?spm=1001.2014.3001.5501)


