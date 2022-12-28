<div align="center">

# Results (raw trajectories) of our methods

</div>

Since our source code is only internal-accessed and not publicly available, we release the results of our methods, including the estimated 6-DoF pose and the ground truth, in the form of `rosbag`. We strongly recommend the peers evaluate their proposed works using our [dataset](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM) and do the comparison with the raw results from our methods using their own accuracy criterion. 

Tips: We recommend this [Python package for the evaluation of odometry and SLAM](https://github.com/MichaelGrupp/evo).

# Our own dataset
Evaluation Results in [Data Sequence for Event-based Stereo Visual-inertial Odometry](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#Data-sequence-for-event-based-stereo-visual-inertial-odometry):
<div align="center"> 
 
Sequence Name|PL-EVIO|ESIO|ESVIO|
:--|:--:|:--:|:--:
hku_agg_translation|---|---|---
hku_agg_rotation|---|---|---
hku_agg_flip|---|---|---
hku_agg_walk|---|---|---
hku_hdr_circle|---|---|---
hku_hdr_slow|---|---|---
hku_hdr_tran_rota|---|---|---
hku_hdr_agg|---|---|---
hku_dark_normal|---|---|---

</div>



<div align="center">

Sequence Name|[EIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#1-iros2022) in DAVIS346|[EIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#1-iros2022) in Dvxplorer|[PL-EIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#2-pl-evio) in DAVIS346|[PL-EIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#1-iros2022](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#2-pl-evio) in Dvxplorer
:--|:--:|:--:|:--:|:--:|:--:
vicon_aggressive_hdr|2021-12|23.0g|---|HDR, Aggressive Motion|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/ESxBPJlRT4FApeMZgwvAo4YBuAhoOT5tcb_A9dAvPSEeeg?e=CRDVrD)
vicon_dark1|2021-12|10.5g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EaY7bfm8ZytGvlFP3v1TNHgBXMubjQvjiuoiZVqqEmA2jA?e=OyZyyU)
vicon_dark2|2021-12|16.6g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/Ed1hZLF4mOJJlz8nuk92evYByN9PkbrJE_xS8yuKy14ZUg?e=gYqWbg)
vicon_darktolight1|2021-12|17.2g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EQwioJi0GqlKmc7j4BcDyQEB-YrX6HSk_FsEavKFYoihYw?e=24ZYdR)
vicon_darktolight2|2021-12|14.4g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EWLO58HfLOxNpFdEQzJgZaoBq4Mo74ceZGcgUYlMLhUJbg?e=JNjn1x)
vicon_hdr1|2021-12|13.7g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EfGW22iMVwZEoVCOdZ9cuHYB2_ZUXR0VA4QJBrRZMftzjA?e=BSuYih)
vicon_hdr2|2021-12|16.9g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EVUTYAGK1a9HslLSffS3y9gBMQZYoZVxWPwaQUGLXzqVHQ?e=9N2zxZ)
vicon_hdr3|2021-12|11.0g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/Eafi0sYdsrpBrkbDt06gqf4BDAj8_MvzTETE1Kx8E6dpSA?e=3GC44d)
vicon_hdr4|2021-12|19.6g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EXt_PrUjWgxNimNDCH9oM2gBcypymHdVrMh5r0hQf1AdAA?e=cUfNMA)
vicon_lighttodark1|2021-12|17.0g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EfOYBysbkRtApSy6-qaMHVEBO7z92UZiQRRhYWnzCW-M1Q?e=sdvcV4)
vicon_lighttodark2|2021-12|12.0g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EXjiHBhmoMlOvtP_T-WP2sgBhJKu9oL9ZpMUIOq-trG4ww?e=rAnaKQ)

</div>


# Public dataset
Evaluation Results in [VECtor](https://star-datasets.github.io/vector/):
<div align="center">
  
Sequence Name|PL-EVIO|ESIO|ESVIO|
:--|:--:|:--:|:--:
board-slow|---|---|---
corner-slow|---|---|---
robot-normal|---|---|---
robot-fast|---|---|---
desk-normal|---|---|---
desk-fast|---|---|---
sofa-normal|---|---|---
sofa-fast|---|---|---
mountain-normal|---|---|---
mountain-fast|---|---|---
hdr-normal|---|---|---
hdr-fast|---|---|---
corridors-dolly|---|---|---
corridors-walk|---|---|---
school-dolly|---|---|---
school-scooter|---|---|---
units-dolly|---|---|---
units-scooter|---|---|---

</div>
