<div align="center">

# Results (Raw Trajectories) of Our Methods

</div>

For the benfit of the community, we release the results of our methods, including the estimated 6-DoF pose and the ground truth, in the form of `rosbag`. 
Since our source code is only internal-accessed and not publicly available, we strongly recommend the peers to evaluate their proposed works using our [dataset](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM) and do the comparison with the raw results from our methods using their own accuracy criterion.

The evaluations of our works in different data sequences are also recorded in video which can be seen in our bilibili: [Guan Weipeng](https://space.bilibili.com/499377825?spm_id_from=333.1007.0.0) or [Chen Peiyu](https://space.bilibili.com/279299582/?spm_id_from=333.999.0.0).

Tips: we recommend this [Python package for the evaluation of odometry and SLAM](https://github.com/MichaelGrupp/evo).

- [Our Own Dataset](#our-own-dataset)
- [Public Dataset](#public-dataset)
  - [VECtor](#VECtor)
  - [DAVIS 240C Datasets](#DAVIS-240C-Datasets)

# Our Own Dataset
Evaluation results in the [dataset](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#Data-sequence-for-event-based-stereo-visual-inertial-odometry) which are designed for stereo event-based VIO. 
Although this dataset uses stereo event cameras, it also would be good choice for the evaluation of monocular event-based VIO method.
[The results (raw trajectory) of PL-EVIO, ESIO and ESVIO](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/chenpyhk_connect_hku_hk/EsuuA9IzsolAl_J9GPB54R0B6NsJgJqa_wgvgTK3YZDLLg?e=639B7L).

The accuracy is measured with absolute trajectory error (ATE) aligning the estimated trajectory with ground truth using 6-DOF transformation (in SE3), which is calculated by the public available tool [evo](https://github.com/MichaelGrupp/evo).
`Unit: m` (e.g. 0.24 means the average error would be 0.24m in the sequence). 
For example, you can run the following command to evaluate the accuracy:

~~~
evo_ape bag ESVIO_hku_hdr_agg.bag /cpy_uav/viconros/odometry /pose_graph/evio_odometry -va -p
~~~

<div align="center"> 
 
Sequence Name|[PL-EVIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#2-pl-evio)|[ESIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#3-esvio)|[ESVIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#3-esvio)|
:--|:--:|:--:|:--:
hku_agg_translation|0.048|0.35|0.063
hku_agg_rotation|0.15|0.51|0.11
hku_agg_flip|0.15|1.23|0.14
hku_agg_walk|0.37|1.14|0.27
hku_hdr_circle|0.068|0.23|0.081
hku_hdr_slow|0.069|0.17|0.059
hku_hdr_tran_rota|0.068|0.60|0.065
hku_hdr_agg|0.14|1.37|0.10
hku_dark_normal|1.25|0.32|0.39
</div>


Evaluation results from monocular purely event-based VIO using different resolution event cameras `DAVIS346 (346x260)` and `DVXplorer (640x480)` in [dataset](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#Data-sequence-for-event-based-monocular-visual-inertial-odometry):

<div align="center">

Sequence Name|[EIO](https://ieeexplore.ieee.org/document/9981970) in DAVIS346|[EIO](https://ieeexplore.ieee.org/document/9981970) in DVXplorer|[PL-EIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#2-pl-evio) in DAVIS346|[PL-EIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#2-pl-evio) in DVXplorer
:--|:--:|:--:|:--:|:--:
vicon_aggressive_hdr|---|---|---|---
vicon_dark1|---|---|---|---
vicon_dark2|---|---|---|---
vicon_darktolight1|---|---|---|---
vicon_darktolight2|---|---|---|---
vicon_hdr1|---|---|---|---
vicon_hdr2|---|---|---|---
vicon_hdr3|---|---|---|---
vicon_hdr4|---|---|---|---
vicon_lighttodark1|---|---|---|---
vicon_lighttodark2|---|---|---|---

</div>


# Public Dataset
## VECtor
Evaluation results in [VECtor](https://star-datasets.github.io/vector/) which are designed for stereo event-based VIO.
Although this dataset uses stereo event cameras, it also would be good choice for the evaluation of monocular event-based VIO method.
[The results (raw trajectory) of PL-EVIO and ESVIO](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/chenpyhk_connect_hku_hk/EkYd25BE8hdGl08Stg8p8ecBHfrLkxyD_M6jSexAnBkxOQ?e=033lEb).

The accuracy is measured with absolute trajectory error (ATE) aligning the estimated trajectory with ground truth using 6-DOF transformation (in SE3), which is calculated by the public available tool [evo](https://github.com/MichaelGrupp/evo).
`Unit: m` (e.g. 0.24 means the average error would be 0.24m in the sequence). 
For example, you can run the following command to evaluate the accuracy:

~~~
evo_ape bag ESVIO_MVSEC_indoor_flying_1.bag /davis/left/odometry /pose_graph/evio_odometry -va -p
~~~
<div align="center">
  
Sequence Name|[PL-EVIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#2-pl-evio)|[ESVIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#3-esvio)|
:--|:--:|:--:
board-slow|---|---
corner-slow|0.017|0.012
robot-normal|0.027|0.043
robot-fast|0.037|0.042
desk-normal|0.31|0.052
desk-fast|0.043|0.042
sofa-normal|0.058|0.047
sofa-fast|0.050|0.052
mountain-normal|0.32|0.044
mountain-fast|0.031|0.039
hdr-normal|0.12|0.017
hdr-fast|0.036|0.039
corridors-dolly|1.23|0.88
corridors-walk|0.72|0.34
school-dolly|3.11|0.53
school-scooter|1.39|0.63
units-dolly|13.82|8.12
units-scooter|11.66|6.64

</div>

## MVSEC
Evaluation results in [MVSEC](https://daniilidis-group.github.io/mvsec/) which are designed for stereo event-based VIO.
Although this dataset uses stereo event cameras, it also would be good choice for the evaluation of monocular event-based VIO method.
[The results (raw trajectory) of PL-EVIO and ESVIO](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/chenpyhk_connect_hku_hk/EpemLX57EuNKq1mYJUNvFXIB1TUnbuGpcrfFC0sFstuy0Q?e=fEXHsd).

The accuracy is measured with absolute trajectory error (ATE) aligning the estimated trajectory with ground truth using 6-DOF transformation (in SE3), which is calculated by the public available tool [evo](https://github.com/MichaelGrupp/evo).
`Unit: m` (e.g. 0.24 means the average error would be 0.24m in the sequence). 
For example, you can run the following command to evaluate the accuracy:

~~~
evo_ape bag ESVIO_VECtor_corner_slow.bag /gt/pose /pose_graph/evio_odometry -va -p
~~~
<div align="center">
  
Sequence Name|[PL-EVIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#2-pl-evio)|[ESVIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#3-esvio)|
:--|:--:|:--:
Indoor Flying 1|0.36|0.25
Indoor Flying 2|0.30|0.30
Indoor Flying 3|0.34|0.25
Indoor Flying 4|0.44|0.46

</div>


## DAVIS 240C Datasets
Evaluation results in [DAVIS 240C Datasets](https://rpg.ifi.uzh.ch/davis_data.html). We also refer the raw results of the other event-based VIO works (EIO: purely event-based VIO; EVIO: Event+Image VIO) as following:
<div align="center">
  
Sequence Name|[CVPR17 EIO](https://openaccess.thecvf.com/content_cvpr_2017/papers/Zhu_Event-Based_Visual_Inertial_CVPR_2017_paper.pdf)|[BMVC17 EIO](https://rpg.ifi.uzh.ch/docs/BMVC17_Rebecq.pdf)|[Ultimate SLAM EIO](https://rpg.ifi.uzh.ch/docs/RAL18_VidalRebecq.pdf)|[Ultimate SLAM EVIO](https://rpg.ifi.uzh.ch/docs/RAL18_VidalRebecq.pdf)|[3DV19 EIO](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/380462/1/3DV2019.pdf)|[RAL22 EVIO](https://arxiv.org/pdf/2204.05880.pdf)|[IROS22 EIO](https://ieeexplore.ieee.org/document/9981249)|[Our IROS22 EIO](https://ieeexplore.ieee.org/document/9981970)|[PL-EVIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#2-pl-evio)|
:--|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:
boxes_translation      | 2.69 | 0.57 | 0.76             |0.27 | 2.55 | 0.48 |1.0| 0.34  | 0.06
hdr_boxes              | 1.23 | 0.92 | 0.67             |0.37 | 1.75 | 0.46 |1.8| 0.40  | 0.10
boxes_6dof             | 3.61 | 0.69 | 0.44             |0.30 | 2.03 | 0.84 |1.5| 0.61  | 0.21
dynamic_translation    | 1.90 | 0.47 | 0.59             |0.18 | 1.32 | 0.40 |0.9| 0.26  | 0.24
dynamic_6dof           | 4.07 | 0.54 | 0.38             |0.19 | 0.52 | 0.79 |1.5| 0.43  | 0.48
poster_translation     | 0.94 | 0.89 | 0.15             |0.12 | 1.34 | 0.35 |1.9| 0.40  | 0.54
hdr_poster             | 2.63 | 0.59 | 0.49             |0.31 | 0.57 | 0.65 |2.8| 0.40  | 0.12
poster_6dof            | 3.56 | 0.82 | 0.30             |0.28 | 1.50 | 0.35 |1.2| 0.26  | 0.14

</div>

Tips:
The estimated and ground-truth trajectories were aligned with a 6-DOF transformation (in SE3), using 5 seconds [0-5s] of the resulting trajectory. 
The result is obtained through computing the mean position error (Euclidean distance in meters) as percentages of the total traveled distance of the ground truth. 
`Unit: %/m` (e.g. 0.24 means the average error would be 0.24m for 100m motion). 
BTW: both [BMVC17](https://rpg.ifi.uzh.ch/docs/BMVC17_Rebecq.pdf) and [Ultimate SLAM](https://rpg.ifi.uzh.ch/docs/RAL18_VidalRebecq.pdf) release their raw results of this dataset in [here](https://rpg.ifi.uzh.ch/ultimateslam.html).
However, it seems that the released results is worse than the from of the paper.
