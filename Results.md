<div align="center">

# Results (Raw Trajectories) of Our Methods

</div>
For the benfit of the community, we release the results of our methods, including the estimated 6-DoF pose and the ground truth, in the form of `rosbag` . 
Since our source code is only internal-accessed and not publicly available, we strongly recommend the peers evaluate their proposed works using our [dataset](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM) and do the comparison with the raw results from our methods using their own accuracy criterion. 

Tips: we recommend this [Python package for the evaluation of odometry and SLAM](https://github.com/MichaelGrupp/evo).

- [Our Own Dataset](#our-own-dataset)
- [Public Dataset](#public-dataset)
  - [VECtor](#VECtor)
  - [DAVIS 240C Datasets](#DAVIS-240C-Datasets)

# Our Own Dataset
Evaluation results in the [data sequences](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#Data-sequence-for-event-based-stereo-visual-inertial-odometry):
<div align="center"> which are designed for stereo event-based VIO. 
 
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

Evaluation results from monocular purely event-based VIO using different resolution event camera in [dataset](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#Data-sequence-for-event-based-monocular-visual-inertial-odometry):

<div align="center">

Sequence Name|[EIO](https://ieeexplore.ieee.org/document/9981970) in DAVIS346|[EIO](https://ieeexplore.ieee.org/document/9981970) in Dvxplorer|[PL-EIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#2-pl-evio) in DAVIS346|[PL-EIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#2-pl-evio) in Dvxplorer
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
Evaluation results in [VECtor](https://star-datasets.github.io/vector/):
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

## DAVIS 240C Datasets
Evaluation results in [DAVIS 240C Datasets](https://rpg.ifi.uzh.ch/davis_data.html) (EIO: purely event-based VIO; EVIO: Event+Image VIO):
<div align="center">
  
Sequence Name|[CVPR17 EIO](https://openaccess.thecvf.com/content_cvpr_2017/papers/Zhu_Event-Based_Visual_Inertial_CVPR_2017_paper.pdf)|[BMVC17 EIO](https://rpg.ifi.uzh.ch/docs/BMVC17_Rebecq.pdf)|[Ultimate SLAM EIO](https://rpg.ifi.uzh.ch/docs/RAL18_VidalRebecq.pdf)|[Ultimate SLAM EVIO](https://rpg.ifi.uzh.ch/docs/RAL18_VidalRebecq.pdf)|[3DV19 EIO](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/380462/1/3DV2019.pdf)|[RAL22 EVIO](https://arxiv.org/pdf/2204.05880.pdf)|[IROS22 EIO](https://ieeexplore.ieee.org/document/9981249)|Our [EIO](https://ieeexplore.ieee.org/document/9981970)|PL-EVIO|
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

BTW: both [BMVC17](https://rpg.ifi.uzh.ch/docs/BMVC17_Rebecq.pdf) and [Ultimate SLAM](https://rpg.ifi.uzh.ch/docs/RAL18_VidalRebecq.pdf) release their raw results of this dataset in [here](https://rpg.ifi.uzh.ch/ultimateslam.html). However, it seems that the released results is worse than the results of the paper.
