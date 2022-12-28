<div align="center">

# Results (raw trajectories) of our methods

</div>

Since our source code is only internal-accessed and not publicly available, we release the results of our methods, including the estimated 6-DoF pose and the ground truth, in the form of `rosbag`. We strongly recommend the peers evaluate their proposed works using our [dataset](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM) and do the comparison with the raw results from our methods using their own accuracy criterion. 

Tips: We recommend this [Python package for the evaluation of odometry and SLAM](https://github.com/MichaelGrupp/evo).

# Our own dataset
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

Sequence Name|[EIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#1-iros2022) in DAVIS346|[EIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#1-iros2022) in Dvxplorer|[PL-EIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#2-pl-evio) in DAVIS346|[PL-EIO](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#2-pl-evio) in Dvxplorer
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


# Public dataset
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

Evaluation results in [DAVIS 240C Datasets](https://rpg.ifi.uzh.ch/davis_data.html):
<div align="center">
  
Sequence Name|Our EIO(https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#1-iros2022)|
:--|:--:
boxes_6dof|---
boxes_translation|---
dynamic_6dof|---
dynamic_translation|---
hdr_boxes|---
hdr_poster|---
poster_6dof|---
poster_translation|---

</div>



[here](https://rpg.ifi.uzh.ch/ultimateslam.html)
