<div align="center">

# Results (raw trajectories) of our methods

</div>

Since our source code is only internal-accessed and not publicly available, we release the results of our methods, including the estimated 6-DoF pose and the ground truth, in the form of `rosbag`. We strongly recommend the peers evaluate their proposed works using our [dataset](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM) and do the comparison with the raw results from our methods using their own accuracy criterion. 

Tips: We recommend this [Python package for the evaluation of odometry and SLAM](https://github.com/MichaelGrupp/evo).

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
 
 Evaluation in [VECtor](https://star-datasets.github.io/vector/)
  
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
