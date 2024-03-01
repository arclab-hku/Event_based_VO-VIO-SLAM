[![HitCount](https://hits.dwyl.com/arclab-hku/Event_based_VO-VIO-SLAM.svg?style=flat-square)](http://hits.dwyl.com/arclab-hku/Event_based_VO-VIO-SLAM)

<div align="center">

# Event-based Vision for VO/VIO/SLAM in Robotics

### Author: [Guan Weipeng](https://kwanwaipang.github.io/), [Chen Peiyu](https://github.com/cpymaple)
</div>

This is the repositorie that collects the dataset we used in our papers.
We also conclude our works in the field of event-based vision.
We hope that we can make some contributions for the development of event-based vision in robotics.

### If you have any suggestions or questions, do not hesitate to propose an issue. 

### If you find this repositorie is helpful in your research, a simple star or citation of our works should be the best affirmation for us. :blush: 


- [Dataset for Stereo EVIO](#Dataset-for-stereo-evio)
  - [Acquisition Platform](#acquisition-platform)
  - [Driver Installation](#driver-installation)
  - [Data Sequence](#data-sequence)
- [Dataset for Monocular EVIO](#Dataset-for-monocular-evio)
  - [Acquisition Platform](#acquisition-platform-1)
  - [Data Sequence](#data-sequence-1)
- [Modified Public Dataset](#Modified-public-dataset)
  - [Modified VECtor Dataset](#Modified-vector-dataset)
  - [Modified DSEC Dataset](#Modified-dsec-dataset)
- [Our Works in Event-based Vision](#our-works-in-event-based-vision)
  - [1. IROS2022](#1-iros2022)
  - [2. PL-EVIO](#2-pl-evio)
  - [3. ESVIO](#3-esvio)
  - [4. ECMD](#4-ecmd)
  - [5. EVI-SAM](#5-evi-sam)
- [Using Our Methods as Comparison](#using-Our-Methods-as-Comparison)
- [Recommendation](#recommendation)


# Dataset for Stereo EVIO
This dataset contains stereo event data at 60HZ and stereo image frames at 30Hz with resolution in 346 × 260, as well as IMU data at 1000Hz. 
Timestamps between all sensors are synchronized in hardware. 
We also provide ground truth poses from a motion capture system VICON at 50Hz during the beginning and end of each sequence, which can be used for trajectory evaluation.
To alleviate disturbance from the motion capture system’s infrared light on the event camera, we add an infrared filter on the lens surface of the DAVIS346 camera.
Note that this might cause the degradation of perception for both the event and image camera during the evaluation, but it can also further increase the challenge of our dataset for the only image-based method.

*This is a very challenge dataset for event-based VO/VIO, features aggressive motion and HDR scenarios. [EVO](https://github.com/uzh-rpg/rpg_dvs_evo_open), [ESVO](https://github.com/HKUST-Aerial-Robotics/ESVO), [Ultimate SLAM](https://github.com/uzh-rpg/rpg_ultimate_slam_open) are failed in most of the sequences*.
We think that parameter tuning is infeasible, therefore, we suggest the users use same set of parameters during the evaluation.
We hope that our dataset can help to push the boundary of future research on event-based VO/VIO algorithms, especially the ones that are really useful and can be applied in practice.



## Acquisition Platform
<div align="center">
<a target="_blank"><img src="ESVIO/quadrotor_flight.jpg" alt="image" width="80%" /></a>
<p> The Platform for Data Collection </p>
</div> 

* The configuration file is in [link](https://github.com/arclab-hku/Datasequence_Event_based_SLAM/tree/main/ESVIO)
* The DAVIS comprises an image camera and event camera on the same pixel array, thus calibration can be done using standard image-based methods, such as [Kalibr](https://github.com/ethz-asl/kalibr)
* We also provide the rosbag for stereo cameras and IMU calibration: [Calibration_bag](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/chenpyhk_connect_hku_hk/EqgJ7ahYWgpIomCbkSSOqqkBMpslIkhlwg7T9GTFMgyNjw?e=UchdI0).
* [Event Camera Calibration using Kalibr and imu_utils](https://blog.csdn.net/gwplovekimi/article/details/120948986)

## Driver Installation
We thanks the [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) for intructions of event camera driver.
<!-- We modified the source code of the [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) with consistent image size. -->
We add the function of the hardware synchronized for stereo setup, the source code is available in [link](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/driver%20code).
After installing the driver, the user can directly run the following command to run your stereo event camera:
~~~
roslaunch stereo_davis_open.launch
~~~

Tips: Users need to adjust the lens of the camera, such as the focal length, aperture.
Filters are needed for avoiding the interfere from infrared light under the motion capture system.
For the dvxplorer, the sensitive of event generation should be set, e.g. `bias_sensitivity`.
Users can visualize the event streams to see whether it is similiar to the edge map of the testing environments, and then fine-tune it.
Otherwise, the event sensor would output noise and is useless just like [M2DGR](https://github.com/SJTU-ViSYS/M2DGR).

## Data Sequence

In our VICON room:

<div align="center">

Sequence Name|Collection Date|Total Size|Duration|Features|One Drive|Baidu Disk
:--|:--:|:--:|:--:|:--:|:--:|:--:
hku_agg_translation|2022-10|3.63g|---|aggressive|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EfM2ytBNx7dIiIX4QkMQVGIBzVtMHEf4pl4EWA81iQZKEw?e=T6RA57)|[Rosbag](https://pan.baidu.com/s/173zuBK6W5AWQTLwE6yG1Nw?pwd=0yf2)
hku_agg_rotation|2022-10|3.70g|---|aggressive|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/ER6-1BaiPJVOjlyG0Pau-vAB3oJ8eHK7hTVb2GmOUTjMpg?e=SQdYxH)|[Rosbag](https://pan.baidu.com/s/1bC4XS8TfRo1A0UHqXguR2w?pwd=9vy0)
hku_agg_flip|2022-10|3.71g|---|aggressive|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EVPphYktymhEh4xF6Bp-S1EBdmHVj-YlBeDK1iu4_CakMg?e=6LuKuv)|[Rosbag](https://pan.baidu.com/s/1RokY4gx5yD0CLA4DHrlDfQ?pwd=thad)
hku_agg_walk|2022-10|4.52g|---|aggressive|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EYK8-WqZ001Pg6Kzdrau4NQBO7k21gahDB-22l1nyeKPkg?e=4pm4me)|[Rosbag](https://pan.baidu.com/s/1nY9YVAVuD4gz-DI-y7S28Q?pwd=bpqr)
hku_hdr_circle|2022-10|2.91g|---|hdr|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EZSKnWsvcbJHn9n0C69xAaUBtW7rnvBC7K59hximA8VrWg?e=TkgRye)|[Rosbag](https://pan.baidu.com/s/1HGwjBiTfeJi1ZipflbWRBQ?pwd=k29n)
hku_hdr_slow|2022-10|4.61g|---|hdr|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EfLPELUuMHtOhFilZytwbPkB0wKAoi7YcJP8ERG2f2HrSA?e=PuLVxP)|[Rosbag](https://pan.baidu.com/s/1lV5vdgfMUsfZPI9I4jZFGg?pwd=o5iz)
hku_hdr_tran_rota|2022-10|3.37g|---|aggressive & hdr|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/ERcfiPRffxJIjTVtZFADEZ8BnlC7vYTULNfi3myBM-17rA?e=d5lJuJ)|[Rosbag](https://pan.baidu.com/s/1n0rAgehFXRURgQnIUjFaJQ?pwd=tfas)
hku_hdr_agg|2022-10|4.43g|---|aggressive & hdr|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EZnpX0eqUc5MsR_mkNi7IEsBbLOI_GM9NRZebZvRQZHYEQ?e=qDGvN5)|[Rosbag](https://pan.baidu.com/s/1chGSI4_wfBudAmG791tPiA?pwd=rzyf)
hku_dark_normal|2022-10|4.24g|---|dark & hdr|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/Edhb8MveJlVEhnltVt4vBIsB2kJu3K4t8dW1MTLgsJ5gLQ?e=wqwzNs)|[Rosbag](https://pan.baidu.com/s/1z9sSwr9J_ZbDuquK0ONUug?pwd=b28q)

</div>

Outdoor large-scale (outdoor without ground truth):

The path length of this data sequence is about 1866m, which covers the place around 310m in length, 170m in width, and 55m in height changes, from Loke Yew Hall to the Eliot Hall and back to the Loke Yew Hall in HKU campus.
That would be a nice travel for your visiting the HKU :heart_eyes: Try it！

<div align="center">
  
Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag
:--|:--:|:--:|:--:|:--:|:--:
hku_outdoor_large-scale|2022-11|67.4g|34.9minutes|Indoor+outdoor; large-scale|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EaqqkI6DvapGsJ0PiYtYSXYBdgUM4szEKdozxklqfBDRcg?e=rHaYXr)
  
</div>



# Dataset for Monocular EVIO
You can use these data sequence to test your monocular EVIO in different resolution event cameras.
The`DAVIS346 (346x260)` and `DVXplorer (640x480)`are attached together (shown in Figure) for facilitating comparison. 
All the sequences are recorded in HDR scenarios with very low illumination or strong illumination changes through switching the strobe flash on and off.
We also provide indoor and outdoor large-scale data sequence.

## Acquisition Platform

<div align="center">
<a target="_blank"><img src="IROS2022/sensor_setup.png" alt="image" width="100%" /></a>
<p> The Platform for Data Collection </p>
</div>

* The configuration file is in [link](https://github.com/arclab-hku/Datasequence_Event_based_SLAM/tree/main/IROS2022)


## Data Sequence
With VICON as ground truth:

<div align="center">

Sequence Name|Collection Date|Total Size|Duration|Features|One Drive|Baidu Disk
:--|:--:|:--:|:--:|:--:|:--:|:--:
vicon_aggressive_hdr|2021-12|23.0g|---|HDR, Aggressive Motion|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/ESxBPJlRT4FApeMZgwvAo4YBuAhoOT5tcb_A9dAvPSEeeg?e=CRDVrD)|[Rosbag](https://pan.baidu.com/s/1VDn5AHfkr-5bkuxedRycAQ?pwd=7dwt)
vicon_dark1|2021-12|10.5g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EaY7bfm8ZytGvlFP3v1TNHgBXMubjQvjiuoiZVqqEmA2jA?e=OyZyyU)|[Rosbag](https://pan.baidu.com/s/1XhPKF7nL0KUCTcMsSDLpIQ?pwd=4r4l)
vicon_dark2|2021-12|16.6g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/Ed1hZLF4mOJJlz8nuk92evYByN9PkbrJE_xS8yuKy14ZUg?e=gYqWbg)|[Rosbag](https://pan.baidu.com/s/1Dm21VKYeHNFhPhhgL1vDyw?pwd=c1g3)
vicon_darktolight1|2021-12|17.2g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EQwioJi0GqlKmc7j4BcDyQEB-YrX6HSk_FsEavKFYoihYw?e=24ZYdR)|[Rosbag](https://pan.baidu.com/s/1cRDwKgMRIF37MD8YLbh5BA?pwd=g2sy)
vicon_darktolight2|2021-12|14.4g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EWLO58HfLOxNpFdEQzJgZaoBq4Mo74ceZGcgUYlMLhUJbg?e=JNjn1x)|[Rosbag](https://pan.baidu.com/s/1-NNRUU-5D9C-MaWGqTrarA?pwd=zzwh)
vicon_hdr1|2021-12|13.7g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EfGW22iMVwZEoVCOdZ9cuHYB2_ZUXR0VA4QJBrRZMftzjA?e=BSuYih)|[Rosbag](https://pan.baidu.com/s/1btjK2A61mXJpSP3Ib5wxhQ?pwd=feyv)
vicon_hdr2|2021-12|16.9g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EVUTYAGK1a9HslLSffS3y9gBMQZYoZVxWPwaQUGLXzqVHQ?e=9N2zxZ)|[Rosbag](https://pan.baidu.com/s/1MZSU9744zomKmhdsRlmxdQ?pwd=8awi)
vicon_hdr3|2021-12|11.0g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/Eafi0sYdsrpBrkbDt06gqf4BDAj8_MvzTETE1Kx8E6dpSA?e=3GC44d)|[Rosbag](https://pan.baidu.com/s/1v1d_JYW6tO0Tm-2ZwDMrEA?pwd=uox1)
vicon_hdr4|2021-12|19.6g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EXt_PrUjWgxNimNDCH9oM2gBcypymHdVrMh5r0hQf1AdAA?e=cUfNMA)|[Rosbag](https://pan.baidu.com/s/1zy97jnPdeUvoMUevsUvHcg?pwd=np1a)
vicon_lighttodark1|2021-12|17.0g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EfOYBysbkRtApSy6-qaMHVEBO7z92UZiQRRhYWnzCW-M1Q?e=sdvcV4)|[Rosbag](https://pan.baidu.com/s/1suptygmLuZNtig3EjjhqqQ?pwd=5r2j)
vicon_lighttodark2|2021-12|12.0g|---|HDR|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EXjiHBhmoMlOvtP_T-WP2sgBhJKu9oL9ZpMUIOq-trG4ww?e=rAnaKQ)|[Rosbag](https://pan.baidu.com/s/1VcJNUuuk0grCPZwyKn8z6A?pwd=ems2)

</div>

indoor (no ground truth):

<div align="center">

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag (Baidu Disk)
:--|:--:|:--:|:--:|:--:|:--:
indoor_aggressive_hdr_1|2021-12|16.62g|---|HDR, Aggressive Motion|[Rosbag](https://pan.baidu.com/s/1xdljGoQEMBAIC55wxQhgcQ?pwd=nadf)
indoor_aggressive_hdr_2|2021-12|15.66g|---|HDR, Aggressive Motion|[Rosbag](https://pan.baidu.com/s/13PcgEKDU4EPlOwwPo9y_pg?pwd=yahy)
indoor_aggressive_test_1|2021-12|17.94g|---|Aggressive Motion|[Rosbag](https://pan.baidu.com/s/1qchTTgwDtJoMY10slygIPQ?pwd=1xma)
indoor_aggressive_test_2|2021-12|8.385g|---|Aggressive Motion|[Rosbag](https://pan.baidu.com/s/1utMd8Leeu1pCRp8Jt-ynnw?pwd=w80j)
indoor_1|2021-12|3.45g|---|---|[Rosbag](https://pan.baidu.com/s/1xKOSpBET2UeXjXs9d_sdug?pwd=uxgm)
indoor_2|2021-12|5.31g|---|---|[Rosbag](https://pan.baidu.com/s/1say6YL8KJTS2W1ekh1MZXw?pwd=42nx)
indoor_3|2021-12|5.28g|---|---|[Rosbag](https://pan.baidu.com/s/1ZfafBCgThVqq8tksaIJFMw?pwd=8bke)
indoor_4|2021-12|6.72g|---|---|[Rosbag](https://pan.baidu.com/s/1OfM0IbvIW_sFZEQnAnDbDQ?pwd=1euv)
indoor_5|2021-12|13.79g|---|---|[Rosbag](https://pan.baidu.com/s/1M_jrtrgNCpAubCTI_pKhRQ?pwd=yp63)
indoor_6|2021-12|20.39g|---|---|[Rosbag](https://pan.baidu.com/s/1cgv25oD-fTjcFpzuy5_seA?pwd=xn0h)

</div>

Outdoor (no ground truth):

<div align="center">

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag (Baidu Disk)
:--|:--:|:--:|:--:|:--:|:--:
indoor_outdoor_1|2021-12|20.87g|---|******|[Rosbag](https://pan.baidu.com/s/1AYwnhOZZu2va0bszIEbg-g?pwd=xtqb)
indoor_outdoor_2|2021-12|39.5g|---|******|[Rosbag](https://pan.baidu.com/s/1ubxhjOx4v0Z-5ZyjWhdrNw?pwd=8b6t)
outdoor_1|2021-12|5.52g|---|******|[Rosbag](https://pan.baidu.com/s/1841QZOil0NX-K581zusx3g?pwd=mpty)
outdoor_2|2021-12|5.27g|---|******|[Rosbag](https://pan.baidu.com/s/1bSIfqt6Giado7fcGA4bErQ?pwd=ywyg)
outdoor_3|2021-12|6.83g|---|******|[Rosbag](https://pan.baidu.com/s/1V3QQqT__n71JqvbNJ4Lihg?pwd=no7f)
outdoor_4|2021-12|7.28g|---|******|[Rosbag](https://pan.baidu.com/s/1pjmK_1b8PLxYeOP4S0UooQ?pwd=dgbh)
outdoor_5|2021-12|7.26g|---|******|[Rosbag](https://pan.baidu.com/s/1sFMEDagu5Q84kGzrnYVwMw?pwd=amba)
outdoor_6|2021-12|5.38g|---|******|[Rosbag](https://pan.baidu.com/s/1D_LofnWzC77fxY5nZ9ENnQ?pwd=vb83)
outdoor_round1|2021-12|11.27g|---|******|[Rosbag](https://pan.baidu.com/s/1tjuQwCUCmHPxFqO7WFcX0g?pwd=aym5)
outdoor_round2|2021-12|13.34g|---|******|[Rosbag](https://pan.baidu.com/s/1LRAPnohws1KV_bhZSeC9_g?pwd=9zff)
outdoor_round3|2021-12|37.26g|---|******|[Rosbag](https://pan.baidu.com/s/1si1BCYC8W77_xSJDrftZ8g?pwd=wl7c)

</div>

On quadrotor platform (sample sequence in our PL-EVIO work):

We also provide the data squences that are collected in the flighting quadrotor platform using DAVIS346.

<div align="center">
<a target="_blank"><img src="PL-EVIO/sensor_setup.jpg" alt="image" width="100%" /></a>
<p> The Platform for Data Collection </p>
</div>

* The configuration file is in [link](https://github.com/arclab-hku/Datasequence_Event_based_SLAM/tree/main/PL-EVIO)

<div align="center">

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag
:--|:--:|:--:|:--:|:--:|:--:
Vicon_dvs_fix_eight|2022-08|1.08g|---|quadrotor flighting|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EeObT20UhbdDpemA3ZFwn7oB0UmbAmgqVObiQYwlZiBQCQ?e=j5H4ZU)
Vicon_dvs_varing_eight|2022-08|1.48g|---|quadrotor flighting|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/wpguan_connect_hku_hk/EWZPnY_Jr1lBiS2uglBysOIBEKdnHyyIGFqgg_oiVXT0BQ?e=UgrBCm)
outdoor_large_scale1|2022-08|9.38g|16 minutes|******|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EY7bTDAc6T5KkRgSB_VhhqYBnVdBYE80dJHwil7sVAeLMw?e=SN4PVU)
outdoor_large_scale2|2022-08|9.34g|16 minutes|******|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EQ5iXzEXjOFNvbqTMuuK03UBkcY7lDOCRuX0HwyZpR2blw?e=cCJQsu)

</div>



# Modified Public Dataset
## Modified VECtor Dataset

[VECtor dataset](https://star-datasets.github.io/vector/) covering the full spectrum of motion dynamics, environment complexities, and illumination conditions for both small and large-scale scenarios.
We modified the frequency of the event_left and event_right (60Hz) and the message format from "prophesee_event_msgs/EventArray" to "dvs_msgs/EventArray" in the [VECtor dataset](https://star-datasets.github.io/vector/), so that there is more event information in each frame and we can extract effective point and line features from the event stream. We release this modified VECtor Dataset to facilitate research on event camera. For the convenience of the user, we also fuse the individual rosbag from different sensors together (left_camera, right_camera, left_event, right_event, imu, groundtruth).

<div align="center">
<a target="_blank"><img src="Others/overview of Vector.png" alt="image" width="100%" /></a>
<p> Overview of Vector dataset </p>
</div> 

<div align="center">

Sequence Name|Total Size|One Drive|Baidu Disk
:--|:--:|:--:|:--:
board-slow|3.18g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EQcIz-Kf18pMl301YDr8KhQBfeziKZlb1zRBMWZBIezKLg?e=GTWE3t)|[Rosbag]()
corner-slow|3.51g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/ET0mYH9gDkVHuBmveuxPa8MB__oW7ti6H4a_JxduDglICw?e=TNMkyl)|[Rosbag](https://pan.baidu.com/s/1r_LrLyDW0PY0aZ4WMf1XKg?pwd=8km1)
robot-normal|3.39g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EUwmAXpA39hIvMTRIJvcZhQBBrj95f6E-MhKkVuvovqadw?e=mLFIcb)|[Rosbag](https://pan.baidu.com/s/1AlT_jS5JKoTGmHdvUXRe7w?pwd=8vu9)
robot-fast|4.23g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EUxG6axtQJxEquh79ZXDsX8BhhGq3QwRjW4MBz8xTXgPcg?e=In6eTJ)|[Rosbag](https://pan.baidu.com/s/1e_VdXYlnoEDjDrRpB3I8sg?pwd=3upg)
desk-normal|8.82g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/ESIpNEnxygNIhevd_2eMx3IB8a2qke2CqFWI6E_tCsN39Q?e=dtsnNu)|[Rosbag](https://pan.baidu.com/s/1e0XTXyeCLW90ywnv_Xf7og?pwd=nrxy)
desk-fast|10.9g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EbtCM0It1R1FlEBk7XrcyWYB_CmOMNWtgL-8oGGg0uGylA?e=53WJlT)|[Rosbag](https://pan.baidu.com/s/1uXJbGoPff1T-r3IVSwTaCQ?pwd=mj34)
sofa-normal|10.8g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EURPc2bQMkhOqw-ppGHPfqkBSCTLLucJCPHS53KZiJq9NA?e=147MeU)|[Rosbag](https://pan.baidu.com/s/1RrgQvoLx9Rg0uK3KueHRuQ?pwd=4suu)
sofa-fast|6.7g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/ETQ16aZMg_RKqD7r7rTzpzUB3L1RNrUsxOqP2StB8PSPtA?e=Zh32Mn)|[Rosbag](https://pan.baidu.com/s/1KX1dHioLfEvZSk278TdPsA?pwd=52tn)
mountain-normal|10.9g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EfXGQD3k9uJDpgE6dkdo1-4BOueKcH3gLV-Y5mxZ6J-FlA?e=Lk6pht)|[Rosbag](https://pan.baidu.com/s/1Paz-OtYVIlRYAKi4_TcHLg?pwd=qxld)
mountain-fast|16.6g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/ERy_KiwAVmRHuHpMfNEruRkB3N8AKNcoz4PhM-D3BNVYhg?e=6Yuuau)|[Rosbag](https://pan.baidu.com/s/1Eg4hu4YvF-LKZtlU5pnSsA?pwd=s7w1)
hdr-normal|7.73g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/Ea0NpmfVv1ZPuzuMji23zugBwcAx5jpk1AIWSdsyOwJwCA?e=viYiOp)|[Rosbag](https://pan.baidu.com/s/1kBGXDhxF1bWc3DEXOU2MSg?pwd=wdlo)
hdr-fast|13.1g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/ERggGm5O8mZKgMFcRGL9PrMBTerkSbiNZujROqQtUqBeNg?e=xCMtrZ)|[Rosbag](https://pan.baidu.com/s/1ykGfjJZuJ5hm7pm9WChxCw?pwd=swcj)
corridors-dolly|7.78g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/Ebd9sRvWt5NDuQm98pFt2moB8tUBXW6jVe5KEnzIu5QVhQ?e=G4PgKQ)|[Rosbag](https://pan.baidu.com/s/1W78Ag2auBO3rMUd5hw3LTw?pwd=d282)
corridors-walk|8.56g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EagcclyPjMRPsTjM_0DtE-wBhZKCGiIXFHggPAWiX-OaBw?e=VMgWiE)|[Rosbag](https://pan.baidu.com/s/1PVrk3UF6XcT6bknlCp6x9A?pwd=4ixi)
school-dolly|12.0g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EX4nOOrn0SFDrn14xncjINYBJA-YGjucLRUaJtEisoU8AQ?e=Tu3bAv)|[Rosbag](https://pan.baidu.com/s/1EIeg1SumhFMADTlMKF6hHA?pwd=edn6)
school-scooter|5.91g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EXrOFgvdxh5Oja9wi7Kin_4Bbzgc15QtFkjYjVCqy20xWg?e=V5hiMq)|[Rosbag](https://pan.baidu.com/s/1fa8IHTSsTDsanJRFgOsoCw?pwd=0epu)
units-dolly|18.5g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/Ea-XpjMCUoJDuAQ9mwVo6IcBzYTz-twRRL2VfQmfUkq02g?e=yJ9VSb)|[Rosbag](https://pan.baidu.com/s/1ub8YRaNSyIPhT31FpJcSvA?pwd=vtvs)
units-scooter|11.6g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/ERKuQIFBDP5FgxA_fqkTP0MB8xsVJ9l3aVUlDGjoZIK1bQ?e=FL3yDk)|[Rosbag](https://pan.baidu.com/s/13TKhx6Leysdw2TJ5FU6nBg?pwd=q08j)

</div>


## Modified DSEC Dataset
[DSEC](https://dsec.ifi.uzh.ch/) is a stereo camera dataset in driving scenarios that contains data from two monochrome event cameras and two global shutter color cameras in favorable and challenging illumination conditions. 
In addition, it also collects Lidar data, IMU and RTK GPS measurements.
However, the data sequence of different sensors in DSEC are divided and in different data formats, which is very unfriendly to users.
Therefore, we convert them into same rosbag which might be easier for event-based VIO evaluation.
The code of processing the data can be also available in [here](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/tree/main/Others/process_data).

<div align="center">
<a target="_blank"><img src="Others/overview of DSEC.gif" alt="image" width="100%" /></a>
<p> Overview of DSEC dataset </p>
</div> 

<div align="center">

Sequence Name|Total Size|One Drive
:--|:--:|:--:
zurich city 04 (a)|13.8g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EZAES7i9y39ElJ9iuyGcuwUBnJM9V6Mds3JatJmBuvME8g?e=xhgsN8)
zurich city 04 (b)|5.33g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/Edz6DSndhcRKsNWsrnCPw0kBG1MyxwVi-nr2ErtYHKVnig?e=yH9tEV)
zurich city 04 (c)|18.7g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/Eek93pMg9CBKotKSv-tU8g0Baxu9nOcGFja8JTA6GFsHrw?e=G7z4TO)
zurich city 04 (d)|15.5g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/Ef4FUqhPxzlIr_xWMR0NJCYBloeiiXuoTu-vnqNNosEu8g?e=DYDttw)
zurich city 04 (e)|4.94g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EeWJC5upKztDpfWrg4oCo0QBNfPnXGNoTCh3QlZyFgpOSg?e=GWvLvQ)
zurich city 04 (f)|15.1g|[Rosbag](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chenpyhk_connect_hku_hk/EVePcqU3ex1ColT58_ZtOBkB2JRie3ZZzXngkGmL8v873Q?e=F6bQDD)

</div>



# Our Works in Event-based Vision
## 1. IROS2022
This work proposed event inertial odometry (EIO).
We do not rely on the use of image-based corner detection but design a asynchronously detected and uniformly distributed event-cornerdetector from events-only data.
The event-corner features tracker are then integrated into a sliding windows graph-based optimization framework that tightly fuses the event-corner features with IMU measurement to estimate the 6-DoF ego-motion.
* PDF can be downloaded [here](https://ieeexplore.ieee.org/document/9981970)
* [Results (raw trajectories)](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/Results_for_comparison.md)
<!-- * Code is available in [internal-accessed link](https://github.com/arclab-hku/EVIO/tree/evio_mono_noetic) -->

<div align="center">
<a href="https://b23.tv/Xe8MZyt" target="_blank"><img src="IROS2022/cover.jpg" alt="video" width="100%" /></a>
<p> Demo Video (click the image to open video demo) </p>
</div>

~~~
@inproceedings{GWPHKU:EIO,
  title={Monocular Event Visual Inertial Odometry based on Event-corner using Sliding Windows Graph-based Optimization},
  author={Guan, Weipeng and Lu, Peng},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={2438-2445},
  year={2022},
  organization={IEEE}
}
~~~

## 2. PL-EVIO 
This work proposed the event-based visual-inertial odometry (EVIO) framework with point and line features, including: pruely event (PL-EIO) and event+image (PL-EVIO).
It is reliable and accurate enough to provide onboard pose feedback control for the quadrotor to achieve aggressive motion, e.g. flipping.
* This work is accepted by T-ASE and simultaneously transferred to ICRA2024. PDF can be downloaded [here](https://ieeexplore.ieee.org/document/10287884)
* [Results (raw trajectories)](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/Results_for_comparison.md)
* An extended version of our PL-EVIO: realizing high-accurate 6-DoF pose tracking and 3D semi-dense mapping (monocular event only) can be seen in [Link](https://www.bilibili.com/video/BV1924y1y7pn/?spm_id_from=333.999.0.0&vd_source=a88e426798937812a8ffc1a9be5a3cb7)
<!-- * Code is available in [internal-accessed link](https://github.com/arclab-hku/EVIO/tree/PL-EIO)  -->


<div align="center">
<a href="https://b23.tv/OE3QM6j" target="_blank"><img src="PL-EVIO/cover.jpg" alt="video" width="100%" /></a>
<p> Demo Video (click the image to open video demo) </p>
</div>

<div align="center">
<a href="https://www.bilibili.com/video/BV1i24y1R7KV/?spm_id_from=333.999.list.card_archive.click&vd_source=a88e426798937812a8ffc1a9be5a3cb7" target="_blank"><img src="PL-EVIO/PLEVIO_flip_3.gif" alt="video" width="100%" /></a>
<p> Onboard Quadrotor Flip using Our PL-EVIO (click the gif to open video demo)</p>
</div>

<!-- <div align="center">
<a href="https://www.bilibili.com/video/BV1i24y1R7KV/?spm_id_from=333.999.list.card_archive.click&vd_source=a88e426798937812a8ffc1a9be5a3cb7" target="_blank"><img src="PL-EVIO/flip.jpg" alt="video" width="100%" /></a>
<p> Onboard Quadrotor Flip using Our PL-EVIO (click the image to open) </p>
</div> -->

~~~
@article{GWPHKU:PL-EVIO,
  title={PL-EVIO: Robust Monocular Event-based Visual Inertial Odometry with Point and Line Features},
  author={Guan, Weipeng and Chen, Peiyu and Xie, Yuhan and Lu, Peng},
  journal={IEEE Transactions on Automation Science and Engineering},
  year={2023}
}
~~~

## 3. ESVIO
This work proposed the first stereo event-based visual inertial odometry framework, including ESIO (purely event-based) and ESVIO (event with image-aided).
The stereo event-corner features are temporally and spatially associated through an event-based representation with spatio-temporal and exponential decay kernel.
The stereo event tracker are then tightly coupled into a sliding windows graph-based optimization framework for the estimation of ego-motion.
* This work is accepted by RAL and simultaneously transferred to [IROS2023](https://www.bilibili.com/video/BV1Ju411P7sq/?spm_id_from=333.999.0.0&vd_source=a88e426798937812a8ffc1a9be5a3cb7). PDF can be downloaded [here](https://ieeexplore.ieee.org/document/10107754)
* The supplementary material is available in [link](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/tree/main/ESVIO/supply)
* [Results (raw trajectories)](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/Results_for_comparison.md)
<!-- * Code is available in [internal-accessed link](https://github.com/arclab-hku/ESVIO) -->

<div align="center">
<a href="https://www.bilibili.com/video/BV1ve4y1M7v4/?share_source=copy_web&vd_source=a722388e07ea53f32d00aed0a0117f3c" target="_blank"><img src="ESVIO/ESVIO_hdr_flight _gif.gif" alt="video" width="100%" /></a>
<p> Onboard Quadrotor Flight using Our ESVIO as State Estimator (click the gif to open video demo)</p>
</div>

~~~
@article{GWPHKU:ESVIO,
  title={ESVIO: Event-based Stereo Visual Inertial Odometry},
  author={Chen, Peiyu and Guan, Weipeng and Lu, Peng},
  journal={IEEE Robotics and Automation Letters},
  year={2023},
  volume={8},
  number={6},
  pages={3661-3668},
  publisher={IEEE}
}
~~~

## 4. ECMD
ECMD is an event-based dataset for autonomous driving.
It provides data from two sets of stereo event cameras with different resolutions (640x480, 346x260), stereo industrial cameras, an infrared camera, a top-installed mechanical LiDAR with two slanted LiDARs, two consumer-level GNSS receivers, and an onboard IMU.
Meanwhile, the ground-truth of the vehicle was obtained using a centimeter-level high-accuracy GNSS-RTK/INS navigation system.
* The dataset is available at [here](https://arclab-hku.github.io/ecmd/). PDF can be downloaded [here](https://ieeexplore.ieee.org/document/10342726).

<div align="center">
<a href="https://www.bilibili.com/video/BV1pN411s79g/?spm_id_from=333.337.search-card.all.click&vd_source=c4be0359ec60c90d434f634ab4075470" target="_blank"><img src="ECMD/homepage_vis.gif" alt="video" width="100%" /></a>
<p> Overview of ECMD (click the gif to open video demo)</p>
</div>

~~~
@article{GWPHKU:ECMD,
  title={ECMD: An Event-Centric Multisensory Driving Dataset for SLAM},
  author={Chen, Peiyu and Guan, Weipeng and Huang, Feng and Zhong, Yihan and Wen, Weisong and Hsu, Li-Ta and Lu, Peng},
  journal={IEEE Transactions on Intelligent Vehicles},
  year={2023}
}
~~~

## 5. EVI-SAM
EVI-SAM is a full event-based SLAM system that tackle the problem of 6-DoF pose tracking and 3D dense mapping using the monocular event camera.
To the best of our knowledge, this is the first framework that employs a non-learning approach to achieve event-based dense and textured 3D reconstruction without GPU acceleration.
Additionally, it is also the first hybrid approach that integrates both direct-based and feature-based methods within an event-based framework.
* The data sequence and the hardware platform of our EVI-SAM is available at [here](***). PDF can be downloaded [here](https://arxiv.org/pdf/2312.11911.pdf).

<div align="center">
<a href="https://www.bilibili.com/video/BV19w411b7te/?spm_id_from=333.999.top_right_bar_window_history.content.click&vd_source=a88e426798937812a8ffc1a9be5a3cb7" target="_blank"><img src="EVI-SAM/first_image.png" alt="video" width="100%" /></a>
<p> Demo Video (click the image to open video demo) </p>
</div>

~~~
@article{GWPHKU:EVI-SAM,
  title={EVI-SAM: Robust, Real-time, Tightly-coupled Event-Visual-Inertial State Estimation and 3D Dense Mapping},
  author={Guan, Weipeng and Chen, Peiyu and Zhao, Huibin and Wang, Yu and Lu, Peng},
  journal={arXiv preprint arXiv:2312.11911},
  year={2023}
}
~~~


# Using Our Methods as Comparison
We strongly recommend the peers to evaluate their proposed method using our dataset, and do the comparison with the raw results from our methods using their own accuracy criterion.

The raw results/trajectories of our methods can be obtained in :point_right: [here](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/Results_for_comparison.md).



# Recommendation
* [Survey on Event-based 3D Reconstruction](https://www.bilibili.com/video/BV1mm4y1c7Rp/?spm_id_from=333.999.0.0)
* [Survey on Event Camera in AR/VR](https://www.bilibili.com/video/BV1ic411X7jC/?spm_id_from=333.999.0.0)
* [Event Camera Calibration using dv-gui](https://blog.csdn.net/gwplovekimi/article/details/121637241?spm=1001.2014.3001.5501)
* [Event Camera Calibration using Kalibr and imu_utils](https://blog.csdn.net/gwplovekimi/article/details/120948986)
* [Event Camera Simulation in Gazebo](https://blog.csdn.net/gwplovekimi/article/details/120347034?spm=1001.2014.3001.5502)
* The survey when I first meet "Event Camera" can be seen in [Blog](https://blog.csdn.net/gwplovekimi/article/details/115908307?spm=1001.2014.3001.5502)
* [Event-based Vision Resources](https://github.com/uzh-rpg/event-based_vision_resources)
* [The course: Event-based Robot Vision, by Prof. Guillermo Gallego](https://www.youtube.com/playlist?list=PL03Gm3nZjVgUFYUh3v5x8jVonjrGfcal8)
<!--
* Useful tools:
   - https://github.com/TimoStoff/event_utils
   - https://github.com/tub-rip/events_viz
-->



# LICENSE
This repositorie is licensed under MIT license. International License and is provided for academic purpose. If you are interested in our project for commercial purposes, please contact [Dr. Peng LU](https://arclab.hku.hk/People.html) for further communication.
