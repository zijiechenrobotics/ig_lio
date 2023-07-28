# iG-LIO

**The code will be released once the paper is accepted.**



This work proposes an incremental Generalized Iterative Closest Point (GICP) based tightly-coupled LiDAR-inertial odometry (LIO), iG-LIO, which addresses the challenges of integrating GICP into real-time LIO. 

![ig_lio_cover](figures/ig_lio_cover.png)

The preview video is available: 

**[YouTube ](https://www.youtube.com/watch?v=VyGc3_tXFSg&t=1s)**

**[bilibili](https://www.bilibili.com/video/BV1fj411X7U6/?vd_source=a23d841c4ace01eddfe9603b21e7891f)**

# Details about all sequences in the paper

We use abbreviations for all sequences due to limited space. The full names of all sequences are presented below.

| Abbreviation |                      Name                      | Distance(km) |
| :----------: | :--------------------------------------------: | :----------: |
|    nclt_1    |                   2012-01-15                   |     7.58     |
|    nclt_2    |                   2012-04-29                   |     3.17     |
|    nclt_3    |                   2012-05-11                   |     6.12     |
|    nclt_4    |                   2012-06-15                   |     4.09     |
|    nclt_5    |                   2013-01-10                   |     1.14     |
|    ncd_1     |              01_short_experiment               |     1.61     |
|    ncd_2     |               02_long_experiment               |     3.06     |
|    ncd_3     |             05_quad_with_dynamics              |     0.48     |
|    ncd_4     |              06_dynamic_spinning               |     0.09     |
|    ncd_5     |               07_parkland_mound                |     0.70     |
|    avia_1    |                hku_main_buiding                |     0.96     |
|    avia_2    | outdoor_Mainbuilding_100Hz_2020-12-24-16-46-29 |     0.14     |
|    avia_3    |     outdoor_run_100Hz_2020-12-27-17-12-19      |     0.09     |

# Mapping Results

We aligned the mapping results of iG-LIO with Google Earth and found that iG-LIO retains global consistency maps.

## NCLT 2012-05-11

![ig_nclt](figures/ig_nclt.png)

## Newer College Dataset 02_long_experiment

![ig_ncd](figures/ig_ncd.png)

## hku_main_building

![ig_hku](figures/ig_hku.png)

# Acknowledgements

Thanks for the below great open-source project for providing references to this work.

1. LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time)
2. [FAST-LIO](https://github.com/hku-mars/FAST_LIO)
3. [Faster-LIO](https://github.com/gaoxiang12/faster-lio)
4. [LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM)
5. [SLICT](https://github.com/brytsknguyen/slict)