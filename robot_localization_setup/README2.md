# robot_localization_setup
This ros1 package contains the necessary code to fuse the sensor data for obtaining the odometry of AUV.     

## Background :    

Currently there is no way for us to localize the robot and calculate it's odometry.   
So we will fuse the data from 2 sensors (AHRS & DVL) to calculate the odometry and also publish the dynamic odometry transform.   

In the package you only need to worry about the `src` folder containing the [sensor_fusion.py](src/sensor_fusion.py) file and the `launch` folder containing the [sensor_transform.launch](launch/sensor_transfom.launch) & [sensor_fusion.launch](launch/sensor_fusion.launch) files and ignore the [ekf_localization.launch](launch/ekf_localization.launch) file.

Also ignore the `params` folder as I was trying to use the *robot_localization* package but due to improper transforms between frames, I couldn't get it to work.       

- Implementing sensor fusion :       

    In the [sensor_transform.launch](launch/sensor_transfom.launch) file, i'm using `dvl_link` as the frame_id for the dvl as i used this name in the `vel_correction_pkg` but in case if you changed this name to kinematic_transform or something else, then just write that name instead.     

    Also i published some static transforms from base_link to the 2 sensors (considering the base_link is already in NED frame).    

    One more thing that I want you to do before running this code is to remove the default  transform on `/tf` topic from world to imu_link by making changes in either the yaml file or the launch file for the `xsens driver node` that is publishing the imu data. This transform was creating issues in the sensor fusion node so I tested sensor_fusion after removing this topic completely from the rosbag and published a new transform. So I want you to remove the tf part from the xsens driver that is publishing the /tf data and after that just run the [sensor_transform.launch](launch/sensor_transfom.launch) file (to publish static transform).      

    After that when all the sensor data is available in the ros environment, run the [sensor_fusion.launch](launch/sensor_fusion.launch) file.


## Running the package :   

Make sure all te other data is getting publishes correclty and there is no other transform initially.

- Run the static transforms for the 2 sensors from base_link

```bash
roslaunch robot_localization_setup sensor_transform.launch
```    

- Run the fusion node     

```bash
roslaunch robot_localization_setup sensor_fusion.launch
```     

- To visualize the odometry in rviz, open rviz and select `global_ned` as the Fixed frame in the *Global Options*.     

- The add the `TF` field and uncheck the `global_ned` frame.    

- Now you would see the odom frame as an NED frame and base_link moving wrt. odom in an NED coordinate frame.