# vel_correction_pkg
This ros1 package contains rhe code to correcting the auv velocity data by using the sensors IMU and DVL.

## Background : 

There is a misalignment of data from the 2 sensors (DVL and AHRS).     
For this we are going to use a ros node that will extract the data from those 2 sensors in real time and publish the correct data onto a certain topic.

- Implementing Kinematic Transform between AHRS & DVL : 

    Currently the local frame of DVL is already aligned with the global NED reference frame of our bot. So linear velocity data from the DVL is in the correct frame (though some scaling of their values needs to be done, but currently let's not bother about that).      

    But the local frame of AHRS is at some rotation wrt. the global NED frame. Due to this, the data from AHRS is coming in its own local frame which we can't fuse with the DVL directly.       

    I have attached a [notes.pdf](src/notes.pdf) file that you can use to visualize how exactly does this misalignment looks like.     

    So basically for this, we are implementing a kinematic transform between the data of those 2 sensors.    

    In the transform we are using the linear velocity from DVL on the ros topic `/dvl/data` and angular velocity from the AHRS on the topic `/imu/angular_velocity` and then calculating a corrected linear velocity of the bot.      

    But before using the angular velocity in the correction formula, we need to make sure that angular velocity has been transformed from the local frame of AHRS to the global NED frame.       

    The node [vel_correction_node.py](src/vel_correction_node.py) requires an offset vector (in global NED frame) that basically has the information about how far apart those 2 sensors are in the global NED frame.  
    So you need to measure the difference and then update the value accordingly in the node. 
    Also currently the node is using the raw angular velocity (in the local AHRS frame) for updation but you need to convert that raw angular velocity to the correct ned frame and then use the updated angular velocity in the correction formula. 
    How that correction is happening has been explained in the [notes.pdf](notes.pdf) file.    

    This node also publishes a crude odometry uding the corrected velocity data.

## Running the package :  

Once you have corrected the offset_vector & converted the angular velocity data to correct frame, run the correction node    

```bash
rosrun vel_correction_pkg vel_correction_node.py
```    

- This node publishes the corrected velocity data on the topic `/plant/corrected_velocity` with message type *TwistStamped* having frame_id `dvl_link`.     
- This node also publishes the crude odometry (using only the corrected velocity data) on the topic `plant/pose_from_corrected_vel` with message type *PoseStamped* having frame_id `dvl_link`.