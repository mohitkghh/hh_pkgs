# hh_pkgs
This repository contains the necessary ros1 packages used for the AUV development.      
Please go through the separate README.md files for each paackage for their usage.

## 1. [README for `vel_correction_pkg`](vel_correction_pkg/README1.md)     
    - Contains the code for fusing the imu angular velocity & dvl velocity data to get the correct velocity of the auv in world frame.     

## 2. [README for `robot_localization_setup`](robot_localization_setup/README2.md)       
    - Contains the code for fusing the raw imu and corrected velocity data to get the odometry of the bot.      

## 3. [README for `testing_pkg`](testing_pkg/README3.md)      
    - Contains the code for analyzing various tests on sensor data such as FFT of imu data for vibrations.