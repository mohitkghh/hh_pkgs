# testing_pkg     
This package contains various nodes and utility scripts to test & analyze the data.    

## IMU FFT analysis :   
- The [imu_fft_psd_node.py](src/imu_fft_psd_node.py) file contains the code to publish the Fast Fourier Transform (fft) & Power Spectral Density (psd) of imu's linear acceleration data.          

- To run the fft+psd node, make sure your imu data is available in the ros environment   

```bash
rosrun testing_pkg imu_fft_psd_node.py
```       

- The fft & psd data gets published on `/imu/fft/*` & `/imu/psd` topics respectively.    
