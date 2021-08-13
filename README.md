# Projects with ESP32 and Arduino

I present some helpful code to use sensors with the popular chip **ESP32** in this repository. All codes are developed in **Aruco IDE.**

<div align="center">
<table> 
  <tr> 
    <td> <img src="/Images/arduino.png" width="100" height="75" alt="arduino"/> </td>
    <td> <img src="/Images/esp32.webp" width="200" height="200" alt="esp32"/> </td>
    <td><img src="/Images/LSM9DS1.jpg" width="100" height="100" alt="LSM9DS1"/></td>
    <td><img src="/Images/VL53L1X.jpeg" width="150" height="150" alt="VL53L1X"/></td>
  </tr>
</table>
</div>

+ **AHRS_LSM9DS1_IMU**: Test the __[Sebastian Madgwick's](http://www.x-io.co.uk/category/open-source/)__ orientation filter on __[LSM9DS1](https://www.st.com/en/mems-and-sensors/lsm9ds1.html)__ IMU.

+ **Calibration_VL53L1X_ToF**: Offset calibration of the Time-of-Flight ranging sensor __[VL53L1X](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html)__.

+ **Get_Unix_Timestamp**: Get Web Unix Timestamp.
+ **I2C_Scanner**: Displays the I2C address of the devices.
+ **LedBlink_Lilygo**: LilyGo **ESP32-MINI-32-V1.3 board** LED blink. LED can change blink frequency by changing delay time.
+ **Send_Remotely_Distance_Euler_LSM9DS1_VL53L1X**: Read and send remotely distance measured by **LSM9DS1** and orientation (Euler angles) of **VL53L1X** through Wi-Fi. Visualize the **VL53L1X** orientation using the **PDE** viewer.
+ **Send_Remotely_Distance_Quaternion_LSM9DS1_VL53L1X**: Read and send remotely distance measured by **LSM9DS1** and orientation (quaternion) of **VL53L1X** through Wi-Fi. Visualize the **VL53L1X** orientation using the **PDE** viewer.
+ **SendData_Http**: Test HTTP connexion. 
+ **Test_LSM9DS1_9DoF_IMU**: Read and send orientation (Euler angles) **VL53L1X** remotely through Wi-Fi.
+ **Test_ThingSpeak**: Send and Visualize **ESP32** data to the ThingSpeak platform.
+ **Test_VL53L1X_ToF**: Measure distance using the **VL53L1X** range sensor. 

## NOTE:

<p style="background:black">
<code style="background:black;color:red">If you find any of these codes helpful, please share my <a href="https://github.com/LuisOrtizF">GitHub</a> and STAR</code>:star:<code style="background:black;color:red">this repository to help other enthusiasts to find these tools. Remember, the knowledge must be shared. Otherwise, it is useless and lost in time.
</code>
</p>

## References:

Please cite the following papers if you use any of these codes in your projects:

```
@ARTICLE{Ortiz2021,
    AUTHOR  = {Ortiz-Fernandez, Luis E. and Cabrera-Avila, Elizabeth V. and Silva, Bruno M. F. da and Gonçalves, Luiz M. G.},
    TITLE   = {Smart Artificial Markers for Accurate Visual Mapping and Localization},
    JOURNAL = {Sensors},
    VOLUME  = {21},
    YEAR    = {2021},
    NUMBER  = {2},
    URL     = {https://www.mdpi.com/1424-8220/21/2/625},
    ISSN    = {1424-8220},
    DOI     = {10.3390/s21020625}
}
```
