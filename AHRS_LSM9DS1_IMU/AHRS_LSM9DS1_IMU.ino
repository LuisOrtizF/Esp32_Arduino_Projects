#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

#define SerialDebug true  // set to true to get Serial output for debugging

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
///float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta 1,3603495231756633

uint32_t delt_t = 0, count = 0;  // used to control display output rate

float deltat = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;  

float pitch, yaw, roll;
// used to calculate integration interval
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
  if (imu.begin() == false)
  {
    Serial.println("Error: Failed to communicate with LSM9DS1!");
    while (1);
  }

    imu.settings.gyro.sampleRate = 4; //238 Hz
    imu.settings.gyro.bandwidth = 1; // GBW_med, 29 Hz at Godr = 238 Hz
    imu.settings.accel.sampleRate = 4; //238 Hz
    imu.settings.accel.bandwidth = -3; // 50 Hz
    imu.settings.mag.sampleRate = 4; //10 Hz
    imu.settings.mag.XYPerformance = 2; // high performance
    imu.settings.mag.ZPerformance = 2; // high performance
 
  imu.calibrate(true);
  imu.calibrateMag(true);
}

void loop()
{  
  if ( imu.accelAvailable() )
    imu.readAccel();
   
  if ( imu.gyroAvailable() )
    imu.readGyro();

  if ( imu.magAvailable() )
    imu.readMag();

  ax = imu.calcAccel(imu.ax);
  ay = imu.calcAccel(imu.ay);
  az = imu.calcAccel(imu.az);
  gx = imu.calcGyro(imu.gx);
  gy = imu.calcGyro(imu.gy);
  gz = imu.calcGyro(imu.gz);
  mx = imu.calcMag(imu.mx);
  my = imu.calcMag(imu.my);
  mz = imu.calcMag(imu.mz);
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  
  // Sensors x, y, and z axes of the accelerometer and gyro are aligned. The magnetometer  
  // the magnetometer z-axis (+ up) is aligned with the z-axis (+ up) of accelerometer and gyro, but the magnetometer
  // x-axis is aligned with the -x axis of the gyro and the magnetometer y axis is aligned with the y axis of the gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the LSM9DS1, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  
  MadgwickQuaternionUpdate(ax, -ay, az, gx*PI/180.0f, -gy*PI/180.0f, gz*PI/180.0f,  -mx,  -my, mz);

  // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;

  Serial.println(deltat,5);
  
  if (delt_t > 500) { // update LCD once per half-second independent of read rate
               
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    //yaw   -= 21.2f; 
    roll  *= 180.0f / PI;
     
    if(SerialDebug) {
    //     Serial.print(roll);
    //     Serial.print(" ");
    //     Serial.print(pitch);
    //     Serial.print(" ");
    //     Serial.println(yaw);

    //      Serial.print(q[0]); Serial.print(" ");
    //      Serial.print(q[1]); Serial.print(" ");
    //      Serial.print(q[2]); Serial.print(" ");
    //      Serial.println(q[3]);
    }

    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform orientation for 
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!

    count = millis();    
  }
}






       







       
