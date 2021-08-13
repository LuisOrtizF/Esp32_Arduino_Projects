// Include SparkFunLSM9DS1 library and its dependencies
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#include <WiFi.h>
#include "time.h"

#include <Eigen.h>// Calls main Eigen matrix class library
#include <Eigen/Geometry>
using namespace Eigen; // Eigen related statement; simplifies syntax for declaration of matrices

// Use the LSM9DS1 class to create an object.
LSM9DS1 imu;

#define SerialDebug true // set to true to get Serial output for debugging

int16_t tempCount; // temperature raw count output
float   temperature; // Stores the LSM9DS1gyro internal chip temperature in degrees Celsius

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0; // used to control display output rate
float pitch, yaw, roll;
float pitch_e, yaw_e, roll_e;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion 
// used to calculate integration interval
float eInt[3] = {0.0f, 0.0f, 0.0f}; // vector to hold integral error for Mahony method

// Variables for WIFI conection and get timestamp
const char* ssid       = "x";
const char* password   = "x";
//const char* ssid       = "x";
//const char* password   = "x";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

#define LEDPin 5

uint16_t initLSM9DS1()
{
  // Setup general device parameters
  // use I2C to communicate with the IMU
  imu.settings.device.commInterface = IMU_MODE_I2C;
  // sets the I2C address of the magnetometer.
  imu.settings.device.mAddress = 0x1E;
  // sets the I2C address of the accelerometer/gyroscope.
  imu.settings.device.agAddress = 0x6B;
  // cablibre accelerometer and gyroscope 
  imu.calibrate(true);
  // // cablibre magnetometer
  imu.calibrateMag(true);
  
  return imu.begin();
}

void printLocalTime()
{
  time_t now;
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Error: Failed to obtain time");
    return;
  }
  time(&now);
  Serial.print(now);
  Serial.println();
}

void setup() 
{
  Serial.begin(115200);
 
  uint16_t status = initLSM9DS1();

  if (!status)
  {
    Serial.println("Error: Failed to communicate with LSM9DS1!");
    while (1)
      ;
  }

  Serial.println("1. LSM9DS1 Conected.");

  //connect to WiFi
  pinMode(LEDPin, OUTPUT);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {    
      // Turn the LED on and wait.
      digitalWrite(LEDPin, HIGH);  
      delay(250);
      
      // Turn the LED off and wait.
      digitalWrite(LEDPin, LOW);
      delay(250);
  }
  Serial.println("2. WiFi Conected.");
  
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

void loop()
{
   
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
    imu.readGyro();
  if ( imu.accelAvailable() )
    imu.readAccel();
  if ( imu.magAvailable() )
    imu.readMag();
  if (imu.tempAvailable())
    imu.readTemp();

  gx = imu.calcGyro(imu.gx);
  gy = imu.calcGyro(imu.gy);
  gz = imu.calcGyro(imu.gz);
  ax = imu.calcAccel(imu.ax);
  ay = imu.calcAccel(imu.ay);
  az = imu.calcAccel(imu.az);
  mx = imu.calcMag(imu.mx);
  my = imu.calcMag(imu.my);
  mz = imu.calcMag(imu.mz);

  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  // Sensors x, y, and z axes of the accelerometer and gyro are aligned. The magnetometer  
  // the magnetometer z-axis (+ up) is aligned with the z-axis (+ up) of accelerometer and gyro, but the magnetometer
  // x-axis is aligned with the -x axis of the gyro and the magnetometer y axis is aligned with the y axis of the gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the LSM9DS1, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -mx,  my, mz);

  // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;
  
  if (delt_t > 500) { // update LCD once per half-second independent of read rate
    
  //    if(SerialDebug) {        
  //        Serial.print(" q0 = "); Serial.print(q[0]);
  //        Serial.print(" qx = "); Serial.print(q[1]); 
  //        Serial.print(" qy = "); Serial.print(q[2]); 
  //        Serial.print(" qz = "); Serial.println(q[3]); 
  //    }               
 
    tempCount = imu.temperature;
    if (tempCount > 100)
      temperature = ((float) tempCount/256. + 25.0); // Gyro chip temperature in degrees Centigrade
    else
      temperature = tempCount;
          
    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth. 
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   

    roll  *= 180.0f / PI;
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 21.0f; // Declination at Natal, RN 2019

    if(SerialDebug) {
      Serial.print("Roll, Pitch, Yaw, Temp, Time: ");
      Serial.print(roll, 2);
      Serial.print(", ");
      Serial.print(pitch, 2);
      Serial.print(", ");
      Serial.print(yaw, 2);
      Serial.print(", ");
      //    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
      // Print temperature in degrees Centigrade   
      Serial.print(temperature, 1);
      delay(250);
      Serial.print(", ");
      printLocalTime();
    }

    //    Quaternionf Q;
    //    Q.w() = q[0];
    //    Q.x() = q[1];
    //    Q.y() = q[2];
    //    Q.z() = q[3];
    //
    //    Vector3f angles = Q.toRotationMatrix().eulerAngles(2, 1, 0);
    //    yaw_e = angles[0]; pitch_e = angles[1]; roll_e = angles[2]; 
    //
    //    roll_e  *= 180.0f / PI;
    //    pitch_e *= 180.0f / PI;
    //    yaw_e   *= 180.0f / PI; 
    //    yaw_e   -= 21.0f; // Declination at Natal, RN 2019
        
    //    if(SerialDebug) {
    //      Serial.print("Roll_e, Pitch_e, Yaw_e: ");
    //      Serial.print(roll_e, 2);
    //      Serial.print(", ");
    //      Serial.print(pitch_e, 2);
    //      Serial.print(", ");
    //      Serial.println(yaw_e, 2);
    //      Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    //    }

    count = millis(); 
    sumCount = 0;
    sum = 0;
  }
}
