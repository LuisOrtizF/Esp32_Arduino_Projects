// Include LSM9DS1 library
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

// Include VL53L1X library
#include "SparkFun_VL53L1X.h"

// Include WiFi library
#include <WiFi.h>
#include <HTTPClient.h>

// Include for multitasking
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

// Create an LSM9DS1 object
LSM9DS1 imu;

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f); 
float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // algorithm gain
float sampleFrec = 25.0f;
float deltat = 1/sampleFrec;        // integration interval for both filter schemes
                 
// vector to hold quaternion qw, qx, qy, qz
// quaternion of sensor frame relative to auxiliary frame
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; 

// Create an VL53L1X object
SFEVL53L1X distanceSensor;

// variables for WIFI conection
//const char* ssid       = "WiFiDCA";
//const char* password   = "udtqcssondodt";
//const char* serverName = "http://192.168.0.12/post-esp-data.php";

const char* ssid       = "x";
const char* password   = "x";
const char* serverName = "http://x.x.x.x/post-esp-data.php";
String apiKeyValue = "tPmAT5Ab3j7F9";

// variable to hold ESP32 unique ID 
String markeriD;

// variable dual core
TaskHandle_t Task1, Task2;
SemaphoreHandle_t baton;

// used to control display output rate
unsigned long microsPerReading, microsPrevious;

void codeForTask1( void * parameter )
{  
  // variables to hold latest sensor data values
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  float roll, pitch, yaw;
  unsigned long microsNow;
    
  for (;;) 
  {
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;
    
    xSemaphoreTake( baton, portMAX_DELAY );

    // update the sensor values whenever new data is available
    if( imu.gyroAvailable() )
      imu.readGyro();
    if( imu.accelAvailable() )
      imu.readAccel();
    if( imu.magAvailable() )
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

    // Sensors x, y, and z axes of the accelerometer and gyro are aligned  
    // Pass gyro rate as rad/s
    MadgwickAHRSupdate(gx*PI/180.0f, -gy*PI/180.0f, gz*PI/180.0f, ax, -ay, az, -mx, -my, mz);
    
    microsNow = micros();
    
    if (microsNow - microsPrevious >= microsPerReading) 
    {
      
      roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      pitch = -1.0f *asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
      
      roll  *= 180.0f / PI;
      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI; 
      yaw   -= 21.2f;
      
      Serial.print(roll); Serial.print(" ");
      Serial.print(pitch); Serial.print(" ");
      Serial.println(yaw);
      
      //      Serial.print(q[0]); Serial.print(" ");
      //      Serial.print(q[1]); Serial.print(" ");
      //      Serial.print(q[2]); Serial.print(" ");
      //      Serial.println(q[3]);
      
      microsPrevious = microsPrevious + microsPerReading;
    }
    
    xSemaphoreGive( baton );
    delay(1);
  }
}

void codeForTask2( void * parameter )
{
  int httpResponseCode;
  HTTPClient http;
  int distance;
    
  for (;;) 
  {
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;

    xSemaphoreTake( baton, portMAX_DELAY );

    // DISTANCE
    while (!distanceSensor.checkForDataReady())
      delay(1);
    
    distance = distanceSensor.getDistance();
    distanceSensor.clearInterrupt();

    // SEND DATA TO DATABASE
    // check WiFi connection status
    if(WiFi.status() == WL_CONNECTED)
    {
      // domain name with URL path or IP address with path
      http.begin(serverName);
      // specify content-type header
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      // prepare HTTP POST request data
      String httpRequestData = "api_key="   + apiKeyValue     + 
                                "&device="   + markeriD        +
                                "&q0="       + String(q[0])    +
                                "&q1="       + String(q[1])    + 
                                "&q2="       + String(q[2])    +
                                "&q3="       + String(q[3])    +
                                "&distance=" + String(distance)+   
                                "";                         
      // send HTTP POST request
      httpResponseCode = http.POST(httpRequestData);
      // free resources
      http.end();
    }
    
    //    if (httpResponseCode>0){
    //      Serial.print("WEBStatus= "); Serial.println("OK");
    //    }
    //    else{
    //      Serial.print("WEBStatus= "); Serial.println("FAIL!");
    //    }
    
    xSemaphoreGive( baton );
    delay(1);
  }
}

void setup()
{
  Serial.begin(9600); 
  Wire.begin();

  // The ESP32 ID is essentially its MAC address(length: 6 bytes).
  char deviceiD[23];
  uint64_t chipid = ESP.getEfuseMac();
  uint16_t chip = (uint16_t)(chipid >> 32);
  snprintf(deviceiD, 23, "%04X%08X", chip, (uint32_t)chipid);
  String deviceiD_str(deviceiD);

  if(deviceiD_str == "2C6B9E286F24")
    markeriD = "0";
  else if(deviceiD_str == "E46A9E286F24")
    markeriD = "1";
  else if(deviceiD_str == "DC6A9E286F24")
    markeriD = "2";
  else if(deviceiD_str == "F4649E286F24")
    markeriD = "3";
  else if(deviceiD_str == "D0649E286F24")
    markeriD = "4";
  else if(deviceiD_str == "AC6B9E286F24")
    markeriD = "5";
  else if(deviceiD_str == "5C639E286F24")
    markeriD = "6";
  else if(deviceiD_str == "F46A9E286F24")
    markeriD = "7";
  else if(deviceiD_str == "24699E286F24")
    markeriD = "8";
  else if(deviceiD_str == "D4649E286F24")
    markeriD = "9";
  else if(deviceiD_str == "18699E286F24")
    markeriD = "10";
  else if(deviceiD_str == "886B9E286F24")
    markeriD = "11";
  else if(deviceiD_str == "F4629E286F24")
    markeriD = "12";
  else if(deviceiD_str == "00E851286F24")
    markeriD = "13";
  else
    markeriD = "14";

  if(imu.begin() == false)
  {
    Serial.println("\nError: LSM9DS1 not conected!");
    while (1);
  }

  imu.settings.gyro.sampleRate = 4;   // 238 Hz
  imu.settings.gyro.bandwidth = 1;    // GBW_med, 29 Hz at Godr = 238 Hz
  imu.settings.accel.sampleRate = 4;  // 238 Hz
  imu.settings.accel.bandwidth = -3;  // 50 Hz
  imu.settings.mag.sampleRate = 4;    // 10 Hz
  imu.settings.mag.XYPerformance = 2; // high performance
  imu.settings.mag.ZPerformance = 2;  // high performance

  //  GBW_low = 0,  // 14 Hz at Godr = 238 Hz,  33 Hz at Godr = 952 Hz
  //  GBW_med,      // 29 Hz at Godr = 238 Hz,  40 Hz at Godr = 952 Hz
  //  GBW_high,     // 63 Hz at Godr = 238 Hz,  58 Hz at Godr = 952 Hz
  //  GBW_highest   // 78 Hz at Godr = 238 Hz, 100 Hz at Godr = 952 Hz

  imu.calibrate(true);
  imu.calibrateMag(true);
  
  Serial.println("\n1. LSM9DS1 Conected.");

  // Distance Sensor Initialization
  bool state = false;

  while(!state){
    state = distanceSensor.checkBootState();
    delay(2);
  }

  if(distanceSensor.init() == 0){
    Serial.println("\n2. VL53L1X conected.");
  }
  else{
    Serial.println("\nError: VL53L1X not conected!");
    while (1);
  }

  // modify the default configuration
  distanceSensor.setDistanceModeShort();
  distanceSensor.setOffset(6); //0=0, 1=6
  distanceSensor.setIntermeasurementPeriod(500);
  distanceSensor.setTimingBudgetInMs(500);

  // enable the ranging
  distanceSensor.startRanging();

  WiFi.begin(ssid, password);
  
  while(WiFi.status() != WL_CONNECTED) 
  {    
      Serial.print("\t.");
      delay(500);
  }

  Serial.print("\n3. WiFi Conected: ");
  Serial.println(WiFi.localIP());

  baton = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    codeForTask1,
    "dataTask",
    10000,
    NULL,
    1,
    &Task1,
    0);
  delay(500);  // needed to start-up task1

  xTaskCreatePinnedToCore(
    codeForTask2,
    "sendTask",
    10000,
    NULL,
    1,
    &Task2,
    1);

  delay(500);
  
  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / sampleFrec;
  microsPrevious = micros();
  
} 

void loop() 
{
  delay(1);
}
