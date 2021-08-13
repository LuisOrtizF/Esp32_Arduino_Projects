#include <Wire.h>
#include "SparkFun_VL53L1X.h"

SFEVL53L1X distanceSensor;

void setup(void)
{
  Wire.begin();
  Serial.begin(115200);

  bool state = false;

  while(!state){
    state = distanceSensor.checkBootState();
    delay(5);
  }

  /* Sensor Initialization */
  if(distanceSensor.init() == 0)
      Serial.println("3. VL53L1X conected.");
  else
      Serial.println("Error: VL53L1X not conected!");

  /* Modify the default configuration */
  distanceSensor.setDistanceModeShort(); 
  distanceSensor.setROI(16,16);
  distanceSensor.setOffset(2047);
  //We'd recommend 20, 33, and 100 ms for short, medium and long distance modes respectively.
  //Must be greater than or equal to the timing budget.  
  distanceSensor.setIntermeasurementPeriod(500);
  //The TB can be adjusted to improve the standard deviation 
  //(SD) of the measurement. Increasing the TB,
  //decreases the SD but increases the power consumption. 
  distanceSensor.setTimingBudgetInMs(500);

  /* enable the ranging*/
  distanceSensor.startRanging();

}

void loop(void)
{
  /* ranging loop */
  bool dataReady = false;

  while(!dataReady)
    dataReady = distanceSensor.checkForDataReady();

  byte rangeStatus = distanceSensor.getRangeStatus();
  Serial.print("\tRange Status: ");
  
  // Make it human readable
  switch (rangeStatus)
  {
    case 0:
    Serial.print("Good");
    break;
    case 1:
    Serial.print("Signal fail");
    break;
    case 2:
    Serial.print("Sigma fail");
    break;
    case 7:
    Serial.print("Wrapped target fail");
    break;
    default:
    Serial.print("Unknown: ");
    Serial.print(rangeStatus);
    break;
  }
  
  Serial.println();
  
  int distance = distanceSensor.getDistance();
  Serial.print("Distance(mm): ");
  Serial.println(distance);
  
  distanceSensor.clearInterrupt();
  
}
