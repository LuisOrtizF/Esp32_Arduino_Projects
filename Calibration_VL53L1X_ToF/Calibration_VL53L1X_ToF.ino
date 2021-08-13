#include <Wire.h>
#include "SparkFun_VL53L1X.h"

SFEVL53L1X distanceSensor;

void setup(void)
{
  Wire.begin();
  
  Serial.begin(9600);

  delay(2000);

  Serial.println();
  Serial.println("***********************************************************************************************");
  Serial.println("                                    Offset calibration");
  Serial.println("Place a light grey (17 % gray) target at a distance of 140mm in front of the VL53L1X sensor.");
  Serial.println("***********************************************************************************************");
  Serial.println();
  
  // Distance Sensor Initialization
  bool state = false;

  while(!state)
  {
    state = distanceSensor.checkBootState();
    delay(2);
  }

  /* Sensor Initialization */
  if(distanceSensor.init() == 0)
  {
    Serial.println("\n1. VL53L1X conected.");
  }
  else{
    Serial.println("\nError: VL53L1X not conected!");
    while (1)
      ;
  }

  // Short mode max distance is limited to 1.3 m but has a better ambient immunity.
  // Above 1.3 meter error 4 is thrown (wrap around).
  distanceSensor.setDistanceModeShort();
  distanceSensor.setTimingBudgetInMs(50);
  // measure periodically. Intermeasurement period must be >/= timing budget.
  distanceSensor.setIntermeasurementPeriod(100);
  distanceSensor.startRanging(); // Start once

  int tinit = 0;

  while (tinit < 15)
  {
    while (!distanceSensor.checkForDataReady())
      delay(1);
    
    int d_before = distanceSensor.getDistance();
    
    distanceSensor.clearInterrupt();
    
    Serial.print("Distance before calibration (mm): ");
    Serial.println(d_before);
    
    tinit++;
  }

  /*
   * Place a target, 17 % gray, at a distance of 140 mm from the sensor and call the VL53L1X_CalibrateOffset (dev, 140, &offset) function.
   * The calibration may take a few seconds. The offset correction is applied to the sensor at the end of calibration.
   *
   * The calibration function takes 50 measurements and then takes the difference between the target distance
   * and the average distance and then calls setOffset() with this value. Thats all. No magic.
   */

  distanceSensor.calibrateOffset(140);
  Serial.print("2. Result of offset calibration. RealDistance - MeasuredDistance = ");
  Serial.print(distanceSensor.getOffset());
  Serial.print(" mm");
  Serial.println();

  // TEST CALIBRATION

  distanceSensor.setOffset(distanceSensor.getOffset());

  distanceSensor.startRanging(); // Start once

  int tend = 0;

  while (tend < 15)
  {
    while (!distanceSensor.checkForDataReady())
      delay(1);
    
    int d_after = distanceSensor.getDistance();
    
    distanceSensor.clearInterrupt();
    
    Serial.print("Distance after calibration (mm): ");
    Serial.println(d_after);
    
    tend++;
  }

  //  //Crosstalk calibration procedure
  //  //Crosstalk calibration should be conducted in a dark environment, with no IR contribution.
  //  //Place a 17 % reflectance chart at the crosstalk calibration distance (xcd)
  //  //Zero means there is no crosstalk compensation. The unit is cps (counts per second).
  //  //distanceSensor.calibrateXTalk(review UM2510 ST doc);
}

void loop(void)
{

}
