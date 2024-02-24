# MicroMouse Sensors

Please note that some of sensors are interfaced with a unique address over an I2C bus while others are directly connected to digital read pins that that just transiting binary data 

## Accelerometer and Gyro (MPU)

### Code

```c++
# include <Wire.h>

const int MPU_addr=0x68;  // I2C address of MPU-6500
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6500)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // read 14 bytes from MPU-6500
  AcX=Wire.read()<<8|Wire.read();     // concatenate high and low byte for each axis
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  Tmp=Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();

  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  // convert to Celsius
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(1000);
}
```

### Example Data

```bash
AcX = -707 | AcY = -1 | AcZ = 3674 | Tmp = 50.93 | GyX = 112 | GyY = -92 | GyZ = 15
...
```

### Comments

> I2C

> Request Stream `Wire.requestFrom(MPU_addr,14,true);` and process data 

> The accuracy of this sensor is not the best

## Encoders

### Code

```c++
#define pinIrA 22
#define pinIrB 24

void setup() {
  Serial.begin(9600);

  // Encoder (A)
  pinMode(pinIrA, INPUT);
  // Encoder (B)
  pinMode(pinIrB, INPUT);
}

void loop() {

  if (digitalRead(pinIrA) == 0) {
    Serial.write("Detected (A)\n");
  }

  if (digitalRead(pinIrB) == 0) {
    Serial.write("Detected (B)\n");
  }

}
```

### Example Data

```bash
Detected (A)
Detected (B)
```

### Comments

> Digital Pin (NOT I2C)

> Conditional stream when read `digitalRead(pinIrA) == 0`

> The sensor is bulky and may not be of the best use in the final design due to the size constraints

## Time of Flight (TOF)

### Code

```c++

#include <Arduino.h>

/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;


// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
#define HIGH_ACCURACY


void setup()
{
  Serial.begin(9600);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
}

void loop()
{
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
}
```

### Example Data

```bash
123
400
...
```

### Comments

> I2C

> Streamed in loop with `sensor.readRangeSingleMillimeters()`

> Works perfectly, surface has to be close to perpendicular to sensor beam
