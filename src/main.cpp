#include <Arduino.h>

// /* This example shows how to get single-shot range
//  measurements from the VL53L0X. The sensor can optionally be
//  configured with different ranging profiles, as described in
//  the VL53L0X API user manual, to get better performance for
//  a certain application. This code is based on the four
//  "SingleRanging" examples in the VL53L0X API.

//  The range readings are in units of mm. */

// #include <Wire.h>
// #include <VL53L0X.h>

// VL53L0X sensor;

// // Uncomment this line to use long range mode. This
// // increases the sensitivity of the sensor and extends its
// // potential range, but increases the likelihood of getting
// // an inaccurate reading because of reflections from objects
// // other than the intended target. It works best in dark
// // conditions.

// // #define LONG_RANGE

// // Uncomment ONE of these two lines to get
// // - higher speed at the cost of lower accuracy OR
// // - higher accuracy at the cost of lower speed

// //#define HIGH_SPEED
// #define HIGH_ACCURACY

// void setup()
// {
//   Serial.begin(9600);
//   Wire.begin();

//   sensor.setTimeout(500);
//   if (!sensor.init())
//   {
//     Serial.println("Failed to detect and initialize sensor!");
//     while (1) {}
//   }

// #if defined LONG_RANGE
//   // lower the return signal rate limit (default is 0.25 MCPS)
//   sensor.setSignalRateLimit(0.1);
//   // increase laser pulse periods (defaults are 14 and 10 PCLKs)
//   sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
//   sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
// #endif

// #if defined HIGH_SPEED
//   // reduce timing budget to 20 ms (default is about 33 ms)
//   sensor.setMeasurementTimingBudget(20000);
// #elif defined HIGH_ACCURACY
//   // increase timing budget to 200 ms
//   sensor.setMeasurementTimingBudget(200000);
// #endif
// }

// void loop()
// {
//   Serial.print(sensor.readRangeSingleMillimeters());
//   if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

//   Serial.println();
// }

// #include "Adafruit_VL53L0X.h"

// Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// void setup() {
//   lox.begin(0x2B);
// }

// void loop(){

// }

// #include <Wire.h>

// void setup()
// {
//   pinMode(15, OUTPUT);
//   Wire.begin();

//   Serial.begin(9600);
//   while (!Serial)
//     ; // Leonardo: wait for serial monitor
//   Serial.println("\nI2C Scanner");
// }

// void loop()
// {
//   digitalWrite(15, LOW);
//   byte error, address;
//   int nDevices;

//   Serial.println("Scanning...");

//   nDevices = 0;
//   for (address = 1; address < 127; address++)
//   {
//     // The i2c_scanner uses the return value of
//     // the Write.endTransmisstion to see if
//     // a device did acknowledge to the address.
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0)
//     {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.print(address, HEX);
//       Serial.println("  !");

//       nDevices++;
//     }
//     else if (error == 4)
//     {
//       Serial.print("Unknown error at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.println(address, HEX);
//     }
//   }
//   if (nDevices == 0)
//     Serial.println("No I2C devices found\n");
//   else
//     Serial.println("done\n");

//   delay(5000); // wait 5 seconds for next scan
// }

# define LED_WHITE 23
# define LED_BLUE 19
# define LED_ORANGE 18
# define LED_RED 5

# define INT1 26
# define INT2 25
# define INT3 33
# define INT4 32

# define BUTTON1 34
# define BUTTON2 35

void motorForward() {
  digitalWrite(INT1, HIGH);
  digitalWrite(INT2, LOW);
  digitalWrite(INT3, HIGH);
  digitalWrite(INT4, LOW);
}

void motorBackward() {
  digitalWrite(INT1, LOW);
  digitalWrite(INT2, HIGH);
  digitalWrite(INT3, LOW);
  digitalWrite(INT4, HIGH);
}

void motorStop() {
  digitalWrite(INT1, LOW);
  digitalWrite(INT2, LOW);
  digitalWrite(INT3, LOW);
  digitalWrite(INT4, LOW);
}

int button1State = 0;
int button2State = 0;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  // pinMode(23, OUTPUT); 
  // Set motor control pins as OUTPUT
  pinMode(INT1, OUTPUT);
  pinMode(INT2, OUTPUT);
  pinMode(INT3, OUTPUT);
  pinMode(INT4, OUTPUT);

  // // LEDs 
  // pinMode(LED_WHITE, OUTPUT);
  // pinMode(LED_BLUE, OUTPUT);
  // pinMode(LED_ORANGE, OUTPUT);
  // pinMode(LED_RED, OUTPUT);

  // // Buttons
  // pinMode(BUTTON1, INPUT);
  // pinMode(BUTTON2, INPUT);
}

// the loop function runs over and over again forever
void loop() {
  // button1State = digitalRead(BUTTON1);
  // button2State = digitalRead(BUTTON2);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
//   if (button1State == HIGH) {
//     // turn LED on:
//     digitalWrite(LED_WHITE, HIGH);
//   } else {
//     // turn LED off:
//     digitalWrite(LED_WHITE, LOW);
//   }


// if (button2State == HIGH) {
//     // turn LED on:
//     digitalWrite(LED_BLUE, HIGH);
//   } else {
//     // turn LED off:
//     digitalWrite(LED_BLUE, LOW);
//   }

  // Move the motor forward
  motorForward();
  delay(2000);  // Wait for 2 seconds

  // Stop the motor
  motorStop();
  delay(1000);  // Wait for 1 second

  // Move the motor backward
  motorBackward();
  delay(2000);  // Wait for 2 seconds

  // Stop the motor
  motorStop();
  delay(1000);  // Wait for 1 second


  // digitalWrite(23, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(1000);                      // wait for a second
  // digitalWrite(23, LOW);   // turn the LED off by making the voltage LOW
  // delay(1000);                      // wait for a second
}
