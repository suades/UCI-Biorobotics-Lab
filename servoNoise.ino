#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <Servo.h>

Servo myservo;
int servoPin = 3;       // Pin that the servomotor is connected to
int pos = 90;            // variable to store the servo position, varies from 0 to 180

LIS3MDL mag;
LSM6 imu;

  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  // example: min: { -1312,  -1675,   +866}    max: { +2954,  +2954,  +3612}
  LIS3MDL::vector<int16_t> m_min = {-832, +142, -4932};
  LIS3MDL::vector<int16_t> m_max = {+3223, +3686, +771};

// Time variables
unsigned long currentMillis;  // time in milliseconds
unsigned long oldMillis;  // previous measurement of time in millseconds
unsigned long oldServoMillis;  // previous measurement of time when we moved servo in millseconds

void setup() {
  Serial.begin(9600);
  Wire.begin();
  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize LIS3MDL magnetometer!");
    while (1);
  }
  mag.enableDefault();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize LSM6 IMU!");
    while (1);
  }
  imu.enableDefault();
  currentMillis = millis(); // initialize time
  oldMillis = currentMillis;
  

  myservo.attach(servoPin);               // attaches the servo toservoPin
}

void loop() {
  currentMillis = (float)millis();
  if ((currentMillis - oldMillis) > 10)
  {
    oldMillis = millis();
    mag.read();
    imu.read();
  /*
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector and
  north, in degrees.
  
  The default vector is chosen by the library to point along the
  surface of the PCB, in the direction of the top of the text on the
  silkscreen. This is the +X axis on the Pololu LSM303D carrier and
  the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
  carriers.
  
  To use a different vector as a reference, use the version of heading()
  that takes a vector argument; for example, use
  compass.heading((LSM303::vector<int>){0, 0, 1});
  to use the +Z axis as a reference.
  */
    float heading = computeHeading();
    
    Serial.print("0.0 ");  // this sets the lower limit of the serial plotter graph
    Serial.print("360.0 "); // this sets the upper limit of the serial plotter graph
    Serial.print(pos);  // servomotor angle
    Serial.print(" ");
    Serial.println(heading); // magnetometer heading
  
    if ((currentMillis - oldServoMillis) > 1000)  // move servomotor a little bit every 1000 milliseconds
    {
      if (pos == 90)
        {pos = 80;
         myservo.write(pos);}
      else
        {pos = 90;
         myservo.write(pos);}
      oldServoMillis = currentMillis;
    }
  }
}

template <typename T> float computeHeading(LIS3MDL::vector<T> from)
{
  LIS3MDL::vector<int32_t> temp_m = {mag.m.x, mag.m.y, mag.m.z};

  // copy acceleration readings from LSM6::vector into an LIS3MDL::vector
  LIS3MDL::vector<int16_t> a = {imu.a.x, imu.a.y, imu.a.z};

  // subtract offset (average of min and max) from magnetometer readings
  temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
  temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
  temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

  // compute E and N
  LIS3MDL::vector<float> E;
  LIS3MDL::vector<float> N;
  LIS3MDL::vector_cross(&temp_m, &a, &E);
  LIS3MDL::vector_normalize(&E);
  LIS3MDL::vector_cross(&a, &E, &N);
  LIS3MDL::vector_normalize(&N);

  // compute heading
  float heading = atan2(LIS3MDL::vector_dot(&E, &from), LIS3MDL::vector_dot(&N, &from)) * 180 / PI;
  if (heading < 0) heading += 360;
  return heading;
}

/*
Returns the angular difference in the horizontal plane between a
default vector (the +X axis) and north, in degrees.
*/
float computeHeading()
{
  return computeHeading((LIS3MDL::vector<int>){1, 0, 0});
}
