#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// setup ROS node handlers
ros::NodeHandle  nh;
std_msgs::Float32MultiArray imu_vector;
ros::Publisher pub_imu_vec("imu_data", &imu_vector);

// setup floats for acceleration vector
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

// setup floats for magnetometer vector
float magX = 0.0;
float magY = 0.0;
float magZ = 0.0;

// setup floats for gyro vector
float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void setup() 
{
  Serial.begin(115200);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println("LSM9DS1 data read demo");

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();

  // init ROS node
  nh.initNode();

  // advertise imu publisher
  nh.advertise(pub_imu_vec);

  // set data length for imu data
  imu_vector.data_length = 9;
}

void loop()
{
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  // update acceleration vector componenets
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;

  // update magnetometer vector components
  magX = m.magnetic.x;
  magY = m.magnetic.y;
  magZ = m.magnetic.z;

  // update gyro vector components
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;

  // populate imu_vector with data
  imu_vector.data[0] = accelX;
  imu_vector.data[1] = accelY;
  imu_vector.data[2] = accelZ;

  imu_vector.data[3] = magX;
  imu_vector.data[4] = magY;
  imu_vector.data[5] = magZ;

  imu_vector.data[6] = gyroX;
  imu_vector.data[7] = gyroY;
  imu_vector.data[8] = gyroZ;

  // publish the vector
  pub_imu_vec.publish(&imu_vector);

  nh.spinOnce();

  delay(200);
}
