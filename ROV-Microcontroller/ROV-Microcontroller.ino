#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>

#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

// macros and pre-proccessor definitions
#define IMU_MESSAGE_LEN 9

// init global state variables updated in subscriber callbacks
int VertMotionSpeed = 0;
int HorizontalMotionSpeed = 0;
int lightSetting = 0;
int rotationSpeed = 0;
int cruiseEnabled = 0;

// i2c initialization
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// setup ROS node handlers
ros::NodeHandle  nh;

// setup ros message body and publisher instance
std_msgs::Float32MultiArray imu_vector;
ros::Publisher pub_imu_vec("imu_data", &imu_vector);

/**************** BEGIN UNTESTED CODE **********************/

// define subscriber callbacks
void VertMotionSpeedCb(const std_msgs::Int8& msg){
  VertMotionSpeed = msg.data;
}

void HorizontalMotionSpeedCb(const std_msgs::Int8& msg){
  HorizontalMotionSpeed = msg.data;
}

void LightSettingCb(const std_msgs::Int8& msg){
  lightSetting = msg.data;
}

void rotationSpeedCb(const std_msgs::Int8& msg){
  rotationSpeed = msg.data;
}

void cruiseEnabledCb(const std_msgs::Int8& msg){
  cruiseEnabled = msg.data;
}

// setup ros subcribers
ros::Subscriber<std_msgs::Int8> VertMotionSpeedSub("VertMotionSpeed", &VertMotionSpeedCb);
ros::Subscriber<std_msgs::Int8> HorizontalMotionSpeedSub("HorizontalMotionSpeed", &HorizontalMotionSpeedCb);
ros::Subscriber<std_msgs::Int8> LightSettingSub("LightSetting", &LightSettingCb);
ros::Subscriber<std_msgs::Int8> rotationSpeedSub("rotationSpeed", &rotationSpeedCb);
ros::Subscriber<std_msgs::Int8> cruiseEnabledSub("cruiseEnabled", &cruiseEnabledCb);

/**************** END UNTESTED CODE **********************/

void setupSensor()
{
  // Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  
  // Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  // Setup the gyroscope
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

  // setup imu_vector message array
  imu_vector.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  imu_vector.layout.dim[0].label = "imu_vec";
  imu_vector.layout.dim[0].size = IMU_MESSAGE_LEN;
  imu_vector.layout.dim[0].stride = IMU_MESSAGE_LEN;
  imu_vector.layout.data_offset = 0;
  imu_vector.data = (float *)malloc(sizeof(float)*IMU_MESSAGE_LEN);
  imu_vector.data_length = IMU_MESSAGE_LEN;

  // advertise imu_vector publisher
  nh.advertise(pub_imu_vec);

/**************** BEGIN UNTESTED CODE **********************/

  // setup subscribers
  nh.subscribe(VertMotionSpeedSub);
  nh.subscribe(HorizontalMotionSpeedSub);
  nh.subscribe(LightSettingSub);
  nh.subscribe(rotationSpeedSub);
  nh.subscribe(cruiseEnabledSub);

/**************** END UNTESTED CODE **********************/

}

void loop()
{
  // attempt to read from sensor
  lsm.read();

  // get new sensor event handler
  sensors_event_t a, m, g, temp;

  // update event handlers
  lsm.getEvent(&a, &m, &g, &temp); 

  // populate imu_vector with data from IMU
  imu_vector.data[0] = a.acceleration.x;
  imu_vector.data[1] = a.acceleration.y;
  imu_vector.data[2] = a.acceleration.z;

  imu_vector.data[3] = m.magnetic.x;
  imu_vector.data[4] = m.magnetic.y;
  imu_vector.data[5] = m.magnetic.z;

  imu_vector.data[6] = g.gyro.x;
  imu_vector.data[7] = g.gyro.y;
  imu_vector.data[8] = g.gyro.z;

  // publish the vector
  pub_imu_vec.publish(&imu_vector);

  nh.spinOnce();
  delay(250);
}
