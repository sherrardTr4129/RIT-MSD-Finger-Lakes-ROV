#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>

#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include "MS5837.h"
#include "Servo.h"

/************ pre-proccessor definitions ***************/
#define IMU_MESSAGE_LEN      9
#define PRESS_MESSAGE_LEN    3
#define FLUID_DENSITY        997 // kg/m^3 (freshwater, 1029 for seawater)

// Define Motor Pins (needs to be changed)
#define LEFT_VERT_MOTOR_PIN      7
#define RIGHT_VERT_MOTOR_PIN     8
#define LEFT_HORIZ_MOTOR_PIN     9
#define RIGHT_HORIZ_MOTOR_PIN    10

// Define Light Controller Pins (needs to be changed)
#define LIGHT_PIN_1       6
#define LIGHT_PIN_2       5
#define LIGHT_PIN_3       4

// Define Motor Speed Settings (needs to be changed)
#define LEFT_VERT_MOTOR_STOP        1500
#define LEFT_VERT_MOTOR_FWD_SLOW    1600
#define LEFT_VERT_MOTOR_FWD_FAST    1850
#define LEFT_VERT_MOTOR_REV_SLOW    1300
#define LEFT_VERT_MOTOR_REV_FAST    1100

#define RIGHT_VERT_MOTOR_STOP        1500
#define RIGHT_VERT_MOTOR_FWD_SLOW    1600
#define RIGHT_VERT_MOTOR_FWD_FAST    1850
#define RIGHT_VERT_MOTOR_REV_SLOW    1300
#define RIGHT_VERT_MOTOR_REV_FAST    1100

#define LEFT_HORIZ_MOTOR_STOP        1500
#define LEFT_HORIZ_MOTOR_FWD_SLOW    1600
#define LEFT_HORIZ_MOTOR_FWD_FAST    1850
#define LEFT_HORIZ_MOTOR_REV_SLOW    1300
#define LEFT_HORIZ_MOTOR_REV_FAST    1100

#define RIGHT_HORIZ_MOTOR_STOP        1500
#define RIGHT_HORIZ_MOTOR_FWD_SLOW    1600
#define RIGHT_HORIZ_MOTOR_FWD_FAST    1850
#define RIGHT_HORIZ_MOTOR_REV_SLOW    1300
#define RIGHT_HORIZ_MOTOR_REV_FAST    1100

// Define Light Controller States (needs to be changed)
#define LIGHT_PIN_ON      200
#define LIGHT_PIN_OFF     0

// init global state variables updated in subscriber callbacks
int VertMotionSpeed = 0;
int HorizontalMotionSpeed = 0;
int lightSetting = 0;
int rotationSpeed = 0;
int cruiseEnabled = 0;

// Init Servo Objects For Motor Controllers
Servo RightVert;
Servo LeftVert;
Servo RightHoriz;
Servo LeftHoriz;

// i2c initialization
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// init pressure sensor
MS5837 PresSensor;

// setup ROS node handlers
ros::NodeHandle  nh;

// setup ros message body and publisher instance
std_msgs::Float32MultiArray imu_vector;
ros::Publisher pub_imu_vec("imu_data", &imu_vector);

std_msgs::Float32MultiArray press_vector;
ros::Publisher pub_BR_vec("BR_data", &press_vector);

/**************** BEGIN UNTESTED CODE **********************/
// define subscriber callbacks
void VertMotionSpeedCb(const std_msgs::Int8& msg) {
  VertMotionSpeed = msg.data;
}

void HorizontalMotionSpeedCb(const std_msgs::Int8& msg) {
  HorizontalMotionSpeed = msg.data;
}

void LightSettingCb(const std_msgs::Int8& msg) {
  lightSetting = msg.data;
}

void rotationSpeedCb(const std_msgs::Int8& msg) {
  rotationSpeed = msg.data;
}

void cruiseEnabledCb(const std_msgs::Int8& msg) {
  cruiseEnabled = msg.data;
}

// setup ros subcribers
ros::Subscriber<std_msgs::Int8> VertMotionSpeedSub("VertMotionSpeed", &VertMotionSpeedCb);
ros::Subscriber<std_msgs::Int8> HorizontalMotionSpeedSub("HorizontalMotionSpeed", &HorizontalMotionSpeedCb);
ros::Subscriber<std_msgs::Int8> LightSettingSub("LightSetting", &LightSettingCb);
ros::Subscriber<std_msgs::Int8> rotationSpeedSub("rotationSpeed", &rotationSpeedCb);
ros::Subscriber<std_msgs::Int8> cruiseEnabledSub("cruiseEnabled", &cruiseEnabledCb);

// init functions to write speed values to motors and light controller
void setLightControllerState(int setting)
{
  switch (setting)
  {
    case 0:
      analogWrite(LIGHT_PIN_1, LIGHT_PIN_OFF);
      analogWrite(LIGHT_PIN_2, LIGHT_PIN_OFF);
      analogWrite(LIGHT_PIN_3, LIGHT_PIN_OFF);
      break;

    case 1:
      analogWrite(LIGHT_PIN_1, LIGHT_PIN_ON);
      analogWrite(LIGHT_PIN_2, LIGHT_PIN_OFF);
      analogWrite(LIGHT_PIN_3, LIGHT_PIN_OFF);
      break;

    case 2:
      analogWrite(LIGHT_PIN_1, LIGHT_PIN_ON);
      analogWrite(LIGHT_PIN_2, LIGHT_PIN_ON);
      analogWrite(LIGHT_PIN_3, LIGHT_PIN_OFF);
      break;

    case 3:
      analogWrite(LIGHT_PIN_1, LIGHT_PIN_ON);
      analogWrite(LIGHT_PIN_2, LIGHT_PIN_ON);
      analogWrite(LIGHT_PIN_3, LIGHT_PIN_ON);
      break;
  }
}

void setVertMotorSpeeds(int setting)
{
  switch (setting)
  {
    case -2:
      LeftVert.writeMicroseconds(LEFT_VERT_MOTOR_REV_FAST);
      RightVert.writeMicroseconds(RIGHT_VERT_MOTOR_REV_FAST);
      break;

    case -1:
      LeftVert.writeMicroseconds(LEFT_VERT_MOTOR_REV_SLOW);
      RightVert.writeMicroseconds(RIGHT_VERT_MOTOR_REV_SLOW);
      break;

    case 0:
      LeftVert.writeMicroseconds(LEFT_VERT_MOTOR_STOP);
      RightVert.writeMicroseconds(RIGHT_VERT_MOTOR_STOP);
      break;

    case 1:
      LeftVert.writeMicroseconds(LEFT_VERT_MOTOR_FWD_SLOW);
      RightVert.writeMicroseconds(RIGHT_VERT_MOTOR_FWD_SLOW);
      break;

    case 2:
      LeftVert.writeMicroseconds(LEFT_VERT_MOTOR_FWD_FAST);
      RightVert.writeMicroseconds(RIGHT_VERT_MOTOR_FWD_FAST);
      break;
  }
}

void setHorizMotorSpeeds(int setting)
{
  switch (setting)
  {
    case -2:
      LeftHoriz.writeMicroseconds(LEFT_HORIZ_MOTOR_REV_FAST);
      RightHoriz.writeMicroseconds(RIGHT_HORIZ_MOTOR_REV_FAST);
      break;

    case -1:
      LeftHoriz.writeMicroseconds(LEFT_HORIZ_MOTOR_REV_SLOW);
      RightHoriz.writeMicroseconds(RIGHT_HORIZ_MOTOR_REV_SLOW);
      break;

    case 0:
      LeftHoriz.writeMicroseconds(LEFT_HORIZ_MOTOR_STOP);
      RightHoriz.writeMicroseconds(RIGHT_HORIZ_MOTOR_STOP);
      break;

    case 1:
      LeftHoriz.writeMicroseconds(LEFT_HORIZ_MOTOR_FWD_SLOW);
      RightHoriz.writeMicroseconds(RIGHT_HORIZ_MOTOR_FWD_SLOW);
      break;

    case 2:
      LeftHoriz.writeMicroseconds(LEFT_HORIZ_MOTOR_FWD_FAST);
      RightHoriz.writeMicroseconds(RIGHT_HORIZ_MOTOR_FWD_FAST);
      break;
  }
}

void setRotationalSpeed(int setting)
{
  switch (setting)
  {
    case -2:
      LeftHoriz.writeMicroseconds(LEFT_HORIZ_MOTOR_REV_FAST);
      RightHoriz.writeMicroseconds(RIGHT_HORIZ_MOTOR_FWD_FAST);
      break;

    case -1:
      LeftHoriz.writeMicroseconds(LEFT_HORIZ_MOTOR_REV_SLOW);
      RightHoriz.writeMicroseconds(RIGHT_HORIZ_MOTOR_FWD_SLOW);
      break;

    case 0:
      LeftHoriz.writeMicroseconds(LEFT_HORIZ_MOTOR_STOP);
      RightHoriz.writeMicroseconds(RIGHT_HORIZ_MOTOR_STOP);
      break;

    case 1:
      LeftHoriz.writeMicroseconds(LEFT_HORIZ_MOTOR_FWD_SLOW);
      RightHoriz.writeMicroseconds(RIGHT_HORIZ_MOTOR_REV_SLOW);
      break;

    case 2:
      LeftHoriz.writeMicroseconds(LEFT_HORIZ_MOTOR_FWD_FAST);
      RightHoriz.writeMicroseconds(RIGHT_HORIZ_MOTOR_REV_FAST);
      break;
  }
}
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


  /**************** BEGIN UNTESTED CODE **********************/
  // setup Pressure Sensor
  Wire.begin();
  if (!PresSensor.init())
  {
    Serial.println("Could Not Initialize Blue Robotics Pressure/Temp Sensor!");
    while (1);
  }

  PresSensor.setModel(MS5837::MS5837_30BA);
  PresSensor.setFluidDensity(FLUID_DENSITY);

  // attach servo objects to pins
  RightVert.attach(RIGHT_VERT_MOTOR_PIN);
  LeftVert.attach(LEFT_VERT_MOTOR_PIN);
  RightHoriz.attach(RIGHT_HORIZ_MOTOR_PIN);
  LeftHoriz.attach(LEFT_HORIZ_MOTOR_PIN);

  //Initilize Motors With Stop Signals
  Serial.println("Initializing Motors!");
  RightVert.writeMicroseconds(RIGHT_VERT_MOTOR_STOP);
  LeftVert.writeMicroseconds(LEFT_VERT_MOTOR_STOP);
  RightHoriz.writeMicroseconds(RIGHT_HORIZ_MOTOR_STOP);
  LeftHoriz.writeMicroseconds(LEFT_HORIZ_MOTOR_STOP);

  //delay for seven seconds to allow for ESCs to recognize stop signals
  delay(7000);
  Serial.println("Done Initializing Motors!");
  /**************** END UNTESTED CODE **********************/

  // init ROS node
  nh.initNode();

  // setup imu_vector message array
  imu_vector.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  imu_vector.layout.dim[0].label = "imu_vec";
  imu_vector.layout.dim[0].size = IMU_MESSAGE_LEN;
  imu_vector.layout.dim[0].stride = IMU_MESSAGE_LEN;
  imu_vector.layout.data_offset = 0;
  imu_vector.data = (float *)malloc(sizeof(float) * IMU_MESSAGE_LEN);
  imu_vector.data_length = IMU_MESSAGE_LEN;

  /**************** BEGIN UNTESTED CODE **********************/
  // setup pressure_vector message array
  press_vector.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  press_vector.layout.dim[0].label = "press_vec";
  press_vector.layout.dim[0].size = PRESS_MESSAGE_LEN;
  press_vector.layout.dim[0].stride = PRESS_MESSAGE_LEN;
  press_vector.layout.data_offset = 0;
  press_vector.data = (float *)malloc(sizeof(float) * PRESS_MESSAGE_LEN);
  press_vector.data_length = PRESS_MESSAGE_LEN;

  // advertise press_vector publisher
  nh.advertise(pub_BR_vec);
  /**************** END UNTESTED CODE **********************/

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
  // attempt to read from IMU
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

  /**************** BEGIN UNTESTED CODE **********************/
  // Update pressure and temperature readings
  PresSensor.read();

  // populate BR data vector with obtained values.
  press_vector.data[0] = (float) PresSensor.pressure();
  press_vector.data[1] = (float) PresSensor.temperature();
  press_vector.data[2] = (float) PresSensor.depth();

  // publish BR sensor data vector
  pub_BR_vec.publish(&press_vector);
  /**************** END UNTESTED CODE **********************/

  // publish the IMU vector
  pub_imu_vec.publish(&imu_vector);

  /**************** BEGIN UNTESTED CODE **********************/
  //update light controller states
  setLightControllerState(lightSetting);

  // update motor speeds based on controller input
  if (cruiseEnabled != 1)
  {
    setVertMotorSpeeds(VertMotionSpeed);
    delay(100); // motor update delay

    setHorizMotorSpeeds(HorizontalMotionSpeed);
    delay(100); // motor update delay

    setRotationalSpeed(rotationSpeed);
    delay(100); // motor update delay
  }

  else if (cruiseEnabled == 1)
  {
    // if cruise control enabled, create copies of current speed settings
    int VertMotionSpeedTemp = VertMotionSpeed;
    int HorizontalMotionSpeedTemp = HorizontalMotionSpeed;
    int rotationSpeedTemp = rotationSpeed;

    // keep updating motors with current speed settings
    while (cruiseEnabled == 1)
    {
      setVertMotorSpeeds(VertMotionSpeedTemp);
      delay(100);

      setHorizMotorSpeeds(HorizontalMotionSpeedTemp);
      delay(100);

      setRotationalSpeed(rotationSpeedTemp);
      delay(100);
    }
  }
  /**************** END UNTESTED CODE **********************/

  nh.spinOnce();
  delay(50); // loop refresh delay
}
