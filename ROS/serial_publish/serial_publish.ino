// Sketch to publish force or sensor data to serial to be read by pySerial in ROS
// or publish sensor value to ROS from Arduino using predefined calibration matrices

// #define ROS  //Publish forces to ROS as a publisher node
#define SP  //Read data directly from serial port with pySerial
#define FORCE  //Print calculated force data
// #define SENSOR  //Print raw sensor data

#include "ros.h"
#include <geometry_msgs/Wrench.h>
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

BLA::Matrix<7> leftSensorData = { 0, 0, 0, 0, 0, 0, 1 };
BLA::Matrix<7> rightSensorData = { 0, 0, 0, 0, 0, 0, 1 };

BLA::Matrix<3> FL = { 0, 0, 0 };
BLA::Matrix<3> FR = { 0, 0, 0 };

BLA::Matrix<3> FLinit = { 0, 0, 0 };
BLA::Matrix<3> FRinit = { 0, 0, 0 };

BLA::Matrix<3, 7> leftCalMat = {1.2420, 0.0352, -0.0076, -1.2408, 0.3257, 0.1642, 0.0906, 
                                0.2667, 0.9489, -0.4970, -0.2718, 0.2564, -1.3695, 0.0038, 
                                3.7836, -2.3230, -1.5742, -4.1823, 2.0285, 2.2484, 0.0852};

BLA::Matrix<3, 7> rightCalMat = {1.2420, 0.0352, -0.0076, -1.2408, 0.3257, 0.1642, 0.0906, 
                                 0.2667, 0.9489, -0.4970, -0.2718, 0.2564, -1.3695, 0.0038, 
                                 3.7836, -2.3230, -1.5742, -4.1823, 2.0285, 2.2484, 0.0852};

const int pinRead[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};
// int sequence = 0;

#ifdef ROS
ros::NodeHandle nh;

geometry_msgs::Wrench leftWrench;
geometry_msgs::Wrench rightWrench;

ros::Publisher leftWrench_pub("leftWrench", &leftWrench);
ros::Publisher rightWrench_pub("rightWrench", &rightWrench);
#endif

void setup() {

  Serial.begin(57600);

#ifdef ROS
  nh.initNode();
  nh.advertise(leftWrench_pub);
  nh.advertise(rightWrench_pub);
#endif

  for (int i = 0; i < 12; i++) {
    if(i < 6){
      leftSensorData(i) = analogRead(pinRead[i]) * (3.3 / 1024.0);
    }
    else{
      rightSensorData(i-6) = analogRead(pinRead[i]) * (3.3 / 1024.0);
    }
  }
  FLinit = leftCalMat * leftSensorData;
  FRinit = rightCalMat * rightSensorData;
}

void loop() {
  for (int i = 0; i < 12; i++) {
    if(i < 6){
      leftSensorData(i) = analogRead(pinRead[i]) * (3.3 / 1024.0);
    }
    else{
      rightSensorData(i-6) = analogRead(pinRead[i]) * (3.3 / 1024.0);
    }
  }
  FL = leftCalMat * leftSensorData - FLinit;
  FR = rightCalMat * rightSensorData - FRinit;
  
#ifdef SP
  // Serial.print(sequence);
  // Serial.print(", ");
  #ifdef SENSOR
    // Serial.print("Sensor data: Left: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(leftSensorData(i));
      Serial.print(", ");
    }
      // Serial.print(leftSensorData(5));
      // Serial.print(", Right: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(rightSensorData(i));
      while (i < 5){
        Serial.print(", ");
        break;
      }
    }
    Serial.print('\n');
  #endif

  #ifdef FORCE
    // Serial.print(" Force: Left: ");
    for (int i = 0; i < 3; i++) {
      Serial.print(FL(i));
      Serial.print(", ");
    }
    // Serial.print(FL(2));
    // Serial.print(", Right: ");
    for (int i = 0; i < 3; i++) {
      Serial.print(FR(i));
      while (i < 2){
        Serial.print(", ");
        break;
      }
    }
    Serial.print('\n');
  #endif
  // sequence++;
  delay(10);
#endif

#ifdef ROS
  leftWrench.force.x = FL(0);
  leftWrench.force.y = FL(1);
  leftWrench.force.z = FL(2);
  rightWrench.force.x = FR(0);
  rightWrench.force.y = FR(1);
  rightWrench.force.z = FR(2);

  leftWrench_pub.publish(&leftWrench);
  rightWrench_pub.publish(&rightWrench);

  nh.spinOnce();
#endif
}